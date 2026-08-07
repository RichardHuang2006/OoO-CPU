#include "cpu.h"

#include <algorithm>

#include "alu.h"

namespace {

// OP-IMM is the only class taking its second operand from the immediate;
// ADDI and ADD share an Op because the function unit does not care where the
// operand came from.
bool uses_immediate(const Decoded& d) { return (d.raw & 0x7Fu) == 0x13u; }

// The proxy-kernel exit syscall: ecall with a7 == 93 halts and yields a0.
constexpr uint32_t SYS_EXIT = 93;
constexpr ArchReg  REG_A0   = 10;
constexpr ArchReg  REG_A7   = 17;

// Bytes touched by a memory op, which is what decides overlap in the queues.
uint8_t access_size(Op op) {
    switch (op) {
    case Op::LB: case Op::LBU: case Op::SB: return 1;
    case Op::LH: case Op::LHU: case Op::SH: return 2;
    default:                                return 4;
    }
}

// Widen loaded bytes the way the opcode asks. Forwarded and fetched data go
// through the same widening, so the two paths cannot disagree.
uint32_t extend_load(Op op, uint32_t v) {
    switch (op) {
    case Op::LB: return static_cast<uint32_t>(static_cast<int32_t>(static_cast<int8_t> (v)));
    case Op::LH: return static_cast<uint32_t>(static_cast<int32_t>(static_cast<int16_t>(v)));
    default:     return v;
    }
}

// x1 and x5 are the link registers, which is how the ISA tells the predictor
// a jump is really a call or a return.
bool is_link(ArchReg r) { return r == 1 || r == 5; }

BranchKind kind_of(const Decoded& d) {
    if (!d.is_branch) return BranchKind::NONE;
    if (d.op == Op::JAL)  return is_link(d.rd) ? BranchKind::CALL : BranchKind::JUMP;
    if (d.op == Op::JALR) {
        if (is_link(d.rd))  return BranchKind::CALL;
        if (is_link(d.rs1)) return BranchKind::RETURN;
        return BranchKind::JUMP;
    }
    return BranchKind::CONDITIONAL;
}

}  // namespace

Cpu::Cpu(Memory& mem, const Config& cfg, uint32_t entry_pc)
    : mem_(mem),
      cfg_(cfg),
      rob_(cfg),
      prf_(cfg),
      free_list_(cfg),
      rat_(cfg),
      iq_(cfg),
      lsq_(cfg),
      bpred_(cfg),
      ckpt_fe_(cfg.num_checkpoints),
      inflight_(cfg.rob_size),
      pc_(entry_pc),
      arch_pc_(entry_pc),
      queue_cap_(std::max(1u, cfg.width) * 2) {
    for (ArchReg a = 0; a < 32; ++a) arch_rat_[a] = a;

    fu_free_at_[static_cast<int>(Fu::ALU)].assign(cfg.num_alu, 0);
    fu_free_at_[static_cast<int>(Fu::BRANCH)].assign(cfg.num_branch, 0);
    fu_free_at_[static_cast<int>(Fu::MUL)].assign(cfg.num_mul, 0);
    fu_free_at_[static_cast<int>(Fu::DIV)].assign(cfg.num_div, 0);
    fu_free_at_[static_cast<int>(Fu::MEM)].assign(cfg.num_mem, 0);

    const uint32_t longest = std::max({cfg.alu_latency, cfg.branch_latency,
                                       cfg.mul_latency, cfg.div_latency,
                                       cfg.mem_latency, 1u});
    cdb_window_ = longest + 2;
    cdb_booked_.assign(static_cast<std::size_t>(cdb_window_), 0);
}

std::array<uint32_t, 32> Cpu::regs() const {
    std::array<uint32_t, 32> out{};
    for (ArchReg a = 0; a < 32; ++a) out[a] = prf_.read(arch_rat_[a]);
    return out;
}

// Stages run in reverse order so each drains its input before the producer
// refills it, giving one uop per stage per cycle at every width.
void Cpu::tick() {
    if (done()) return;

    ++cycle_;
    ++stats_.cycles;

    commit();
    if (done()) return;   // squashed by the halt; no stage may refill

    writeback();
    execute();
    issue();
    dispatch();
    rename();
    decode_stage();
    fetch();
}

bool Cpu::run(uint64_t max_cycles) {
    while (!done() && cycle_ < max_cycles) tick();
    return done();
}

// ---------------------------------------------------------------- fetch ---
// Every PC is offered to the predictor, which answers from the target cache
// alone — a PC it has never committed a taken branch for falls through, which
// is exactly what hardware does and the reason a cold branch always costs a
// recovery.
//
// The history and return stack move here, speculatively, because the very next
// prediction depends on them. What the checkpoint keeps is the history from
// before this branch's own shift and the return stack from after its own push
// or pop, which is precisely the state recovery has to get back to.
void Cpu::fetch() {
    if (fetch_stalled_) return;

    for (uint32_t n = 0; n < cfg_.width && fetch_q_.size() < queue_cap_; ++n) {
        Uop u;
        u.pc  = pc_;
        u.raw = mem_.load_u32(pc_);

        const uint32_t ghr_before = bpred_.ghr();
        const BranchPredictor::Prediction pr = bpred_.predict(pc_);
        u.pred_taken  = pr.taken;
        u.pred_target = pr.target;
        u.btb_hit     = pr.btb_hit;
        u.from_ras    = pr.from_ras;
        u.pht_index   = pr.pht_index;
        u.fe_snap     = {ghr_before, bpred_.ras()};

        pc_ = pr.taken ? pr.target : pc_ + 4;
        fetch_q_.push_back(u);
        ++stats_.fetched;

        // One redirect per cycle: the rest of this bundle lives somewhere else.
        if (pr.taken) break;
    }
}

// --------------------------------------------------------------- decode ---
void Cpu::decode_stage() {
    for (uint32_t n = 0; n < cfg_.width; ++n) {
        if (fetch_q_.empty()) break;
        if (decode_q_.size() >= queue_cap_) break;   // back-pressure from rename

        Uop u = fetch_q_.front();
        fetch_q_.pop_front();
        u.dec     = decode(u.raw);
        u.next_pc = u.pc + 4;
        u.bkind   = kind_of(u.dec);
        if (u.dec.op == Op::INVALID) u.trap = TrapCause::ILLEGAL;

        decode_q_.push_back(u);
        ++stats_.decoded;
        if (record_decode_) decode_log_.push_back({cycle_, u.pc, u.raw});

        // Nothing past a trap is worth fetching: it either never runs or is
        // squashed when the trap commits.
        if (u.dec.kind == OpKind::TRAP) {
            fetch_q_.clear();
            fetch_stalled_ = true;
            break;
        }
    }
}

// --------------------------------------------------------------- rename ---
// Sources become the physical tags the RAT currently points at; the
// destination takes a fresh register and the displaced mapping rides in the
// ROB entry until commit hands it back. That is the whole trick: no two
// writers of one architectural register ever share storage, so WAW and WAR
// stop existing and only true dependences survive into the issue queue.
void Cpu::rename() {
    for (uint32_t n = 0; n < cfg_.width; ++n) {
        if (decode_q_.empty()) break;
        if (rename_q_.size() >= queue_cap_) break;   // back-pressure from dispatch
        if (rob_.full()) { stats_.stall(Stall::ROB_FULL); break; }

        Uop u = decode_q_.front();
        const bool needs_reg = u.dec.writes_rd;   // already false for rd == x0

        // A branch that cannot be checkpointed cannot be recovered from, so it
        // waits for a slot rather than going unprotected.
        if (u.dec.is_branch && rat_.num_free_checkpoints() == 0) {
            stats_.stall(Stall::CHECKPOINT);
            break;
        }

        PhysReg dest = INVALID_PHYSREG;
        if (needs_reg) {
            const std::optional<PhysReg> got = free_list_.alloc();
            if (!got) { stats_.stall(Stall::PHYSREG); break; }
            dest = *got;
        }
        decode_q_.pop_front();

        u.src1 = rat_.map(u.dec.rs1);
        u.src2 = rat_.map(u.dec.rs2);
        if (needs_reg) {
            u.stale = rat_.map(u.dec.rd);
            u.dest  = dest;
            rat_.set(u.dec.rd, dest);
            prf_.mark_pending(dest);
        }

        // Snapshotted after its own destination is mapped, because the branch
        // itself survives recovery and its result must survive with it.
        if (u.dec.is_branch) {
            u.ckpt = *rat_.alloc_checkpoint();
            ckpt_fe_[u.ckpt] = u.fe_snap;
        }

        RobEntry e;
        e.pc         = u.pc;
        e.next_pc    = u.next_pc;
        e.dest_arch  = needs_reg ? u.dec.rd : INVALID_ARCHREG;
        e.dest_phys  = u.dest;
        e.stale_phys = u.stale;
        e.is_branch  = u.dec.is_branch;
        e.is_store   = u.dec.is_store;
        e.ckpt       = u.ckpt;
        u.rob = rob_.allocate(e);
        u.seq = rob_.at(u.rob).seq;
        inflight_[u.rob] = u;

        rename_q_.push_back(u);
        ++stats_.renamed;
        if (record_rename_) {
            rename_log_.push_back({cycle_, u.seq, u.pc, needs_reg ? u.dec.rd : INVALID_ARCHREG,
                                   u.dest, u.stale, u.src1, u.src2});
        }
    }
}

// ------------------------------------------------------------- dispatch ---
// The uop takes its issue-queue seat here, and a memory op also takes its
// place in line in the load or store queue. Those seats are handed out in
// program order, which is what makes "older than me" answerable later without
// searching. Ready bits are sampled from the PRF once, on the way in; after
// that the entry only learns from tag broadcasts.
void Cpu::dispatch() {
    for (uint32_t n = 0; n < cfg_.width; ++n) {
        if (rename_q_.empty()) break;
        if (iq_.full()) { stats_.stall(Stall::IQ_FULL); break; }

        Uop u = rename_q_.front();

        if (u.dec.is_load) {
            const std::optional<uint32_t> slot =
                lsq_.alloc_load(u.seq, access_size(u.dec.op));
            if (!slot) { stats_.stall(Stall::LQ_FULL); break; }
            u.lsq_idx = *slot;
        } else if (u.dec.is_store) {
            const std::optional<uint32_t> slot =
                lsq_.alloc_store(u.seq, access_size(u.dec.op));
            if (!slot) { stats_.stall(Stall::SQ_FULL); break; }
            u.lsq_idx = *slot;
        }
        rename_q_.pop_front();
        inflight_[u.rob] = u;

        IssueQueue::Entry e;
        e.seq        = u.seq;
        e.rob        = u.rob;
        e.kind       = u.dec.kind;
        e.src1       = u.src1;
        e.src2       = u.src2;
        e.dest       = u.dest;
        // An unused source decodes to x0, which maps to p0 and is always
        // ready, so no operand needs a "do I read this" flag.
        e.src1_ready = prf_.is_ready(u.src1);
        e.src2_ready = prf_.is_ready(u.src2);
        e.latency    = latency_of(u.dec);
        iq_.insert(e);
        ++stats_.dispatched;
    }
}

// ---------------------------------------------------------------- issue ---
Cpu::Fu Cpu::unit_of(const Decoded& d) {
    switch (d.kind) {
    case OpKind::ALU:    return Fu::ALU;
    case OpKind::BRANCH: return Fu::BRANCH;
    case OpKind::MUL:    return Fu::MUL;
    case OpKind::DIV:    return Fu::DIV;
    case OpKind::LOAD:
    case OpKind::STORE:  return Fu::MEM;
    case OpKind::NOP:
    case OpKind::TRAP:   break;
    }
    return Fu::NONE;
}

Stall Cpu::port_stall(Fu f) {
    switch (f) {
    case Fu::ALU:    return Stall::ALU_PORT;
    case Fu::BRANCH: return Stall::BRANCH_PORT;
    case Fu::MUL:    return Stall::MUL_PORT;
    case Fu::DIV:    return Stall::DIV_PORT;
    case Fu::MEM:    return Stall::MEM_PORT;
    case Fu::NONE:   break;
    }
    return Stall::MEM_PORT;
}

uint32_t Cpu::latency_of(const Decoded& d) const {
    uint32_t l = 1;
    switch (d.kind) {
    case OpKind::ALU:    l = cfg_.alu_latency;    break;
    case OpKind::BRANCH: l = cfg_.branch_latency; break;
    case OpKind::MUL:    l = cfg_.mul_latency;    break;
    case OpKind::DIV:    l = cfg_.div_latency;    break;
    case OpKind::LOAD:   l = cfg_.mem_latency;    break;
    case OpKind::STORE:  l = 1;                   break;   // the write is at commit
    case OpKind::NOP:
    case OpKind::TRAP:   l = 1;                   break;
    }
    return std::max(1u, l);   // a zero-cycle unit would finish before it started
}

int Cpu::free_unit(Fu f) const {
    if (f == Fu::NONE) return 0;
    const std::vector<uint64_t>& pool = fu_free_at_[static_cast<int>(f)];
    for (std::size_t i = 0; i < pool.size(); ++i) {
        if (pool[i] <= cycle_) return static_cast<int>(i);
    }
    return -1;
}

bool Cpu::reserve_cdb(uint64_t at_cycle) {
    const std::size_t slot = static_cast<std::size_t>(at_cycle % cdb_window_);
    if (cdb_booked_[slot] >= cfg_.num_cdb) return false;
    ++cdb_booked_[slot];
    return true;
}

// Oldest ready first, but a uop blocked on a unit or a writeback port only
// blocks itself — the next candidate still gets a look. That is the point of
// the whole machine: program order stopped constraining execution at rename.
void Cpu::issue() {
    const std::vector<IssueQueue::Entry> ready = iq_.select(iq_.size());
    uint32_t issued = 0;
    bool     redirect = false;
    Uop      offender;

    for (const IssueQueue::Entry& e : ready) {
        if (issued >= cfg_.width) break;

        Uop u = inflight_[e.rob];

        const Fu  f    = unit_of(u.dec);
        const int unit = free_unit(f);
        if (unit < 0) { stats_.stall(port_stall(f)); continue; }

        uint32_t lat = e.latency;
        u.val1 = prf_.read(u.src1);
        u.val2 = prf_.read(u.src2);

        if (u.dec.is_load) {
            // A load cannot book a writeback port at issue: whether it takes
            // one cycle or all of memory latency depends on what the store
            // queue says, which is only known now.
            if (!execute_load(u, lat)) {
                // Address generation happened, so the unit is spent either
                // way; the load keeps its seat and asks again next cycle.
                fu_free_at_[static_cast<int>(Fu::MEM)][static_cast<std::size_t>(unit)] =
                    cycle_ + 1;
                ++stats_.load_replays;
                stats_.stall(Stall::STORE_ORDER);
                continue;
            }
        } else {
            // Everything else knows its latency at issue, so it books the
            // writeback port for the cycle the value lands and does not go
            // without one.
            if (u.dec.writes_rd && !reserve_cdb(cycle_ + lat)) {
                stats_.stall(Stall::CDB);
                continue;
            }
            if (u.trap == TrapCause::NONE) execute_uop(u);
            if (u.dec.is_store) {
                lsq_.resolve_addr(u.lsq_idx, u.mem_addr);
                lsq_.resolve_data(u.lsq_idx, u.store_data);
            }
        }

        if (f != Fu::NONE) {
            fu_free_at_[static_cast<int>(f)][static_cast<std::size_t>(unit)] =
                cycle_ + (pipelined(f) ? 1 : lat);
        }
        u.wb_cycle = cycle_ + lat;

        // The guess is settled here. If it was wrong, the oldest offender in
        // the cycle is the one that recovers: anything younger is on a path
        // that never existed, including a younger branch that also "resolved".
        const uint32_t predicted = u.pred_taken ? u.pred_target : u.pc + 4;
        if (u.next_pc != predicted) {
            rob_.at(u.rob).mispredicted = true;
            if (!redirect || u.seq < offender.seq) {
                redirect = true;
                offender = u;
            }
        }

        inflight_[u.rob] = u;
        iq_.erase(e.seq);

        // A single-cycle unit has its answer in the cycle it started, so it
        // goes straight to the writeback queue rather than through execute.
        if (lat > 1)               executing_.push_back(u);
        else if (u.dec.is_load)    wb_slow_.push_back(u);
        else                       wb_fast_.push_back(u);

        ++issued;
        ++stats_.issued;
        if (u.dec.is_load)  ++stats_.loads;
        if (u.dec.is_store) ++stats_.stores;
        if (record_issue_) issue_log_.push_back({cycle_, u.seq, u.pc, u.dest, u.wb_cycle});
    }

    // Slots lost with work waiting are lost to a producer, not to a resource.
    if (issued < cfg_.width && ready.size() == issued && !iq_.empty()) {
        stats_.stall(Stall::OPERANDS);
    }

    if (redirect) recover(offender);
}

void Cpu::release_cdb(const Uop& u) {
    if (!u.dec.writes_rd || u.dec.is_load) return;    // never booked one
    if (u.wb_cycle <= cycle_) return;                 // already spent or cleared
    uint32_t& booked = cdb_booked_[static_cast<std::size_t>(u.wb_cycle % cdb_window_)];
    if (booked > 0) --booked;
}

// Everything younger than the branch is undone in one cycle, and fetch — which
// runs later in this same cycle — restarts on the real path.
//
// Registers go back youngest first, and each entry returns the register it
// allocated rather than the stale one it displaced: that stale mapping still
// belongs to an older entry, which will hand it back at its own commit.
// Getting that direction backwards is the classic recovery bug, and it does
// not show up until some unrelated register is quietly wrong much later.
void Cpu::recover(const Uop& br) {
    const std::vector<RobEntry> killed = rob_.truncate_to(br.rob);
    for (const RobEntry& e : killed) {
        free_list_.free(e.dest_phys);
        rat_.free_checkpoint(e.ckpt);
        ++stats_.squashed;
    }

    iq_.squash_after(br.seq);
    lsq_.squash_after(br.seq);
    squash_queue(executing_, br.seq);
    squash_queue(wb_fast_, br.seq);
    squash_queue(wb_slow_, br.seq);

    rat_.restore_checkpoint(br.ckpt);
    bpred_.restore(ckpt_fe_[br.ckpt]);
    // The history the branch shifted in was a guess; replace it with the fact.
    if (br.btb_hit && br.bkind == BranchKind::CONDITIONAL) {
        bpred_.shift_history(br.next_pc != br.pc + 4);
    }
    // The branch survives, so its slot stays reserved and is released once,
    // when it commits, like every other branch's.

    fetch_q_.clear();
    decode_q_.clear();
    rename_q_.clear();
    fetch_stalled_ = false;
    pc_ = br.next_pc;
    ++stats_.mispredicts;
}

// -------------------------------------------------------------- execute ---
// Multi-cycle units hand their result over on the cycle it appears; single
// cycle ops never get here.
void Cpu::execute() {
    std::vector<Uop> still_running;
    still_running.reserve(executing_.size());

    for (const Uop& u : executing_) {
        if (u.wb_cycle > cycle_ + 1) still_running.push_back(u);
        else if (u.dec.is_load)      wb_slow_.push_back(u);
        else                         wb_fast_.push_back(u);
    }
    executing_.swap(still_running);
}

// Three answers, and only one of them touches memory. Forwarding is what lets
// a load see a store that has not committed yet; replaying is what keeps it
// from guessing when an older address is still unknown.
bool Cpu::execute_load(Uop& u, uint32_t& latency) {
    const uint32_t addr = u.val1 + static_cast<uint32_t>(u.dec.imm);
    lsq_.resolve_load_addr(u.lsq_idx, addr);

    const ForwardResult f = lsq_.search_older_stores(u.lsq_idx);
    if (f.kind == Forward::REPLAY) return false;

    u.mem_addr   = addr;
    u.has_result = true;
    if (f.kind == Forward::FORWARD) {
        u.result = extend_load(u.dec.op, f.data);
        latency  = 1;                       // never left the store queue
        ++stats_.load_forwards;
        return true;
    }

    switch (u.dec.op) {
    case Op::LB:  u.result = extend_load(Op::LB, mem_.load_u8 (addr)); break;
    case Op::LH:  u.result = extend_load(Op::LH, mem_.load_u16(addr)); break;
    case Op::LBU: u.result = mem_.load_u8 (addr); break;
    case Op::LHU: u.result = mem_.load_u16(addr); break;
    default:      u.result = mem_.load_u32(addr); break;
    }
    ++stats_.load_memory;
    return true;
}

void Cpu::execute_uop(Uop& u) {
    const Decoded& d   = u.dec;
    const uint32_t rs1 = u.val1;
    const uint32_t rs2 = u.val2;
    const uint32_t imm = static_cast<uint32_t>(d.imm);
    const uint32_t opb = uses_immediate(d) ? imm : rs2;

    switch (d.op) {
    // ---- Integer ALU ------------------------------------------------------
    case Op::ADD:   u.result = alu::add (rs1, opb); u.has_result = true; break;
    case Op::SUB:   u.result = alu::sub (rs1, rs2); u.has_result = true; break;
    case Op::SLL:   u.result = alu::sll (rs1, opb); u.has_result = true; break;
    case Op::SRL:   u.result = alu::srl (rs1, opb); u.has_result = true; break;
    case Op::SRA:   u.result = alu::sra (rs1, opb); u.has_result = true; break;
    case Op::AND:   u.result = alu::and_(rs1, opb); u.has_result = true; break;
    case Op::OR:    u.result = alu::or_ (rs1, opb); u.has_result = true; break;
    case Op::XOR:   u.result = alu::xor_(rs1, opb); u.has_result = true; break;
    case Op::SLT:   u.result = alu::slt (rs1, opb); u.has_result = true; break;
    case Op::SLTU:  u.result = alu::sltu(rs1, opb); u.has_result = true; break;
    case Op::LUI:   u.result = imm;                 u.has_result = true; break;
    case Op::AUIPC: u.result = u.pc + imm;          u.has_result = true; break;

    // ---- M extension ------------------------------------------------------
    case Op::MUL:    u.result = alu::mul   (rs1, rs2); u.has_result = true; break;
    case Op::MULH:   u.result = alu::mulh  (rs1, rs2); u.has_result = true; break;
    case Op::MULHSU: u.result = alu::mulhsu(rs1, rs2); u.has_result = true; break;
    case Op::MULHU:  u.result = alu::mulhu (rs1, rs2); u.has_result = true; break;
    case Op::DIV:    u.result = alu::div   (rs1, rs2); u.has_result = true; break;
    case Op::DIVU:   u.result = alu::divu  (rs1, rs2); u.has_result = true; break;
    case Op::REM:    u.result = alu::rem   (rs1, rs2); u.has_result = true; break;
    case Op::REMU:   u.result = alu::remu  (rs1, rs2); u.has_result = true; break;

    // ---- Control flow -----------------------------------------------------
    case Op::BEQ:  if (alu::beq (rs1, rs2)) u.next_pc = u.pc + imm; break;
    case Op::BNE:  if (alu::bne (rs1, rs2)) u.next_pc = u.pc + imm; break;
    case Op::BLT:  if (alu::blt (rs1, rs2)) u.next_pc = u.pc + imm; break;
    case Op::BGE:  if (alu::bge (rs1, rs2)) u.next_pc = u.pc + imm; break;
    case Op::BLTU: if (alu::bltu(rs1, rs2)) u.next_pc = u.pc + imm; break;
    case Op::BGEU: if (alu::bgeu(rs1, rs2)) u.next_pc = u.pc + imm; break;

    case Op::JAL:
        u.result = u.pc + 4; u.has_result = true;
        u.next_pc = u.pc + imm;
        break;
    case Op::JALR:
        // Target drops its low bit, and the link comes from the old PC, which
        // matters when rd == rs1.
        u.result = u.pc + 4; u.has_result = true;
        u.next_pc = (rs1 + imm) & ~1u;
        break;

    // ---- Loads: handled by execute_load, which has to ask the queue -------
    case Op::LB:
    case Op::LH:
    case Op::LW:
    case Op::LBU:
    case Op::LHU:
        break;

    // ---- Stores: address and data now, memory at commit -------------------
    case Op::SB:
    case Op::SH:
    case Op::SW:
        u.mem_addr   = rs1 + imm;
        u.store_data = rs2;
        break;

    // ---- No-ops and traps -------------------------------------------------
    case Op::FENCE:
    case Op::FENCE_I:
    case Op::ECALL:
    case Op::EBREAK:
        break;
    case Op::INVALID:
        u.trap = TrapCause::ILLEGAL;
        break;
    }
}

// ------------------------------------------------------------ writeback ---
void Cpu::complete(const Uop& u) {
    if (u.trap == TrapCause::NONE && u.has_result && u.dest != INVALID_PHYSREG) {
        prf_.write(u.dest, u.result);
    }
    iq_.wakeup(u.dest);
    rob_.at(u.rob).complete = true;
    inflight_[u.rob] = u;
    ++stats_.wrote_back;
}

// Runs before issue, so a tag broadcast this cycle reaches select this cycle.
// Everything about back-to-back dependent issue rests on that ordering.
void Cpu::writeback() {
    uint32_t ports = cfg_.num_cdb;

    // Booked at issue, so the port is guaranteed to be there.
    while (!wb_fast_.empty()) {
        const Uop& u = wb_fast_.front();
        if (u.dec.writes_rd) {
            if (ports == 0) break;
            --ports;
        }
        complete(u);
        wb_fast_.pop_front();
    }

    // Loads could not book one — their latency is not known at issue — so they
    // take whatever is left and retry next cycle otherwise.
    while (!wb_slow_.empty() && ports > 0) {
        complete(wb_slow_.front());
        wb_slow_.pop_front();
        --ports;
    }

    cdb_booked_[static_cast<std::size_t>(cycle_ % cdb_window_)] = 0;
}

// --------------------------------------------------------------- commit ---
void Cpu::commit() {
    for (uint32_t n = 0; n < cfg_.width; ++n) {
        const RobIndex idx = rob_.head();
        if (idx == INVALID_ROBINDEX || !rob_.at(idx).complete) break;
        rob_.pop_head_if_complete();

        const Uop u = inflight_[idx];
        // Sequence numbers skip over squashed uops, so program order means
        // strictly increasing rather than consecutive.
        if (stats_.retired > 0 && u.seq <= last_committed_seq_) commit_in_order_ = false;
        last_committed_seq_ = u.seq;

        // Every architectural effect that is not a register write happens here
        // and nowhere else, memory included.
        // The queue seats are given up in the same order they were taken.
        if (u.dec.is_load) lsq_.loads().pop_head();

        // The predictor only learns from instructions that really ran, so a
        // wrong path can never train it.
        if (u.dec.is_branch) {
            const bool taken = u.next_pc != u.pc + 4;
            bpred_.commit(u.pc, u.pht_index, u.bkind, taken, u.next_pc);
            rat_.free_checkpoint(u.ckpt);

            ++stats_.branches;
            ++stats_.btb_lookups;
            if (u.btb_hit) ++stats_.btb_hits;
            if (u.from_ras) {
                ++stats_.ras_pops;
                if (u.pred_target == u.next_pc) ++stats_.ras_hits;
            }
        }

        TrapCause cause = u.trap;
        if (cause == TrapCause::NONE) {
            switch (u.dec.op) {
            // A store leaves the queue and becomes visible to the world in
            // the same instant, which is what makes a squash able to erase it
            // by doing nothing at all.
            case Op::SB:
            case Op::SH:
            case Op::SW: {
                const LsqEntry& st = lsq_.stores().nth(0);
                if      (u.dec.op == Op::SB) mem_.store_u8 (st.addr, static_cast<uint8_t> (st.data));
                else if (u.dec.op == Op::SH) mem_.store_u16(st.addr, static_cast<uint16_t>(st.data));
                else                         mem_.store_u32(st.addr, st.data);
                lsq_.stores().pop_head();
                break;
            }

            case Op::ECALL:
                if (reg(REG_A7) == SYS_EXIT) {
                    exit_code_ = reg(REG_A0);
                    halted_    = true;
                } else {
                    cause = TrapCause::ECALL_UNKNOWN;
                }
                break;
            case Op::EBREAK:
                cause = TrapCause::EBREAK;
                break;

            default:
                break;
            }
        }

        // The mapping becomes architectural and the one it displaced goes back
        // to the free list. This is the only place a register is freed on the
        // correct path, which is exactly why two writers of one architectural
        // register can never end up aliasing the same physical storage.
        if (u.dest != INVALID_PHYSREG) {
            arch_rat_[u.dec.rd] = u.dest;
            free_list_.free(u.stale);
        }

        if (cause != TrapCause::NONE) {
            trapped_    = true;
            trap_cause_ = cause;
        }

        arch_pc_ = done() ? u.pc : u.next_pc;   // a trap reports its own PC
        ++stats_.retired;

        // The halting instruction retires; anything behind it does not.
        if (done()) {
            squash_in_flight();
            break;
        }
    }
}

// Youngest first, so each entry hands back the register it allocated — not the
// stale one it displaced, which an older entry still owns.
void Cpu::squash_in_flight() {
    for (const RobEntry& e : rob_.squash_all()) {
        free_list_.free(e.dest_phys);
        rat_.free_checkpoint(e.ckpt);
        ++stats_.squashed;
    }

    rat_.adopt(arch_rat_);
    lsq_.clear();
    fetch_q_.clear();
    decode_q_.clear();
    rename_q_.clear();
    iq_.clear();
    executing_.clear();
    wb_fast_.clear();
    wb_slow_.clear();
    std::fill(cdb_booked_.begin(), cdb_booked_.end(), 0u);
    fetch_stalled_ = false;
}
