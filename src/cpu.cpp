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

}  // namespace

Cpu::Cpu(Memory& mem, const Config& cfg, uint32_t entry_pc)
    : mem_(mem),
      cfg_(cfg),
      rob_(cfg),
      prf_(cfg),
      free_list_(cfg),
      rat_(cfg),
      iq_(cfg),
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
void Cpu::fetch() {
    if (fetch_stalled_) return;

    for (uint32_t n = 0; n < cfg_.width && fetch_q_.size() < queue_cap_; ++n) {
        Uop u;
        u.pc  = pc_;
        u.raw = mem_.load_u32(pc_);
        pc_ += 4;
        fetch_q_.push_back(u);
        ++stats_.fetched;
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
        if (u.dec.op == Op::INVALID) u.trap = TrapCause::ILLEGAL;

        decode_q_.push_back(u);
        ++stats_.decoded;
        if (record_decode_) decode_log_.push_back({cycle_, u.pc, u.raw});

        // Nothing younger than a control transfer or a trap may be fetched:
        // the branch target is unknown until it executes, and letting a younger
        // write land before an ecall reads a7 would corrupt the syscall.
        if (u.dec.is_branch || u.dec.kind == OpKind::TRAP) {
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

        RobEntry e;
        e.pc         = u.pc;
        e.next_pc    = u.next_pc;
        e.dest_arch  = needs_reg ? u.dec.rd : INVALID_ARCHREG;
        e.dest_phys  = u.dest;
        e.stale_phys = u.stale;
        e.is_branch  = u.dec.is_branch;
        e.is_store   = u.dec.is_store;
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
// The uop takes its issue-queue seat here. Ready bits are sampled from the PRF
// once, on the way in; after that the entry only learns from tag broadcasts.
void Cpu::dispatch() {
    for (uint32_t n = 0; n < cfg_.width; ++n) {
        if (rename_q_.empty()) break;
        if (iq_.full()) { stats_.stall(Stall::IQ_FULL); break; }

        const Uop u = rename_q_.front();
        rename_q_.pop_front();

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

bool Cpu::older_store_pending(SeqNum seq) const {
    for (uint32_t k = 0; k < rob_.size(); ++k) {
        const RobEntry& e = rob_.nth_entry(k);
        if (e.seq >= seq) break;              // the ROB is in program order
        if (e.is_store) return true;
    }
    return false;
}

// Oldest ready first, but a uop blocked on a unit or a writeback port only
// blocks itself — the next candidate still gets a look. That is the point of
// the whole machine: program order stopped constraining execution at rename.
void Cpu::issue() {
    const std::vector<IssueQueue::Entry> ready = iq_.select(iq_.size());
    uint32_t issued = 0;

    for (const IssueQueue::Entry& e : ready) {
        if (issued >= cfg_.width) break;

        Uop u = inflight_[e.rob];

        if (u.dec.is_load && older_store_pending(u.seq)) {
            stats_.stall(Stall::STORE_ORDER);
            continue;
        }

        const Fu  f    = unit_of(u.dec);
        const int unit = free_unit(f);
        if (unit < 0) { stats_.stall(port_stall(f)); continue; }

        // A result-producing op books its writeback port now, at the cycle the
        // value will actually land, and does not go without one.
        const uint32_t lat = e.latency;
        if (u.dec.writes_rd && !reserve_cdb(cycle_ + lat)) {
            stats_.stall(Stall::CDB);
            continue;
        }

        if (f != Fu::NONE) {
            fu_free_at_[static_cast<int>(f)][static_cast<std::size_t>(unit)] =
                cycle_ + (pipelined(f) ? 1 : lat);
        }

        u.val1 = prf_.read(u.src1);
        u.val2 = prf_.read(u.src2);
        if (u.trap == TrapCause::NONE) execute_uop(u);
        u.wb_cycle = cycle_ + lat;

        // A control transfer resolves here, and fetch — which runs later this
        // cycle — picks up from the target with no wrong path to undo.
        if (u.dec.is_branch) {
            pc_ = u.next_pc;
            fetch_stalled_ = false;
        }

        inflight_[u.rob] = u;
        iq_.erase(e.seq);

        // A single-cycle unit has its answer in the cycle it started, so it
        // goes straight to the writeback queue rather than through execute.
        if (lat <= 1) wb_fast_.push_back(u);
        else          executing_.push_back({u, cycle_ + lat - 1});

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
}

// -------------------------------------------------------------- execute ---
// Multi-cycle units hand their result over on the cycle it appears; single
// cycle ops never get here.
void Cpu::execute() {
    std::vector<FuOp> still_running;
    still_running.reserve(executing_.size());

    for (const FuOp& op : executing_) {
        if (op.finish <= cycle_) wb_fast_.push_back(op.uop);
        else                     still_running.push_back(op);
    }
    executing_.swap(still_running);
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

    // ---- Loads ------------------------------------------------------------
    case Op::LB: {
        const auto b = static_cast<int8_t>(mem_.load_u8(rs1 + imm));
        u.result = static_cast<uint32_t>(static_cast<int32_t>(b));
        u.has_result = true;
        break;
    }
    case Op::LH: {
        const auto h = static_cast<int16_t>(mem_.load_u16(rs1 + imm));
        u.result = static_cast<uint32_t>(static_cast<int32_t>(h));
        u.has_result = true;
        break;
    }
    case Op::LW:  u.result = mem_.load_u32(rs1 + imm); u.has_result = true; break;
    case Op::LBU: u.result = mem_.load_u8 (rs1 + imm); u.has_result = true; break;
    case Op::LHU: u.result = mem_.load_u16(rs1 + imm); u.has_result = true; break;

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
        if (u.seq != stats_.retired) commit_in_order_ = false;

        // Every architectural effect that is not a register write happens here
        // and nowhere else, memory included.
        TrapCause cause = u.trap;
        if (cause == TrapCause::NONE) {
            switch (u.dec.op) {
            case Op::SB: mem_.store_u8 (u.mem_addr, static_cast<uint8_t> (u.store_data)); break;
            case Op::SH: mem_.store_u16(u.mem_addr, static_cast<uint16_t>(u.store_data)); break;
            case Op::SW: mem_.store_u32(u.mem_addr, u.store_data); break;

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
    for (uint32_t k = rob_.size(); k > 0; --k) {
        const RobEntry& e = rob_.nth_entry(k - 1);
        free_list_.free(e.dest_phys);
        ++stats_.squashed;
    }
    rob_.squash_all();

    rat_.adopt(arch_rat_);
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
