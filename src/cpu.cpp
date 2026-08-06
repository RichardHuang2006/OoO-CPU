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
      inflight_(cfg.rob_size),
      pc_(entry_pc),
      arch_pc_(entry_pc),
      queue_cap_(std::max(1u, cfg.width) * 2) {
    fu_free_at_[static_cast<int>(Fu::ALU)].assign(cfg.num_alu, 0);
    fu_free_at_[static_cast<int>(Fu::BRANCH)].assign(cfg.num_branch, 0);
    fu_free_at_[static_cast<int>(Fu::MUL)].assign(cfg.num_mul, 0);
    fu_free_at_[static_cast<int>(Fu::DIV)].assign(cfg.num_div, 0);
    fu_free_at_[static_cast<int>(Fu::MEM)].assign(cfg.num_mem, 0);
}

// Stages run in reverse order so each drains its input before the producer
// refills it, giving one uop per stage per cycle at every width.
void Cpu::tick() {
    if (done()) return;

    ++cycle_;
    commit();
    if (done()) return;   // squashed by the halt; no stage may refill

    writeback();
    execute();
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
    }
}

// --------------------------------------------------------------- decode ---
// Also dispatch: a decoded uop takes its ROB slot here, in program order, and
// that is where its sequence number comes from.
void Cpu::decode_stage() {
    for (uint32_t n = 0; n < cfg_.width; ++n) {
        if (fetch_q_.empty()) break;
        if (dispatch_q_.size() >= queue_cap_) break;   // back-pressure from issue
        if (rob_.full()) break;

        Uop u = fetch_q_.front();
        fetch_q_.pop_front();
        u.dec     = decode(u.raw);
        u.next_pc = u.pc + 4;
        if (u.dec.op == Op::INVALID) u.trap = TrapCause::ILLEGAL;

        RobEntry e;
        e.pc        = u.pc;
        e.next_pc   = u.next_pc;
        e.dest_arch = u.dec.writes_rd ? u.dec.rd : INVALID_ARCHREG;
        e.is_branch = u.dec.is_branch;
        e.is_store  = u.dec.is_store;
        u.rob = rob_.allocate(e);
        u.seq = rob_.at(u.rob).seq;
        inflight_[u.rob] = u;

        dispatch_q_.push_back(u);
        if (record_decode_) decode_log_.push_back({cycle_, u.seq, u.pc, u.raw});

        // Nothing younger than a control transfer or a trap may be fetched:
        // the branch target is unknown until execute, and letting a younger
        // write land before an ecall reads a7 would corrupt the syscall.
        if (u.dec.is_branch || u.dec.kind == OpKind::TRAP) {
            fetch_q_.clear();
            fetch_stalled_ = true;
            break;
        }
    }
}

// -------------------------------------------------------------- execute ---
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

bool Cpu::older_store_pending(SeqNum seq) const {
    for (uint32_t k = 0; k < rob_.size(); ++k) {
        const RobEntry& e = rob_.nth_entry(k);
        if (e.seq >= seq) break;              // the ROB is in program order
        if (e.is_store) return true;
    }
    return false;
}

void Cpu::execute() {
    // Issue in program order, stopping at the first uop that cannot go.
    for (uint32_t n = 0; n < cfg_.width && !dispatch_q_.empty(); ++n) {
        Uop u = dispatch_q_.front();

        if (!operands_ready(u.dec)) break;
        if (u.dec.is_load && older_store_pending(u.seq)) break;

        const Fu  f    = unit_of(u.dec);
        const int unit = free_unit(f);
        if (unit < 0) break;

        const uint32_t lat = latency_of(u.dec);
        if (f != Fu::NONE) {
            fu_free_at_[static_cast<int>(f)][static_cast<std::size_t>(unit)] =
                cycle_ + (pipelined(f) ? 1 : lat);
        }

        if (u.trap == TrapCause::NONE) execute_uop(u);
        if (u.dec.writes_rd) ++reg_busy_[u.dec.rd];

        // A control transfer resolves here, and fetch — which runs later this
        // cycle — picks up from the target with no wrong path to undo.
        if (u.dec.is_branch) {
            pc_ = u.next_pc;
            fetch_stalled_ = false;
        }

        inflight_[u.rob] = u;
        executing_.push_back({u, cycle_ + lat - 1});
        dispatch_q_.pop_front();
        ++issued_;
    }

    // Results appear in issue order: a long op holds up younger ones behind it,
    // which is what keeps register writes ordered without renaming.
    while (!executing_.empty() && executing_.front().finish <= cycle_) {
        wb_q_.push_back(executing_.front().uop);
        executing_.pop_front();
    }
}

void Cpu::execute_uop(Uop& u) {
    const Decoded& d   = u.dec;
    const uint32_t rs1 = regs_[d.rs1];
    const uint32_t rs2 = regs_[d.rs2];
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
void Cpu::writeback() {
    for (uint32_t n = 0; n < cfg_.num_cdb && !wb_q_.empty(); ++n) {
        const Uop u = wb_q_.front();
        wb_q_.pop_front();

        if (u.trap == TrapCause::NONE && u.has_result && u.dec.writes_rd) {
            regs_[u.dec.rd] = u.result;
            regs_[0] = 0;                     // x0 stays hardwired
        }
        if (u.dec.writes_rd) --reg_busy_[u.dec.rd];

        rob_.at(u.rob).complete = true;
    }
}

// --------------------------------------------------------------- commit ---
void Cpu::commit() {
    for (uint32_t n = 0; n < cfg_.width; ++n) {
        const RobIndex idx = rob_.head();
        if (idx == INVALID_ROBINDEX || !rob_.at(idx).complete) break;
        rob_.pop_head_if_complete();

        const Uop u = inflight_[idx];
        if (u.seq != retired_) commit_in_order_ = false;

        // Every architectural effect that is not a register write happens here
        // and nowhere else, memory included.
        TrapCause cause = u.trap;
        if (cause == TrapCause::NONE) {
            switch (u.dec.op) {
            case Op::SB: mem_.store_u8 (u.mem_addr, static_cast<uint8_t> (u.store_data)); break;
            case Op::SH: mem_.store_u16(u.mem_addr, static_cast<uint16_t>(u.store_data)); break;
            case Op::SW: mem_.store_u32(u.mem_addr, u.store_data); break;

            case Op::ECALL:
                if (regs_[REG_A7] == SYS_EXIT) {
                    exit_code_ = regs_[REG_A0];
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

        if (cause != TrapCause::NONE) {
            trapped_    = true;
            trap_cause_ = cause;
        }

        arch_pc_ = done() ? u.pc : u.next_pc;   // a trap reports its own PC
        ++retired_;

        // The halting instruction retires; anything behind it does not.
        if (done()) {
            squash_in_flight();
            break;
        }
    }
}

void Cpu::squash_in_flight() {
    fetch_q_.clear();
    dispatch_q_.clear();
    executing_.clear();
    wb_q_.clear();
    rob_.squash_all();
    reg_busy_.fill(0);
    fetch_stalled_ = false;
}
