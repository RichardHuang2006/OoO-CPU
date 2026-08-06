#include "cpu.h"

#include "alu.h"

namespace {

// OP-IMM is the only class taking its second operand from the immediate;
// ADDI and ADD share an Op because the function unit does not care where the
// operand came from.
bool uses_immediate(const Decoded& d) { return (d.raw & 0x7Fu) == 0x13u; }

// The single-cycle integer ALU class.
bool execute_alu(const Uop& u, uint32_t rs1, uint32_t opb, uint32_t& out) {
    switch (u.dec.op) {
    case Op::ADD:   out = alu::add (rs1, opb); return true;
    case Op::SUB:   out = alu::sub (rs1, opb); return true;
    case Op::SLL:   out = alu::sll (rs1, opb); return true;
    case Op::SRL:   out = alu::srl (rs1, opb); return true;
    case Op::SRA:   out = alu::sra (rs1, opb); return true;
    case Op::AND:   out = alu::and_(rs1, opb); return true;
    case Op::OR:    out = alu::or_ (rs1, opb); return true;
    case Op::XOR:   out = alu::xor_(rs1, opb); return true;
    case Op::SLT:   out = alu::slt (rs1, opb); return true;
    case Op::SLTU:  out = alu::sltu(rs1, opb); return true;
    case Op::LUI:   out = static_cast<uint32_t>(u.dec.imm); return true;
    case Op::AUIPC: out = u.pc + static_cast<uint32_t>(u.dec.imm); return true;
    default:        return false;
    }
}

}  // namespace

Cpu::Cpu(Memory& mem, const Config& cfg, uint32_t entry_pc)
    : mem_(mem), cfg_(cfg), pc_(entry_pc), arch_pc_(entry_pc) {}

// Stages run in reverse order so each drains its output latch before the
// producer refills it, giving one uop per stage per cycle.
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

void Cpu::fetch() {
    if (if_id_) return;                       // back-pressure from Decode

    Uop u;
    u.seq = next_seq_++;
    u.pc  = pc_;
    u.raw = mem_.load_u32(pc_);
    pc_ += 4;                                 // sequential fetch only, so far
    if_id_ = u;
}

void Cpu::decode_stage() {
    if (id_ex_ || !if_id_) return;

    Uop u = *if_id_;
    u.dec     = decode(u.raw);
    u.next_pc = u.pc + 4;
    if (u.dec.op == Op::INVALID) u.trap = TrapCause::ILLEGAL;

    if_id_.reset();
    id_ex_ = u;
}

void Cpu::execute() {
    if (ex_wb_ || !id_ex_) return;

    Uop u = *id_ex_;
    if (u.trap == TrapCause::NONE) {
        const uint32_t rs1 = regs_[u.dec.rs1];
        const uint32_t rs2 = regs_[u.dec.rs2];
        const uint32_t opb = uses_immediate(u.dec) ? static_cast<uint32_t>(u.dec.imm) : rs2;

        switch (u.dec.kind) {
        case OpKind::ALU:
            u.has_result = execute_alu(u, rs1, opb, u.result);
            if (!u.has_result) u.trap = TrapCause::ILLEGAL;
            break;

        case OpKind::NOP:      // fence / fence.i: nothing to order, nothing to do
        case OpKind::TRAP:     // ecall / ebreak are resolved at commit
            break;

        case OpKind::BRANCH:
        case OpKind::LOAD:
        case OpKind::STORE:
        case OpKind::MUL:
        case OpKind::DIV:
            // No function units for these yet; fail loudly, not silently.
            u.trap = TrapCause::UNIMPLEMENTED;
            break;
        }
    }

    id_ex_.reset();
    ex_wb_ = u;
}

void Cpu::writeback() {
    if (wb_cm_ || !ex_wb_) return;

    const Uop& u = *ex_wb_;
    if (u.trap == TrapCause::NONE && u.has_result && u.dec.writes_rd) {
        regs_[u.dec.rd] = u.result;
        regs_[0] = 0;                         // x0 stays hardwired
    }

    wb_cm_ = u;
    ex_wb_.reset();
}

void Cpu::commit() {
    if (!wb_cm_) return;

    const Uop u = *wb_cm_;
    wb_cm_.reset();

    if (u.seq != retired_) commit_in_order_ = false;

    // Every architectural effect that is not a register write happens here
    // and nowhere else.
    TrapCause cause = u.trap;
    if (cause == TrapCause::NONE && u.dec.kind == OpKind::TRAP) {
        if (u.dec.op == Op::ECALL) {
            if (regs_[17] == 93) {            // a7 == 93 → exit(a0)
                exit_code_ = regs_[10];
                halted_ = true;
            } else {
                cause = TrapCause::ECALL_UNKNOWN;
            }
        } else if (u.dec.op == Op::EBREAK) {
            cause = TrapCause::EBREAK;
        }
    }

    if (cause != TrapCause::NONE) {
        trapped_    = true;
        trap_cause_ = cause;
    }

    arch_pc_ = u.next_pc;
    ++retired_;

    // The halting instruction retires; anything behind it does not.
    if (done()) squash_in_flight();
}

void Cpu::squash_in_flight() {
    if_id_.reset();
    id_ex_.reset();
    ex_wb_.reset();
    wb_cm_.reset();
}
