#pragma once

// The oracle: a straight in-order interpreter for RV32IM.
//
// One instruction per iteration — fetch, decode (decoder.h), execute
// (alu.h), write back, advance PC. No pipeline, no renaming, no
// speculation, nothing that could be wrong in an interesting way. Every
// later phase's `Cpu` is checked against this by comparing all 32
// architectural registers, the exit code, and the retired count
// (DESIGN.md §8.1).
//
// It lives in tests/ because it is test infrastructure, but src/main.cpp
// also drives it so `oooc --hex prog.hex --regs` works before any pipeline
// model exists. Both build targets have -Itests.
//
// The one deliberate duplication in this project: the switch below repeats
// the ISA semantics that the Phase-3 execute stage will implement
// independently. Sharing the code would defeat the entire point — two
// implementations that agree are evidence, one implementation called twice
// is not.

#include <cstdint>
#include <cstdio>

#include "alu.h"
#include "decoder.h"
#include "memory.h"

namespace ref {

// The `exit` syscall from the RISC-V proxy-kernel ABI: `ecall` with
// a7 == 93 halts the machine and yields a0 as the exit code (DESIGN.md §10).
inline constexpr uint32_t SYS_EXIT = 93;
inline constexpr ArchReg  REG_A0   = 10;
inline constexpr ArchReg  REG_A7   = 17;

struct Result {
    uint32_t regs[32] = {};
    uint32_t pc        = 0;   // next PC; on a halt/trap, the halting PC
    uint32_t exit_code = 0;
    uint64_t retired   = 0;

    bool halted  = false;     // ecall, a7 == 93
    bool trapped = false;     // ebreak, illegal encoding, unhandled ecall
    bool budget  = false;     // hit Options::max_insts — neither halt nor trap

    bool done() const { return halted || trapped; }
};

struct Options {
    uint64_t   max_insts = 100'000'000;   // runaway-program backstop
    bool       trace     = false;
    std::FILE* trace_out = stderr;
};

// OP-IMM is the only class where the second ALU operand comes from the
// immediate rather than rs2, and `Decoded` cannot express the difference:
// ADDI and ADD both decode to Op::ADD (deliberately — they are the same
// operation on different operand sources, and the FU does not care). The
// opcode is the discriminator.
inline bool uses_immediate(const Decoded& d) { return (d.raw & 0x7Fu) == 0x13u; }

// Execute exactly one instruction. Updates registers, memory, PC, the
// retired count, and — for a halting instruction — the halt/trap flags.
inline void step(Memory& mem, Result& st, const Options& opts) {
    const uint32_t pc  = st.pc;
    const uint32_t raw = mem.load_u32(pc);
    const Decoded  d   = decode(raw);

    const uint32_t rs1 = st.regs[d.rs1];
    const uint32_t rs2 = st.regs[d.rs2];
    const uint32_t imm = static_cast<uint32_t>(d.imm);
    const uint32_t opb = uses_immediate(d) ? imm : rs2;

    uint32_t next_pc    = pc + 4;
    uint32_t result     = 0;
    bool     has_result = false;

    switch (d.op) {
    // ---- Integer ALU ------------------------------------------------------
    case Op::ADD:   result = alu::add (rs1, opb); has_result = true; break;
    case Op::SUB:   result = alu::sub (rs1, rs2); has_result = true; break;
    case Op::SLL:   result = alu::sll (rs1, opb); has_result = true; break;
    case Op::SRL:   result = alu::srl (rs1, opb); has_result = true; break;
    case Op::SRA:   result = alu::sra (rs1, opb); has_result = true; break;
    case Op::AND:   result = alu::and_(rs1, opb); has_result = true; break;
    case Op::OR:    result = alu::or_ (rs1, opb); has_result = true; break;
    case Op::XOR:   result = alu::xor_(rs1, opb); has_result = true; break;
    case Op::SLT:   result = alu::slt (rs1, opb); has_result = true; break;
    case Op::SLTU:  result = alu::sltu(rs1, opb); has_result = true; break;

    case Op::LUI:   result = imm;      has_result = true; break;
    case Op::AUIPC: result = pc + imm; has_result = true; break;

    // ---- M extension ------------------------------------------------------
    case Op::MUL:    result = alu::mul   (rs1, rs2); has_result = true; break;
    case Op::MULH:   result = alu::mulh  (rs1, rs2); has_result = true; break;
    case Op::MULHSU: result = alu::mulhsu(rs1, rs2); has_result = true; break;
    case Op::MULHU:  result = alu::mulhu (rs1, rs2); has_result = true; break;
    case Op::DIV:    result = alu::div   (rs1, rs2); has_result = true; break;
    case Op::DIVU:   result = alu::divu  (rs1, rs2); has_result = true; break;
    case Op::REM:    result = alu::rem   (rs1, rs2); has_result = true; break;
    case Op::REMU:   result = alu::remu  (rs1, rs2); has_result = true; break;

    // ---- Control flow -----------------------------------------------------
    case Op::BEQ:  if (alu::beq (rs1, rs2)) next_pc = pc + imm; break;
    case Op::BNE:  if (alu::bne (rs1, rs2)) next_pc = pc + imm; break;
    case Op::BLT:  if (alu::blt (rs1, rs2)) next_pc = pc + imm; break;
    case Op::BGE:  if (alu::bge (rs1, rs2)) next_pc = pc + imm; break;
    case Op::BLTU: if (alu::bltu(rs1, rs2)) next_pc = pc + imm; break;
    case Op::BGEU: if (alu::bgeu(rs1, rs2)) next_pc = pc + imm; break;

    case Op::JAL:
        result = pc + 4; has_result = true;
        next_pc = pc + imm;
        break;
    case Op::JALR:
        // The low bit of the computed target is cleared, per the ISA — and
        // note the link value is computed from the *old* PC, which matters
        // when rd == rs1 (`jalr ra, ra, 0`).
        result = pc + 4; has_result = true;
        next_pc = (rs1 + imm) & ~1u;
        break;

    // ---- Loads ------------------------------------------------------------
    case Op::LB: {
        const auto b = static_cast<int8_t>(mem.load_u8(rs1 + imm));
        result = static_cast<uint32_t>(static_cast<int32_t>(b));
        has_result = true;
        break;
    }
    case Op::LH: {
        const auto h = static_cast<int16_t>(mem.load_u16(rs1 + imm));
        result = static_cast<uint32_t>(static_cast<int32_t>(h));
        has_result = true;
        break;
    }
    case Op::LW:  result = mem.load_u32(rs1 + imm); has_result = true; break;
    case Op::LBU: result = mem.load_u8 (rs1 + imm); has_result = true; break;
    case Op::LHU: result = mem.load_u16(rs1 + imm); has_result = true; break;

    // ---- Stores -----------------------------------------------------------
    case Op::SB: mem.store_u8 (rs1 + imm, static_cast<uint8_t> (rs2)); break;
    case Op::SH: mem.store_u16(rs1 + imm, static_cast<uint16_t>(rs2)); break;
    case Op::SW: mem.store_u32(rs1 + imm, rs2); break;

    // ---- No-ops -----------------------------------------------------------
    case Op::FENCE:
    case Op::FENCE_I:
        break;

    // ---- Traps ------------------------------------------------------------
    case Op::ECALL:
        if (st.regs[REG_A7] == SYS_EXIT) {
            st.exit_code = st.regs[REG_A0];
            st.halted = true;
        } else {
            st.trapped = true;
        }
        break;
    case Op::EBREAK:
    case Op::INVALID:
        st.trapped = true;
        break;
    }

    if (has_result && d.writes_rd) st.regs[d.rd] = result;
    st.regs[0] = 0;                                   // x0 is hardwired

    if (opts.trace && opts.trace_out) {
        std::fprintf(opts.trace_out, "%08X: %08X  next=%08X", pc, raw, next_pc);
        if (has_result && d.writes_rd) {
            std::fprintf(opts.trace_out, "  x%u <- %08X", d.rd, result);
        }
        std::fputc('\n', opts.trace_out);
    }

    st.pc = st.done() ? pc : next_pc;                 // a trap reports its own PC
    ++st.retired;
}

// Run from `entry_pc` until the program halts, traps, or exhausts its
// instruction budget.
inline Result run(Memory& mem, uint32_t entry_pc, const Options& opts = Options{}) {
    Result st;
    st.pc = entry_pc;
    while (!st.done()) {
        if (st.retired >= opts.max_insts) { st.budget = true; break; }
        step(mem, st, opts);
    }
    return st;
}

}  // namespace ref
