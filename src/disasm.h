#pragma once

#include <cstdint>
#include <cstdio>
#include <string>

#include "decoder.h"
#include "types.h"

// One decoded instruction as text, for traces and for anything a human reads.
//
// The decoder throws away the distinction between ADD and ADDI — the function
// unit does not care where the second operand came from — so the mnemonic is
// recovered here from the opcode field, the one place the raw word still says
// it. Branch and jump targets print absolute, because a trace viewer wants an
// address it can compare against a PC, not a displacement it would have to add.

namespace disasm_detail {

inline std::string reg(ArchReg r) { return "x" + std::to_string(r); }

inline std::string hex32(uint32_t v) {
    char buf[16];
    std::snprintf(buf, sizeof(buf), "0x%X", v);
    return buf;
}

inline std::string dec(int32_t v) { return std::to_string(v); }

// OP-IMM is the only class taking its second operand from the immediate.
inline bool is_op_imm(const Decoded& d) { return (d.raw & 0x7Fu) == 0x13u; }

inline std::string rrr(const char* m, const Decoded& d) {
    return std::string(m) + " " + reg(d.rd) + "," + reg(d.rs1) + "," + reg(d.rs2);
}
inline std::string rri(const char* m, const Decoded& d) {
    return std::string(m) + " " + reg(d.rd) + "," + reg(d.rs1) + "," + dec(d.imm);
}
inline std::string mem(const char* m, ArchReg data, const Decoded& d) {
    return std::string(m) + " " + reg(data) + "," + dec(d.imm) + "(" + reg(d.rs1) + ")";
}
inline std::string br(const char* m, const Decoded& d, uint32_t pc) {
    return std::string(m) + " " + reg(d.rs1) + "," + reg(d.rs2) + "," +
           hex32(pc + static_cast<uint32_t>(d.imm));
}

}  // namespace disasm_detail

// `pc` only matters for the ops whose printed form names an address.
inline std::string disasm(const Decoded& d, uint32_t pc) {
    using namespace disasm_detail;
    const bool imm_form = is_op_imm(d);

    switch (d.op) {
    case Op::ADD:  return imm_form ? rri("addi",  d) : rrr("add",  d);
    case Op::SLT:  return imm_form ? rri("slti",  d) : rrr("slt",  d);
    case Op::SLTU: return imm_form ? rri("sltiu", d) : rrr("sltu", d);
    case Op::XOR:  return imm_form ? rri("xori",  d) : rrr("xor",  d);
    case Op::OR:   return imm_form ? rri("ori",   d) : rrr("or",   d);
    case Op::AND:  return imm_form ? rri("andi",  d) : rrr("and",  d);
    case Op::SLL:  return imm_form ? rri("slli",  d) : rrr("sll",  d);
    case Op::SRL:  return imm_form ? rri("srli",  d) : rrr("srl",  d);
    case Op::SRA:  return imm_form ? rri("srai",  d) : rrr("sra",  d);
    case Op::SUB:  return rrr("sub", d);

    // The immediate already sits at bits 31..12; print the field, the way an
    // assembler wrote it.
    case Op::LUI:
        return "lui " + reg(d.rd) + "," + hex32(static_cast<uint32_t>(d.imm) >> 12);
    case Op::AUIPC:
        return "auipc " + reg(d.rd) + "," + hex32(static_cast<uint32_t>(d.imm) >> 12);

    case Op::MUL:    return rrr("mul",    d);
    case Op::MULH:   return rrr("mulh",   d);
    case Op::MULHSU: return rrr("mulhsu", d);
    case Op::MULHU:  return rrr("mulhu",  d);
    case Op::DIV:    return rrr("div",    d);
    case Op::DIVU:   return rrr("divu",   d);
    case Op::REM:    return rrr("rem",    d);
    case Op::REMU:   return rrr("remu",   d);

    case Op::BEQ:  return br("beq",  d, pc);
    case Op::BNE:  return br("bne",  d, pc);
    case Op::BLT:  return br("blt",  d, pc);
    case Op::BGE:  return br("bge",  d, pc);
    case Op::BLTU: return br("bltu", d, pc);
    case Op::BGEU: return br("bgeu", d, pc);

    case Op::JAL:
        return "jal " + reg(d.rd) + "," + hex32(pc + static_cast<uint32_t>(d.imm));
    case Op::JALR:
        return "jalr " + reg(d.rd) + "," + dec(d.imm) + "(" + reg(d.rs1) + ")";

    case Op::LB:  return mem("lb",  d.rd, d);
    case Op::LH:  return mem("lh",  d.rd, d);
    case Op::LW:  return mem("lw",  d.rd, d);
    case Op::LBU: return mem("lbu", d.rd, d);
    case Op::LHU: return mem("lhu", d.rd, d);

    // A store names the register it reads, which the decoder keeps in rs2.
    case Op::SB: return mem("sb", d.rs2, d);
    case Op::SH: return mem("sh", d.rs2, d);
    case Op::SW: return mem("sw", d.rs2, d);

    case Op::FENCE:   return "fence";
    case Op::FENCE_I: return "fence.i";
    case Op::ECALL:   return "ecall";
    case Op::EBREAK:  return "ebreak";
    case Op::INVALID: break;
    }
    return "<invalid " + hex32(d.raw) + ">";
}

// Convenience for callers holding only the word.
inline std::string disasm(uint32_t raw, uint32_t pc) { return disasm(decode(raw), pc); }
