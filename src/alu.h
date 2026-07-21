// Functional model. Evaluated at Execute, once operands have been read.
#pragma once
#include "types.h"

// Non-memory, non-branch datapath result.
inline uint32_t alu_execute(const DecodedInst& d, uint32_t a, uint32_t b) {
    const int32_t sa = (int32_t)a, sb = (int32_t)b;
    const uint32_t sh_i = (uint32_t)d.imm & 31;
    const uint32_t sh_r = b & 31;

    switch (d.op) {
        case Op::LUI:   return (uint32_t)d.imm;
        case Op::AUIPC: return d.pc + (uint32_t)d.imm;

        case Op::ADDI:  return a + (uint32_t)d.imm;
        case Op::SLTI:  return sa < d.imm;
        case Op::SLTIU: return a < (uint32_t)d.imm;
        case Op::XORI:  return a ^ (uint32_t)d.imm;
        case Op::ORI:   return a | (uint32_t)d.imm;
        case Op::ANDI:  return a & (uint32_t)d.imm;
        case Op::SLLI:  return a << sh_i;
        case Op::SRLI:  return a >> sh_i;
        case Op::SRAI:  return (uint32_t)(sa >> sh_i);

        case Op::ADD:   return a + b;
        case Op::SUB:   return a - b;
        case Op::SLL:   return a << sh_r;
        case Op::SLT:   return sa < sb;
        case Op::SLTU:  return a < b;
        case Op::XOR:   return a ^ b;
        case Op::SRL:   return a >> sh_r;
        case Op::SRA:   return (uint32_t)(sa >> sh_r);
        case Op::OR:    return a | b;
        case Op::AND:   return a & b;

        case Op::MUL:    return (uint32_t)((int64_t)sa * (int64_t)sb);
        case Op::MULH:   return (uint32_t)(((int64_t)sa * (int64_t)sb) >> 32);
        case Op::MULHU:  return (uint32_t)(((uint64_t)a * (uint64_t)b) >> 32);
        case Op::MULHSU: return (uint32_t)(((int64_t)sa * (int64_t)(uint64_t)b) >> 32);

        // Division by zero and signed overflow are defined by the ISA, not trapping.
        case Op::DIV:
            if (b == 0) return 0xffffffffu;
            if (a == 0x80000000u && b == 0xffffffffu) return 0x80000000u;
            return (uint32_t)(sa / sb);
        case Op::DIVU:  return b == 0 ? 0xffffffffu : a / b;
        case Op::REM:
            if (b == 0) return a;
            if (a == 0x80000000u && b == 0xffffffffu) return 0;
            return (uint32_t)(sa % sb);
        case Op::REMU:  return b == 0 ? a : a % b;

        case Op::FENCE: case Op::FENCE_I: return 0;
        default: return 0;
    }
}

inline bool branch_taken(Op op, uint32_t a, uint32_t b) {
    switch (op) {
        case Op::BEQ:  return a == b;
        case Op::BNE:  return a != b;
        case Op::BLT:  return (int32_t)a <  (int32_t)b;
        case Op::BGE:  return (int32_t)a >= (int32_t)b;
        case Op::BLTU: return a <  b;
        case Op::BGEU: return a >= b;
        default: return false;
    }
}

// Sign/zero extension applied to a load's raw bytes.
inline uint32_t load_extend(Op op, uint32_t raw) {
    switch (op) {
        case Op::LB:  return (uint32_t)(int32_t)(int8_t)(uint8_t)raw;
        case Op::LH:  return (uint32_t)(int32_t)(int16_t)(uint16_t)raw;
        case Op::LBU: return raw & 0xff;
        case Op::LHU: return raw & 0xffff;
        default:      return raw;
    }
}
