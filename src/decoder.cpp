#include "decoder.h"

namespace {

// -- bit-field extraction ---------------------------------------------------
constexpr uint32_t opcode_of(uint32_t x) { return x & 0x7F; }
constexpr uint32_t funct3_of(uint32_t x) { return (x >> 12) & 0x07; }
constexpr uint32_t funct7_of(uint32_t x) { return (x >> 25) & 0x7F; }
constexpr uint32_t rd_of    (uint32_t x) { return (x >> 7)  & 0x1F; }
constexpr uint32_t rs1_of   (uint32_t x) { return (x >> 15) & 0x1F; }
constexpr uint32_t rs2_of   (uint32_t x) { return (x >> 20) & 0x1F; }

// -- sign-extension --------------------------------------------------------
// Extends the low `bits` of `v` to a signed 32-bit int in a way that is
// well-defined under -Wpedantic (no arithmetic on signed overflow).
constexpr int32_t sext(uint32_t v, uint32_t bits) {
    const uint32_t mask = 1u << (bits - 1);
    return static_cast<int32_t>((v ^ mask) - mask);
}

// -- per-format immediate assemblers ---------------------------------------
constexpr int32_t imm_I(uint32_t x) {
    return sext((x >> 20) & 0xFFF, 12);
}
constexpr int32_t imm_S(uint32_t x) {
    const uint32_t v = (((x >> 25) & 0x7F) << 5) | ((x >> 7) & 0x1F);
    return sext(v, 12);
}
constexpr int32_t imm_B(uint32_t x) {
    const uint32_t v = (((x >> 31) & 0x01) << 12) |
                       (((x >>  7) & 0x01) << 11) |
                       (((x >> 25) & 0x3F) <<  5) |
                       (((x >>  8) & 0x0F) <<  1);
    return sext(v, 13);
}
constexpr int32_t imm_U(uint32_t x) {
    // Bits 31..12 are the immediate placed at bits 31..12 of the result;
    // low 12 bits are zero. The result is already sign-correct as int32_t.
    return static_cast<int32_t>(x & 0xFFFFF000u);
}
constexpr int32_t imm_J(uint32_t x) {
    const uint32_t v = (((x >> 31) & 0x0001) << 20) |
                       (((x >> 12) & 0x00FF) << 12) |
                       (((x >> 20) & 0x0001) << 11) |
                       (((x >> 21) & 0x03FF) <<  1);
    return sext(v, 21);
}

Decoded invalid_of(uint32_t raw) {
    Decoded d{};
    d.raw = raw;
    d.op = Op::INVALID;
    d.kind = OpKind::TRAP;
    return d;
}

}  // namespace

Decoded decode(uint32_t raw) {
    Decoded d{};
    d.raw = raw;
    d.rd  = rd_of(raw);
    d.rs1 = rs1_of(raw);
    d.rs2 = rs2_of(raw);

    const uint32_t opcode = opcode_of(raw);
    const uint32_t f3     = funct3_of(raw);
    const uint32_t f7     = funct7_of(raw);

    switch (opcode) {

    // -- U-type ------------------------------------------------------------
    case 0x37: // LUI
        d.op = Op::LUI;
        d.kind = OpKind::ALU;
        d.imm = imm_U(raw);
        d.rs1 = 0; d.rs2 = 0;
        d.writes_rd = (d.rd != 0);
        return d;

    case 0x17: // AUIPC
        d.op = Op::AUIPC;
        d.kind = OpKind::ALU;
        d.imm = imm_U(raw);
        d.rs1 = 0; d.rs2 = 0;
        d.writes_rd = (d.rd != 0);
        return d;

    // -- J-type ------------------------------------------------------------
    case 0x6F: // JAL
        d.op = Op::JAL;
        d.kind = OpKind::BRANCH;
        d.imm = imm_J(raw);
        d.rs1 = 0; d.rs2 = 0;
        d.is_branch = true;
        d.writes_rd = (d.rd != 0);
        return d;

    // -- I-type (jump) -----------------------------------------------------
    case 0x67: // JALR
        if (f3 != 0) return invalid_of(raw);
        d.op = Op::JALR;
        d.kind = OpKind::BRANCH;
        d.imm = imm_I(raw);
        d.rs2 = 0;
        d.is_branch = true;
        d.writes_rd = (d.rd != 0);
        return d;

    // -- B-type ------------------------------------------------------------
    case 0x63: // BRANCH
        d.imm = imm_B(raw);
        d.kind = OpKind::BRANCH;
        d.rd = 0;
        d.is_branch = true;
        switch (f3) {
        case 0x0: d.op = Op::BEQ;  return d;
        case 0x1: d.op = Op::BNE;  return d;
        case 0x4: d.op = Op::BLT;  return d;
        case 0x5: d.op = Op::BGE;  return d;
        case 0x6: d.op = Op::BLTU; return d;
        case 0x7: d.op = Op::BGEU; return d;
        default:  return invalid_of(raw);
        }

    // -- I-type (load) -----------------------------------------------------
    case 0x03: // LOAD
        d.imm = imm_I(raw);
        d.kind = OpKind::LOAD;
        d.rs2 = 0;
        d.is_load = true;
        d.writes_rd = (d.rd != 0);
        switch (f3) {
        case 0x0: d.op = Op::LB;  return d;
        case 0x1: d.op = Op::LH;  return d;
        case 0x2: d.op = Op::LW;  return d;
        case 0x4: d.op = Op::LBU; return d;
        case 0x5: d.op = Op::LHU; return d;
        default:  return invalid_of(raw);
        }

    // -- S-type ------------------------------------------------------------
    case 0x23: // STORE
        d.imm = imm_S(raw);
        d.kind = OpKind::STORE;
        d.rd = 0;
        d.is_store = true;
        switch (f3) {
        case 0x0: d.op = Op::SB; return d;
        case 0x1: d.op = Op::SH; return d;
        case 0x2: d.op = Op::SW; return d;
        default:  return invalid_of(raw);
        }

    // -- I-type (ALU immediate) -------------------------------------------
    case 0x13: // OP-IMM
        d.imm = imm_I(raw);
        d.kind = OpKind::ALU;
        d.rs2 = 0;
        d.writes_rd = (d.rd != 0);
        switch (f3) {
        case 0x0: d.op = Op::ADD;  return d;   // ADDI
        case 0x2: d.op = Op::SLT;  return d;   // SLTI
        case 0x3: d.op = Op::SLTU; return d;   // SLTIU
        case 0x4: d.op = Op::XOR;  return d;   // XORI
        case 0x6: d.op = Op::OR;   return d;   // ORI
        case 0x7: d.op = Op::AND;  return d;   // ANDI
        case 0x1: // SLLI — funct7 must be 0, shamt in rs2 field
            if ((f7 & 0x7E) != 0) return invalid_of(raw);
            d.op = Op::SLL;
            d.imm = static_cast<int32_t>(rs2_of(raw));
            return d;
        case 0x5: // SRLI / SRAI — funct7 bit 5 distinguishes
            if ((f7 & 0x5F) != 0) return invalid_of(raw);
            d.op = (f7 & 0x20) ? Op::SRA : Op::SRL;
            d.imm = static_cast<int32_t>(rs2_of(raw));
            return d;
        }
        return invalid_of(raw);

    // -- R-type (ALU + M extension) ---------------------------------------
    case 0x33: // OP
        d.kind = OpKind::ALU;
        d.imm = 0;
        d.writes_rd = (d.rd != 0);
        if (f7 == 0x01) {
            switch (f3) {
            case 0x0: d.op = Op::MUL;    d.kind = OpKind::MUL; return d;
            case 0x1: d.op = Op::MULH;   d.kind = OpKind::MUL; return d;
            case 0x2: d.op = Op::MULHSU; d.kind = OpKind::MUL; return d;
            case 0x3: d.op = Op::MULHU;  d.kind = OpKind::MUL; return d;
            case 0x4: d.op = Op::DIV;    d.kind = OpKind::DIV; return d;
            case 0x5: d.op = Op::DIVU;   d.kind = OpKind::DIV; return d;
            case 0x6: d.op = Op::REM;    d.kind = OpKind::DIV; return d;
            case 0x7: d.op = Op::REMU;   d.kind = OpKind::DIV; return d;
            }
            return invalid_of(raw);
        }
        if (f7 == 0x00) {
            switch (f3) {
            case 0x0: d.op = Op::ADD;  return d;
            case 0x1: d.op = Op::SLL;  return d;
            case 0x2: d.op = Op::SLT;  return d;
            case 0x3: d.op = Op::SLTU; return d;
            case 0x4: d.op = Op::XOR;  return d;
            case 0x5: d.op = Op::SRL;  return d;
            case 0x6: d.op = Op::OR;   return d;
            case 0x7: d.op = Op::AND;  return d;
            }
        }
        if (f7 == 0x20) {
            switch (f3) {
            case 0x0: d.op = Op::SUB; return d;
            case 0x5: d.op = Op::SRA; return d;
            }
        }
        return invalid_of(raw);

    // -- MISC-MEM (FENCE / FENCE.I) — retire as NOPs, no reorder buffer flush
    case 0x0F:
        d.kind = OpKind::NOP;
        d.rd = 0; d.rs1 = 0; d.rs2 = 0; d.imm = 0;
        switch (f3) {
        case 0x0: d.op = Op::FENCE;   return d;
        case 0x1: d.op = Op::FENCE_I; return d;
        }
        return invalid_of(raw);

    // -- SYSTEM (ECALL / EBREAK) ------------------------------------------
    case 0x73:
        d.kind = OpKind::TRAP;
        d.rd = 0; d.rs1 = 0; d.rs2 = 0; d.imm = 0;
        if (f3 == 0) {
            const uint32_t imm12 = (raw >> 20) & 0xFFF;
            if (imm12 == 0) { d.op = Op::ECALL;  return d; }
            if (imm12 == 1) { d.op = Op::EBREAK; return d; }
        }
        return invalid_of(raw);

    default:
        return invalid_of(raw);
    }
}
