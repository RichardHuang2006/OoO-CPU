#include "decoder.h"

const char* op_name(Op op) {
    switch (op) {
        case Op::LUI: return "lui";       case Op::AUIPC: return "auipc";
        case Op::JAL: return "jal";       case Op::JALR: return "jalr";
        case Op::BEQ: return "beq";       case Op::BNE: return "bne";
        case Op::BLT: return "blt";       case Op::BGE: return "bge";
        case Op::BLTU: return "bltu";     case Op::BGEU: return "bgeu";
        case Op::LB: return "lb";         case Op::LH: return "lh";
        case Op::LW: return "lw";         case Op::LBU: return "lbu";
        case Op::LHU: return "lhu";       case Op::SB: return "sb";
        case Op::SH: return "sh";         case Op::SW: return "sw";
        case Op::ADDI: return "addi";     case Op::SLTI: return "slti";
        case Op::SLTIU: return "sltiu";   case Op::XORI: return "xori";
        case Op::ORI: return "ori";       case Op::ANDI: return "andi";
        case Op::SLLI: return "slli";     case Op::SRLI: return "srli";
        case Op::SRAI: return "srai";     case Op::ADD: return "add";
        case Op::SUB: return "sub";       case Op::SLL: return "sll";
        case Op::SLT: return "slt";       case Op::SLTU: return "sltu";
        case Op::XOR: return "xor";       case Op::SRL: return "srl";
        case Op::SRA: return "sra";       case Op::OR: return "or";
        case Op::AND: return "and";       case Op::FENCE: return "fence";
        case Op::FENCE_I: return "fence.i"; case Op::ECALL: return "ecall";
        case Op::EBREAK: return "ebreak"; case Op::MUL: return "mul";
        case Op::MULH: return "mulh";     case Op::MULHSU: return "mulhsu";
        case Op::MULHU: return "mulhu";   case Op::DIV: return "div";
        case Op::DIVU: return "divu";     case Op::REM: return "rem";
        case Op::REMU: return "remu";     default: return "illegal";
    }
}

const char* fu_name(FU fu) {
    switch (fu) {
        case FU::ALU: return "alu";       case FU::BRANCH: return "branch";
        case FU::MUL: return "mul";       case FU::DIV: return "div";
        case FU::MEM: return "mem";       case FU::SYSTEM: return "system";
        default: return "none";
    }
}

int DecodedInst::mem_bytes() const {
    switch (op) {
        case Op::LB: case Op::LBU: case Op::SB: return 1;
        case Op::LH: case Op::LHU: case Op::SH: return 2;
        case Op::LW: case Op::SW: return 4;
        default: return 0;
    }
}

bool DecodedInst::mem_signed() const {
    return op == Op::LB || op == Op::LH || op == Op::LW;
}

namespace {

inline int32_t sext(uint32_t v, int bits) {
    uint32_t m = 1u << (bits - 1);
    return (int32_t)((v ^ m) - m);
}

inline uint32_t bits(uint32_t x, int hi, int lo) {
    return (x >> lo) & ((1u << (hi - lo + 1)) - 1);
}

inline int32_t imm_i(uint32_t x) { return sext(bits(x, 31, 20), 12); }
inline int32_t imm_s(uint32_t x) { return sext((bits(x, 31, 25) << 5) | bits(x, 11, 7), 12); }
inline int32_t imm_b(uint32_t x) {
    uint32_t v = (bits(x, 31, 31) << 12) | (bits(x, 7, 7) << 11) |
                 (bits(x, 30, 25) << 5)  | (bits(x, 11, 8) << 1);
    return sext(v, 13);
}
inline int32_t imm_u(uint32_t x) { return (int32_t)(x & 0xfffff000u); }
inline int32_t imm_j(uint32_t x) {
    uint32_t v = (bits(x, 31, 31) << 20) | (bits(x, 19, 12) << 12) |
                 (bits(x, 20, 20) << 11) | (bits(x, 30, 21) << 1);
    return sext(v, 21);
}

// x1 (ra) and x5 (t0) are the link registers the RAS tracks.
inline bool is_link_reg(uint8_t r) { return r == 1 || r == 5; }

} // namespace

DecodedInst decode(uint32_t raw, uint32_t pc) {
    DecodedInst d;
    d.pc = pc;
    d.raw = raw;

    const uint32_t opcode = bits(raw, 6, 0);
    const uint32_t rd     = bits(raw, 11, 7);
    const uint32_t funct3 = bits(raw, 14, 12);
    const uint32_t rs1    = bits(raw, 19, 15);
    const uint32_t rs2    = bits(raw, 24, 20);
    const uint32_t funct7 = bits(raw, 31, 25);

    d.rd = (uint8_t)rd; d.rs1 = (uint8_t)rs1; d.rs2 = (uint8_t)rs2;

    auto illegal = [&]() {
        d.op = Op::ILLEGAL; d.illegal = true; d.fu = FU::SYSTEM;
        d.use_rs1 = d.use_rs2 = d.has_rd = false;
        return d;
    };

    switch (opcode) {
    case 0x37: // LUI
        d.op = Op::LUI; d.fu = FU::ALU; d.imm = imm_u(raw); d.has_rd = true;
        break;
    case 0x17: // AUIPC
        d.op = Op::AUIPC; d.fu = FU::ALU; d.imm = imm_u(raw); d.has_rd = true;
        break;
    case 0x6f: // JAL
        d.op = Op::JAL; d.fu = FU::BRANCH; d.imm = imm_j(raw);
        d.has_rd = true; d.is_jump = true;
        d.is_call = is_link_reg((uint8_t)rd);
        break;
    case 0x67: // JALR
        if (funct3 != 0) return illegal();
        d.op = Op::JALR; d.fu = FU::BRANCH; d.imm = imm_i(raw);
        d.has_rd = true; d.use_rs1 = true; d.is_jump = true; d.is_indirect = true;
        d.is_call = is_link_reg((uint8_t)rd);
        // `ret` is jalr x0, 0(ra): pops the RAS unless it is also a call.
        d.is_ret  = !d.is_call && is_link_reg((uint8_t)rs1) && d.imm == 0;
        break;
    case 0x63: // BRANCH
        d.fu = FU::BRANCH; d.imm = imm_b(raw);
        d.use_rs1 = d.use_rs2 = true; d.is_branch = true;
        switch (funct3) {
            case 0: d.op = Op::BEQ; break;   case 1: d.op = Op::BNE; break;
            case 4: d.op = Op::BLT; break;   case 5: d.op = Op::BGE; break;
            case 6: d.op = Op::BLTU; break;  case 7: d.op = Op::BGEU; break;
            default: return illegal();
        }
        break;
    case 0x03: // LOAD
        d.fu = FU::MEM; d.imm = imm_i(raw);
        d.use_rs1 = true; d.has_rd = true; d.is_load = true;
        switch (funct3) {
            case 0: d.op = Op::LB; break;  case 1: d.op = Op::LH; break;
            case 2: d.op = Op::LW; break;  case 4: d.op = Op::LBU; break;
            case 5: d.op = Op::LHU; break;
            default: return illegal();
        }
        break;
    case 0x23: // STORE
        d.fu = FU::MEM; d.imm = imm_s(raw);
        d.use_rs1 = d.use_rs2 = true; d.is_store = true;
        switch (funct3) {
            case 0: d.op = Op::SB; break;  case 1: d.op = Op::SH; break;
            case 2: d.op = Op::SW; break;
            default: return illegal();
        }
        break;
    case 0x13: // OP-IMM
        d.fu = FU::ALU; d.use_rs1 = true; d.has_rd = true; d.imm = imm_i(raw);
        switch (funct3) {
            case 0: d.op = Op::ADDI; break;
            case 2: d.op = Op::SLTI; break;
            case 3: d.op = Op::SLTIU; break;
            case 4: d.op = Op::XORI; break;
            case 6: d.op = Op::ORI; break;
            case 7: d.op = Op::ANDI; break;
            case 1:
                if (funct7 != 0) return illegal();
                d.op = Op::SLLI; d.imm = (int32_t)rs2;
                break;
            case 5:
                if (funct7 == 0)         { d.op = Op::SRLI; }
                else if (funct7 == 0x20) { d.op = Op::SRAI; }
                else return illegal();
                d.imm = (int32_t)rs2;
                break;
            default: return illegal();
        }
        break;
    case 0x33: // OP
        d.use_rs1 = d.use_rs2 = true; d.has_rd = true;
        if (funct7 == 0x01) { // RV32M
            d.fu = (funct3 >= 4) ? FU::DIV : FU::MUL;
            switch (funct3) {
                case 0: d.op = Op::MUL; break;    case 1: d.op = Op::MULH; break;
                case 2: d.op = Op::MULHSU; break; case 3: d.op = Op::MULHU; break;
                case 4: d.op = Op::DIV; break;    case 5: d.op = Op::DIVU; break;
                case 6: d.op = Op::REM; break;    case 7: d.op = Op::REMU; break;
            }
            break;
        }
        d.fu = FU::ALU;
        switch (funct3) {
            case 0:
                if (funct7 == 0)         d.op = Op::ADD;
                else if (funct7 == 0x20) d.op = Op::SUB;
                else return illegal();
                break;
            case 1: if (funct7) return illegal(); d.op = Op::SLL; break;
            case 2: if (funct7) return illegal(); d.op = Op::SLT; break;
            case 3: if (funct7) return illegal(); d.op = Op::SLTU; break;
            case 4: if (funct7) return illegal(); d.op = Op::XOR; break;
            case 5:
                if (funct7 == 0)         d.op = Op::SRL;
                else if (funct7 == 0x20) d.op = Op::SRA;
                else return illegal();
                break;
            case 6: if (funct7) return illegal(); d.op = Op::OR; break;
            case 7: if (funct7) return illegal(); d.op = Op::AND; break;
        }
        break;
    case 0x0f: // MISC-MEM: fences retire as NOPs
        d.fu = FU::ALU;
        if (funct3 == 0)      d.op = Op::FENCE;
        else if (funct3 == 1) d.op = Op::FENCE_I;
        else return illegal();
        break;
    case 0x73: // SYSTEM
        d.fu = FU::SYSTEM; d.is_system = true;
        if (funct3 != 0) return illegal();
        if (bits(raw, 31, 20) == 0)      d.op = Op::ECALL;
        else if (bits(raw, 31, 20) == 1) d.op = Op::EBREAK;
        else return illegal();
        break;
    default:
        return illegal();
    }

    // x0 is hardwired to zero: a write to it has no destination to rename.
    if (d.rd == 0) d.has_rd = false;
    return d;
}
