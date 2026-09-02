#include "instruction.h"

#include <cstdio>

// Layout continues from instruction.h:
//   4a. decode helpers: field extraction, sign extension, immediate assembly
//   4b. RV32I decoding
//   4c. M-extension decoding (funct7 == 0x01 under opcode OP)
//   8.  disassembly

namespace {

// -- 4a. bit-field extraction ------------------------------------------------
constexpr uint32_t opcode_of(uint32_t x) { return x & 0x7F; }
constexpr uint32_t funct3_of(uint32_t x) { return (x >> 12) & 0x07; }
constexpr uint32_t funct7_of(uint32_t x) { return (x >> 25) & 0x7F; }
constexpr uint32_t rd_of    (uint32_t x) { return (x >> 7)  & 0x1F; }
constexpr uint32_t rs1_of   (uint32_t x) { return (x >> 15) & 0x1F; }
constexpr uint32_t rs2_of   (uint32_t x) { return (x >> 20) & 0x1F; }

// -- 4a. sign-extension -------------------------------------------------------
// Extends the low `bits` of `v` without relying on signed overflow.
constexpr int32_t sext(uint32_t v, uint32_t bits) {
    const uint32_t mask = 1u << (bits - 1);
    return static_cast<int32_t>((v ^ mask) - mask);
}

// -- 4a. per-format immediate assemblers --------------------------------------
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
    // Immediate stays at bits 31..12; the low 12 are zero and the result is
    // already sign-correct.
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

// -- 4b./4c. RV32I and M-extension decoding -----------------------------------
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

    // -- I-type (jump) -------------------------------------------------------
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

    // -- I-type (load) -------------------------------------------------------
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

    // -- I-type (ALU immediate) ---------------------------------------------
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

    // -- R-type (ALU + M extension) -------------------------------------------
    case 0x33: // OP
        d.kind = OpKind::ALU;
        d.imm = 0;
        d.writes_rd = (d.rd != 0);
        if (f7 == 0x01) {          // -- 4c. the whole M extension lives here
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

    // -- MISC-MEM (FENCE / FENCE.I) — retire as NOPs
    case 0x0F:
        d.kind = OpKind::NOP;
        d.rd = 0; d.rs1 = 0; d.rs2 = 0; d.imm = 0;
        switch (f3) {
        case 0x0: d.op = Op::FENCE;   return d;
        case 0x1: d.op = Op::FENCE_I; return d;
        }
        return invalid_of(raw);

    // -- SYSTEM (ECALL / EBREAK) ----------------------------------------------
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

// ---------------------------------------------------------------------------
// 8. Disassembly
// ---------------------------------------------------------------------------

const char* reg_name(ArchReg r) {
    static const char* const names[32] = {
        "zero", "ra", "sp", "gp", "tp",  "t0", "t1", "t2",
        "s0",   "s1", "a0", "a1", "a2",  "a3", "a4", "a5",
        "a6",   "a7", "s2", "s3", "s4",  "s5", "s6", "s7",
        "s8",   "s9", "s10","s11","t3",  "t4", "t5", "t6",
    };
    return r < 32 ? names[r] : "x?";
}

namespace {

std::string hex_of(uint32_t v) {
    char buf[16];
    std::snprintf(buf, sizeof(buf), "0x%X", v);
    return buf;
}

// Branch / JAL target: absolute when the caller knows the PC, relative
// otherwise, so a lone decode still disassembles readably.
std::string target_of(uint32_t pc, int32_t imm) {
    if (pc != 0) return hex_of(alu::branch_target(pc, imm));
    char buf[24];
    std::snprintf(buf, sizeof(buf), "pc%+d", imm);
    return buf;
}

std::string r3(const char* m, ArchReg rd, ArchReg a, ArchReg b) {
    return std::string(m) + " " + reg_name(rd) + ", " + reg_name(a) + ", " + reg_name(b);
}
std::string ri(const char* m, ArchReg rd, ArchReg a, int32_t imm) {
    return std::string(m) + " " + reg_name(rd) + ", " + reg_name(a) + ", " +
           std::to_string(imm);
}
std::string mem_form(const char* m, ArchReg r, ArchReg base, int32_t off) {
    return std::string(m) + " " + reg_name(r) + ", " + std::to_string(off) +
           "(" + reg_name(base) + ")";
}

const char* branch_mnemonic(Op op) {
    switch (op) {
    case Op::BEQ:  return "beq";
    case Op::BNE:  return "bne";
    case Op::BLT:  return "blt";
    case Op::BGE:  return "bge";
    case Op::BLTU: return "bltu";
    default:       return "bgeu";
    }
}

}  // namespace

std::string disasm(const Decoded& d, uint32_t pc) {
    const bool imm_form = uses_immediate(d);   // OP-IMM vs OP spelling

    switch (d.op) {
    // ---- ALU, immediate and register forms ---------------------------------
    case Op::ADD:
        if (!imm_form) return r3("add", d.rd, d.rs1, d.rs2);
        // The canonical pseudo-instructions are all ADDI in disguise.
        if (d.rd == 0 && d.rs1 == 0 && d.imm == 0) return "nop";
        if (d.rs1 == 0) return std::string("li ") + reg_name(d.rd) + ", " +
                               std::to_string(d.imm);
        if (d.imm == 0) return std::string("mv ") + reg_name(d.rd) + ", " +
                               reg_name(d.rs1);
        return ri("addi", d.rd, d.rs1, d.imm);
    case Op::SUB:  return r3("sub", d.rd, d.rs1, d.rs2);
    case Op::SLL:  return imm_form ? ri("slli", d.rd, d.rs1, d.imm) : r3("sll", d.rd, d.rs1, d.rs2);
    case Op::SRL:  return imm_form ? ri("srli", d.rd, d.rs1, d.imm) : r3("srl", d.rd, d.rs1, d.rs2);
    case Op::SRA:  return imm_form ? ri("srai", d.rd, d.rs1, d.imm) : r3("sra", d.rd, d.rs1, d.rs2);
    case Op::AND:  return imm_form ? ri("andi", d.rd, d.rs1, d.imm) : r3("and", d.rd, d.rs1, d.rs2);
    case Op::OR:   return imm_form ? ri("ori",  d.rd, d.rs1, d.imm) : r3("or",  d.rd, d.rs1, d.rs2);
    case Op::XOR:  return imm_form ? ri("xori", d.rd, d.rs1, d.imm) : r3("xor", d.rd, d.rs1, d.rs2);
    case Op::SLT:  return imm_form ? ri("slti", d.rd, d.rs1, d.imm) : r3("slt", d.rd, d.rs1, d.rs2);
    case Op::SLTU: return imm_form ? ri("sltiu", d.rd, d.rs1, d.imm) : r3("sltu", d.rd, d.rs1, d.rs2);

    case Op::LUI:
        return std::string("lui ") + reg_name(d.rd) + ", " +
               hex_of((static_cast<uint32_t>(d.imm) >> 12) & 0xFFFFF);
    case Op::AUIPC:
        return std::string("auipc ") + reg_name(d.rd) + ", " +
               hex_of((static_cast<uint32_t>(d.imm) >> 12) & 0xFFFFF);

    // ---- M extension --------------------------------------------------------
    case Op::MUL:    return r3("mul",    d.rd, d.rs1, d.rs2);
    case Op::MULH:   return r3("mulh",   d.rd, d.rs1, d.rs2);
    case Op::MULHSU: return r3("mulhsu", d.rd, d.rs1, d.rs2);
    case Op::MULHU:  return r3("mulhu",  d.rd, d.rs1, d.rs2);
    case Op::DIV:    return r3("div",    d.rd, d.rs1, d.rs2);
    case Op::DIVU:   return r3("divu",   d.rd, d.rs1, d.rs2);
    case Op::REM:    return r3("rem",    d.rd, d.rs1, d.rs2);
    case Op::REMU:   return r3("remu",   d.rd, d.rs1, d.rs2);

    // ---- Control flow --------------------------------------------------------
    case Op::BEQ: case Op::BNE: case Op::BLT:
    case Op::BGE: case Op::BLTU: case Op::BGEU:
        return std::string(branch_mnemonic(d.op)) + " " + reg_name(d.rs1) +
               ", " + reg_name(d.rs2) + ", " + target_of(pc, d.imm);

    case Op::JAL:
        if (d.rd == 0) return std::string("j ") + target_of(pc, d.imm);
        return std::string("jal ") + reg_name(d.rd) + ", " + target_of(pc, d.imm);
    case Op::JALR:
        if (d.rd == 0 && d.rs1 == 1 && d.imm == 0) return "ret";
        if (d.rd == 0 && d.imm == 0) return std::string("jr ") + reg_name(d.rs1);
        return std::string("jalr ") + reg_name(d.rd) + ", " +
               std::to_string(d.imm) + "(" + reg_name(d.rs1) + ")";

    // ---- Memory --------------------------------------------------------------
    case Op::LB:  return mem_form("lb",  d.rd,  d.rs1, d.imm);
    case Op::LH:  return mem_form("lh",  d.rd,  d.rs1, d.imm);
    case Op::LW:  return mem_form("lw",  d.rd,  d.rs1, d.imm);
    case Op::LBU: return mem_form("lbu", d.rd,  d.rs1, d.imm);
    case Op::LHU: return mem_form("lhu", d.rd,  d.rs1, d.imm);
    case Op::SB:  return mem_form("sb",  d.rs2, d.rs1, d.imm);
    case Op::SH:  return mem_form("sh",  d.rs2, d.rs1, d.imm);
    case Op::SW:  return mem_form("sw",  d.rs2, d.rs1, d.imm);

    // ---- System ----------------------------------------------------------------
    case Op::FENCE:   return "fence";
    case Op::FENCE_I: return "fence.i";
    case Op::ECALL:   return "ecall";
    case Op::EBREAK:  return "ebreak";
    case Op::INVALID: break;
    }
    char buf[32];
    std::snprintf(buf, sizeof(buf), "illegal 0x%08X", d.raw);
    return buf;
}
