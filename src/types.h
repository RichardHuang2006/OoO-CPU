// Static instruction description plus the dynamic uop that flows down the pipe.
#pragma once
#include <cstdint>
#include <string>
#include <vector>

enum class Op : uint8_t {
    ILLEGAL = 0,
    // U / J
    LUI, AUIPC, JAL, JALR,
    // B
    BEQ, BNE, BLT, BGE, BLTU, BGEU,
    // loads
    LB, LH, LW, LBU, LHU,
    // stores
    SB, SH, SW,
    // I-type ALU
    ADDI, SLTI, SLTIU, XORI, ORI, ANDI, SLLI, SRLI, SRAI,
    // R-type ALU
    ADD, SUB, SLL, SLT, SLTU, XOR, SRL, SRA, OR, AND,
    // misc
    FENCE, FENCE_I, ECALL, EBREAK,
    // M extension (optional, drives the pipelined multiplier)
    MUL, MULH, MULHSU, MULHU, DIV, DIVU, REM, REMU,
};

const char* op_name(Op op);

// Which function-unit class an op needs.
enum class FU : uint8_t { NONE = 0, ALU, BRANCH, MUL, DIV, MEM, SYSTEM };

const char* fu_name(FU fu);

// Result of decoding one 32-bit word.
struct DecodedInst {
    uint32_t pc  = 0;
    uint32_t raw = 0;
    Op  op = Op::ILLEGAL;
    FU  fu = FU::NONE;

    uint8_t rs1 = 0, rs2 = 0, rd = 0;
    bool use_rs1 = false, use_rs2 = false, has_rd = false;
    int32_t imm = 0;

    bool is_branch = false;   // conditional branch
    bool is_jump   = false;   // JAL / JALR
    bool is_indirect = false; // JALR
    bool is_load   = false;
    bool is_store  = false;
    bool is_system = false;   // ECALL / EBREAK -> trap at commit
    bool is_call   = false;   // JAL/JALR with rd == x1/x5  (RAS push)
    bool is_ret    = false;   // JALR x0, 0(x1/x5)          (RAS pop)
    bool illegal   = false;

    // Anything that can steer the front end.
    bool is_ctrl() const { return is_branch || is_jump; }
    int  mem_bytes() const;
    bool mem_signed() const;
};

// A dynamic instruction. One instance is copied between pipeline latches; the
// out-of-order structures index it by rob_idx.
struct Uop {
    DecodedInst d;
    uint64_t seq = 0;         // global age, monotonically increasing

    int rob_idx = -1;
    int lq_idx  = -1;
    int sq_idx  = -1;
    int chkpt   = -1;         // branch checkpoint id, -1 if none

    // Renaming
    int pdst   = -1;          // new physical destination
    int pstale = -1;          // previous mapping of the arch dest
    int psrc1  = -1;
    int psrc2  = -1;

    // Operands (read from the PRF after select)
    uint32_t val1 = 0, val2 = 0;
    uint32_t result = 0;

    // Front-end prediction state. The RAS is speculative and is snapshotted in
    // Fetch (not Rename) because younger fetches mutate it before this uop
    // reaches the rename stage.
    bool     pred_taken  = false;
    uint32_t pred_target = 0;
    uint32_t ghr_snapshot = 0;      // GHR *before* this branch's own update
    bool     btb_hit = false;
    bool     used_ras = false;
    std::vector<uint32_t> ras_snapshot;
    int      ras_top = 0, ras_depth = 0;

    // Execute outcome
    bool     mispredicted = false;
    bool     actual_taken = false;
    uint32_t actual_target = 0;

    // Memory
    uint32_t addr = 0;
};
