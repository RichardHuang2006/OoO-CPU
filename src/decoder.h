#pragma once

#include <cstdint>
#include "types.h"

// The exact operation. `OpKind` picks the function unit, `Op` picks the
// semantics within it; keeping them separate lets a new op join a class
// without touching the routing.
enum class Op : uint8_t {
    // OpKind::ALU
    ADD, SUB, SLL, SRL, SRA, AND, OR, XOR, SLT, SLTU,
    LUI, AUIPC,
    // OpKind::MUL
    MUL, MULH, MULHSU, MULHU,
    // OpKind::DIV
    DIV, DIVU, REM, REMU,
    // OpKind::BRANCH
    BEQ, BNE, BLT, BGE, BLTU, BGEU,
    JAL, JALR,
    // OpKind::LOAD
    LB, LH, LW, LBU, LHU,
    // OpKind::STORE
    SB, SH, SW,
    // OpKind::NOP
    FENCE, FENCE_I,
    // OpKind::TRAP
    ECALL, EBREAK,
    INVALID,   // unknown opcode / funct combination
};

struct Decoded {
    uint32_t raw;         // original 32-bit instruction word
    Op       op;
    OpKind   kind;        // function-unit class
    ArchReg  rd;          // 0 if the instruction does not use rd
    ArchReg  rs1;         // 0 if unused
    ArchReg  rs2;         // 0 if unused
    int32_t  imm;         // sign-extended once, at decode
    bool     is_branch;   // any control-flow-changing op (Bxx, JAL, JALR)
    bool     is_load;
    bool     is_store;
    bool     writes_rd;   // rd != 0 AND the op semantically writes rd
};

// Decode one RV32IM instruction. Unknown encodings return Op::INVALID with
// kind OpKind::TRAP so the trap stays precise at commit.
Decoded decode(uint32_t raw);
