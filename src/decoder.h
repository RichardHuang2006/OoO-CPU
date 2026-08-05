#pragma once

#include <cstdint>
#include "types.h"

// Specific operation. `OpKind` (types.h) gives the latency / FU class that
// Phase-5 dispatch routes on; `Op` picks the exact semantics that alu.h
// dispatches on inside a class. Keeping the two enums separate means a new
// pipelined multiply variant can drop into OpKind::MUL without adding a case
// to the ALU switch.
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
    OpKind   kind;        // FU / latency class (types.h)
    ArchReg  rd;          // 0 if the instruction does not use rd
    ArchReg  rs1;         // 0 if unused
    ArchReg  rs2;         // 0 if unused
    int32_t  imm;         // sign-extended once, at decode
    bool     is_branch;   // any control-flow-changing op (Bxx, JAL, JALR)
    bool     is_load;
    bool     is_store;
    bool     writes_rd;   // rd != 0 AND the op semantically writes rd
};

// Decode one 32-bit RV32IM instruction. Unknown opcode / funct combinations
// return `Op::INVALID` with `kind = OpKind::TRAP` — the trap is precise at
// commit rather than a decoder-time crash.
Decoded decode(uint32_t raw);
