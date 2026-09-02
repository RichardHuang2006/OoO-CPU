#pragma once

// Everything that describes one RV32IM instruction, independent of any
// pipeline: identifier types, operation kinds, the decoded form, decode(),
// the pure execution semantics, and disassembly. The pipeline (cpu.h) and the
// reference interpreter (tests/ref.h) both build on this file, so instruction
// behavior is defined exactly once and never depends on speculative state.
//
// Layout:
//   1. architectural types and sentinels
//   2. operation kinds (function-unit routing) and exact operations
//   3. the decoded-instruction record
//   4. decode()                          (implementation in instruction.cpp)
//   5. ALU semantics                     (constexpr, usable in static_assert)
//   6. branch comparisons and targets
//   7. multiply / divide, with the ISA-defined corner cases
//   8. disassembly                       (implementation in instruction.cpp)

#include <cstdint>
#include <limits>
#include <string>

// ---------------------------------------------------------------------------
// 1. Architectural types. All are uint32_t; the names only make signatures
//    readable.
// ---------------------------------------------------------------------------

using ArchReg      = uint32_t;   // architectural register, 0..31
using PhysReg      = uint32_t;   // physical register in the unified PRF
using RobIndex     = uint32_t;   // slot in the reorder buffer
using SeqNum       = uint32_t;   // monotonic; defines "older than"
using CheckpointId = uint32_t;   // slot in the branch-checkpoint pool

// Out-of-band sentinels, disjoint from every legal index.
inline constexpr ArchReg      INVALID_ARCHREG    = std::numeric_limits<ArchReg>::max();
inline constexpr PhysReg      INVALID_PHYSREG    = std::numeric_limits<PhysReg>::max();
inline constexpr RobIndex     INVALID_ROBINDEX   = std::numeric_limits<RobIndex>::max();
inline constexpr SeqNum       INVALID_SEQNUM     = std::numeric_limits<SeqNum>::max();
inline constexpr CheckpointId INVALID_CHECKPOINT = std::numeric_limits<CheckpointId>::max();

// ---------------------------------------------------------------------------
// 2. Operation kinds. `OpKind` picks the function unit, `Op` picks the
//    semantics within it; keeping them separate lets a new op join a class
//    without touching the routing.
// ---------------------------------------------------------------------------

enum class OpKind : uint8_t {
    ALU,
    BRANCH,
    MUL,
    DIV,
    LOAD,
    STORE,
    NOP,     // FENCE / FENCE.I
    TRAP,    // ECALL / EBREAK / illegal
};

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

// ---------------------------------------------------------------------------
// 3. The decoded instruction. Filled once at decode; immutable afterwards.
// ---------------------------------------------------------------------------

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

// ---------------------------------------------------------------------------
// 4. Decode one RV32IM instruction. Unknown encodings return Op::INVALID with
//    kind OpKind::TRAP so the trap stays precise at commit.
// ---------------------------------------------------------------------------

Decoded decode(uint32_t raw);

// OP-IMM is the only class taking its second operand from the immediate. ADDI
// and ADD share an Op, since the function unit is indifferent to the operand's
// source; the opcode is what says which one to read.
inline bool uses_immediate(const Decoded& d) { return (d.raw & 0x7Fu) == 0x13u; }

// ---------------------------------------------------------------------------
// 5.-7. Execution semantics: pure two-input primitives with no PC, no memory,
//    and no pipeline state. LUI, AUIPC, JAL, JALR and address arithmetic all
//    reduce to add(). Everything is constexpr so a regression fails to build
//    when checked by a static_assert.
//
//    and/or/xor are C++ tokens, hence the trailing underscores.
// ---------------------------------------------------------------------------

namespace alu {

// ---- 5. Integer ALU (RV32I) ------------------------------------------------
constexpr uint32_t add (uint32_t a, uint32_t b) { return a + b; }
constexpr uint32_t sub (uint32_t a, uint32_t b) { return a - b; }

// RV32 shifts only use the low 5 bits of the shift amount.
constexpr uint32_t sll (uint32_t a, uint32_t b) { return a << (b & 0x1Fu); }
constexpr uint32_t srl (uint32_t a, uint32_t b) { return a >> (b & 0x1Fu); }

// Signed right shift is implementation-defined in C++17, so fill the sign
// bits explicitly.
constexpr uint32_t sra (uint32_t a, uint32_t b) {
    const uint32_t sh = b & 0x1Fu;
    if (sh == 0) return a;                                   // avoids `<< 32` UB
    const uint32_t sign_fill = (a & 0x80000000u) ? (~0u << (32 - sh)) : 0u;
    return (a >> sh) | sign_fill;
}

constexpr uint32_t and_(uint32_t a, uint32_t b) { return a & b; }
constexpr uint32_t or_ (uint32_t a, uint32_t b) { return a | b; }
constexpr uint32_t xor_(uint32_t a, uint32_t b) { return a ^ b; }

constexpr uint32_t slt (uint32_t a, uint32_t b) {
    return static_cast<int32_t>(a) < static_cast<int32_t>(b) ? 1u : 0u;
}
constexpr uint32_t sltu(uint32_t a, uint32_t b) { return a < b ? 1u : 0u; }

// ---- 6. Branch comparisons -------------------------------------------------
constexpr bool beq (uint32_t a, uint32_t b) { return a == b; }
constexpr bool bne (uint32_t a, uint32_t b) { return a != b; }
constexpr bool blt (uint32_t a, uint32_t b) {
    return static_cast<int32_t>(a) < static_cast<int32_t>(b);
}
constexpr bool bge (uint32_t a, uint32_t b) {
    return static_cast<int32_t>(a) >= static_cast<int32_t>(b);
}
constexpr bool bltu(uint32_t a, uint32_t b) { return a < b; }
constexpr bool bgeu(uint32_t a, uint32_t b) { return a >= b; }

// The comparison an Op stands for; false for any non-conditional op.
constexpr bool branch_taken(Op op, uint32_t a, uint32_t b) {
    switch (op) {
    case Op::BEQ:  return beq (a, b);
    case Op::BNE:  return bne (a, b);
    case Op::BLT:  return blt (a, b);
    case Op::BGE:  return bge (a, b);
    case Op::BLTU: return bltu(a, b);
    case Op::BGEU: return bgeu(a, b);
    default:       return false;
    }
}

// ---- 6. Branch and jump targets ---------------------------------------------
// Bxx and JAL are PC-relative; JALR is register-relative and drops the low
// bit. The link value is the sequential PC, taken from the *old* PC, which
// matters when rd == rs1.
constexpr uint32_t branch_target(uint32_t pc, int32_t imm) {
    return pc + static_cast<uint32_t>(imm);
}
constexpr uint32_t jalr_target(uint32_t rs1, int32_t imm) {
    return (rs1 + static_cast<uint32_t>(imm)) & ~1u;
}
constexpr uint32_t link_address(uint32_t pc) { return pc + 4; }

// ---- 7. M extension: MUL family ---------------------------------------------
// Low 32 bits of the product; uint32_t multiplication is defined to wrap.
constexpr uint32_t mul(uint32_t a, uint32_t b) { return a * b; }

// Upper 32 bits of the signed × signed product.
constexpr uint32_t mulh(uint32_t a, uint32_t b) {
    const int64_t p = static_cast<int64_t>(static_cast<int32_t>(a)) *
                      static_cast<int64_t>(static_cast<int32_t>(b));
    return static_cast<uint32_t>(static_cast<uint64_t>(p) >> 32);
}
// Upper 32 bits of the unsigned × unsigned product.
constexpr uint32_t mulhu(uint32_t a, uint32_t b) {
    const uint64_t p = static_cast<uint64_t>(a) * static_cast<uint64_t>(b);
    return static_cast<uint32_t>(p >> 32);
}
// Upper 32 bits of signed × unsigned; the product always fits in int64.
constexpr uint32_t mulhsu(uint32_t a, uint32_t b) {
    const int64_t p = static_cast<int64_t>(static_cast<int32_t>(a)) *
                      static_cast<int64_t>(static_cast<uint32_t>(b));
    return static_cast<uint32_t>(static_cast<uint64_t>(p) >> 32);
}

// ---- 7. M extension: DIV / REM ----------------------------------------------
// Both edge cases are defined by the ISA and must be guarded: native / and %
// would be UB, and INT_MIN / -1 raises SIGFPE on x86.
constexpr uint32_t div(uint32_t a, uint32_t b) {
    if (b == 0) return 0xFFFFFFFFu;                          // divide-by-zero → -1
    const int32_t sa = static_cast<int32_t>(a);
    const int32_t sb = static_cast<int32_t>(b);
    if (sa == std::numeric_limits<int32_t>::min() && sb == -1) {
        return static_cast<uint32_t>(sa);                    // INT_MIN / -1 → INT_MIN
    }
    return static_cast<uint32_t>(sa / sb);
}
constexpr uint32_t divu(uint32_t a, uint32_t b) {
    if (b == 0) return 0xFFFFFFFFu;
    return a / b;
}
constexpr uint32_t rem(uint32_t a, uint32_t b) {
    if (b == 0) return a;                                    // rem-by-zero → dividend
    const int32_t sa = static_cast<int32_t>(a);
    const int32_t sb = static_cast<int32_t>(b);
    if (sa == std::numeric_limits<int32_t>::min() && sb == -1) return 0;
    return static_cast<uint32_t>(sa % sb);
}
constexpr uint32_t remu(uint32_t a, uint32_t b) {
    if (b == 0) return a;
    return a % b;
}

}  // namespace alu

// ---------------------------------------------------------------------------
// 8. Disassembly, for traces and debugging. Never consulted by execution.
// ---------------------------------------------------------------------------

// ABI name of an architectural register: 10 → "a0". Out-of-range → "x?".
const char* reg_name(ArchReg r);

// One instruction as text, e.g. "addi t0, t0, 1" or "beq a0, a1, 0x1010".
// `pc` resolves PC-relative targets; branch and JAL targets print absolute
// when it is supplied and as "pc+off" when it is zero. Common pseudo-forms
// (nop, li, mv, j, jr, ret) print in their pseudo spelling.
std::string disasm(const Decoded& d, uint32_t pc = 0);

inline std::string disasm(uint32_t raw, uint32_t pc = 0) {
    return disasm(decode(raw), pc);
}
