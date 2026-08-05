#pragma once

#include <cstdint>
#include <limits>

// Fundamental identifier types for the pipeline. All aliases are uint32_t so
// they compare, hash, and pack the same way; the aliases exist to make
// function signatures self-documenting (a RobIndex is not interchangeable with
// a PhysReg to a human reader).

using ArchReg  = uint32_t;   // architectural register (0..31 for RV32I)
using PhysReg  = uint32_t;   // physical register in the unified PRF (DESIGN.md §3)
using RobIndex = uint32_t;   // slot in the reorder buffer
using SeqNum   = uint32_t;   // monotonic instruction sequence number; defines "older than"

// Sentinels instead of sprinkling -1 casts through the code. Picked at the
// numeric max so they are trivially disjoint from every legal index — no
// sizing knob approaches 2^32 - 1 — and they compose with the
// std::optional<T> idiom used at Phase-4 boundaries: opt.value_or(INVALID_*).
inline constexpr ArchReg  INVALID_ARCHREG  = std::numeric_limits<ArchReg>::max();
inline constexpr PhysReg  INVALID_PHYSREG  = std::numeric_limits<PhysReg>::max();
inline constexpr RobIndex INVALID_ROBINDEX = std::numeric_limits<RobIndex>::max();
inline constexpr SeqNum   INVALID_SEQNUM   = std::numeric_limits<SeqNum>::max();

// Function-unit routing tag. Every decoded op belongs to exactly one class;
// Phase 5's dispatch and select stages branch on this and nothing else.
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
