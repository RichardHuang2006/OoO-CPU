#pragma once

#include <cstdint>
#include <limits>

// Identifier aliases. All are uint32_t; the names only make signatures
// readable.

using ArchReg      = uint32_t;   // architectural register, 0..31
using PhysReg      = uint32_t;   // physical register in the unified PRF
using RobIndex     = uint32_t;   // slot in the reorder buffer
using SeqNum       = uint32_t;   // monotonic; defines "older than"
using CheckpointId = uint32_t;   // slot in the RAT checkpoint pool

// Out-of-band sentinels, disjoint from every legal index.
inline constexpr ArchReg      INVALID_ARCHREG    = std::numeric_limits<ArchReg>::max();
inline constexpr PhysReg      INVALID_PHYSREG    = std::numeric_limits<PhysReg>::max();
inline constexpr RobIndex     INVALID_ROBINDEX   = std::numeric_limits<RobIndex>::max();
inline constexpr SeqNum       INVALID_SEQNUM     = std::numeric_limits<SeqNum>::max();
inline constexpr CheckpointId INVALID_CHECKPOINT = std::numeric_limits<CheckpointId>::max();

// Function-unit routing tag. Every decoded op belongs to exactly one class.
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
