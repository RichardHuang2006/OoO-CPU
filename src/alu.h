#pragma once

#include <cstdint>
#include <limits>

// Pure two-input primitives for RV32IM. No PC or immediate here: LUI, AUIPC,
// JAL, JALR and address arithmetic all reduce to add().
//
// and/or/xor are C++ tokens, hence the trailing underscores.

namespace alu {

// ---- Integer ALU (RV32I) --------------------------------------------------
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

// ---- M extension: MUL family ---------------------------------------------
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

// ---- M extension: DIV / REM ----------------------------------------------
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

// ---- Branch conditions ---------------------------------------------------
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

}  // namespace alu
