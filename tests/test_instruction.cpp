// Instruction-level tests: architectural types, RV32I and M-extension
// decoding, immediate extraction, the pure ALU / branch / multiply / divide
// semantics, the in-tree assembler, and disassembly. Nothing here constructs
// a pipeline: decode and semantics are testable in isolation by design.

#include <initializer_list>
#include <stdexcept>
#include <type_traits>

#include "test_support.h"

// ------------------------------------------------------- @section("types") ---
SECTION("types") {
    static_assert(std::is_same_v<ArchReg,  uint32_t>);
    static_assert(std::is_same_v<PhysReg,  uint32_t>);
    static_assert(std::is_same_v<RobIndex, uint32_t>);
    static_assert(std::is_same_v<SeqNum,   uint32_t>);

    // sentinels are outside any plausible in-band range
    REQUIRE(INVALID_ARCHREG  > 31u);
    REQUIRE(INVALID_PHYSREG  > (1u << 16));
    REQUIRE(INVALID_ROBINDEX > (1u << 16));
    REQUIRE(INVALID_SEQNUM   > (1u << 16));

    // round-trip through the std::optional<PhysReg> idiom
    auto to_opt = [](PhysReg r) -> std::optional<PhysReg> {
        return (r == INVALID_PHYSREG) ? std::nullopt : std::optional<PhysReg>(r);
    };
    auto from_opt = [](std::optional<PhysReg> o) -> PhysReg {
        return o.value_or(INVALID_PHYSREG);
    };

    REQUIRE(from_opt(to_opt(INVALID_PHYSREG)) == INVALID_PHYSREG);
    REQUIRE(!to_opt(INVALID_PHYSREG).has_value());
    for (PhysReg r : std::initializer_list<PhysReg>{0u, 1u, 63u}) {
        auto opt = to_opt(r);
        REQUIRE(opt.has_value());
        REQUIRE(from_opt(opt) == r);
    }

    // OpKind values are distinct, which is the whole invariant at this stage.
    static_assert(static_cast<int>(OpKind::ALU)  != static_cast<int>(OpKind::TRAP));
    static_assert(static_cast<int>(OpKind::LOAD) != static_cast<int>(OpKind::STORE));
    static_assert(static_cast<int>(OpKind::NOP)  != static_cast<int>(OpKind::ALU));
}

// ------------------------------------------------------ @section("decode") ---
namespace enc {
    // File-local encoders, so the decode cases need no raw hex constants.
    constexpr uint32_t R(uint32_t op, uint32_t rd, uint32_t f3,
                         uint32_t rs1, uint32_t rs2, uint32_t f7) {
        return (f7 << 25) | (rs2 << 20) | (rs1 << 15) | (f3 << 12) | (rd << 7) | op;
    }
    constexpr uint32_t I(uint32_t op, uint32_t rd, uint32_t f3,
                         uint32_t rs1, int32_t imm) {
        const uint32_t u = static_cast<uint32_t>(imm) & 0xFFF;
        return (u << 20) | (rs1 << 15) | (f3 << 12) | (rd << 7) | op;
    }
    constexpr uint32_t S(uint32_t f3, uint32_t rs1, uint32_t rs2, int32_t imm) {
        const uint32_t u = static_cast<uint32_t>(imm) & 0xFFF;
        const uint32_t hi = (u >> 5) & 0x7F;
        const uint32_t lo = u & 0x1F;
        return (hi << 25) | (rs2 << 20) | (rs1 << 15) | (f3 << 12) | (lo << 7) | 0x23;
    }
    constexpr uint32_t B(uint32_t f3, uint32_t rs1, uint32_t rs2, int32_t imm) {
        const uint32_t u = static_cast<uint32_t>(imm) & 0x1FFF;
        const uint32_t b12   = (u >> 12) & 0x1;
        const uint32_t b11   = (u >> 11) & 0x1;
        const uint32_t b10_5 = (u >> 5) & 0x3F;
        const uint32_t b4_1  = (u >> 1) & 0xF;
        return (b12 << 31) | (b10_5 << 25) | (rs2 << 20) | (rs1 << 15) |
               (f3 << 12) | (b4_1 << 8) | (b11 << 7) | 0x63;
    }
    constexpr uint32_t U(uint32_t op, uint32_t rd, uint32_t imm_hi20) {
        return ((imm_hi20 & 0xFFFFF) << 12) | (rd << 7) | op;
    }
    constexpr uint32_t J(uint32_t rd, int32_t imm) {
        const uint32_t u = static_cast<uint32_t>(imm) & 0x1FFFFF;
        const uint32_t b20    = (u >> 20) & 0x1;
        const uint32_t b19_12 = (u >> 12) & 0xFF;
        const uint32_t b11    = (u >> 11) & 0x1;
        const uint32_t b10_1  = (u >> 1)  & 0x3FF;
        return (b20 << 31) | (b10_1 << 21) | (b11 << 20) | (b19_12 << 12) |
               (rd << 7) | 0x6F;
    }
    // pseudos for ECALL / EBREAK / FENCE (system with fixed imm)
    constexpr uint32_t ECALL()   { return 0x00000073u; }
    constexpr uint32_t EBREAK()  { return 0x00100073u; }
    constexpr uint32_t FENCE()   { return 0x0000000Fu; }  // pred=succ=0
    constexpr uint32_t FENCE_I() { return 0x0000100Fu; }
}

SECTION("decode") {
    struct C {
        uint32_t raw;
        Op       op;
        OpKind   kind;
        ArchReg  rd, rs1, rs2;
        int32_t  imm;
        bool     br, ld, st, wrd;
    };

    // ---- U-type, sign-preserved --------------------------------------------
    const C cases[] = {
        {enc::U(0x37, 5, 0xABCDE), Op::LUI,   OpKind::ALU,    5, 0, 0, int32_t(0xABCDE000), 0,0,0, 1},
        {enc::U(0x37, 1, 0x80000), Op::LUI,   OpKind::ALU,    1, 0, 0, int32_t(0x80000000), 0,0,0, 1},
        {enc::U(0x17, 6, 0x12345), Op::AUIPC, OpKind::ALU,    6, 0, 0, int32_t(0x12345000), 0,0,0, 1},
        // rd = 0 → writes_rd false (a defined no-op result)
        {enc::U(0x37, 0, 0x1),     Op::LUI,   OpKind::ALU,    0, 0, 0, int32_t(0x00001000), 0,0,0, 0},

        // ---- J-type ------------------------------------------------------
        {enc::J(1,  8),   Op::JAL, OpKind::BRANCH, 1, 0, 0,   8, 1,0,0, 1},
        {enc::J(0, -12),  Op::JAL, OpKind::BRANCH, 0, 0, 0, -12, 1,0,0, 0},  // `j` pseudo
        {enc::J(5,  1048574), Op::JAL, OpKind::BRANCH, 5, 0, 0,  1048574, 1,0,0, 1}, // +max
        {enc::J(5, -1048576), Op::JAL, OpKind::BRANCH, 5, 0, 0, -1048576, 1,0,0, 1}, // -max

        // ---- I-type ALU (immediate) --------------------------------------
        {enc::I(0x13, 1, 0x0, 2,     5), Op::ADD,  OpKind::ALU, 1, 2, 0,     5, 0,0,0, 1},
        {enc::I(0x13, 1, 0x0, 2,    -1), Op::ADD,  OpKind::ALU, 1, 2, 0,    -1, 0,0,0, 1},
        {enc::I(0x13, 1, 0x0, 2,  2047), Op::ADD,  OpKind::ALU, 1, 2, 0,  2047, 0,0,0, 1},
        {enc::I(0x13, 1, 0x0, 2, -2048), Op::ADD,  OpKind::ALU, 1, 2, 0, -2048, 0,0,0, 1},
        {enc::I(0x13, 3, 0x2, 4,    -1), Op::SLT,  OpKind::ALU, 3, 4, 0,    -1, 0,0,0, 1},
        {enc::I(0x13, 3, 0x3, 4,   100), Op::SLTU, OpKind::ALU, 3, 4, 0,   100, 0,0,0, 1},
        {enc::I(0x13, 3, 0x4, 4,  0xFF), Op::XOR,  OpKind::ALU, 3, 4, 0,  0xFF, 0,0,0, 1},
        {enc::I(0x13, 3, 0x6, 4,  0x01), Op::OR,   OpKind::ALU, 3, 4, 0,  0x01, 0,0,0, 1},
        {enc::I(0x13, 3, 0x7, 4,  0xFF), Op::AND,  OpKind::ALU, 3, 4, 0,  0xFF, 0,0,0, 1},
        // shifts: shamt encoded in rs2 field, f7 selects arithmetic vs. logical
        {enc::R(0x13, 5, 0x1, 6, 5, 0x00), Op::SLL, OpKind::ALU, 5, 6, 0,  5, 0,0,0, 1}, // SLLI
        {enc::R(0x13, 5, 0x5, 6, 7, 0x00), Op::SRL, OpKind::ALU, 5, 6, 0,  7, 0,0,0, 1}, // SRLI
        {enc::R(0x13, 5, 0x5, 6, 7, 0x20), Op::SRA, OpKind::ALU, 5, 6, 0,  7, 0,0,0, 1}, // SRAI

        // ---- R-type ALU --------------------------------------------------
        {enc::R(0x33, 3, 0x0, 1, 2, 0x00), Op::ADD,  OpKind::ALU, 3, 1, 2, 0, 0,0,0, 1},
        {enc::R(0x33, 3, 0x0, 1, 2, 0x20), Op::SUB,  OpKind::ALU, 3, 1, 2, 0, 0,0,0, 1},
        {enc::R(0x33, 3, 0x1, 1, 2, 0x00), Op::SLL,  OpKind::ALU, 3, 1, 2, 0, 0,0,0, 1},
        {enc::R(0x33, 3, 0x2, 1, 2, 0x00), Op::SLT,  OpKind::ALU, 3, 1, 2, 0, 0,0,0, 1},
        {enc::R(0x33, 3, 0x3, 1, 2, 0x00), Op::SLTU, OpKind::ALU, 3, 1, 2, 0, 0,0,0, 1},
        {enc::R(0x33, 3, 0x4, 1, 2, 0x00), Op::XOR,  OpKind::ALU, 3, 1, 2, 0, 0,0,0, 1},
        {enc::R(0x33, 3, 0x5, 1, 2, 0x00), Op::SRL,  OpKind::ALU, 3, 1, 2, 0, 0,0,0, 1},
        {enc::R(0x33, 3, 0x5, 1, 2, 0x20), Op::SRA,  OpKind::ALU, 3, 1, 2, 0, 0,0,0, 1},
        {enc::R(0x33, 3, 0x6, 1, 2, 0x00), Op::OR,   OpKind::ALU, 3, 1, 2, 0, 0,0,0, 1},
        {enc::R(0x33, 3, 0x7, 1, 2, 0x00), Op::AND,  OpKind::ALU, 3, 1, 2, 0, 0,0,0, 1},

        // ---- M extension (both halves) -----------------------------------
        {enc::R(0x33, 5, 0x0, 6, 7, 0x01), Op::MUL,    OpKind::MUL, 5, 6, 7, 0, 0,0,0, 1},
        {enc::R(0x33, 5, 0x1, 6, 7, 0x01), Op::MULH,   OpKind::MUL, 5, 6, 7, 0, 0,0,0, 1},
        {enc::R(0x33, 5, 0x2, 6, 7, 0x01), Op::MULHSU, OpKind::MUL, 5, 6, 7, 0, 0,0,0, 1},
        {enc::R(0x33, 5, 0x3, 6, 7, 0x01), Op::MULHU,  OpKind::MUL, 5, 6, 7, 0, 0,0,0, 1},
        {enc::R(0x33, 5, 0x4, 6, 7, 0x01), Op::DIV,    OpKind::DIV, 5, 6, 7, 0, 0,0,0, 1},
        {enc::R(0x33, 5, 0x5, 6, 7, 0x01), Op::DIVU,   OpKind::DIV, 5, 6, 7, 0, 0,0,0, 1},
        {enc::R(0x33, 5, 0x6, 6, 7, 0x01), Op::REM,    OpKind::DIV, 5, 6, 7, 0, 0,0,0, 1},
        {enc::R(0x33, 5, 0x7, 6, 7, 0x01), Op::REMU,   OpKind::DIV, 5, 6, 7, 0, 0,0,0, 1},

        // ---- Branches ----------------------------------------------------
        {enc::B(0x0, 1, 2,   8), Op::BEQ,  OpKind::BRANCH, 0, 1, 2,    8, 1,0,0, 0},
        {enc::B(0x1, 1, 2,  -8), Op::BNE,  OpKind::BRANCH, 0, 1, 2,   -8, 1,0,0, 0},
        {enc::B(0x4, 1, 2,  16), Op::BLT,  OpKind::BRANCH, 0, 1, 2,   16, 1,0,0, 0},
        {enc::B(0x5, 1, 2, -16), Op::BGE,  OpKind::BRANCH, 0, 1, 2,  -16, 1,0,0, 0},
        {enc::B(0x6, 1, 2,  32), Op::BLTU, OpKind::BRANCH, 0, 1, 2,   32, 1,0,0, 0},
        {enc::B(0x7, 1, 2, -32), Op::BGEU, OpKind::BRANCH, 0, 1, 2,  -32, 1,0,0, 0},

        // ---- Loads -------------------------------------------------------
        {enc::I(0x03, 1, 0x0, 2,  0), Op::LB,  OpKind::LOAD, 1, 2, 0,  0, 0,1,0, 1},
        {enc::I(0x03, 1, 0x1, 2,  4), Op::LH,  OpKind::LOAD, 1, 2, 0,  4, 0,1,0, 1},
        {enc::I(0x03, 1, 0x2, 2, -4), Op::LW,  OpKind::LOAD, 1, 2, 0, -4, 0,1,0, 1},
        {enc::I(0x03, 1, 0x4, 2,  8), Op::LBU, OpKind::LOAD, 1, 2, 0,  8, 0,1,0, 1},
        {enc::I(0x03, 1, 0x5, 2, 12), Op::LHU, OpKind::LOAD, 1, 2, 0, 12, 0,1,0, 1},

        // ---- Stores ------------------------------------------------------
        {enc::S(0x0, 2, 1,   0), Op::SB, OpKind::STORE, 0, 2, 1,   0, 0,0,1, 0},
        {enc::S(0x1, 2, 1,   4), Op::SH, OpKind::STORE, 0, 2, 1,   4, 0,0,1, 0},
        {enc::S(0x2, 2, 1,  -4), Op::SW, OpKind::STORE, 0, 2, 1,  -4, 0,0,1, 0},

        // ---- JALR / MISC-MEM / SYSTEM -----------------------------------
        {enc::I(0x67, 1, 0x0, 2, 4), Op::JALR, OpKind::BRANCH, 1, 2, 0, 4, 1,0,0, 1},
        {enc::FENCE(),   Op::FENCE,   OpKind::NOP,  0, 0, 0, 0, 0,0,0, 0},
        {enc::FENCE_I(), Op::FENCE_I, OpKind::NOP,  0, 0, 0, 0, 0,0,0, 0},
        {enc::ECALL(),   Op::ECALL,   OpKind::TRAP, 0, 0, 0, 0, 0,0,0, 0},
        {enc::EBREAK(),  Op::EBREAK,  OpKind::TRAP, 0, 0, 0, 0, 0,0,0, 0},
    };

    for (const C& c : cases) {
        const Decoded d = decode(c.raw);
        REQUIRE(d.op        == c.op);
        REQUIRE(d.kind      == c.kind);
        REQUIRE(d.rd        == c.rd);
        REQUIRE(d.rs1       == c.rs1);
        REQUIRE(d.rs2       == c.rs2);
        REQUIRE(d.imm       == c.imm);
        REQUIRE(d.is_branch == bool(c.br));
        REQUIRE(d.is_load   == bool(c.ld));
        REQUIRE(d.is_store  == bool(c.st));
        REQUIRE(d.writes_rd == bool(c.wrd));
    }

    // ---- Invalid encodings must trap, not crash --------------------------
    const uint32_t traps[] = {
        0x00000000,                       // opcode 0
        0xFFFFFFFFu,                      // opcode 0x7F, all ones
        enc::I(0x67, 1, 0x1, 2, 0),       // JALR with non-zero funct3
        enc::R(0x33, 1, 0x0, 2, 3, 0x40), // ADD with reserved funct7
        enc::R(0x13, 1, 0x1, 2, 5, 0x20), // SLLI with non-zero funct7 bit
        enc::B(0x2, 1, 2, 0),             // BRANCH funct3=2 (reserved)
        enc::I(0x03, 1, 0x3, 2, 0),       // LOAD funct3=3 (reserved)
        enc::S(0x3, 2, 1, 0),             // STORE funct3=3 (reserved)
        0x00000073u | (0x2u << 20),       // SYSTEM with imm != {0, 1}
    };
    for (uint32_t raw : traps) {
        const Decoded d = decode(raw);
        REQUIRE(d.op == Op::INVALID);
        REQUIRE(d.kind == OpKind::TRAP);
    }

    // ---- Sign-extension boundary spot-checks ----------------------------
    // I-type: +2047 / -2048
    REQUIRE(decode(enc::I(0x13, 1, 0, 0,  2047)).imm ==  2047);
    REQUIRE(decode(enc::I(0x13, 1, 0, 0, -2048)).imm == -2048);
    // S-type: +2047 / -2048
    REQUIRE(decode(enc::S(0x2, 1, 2,  2047)).imm ==  2047);
    REQUIRE(decode(enc::S(0x2, 1, 2, -2048)).imm == -2048);
    // B-type: +4094 / -4096 (13-bit, low bit always 0)
    REQUIRE(decode(enc::B(0x0, 1, 2,  4094)).imm ==  4094);
    REQUIRE(decode(enc::B(0x0, 1, 2, -4096)).imm == -4096);
}

// --------------------------------------------------------- @section("alu") ---
SECTION("alu") {
    // ---- Integer ALU -------------------------------------------------------
    REQUIRE(alu::add(3, 5) == 8u);
    REQUIRE(alu::add(0xFFFFFFFFu, 1) == 0u);                        // wrap
    REQUIRE(alu::sub(3, 5) == static_cast<uint32_t>(-2));

    REQUIRE(alu::sll(1u, 4)  == 16u);
    REQUIRE(alu::sll(1u, 36) == 16u);                               // shamt & 0x1F
    REQUIRE(alu::srl(0x80000000u, 4)  == 0x08000000u);              // logical
    REQUIRE(alu::srl(0x80000000u, 0)  == 0x80000000u);              // 0-shift edge
    REQUIRE(alu::sra(0x80000000u, 4)  == 0xF8000000u);              // arithmetic
    REQUIRE(alu::sra(0x80000000u, 0)  == 0x80000000u);              // 0-shift edge
    REQUIRE(alu::sra(0xFFFFFFFFu, 31) == 0xFFFFFFFFu);              // all-ones preserved
    REQUIRE(alu::sra(0x40000000u, 4)  == 0x04000000u);              // positive → logical

    REQUIRE(alu::and_(0xF0F0u, 0x0FF0u) == 0x00F0u);
    REQUIRE(alu::or_ (0xF0F0u, 0x0FF0u) == 0xFFF0u);
    REQUIRE(alu::xor_(0xF0F0u, 0x0FF0u) == 0xFF00u);

    REQUIRE(alu::slt (static_cast<uint32_t>(-1), 1) == 1u);         // -1 < 1 signed
    REQUIRE(alu::sltu(static_cast<uint32_t>(-1), 1) == 0u);         // -1 > 1 unsigned
    REQUIRE(alu::slt (1u, 1u) == 0u);
    REQUIRE(alu::sltu(1u, 1u) == 0u);

    // ---- M extension: MUL family ------------------------------------------
    REQUIRE(alu::mul  (3u, 5u) == 15u);
    REQUIRE(alu::mul  (0xFFFFFFFFu, 2u) == 0xFFFFFFFEu);            // low 32 wraps
    // (-1) × (-1) = 1 as 64-bit; upper 32 = 0.
    REQUIRE(alu::mulh (static_cast<uint32_t>(-1), static_cast<uint32_t>(-1)) == 0u);
    // 0xFFFFFFFF × 0xFFFFFFFF = 0xFFFFFFFE_00000001; upper 32 = 0xFFFFFFFE.
    REQUIRE(alu::mulhu(static_cast<uint32_t>(-1), static_cast<uint32_t>(-1)) == 0xFFFFFFFEu);
    // (-1) signed × 1 unsigned = -1 as int64; two's-complement upper 32 = -1.
    REQUIRE(alu::mulhsu(static_cast<uint32_t>(-1), 1u) == 0xFFFFFFFFu);

    // ---- M extension: DIV / REM edge cases --------------------------------
    constexpr uint32_t INT_MIN_U = 0x80000000u;
    // The two defined-by-the-ISA cases:
    REQUIRE(alu::div(INT_MIN_U, static_cast<uint32_t>(-1)) == INT_MIN_U);  // no trap
    REQUIRE(alu::div(10u, 0u) == 0xFFFFFFFFu);                              // /0 → -1
    // The remainder counterparts:
    REQUIRE(alu::rem(INT_MIN_U, static_cast<uint32_t>(-1)) == 0u);
    REQUIRE(alu::rem(10u, 0u) == 10u);                                      // %0 → dividend
    // Unsigned variants:
    REQUIRE(alu::divu(10u, 0u) == 0xFFFFFFFFu);
    REQUIRE(alu::remu(10u, 0u) == 10u);
    // Normal signed and unsigned division; truncate toward zero, not floor.
    REQUIRE(alu::div (10u, 3u) == 3u);
    REQUIRE(alu::div (static_cast<uint32_t>(-10), 3u) == static_cast<uint32_t>(-3));
    REQUIRE(alu::div (10u, static_cast<uint32_t>(-3)) == static_cast<uint32_t>(-3));
    REQUIRE(alu::divu(10u, 3u) == 3u);
    REQUIRE(alu::rem (10u, 3u) == 1u);
    REQUIRE(alu::rem (static_cast<uint32_t>(-10), 3u) == static_cast<uint32_t>(-1));
    REQUIRE(alu::remu(10u, 3u) == 1u);

    // ---- Branch conditions ------------------------------------------------
    REQUIRE( alu::beq(5, 5));   REQUIRE(!alu::beq(5, 6));
    REQUIRE( alu::bne(5, 6));   REQUIRE(!alu::bne(5, 5));
    // Signed / unsigned split around -1 vs. 1
    REQUIRE( alu::blt (static_cast<uint32_t>(-1), 1));
    REQUIRE( alu::bge (1, static_cast<uint32_t>(-1)));
    REQUIRE(!alu::blt (1, static_cast<uint32_t>(-1)));
    REQUIRE( alu::bltu(1, static_cast<uint32_t>(-1)));
    REQUIRE( alu::bgeu(static_cast<uint32_t>(-1), 1));
    REQUIRE(!alu::bltu(static_cast<uint32_t>(-1), 1));

    // branch_taken dispatches to the comparison its Op names, and is false
    // for anything that is not a conditional branch.
    REQUIRE( alu::branch_taken(Op::BEQ,  7, 7));
    REQUIRE(!alu::branch_taken(Op::BEQ,  7, 8));
    REQUIRE( alu::branch_taken(Op::BNE,  7, 8));
    REQUIRE( alu::branch_taken(Op::BLT,  static_cast<uint32_t>(-1), 1));
    REQUIRE(!alu::branch_taken(Op::BLTU, static_cast<uint32_t>(-1), 1));
    REQUIRE( alu::branch_taken(Op::BGE,  1, static_cast<uint32_t>(-1)));
    REQUIRE( alu::branch_taken(Op::BGEU, static_cast<uint32_t>(-1), 1));
    REQUIRE(!alu::branch_taken(Op::JAL,  1, 1));
    REQUIRE(!alu::branch_taken(Op::ADD,  1, 1));

    // ---- Jump and return target arithmetic ---------------------------------
    REQUIRE(alu::branch_target(0x1000, 16)  == 0x1010u);
    REQUIRE(alu::branch_target(0x1000, -16) == 0x0FF0u);
    REQUIRE(alu::jalr_target(0x2000, 5) == 0x2004u);      // low bit dropped
    REQUIRE(alu::jalr_target(0x2001, 4) == 0x2004u);
    REQUIRE(alu::link_address(0x1000) == 0x1004u);

    // Compile-time checks, so a regression in the pure primitives fails to
    // build rather than fails to run.
    static_assert(alu::add(3, 5) == 8u);
    static_assert(alu::div(0x80000000u, 0xFFFFFFFFu) == 0x80000000u);
    static_assert(alu::rem(0x80000000u, 0xFFFFFFFFu) == 0u);
    static_assert(alu::sra(0x80000000u, 4)           == 0xF8000000u);
    static_assert(alu::branch_taken(Op::BLT, 0xFFFFFFFFu, 1u));
    static_assert(alu::jalr_target(0x2001u, 4) == 0x2004u);
}

// --------------------------------------------------------- @section("asm") ---
SECTION("asm") {
    using namespace asmc;

    // ---- Every instruction encodes byte-exact vs. the enc:: reference ---
    // enc:: was itself pinned against the decoder above, so this is a real
    // differential check rather than a restatement.
    {
        Assembler a;
        a.addi (a0, a1, 5);
        a.sub  (t0, t1, t2);
        a.sll  (s0, s1, s2);
        a.slli (s0, s1, 5);
        a.slti (a0, a1, -1);
        a.sltiu(a0, a1, 100);
        a.xori (a0, a1, 0xFF);
        a.ori  (a0, a1, 0x1);
        a.andi (a0, a1, 0xFF);
        a.srli (s0, s1, 7);
        a.srai (s0, s1, 7);
        a.slt  (s0, s1, s2);
        a.sltu (s0, s1, s2);
        a.xor_ (s0, s1, s2);
        a.srl  (s0, s1, s2);
        a.sra  (s0, s1, s2);
        a.or_  (s0, s1, s2);
        a.and_ (s0, s1, s2);
        a.lui  (a0, 0x12345);
        a.auipc(a0, 0x12345);
        a.lb   (a0, sp, -4);
        a.lh   (a0, sp,  0);
        a.lw   (a0, sp,  4);
        a.lbu  (a0, sp,  8);
        a.lhu  (a0, sp, 12);
        a.sb   (a0, sp, -4);
        a.sh   (a0, sp,  0);
        a.sw   (a0, sp,  4);
        a.jalr (ra, a0, 4);
        a.mul   (a0, a1, a2);
        a.mulh  (a0, a1, a2);
        a.mulhsu(a0, a1, a2);
        a.mulhu (a0, a1, a2);
        a.div_  (a0, a1, a2);
        a.divu  (a0, a1, a2);
        a.rem   (a0, a1, a2);
        a.remu  (a0, a1, a2);
        a.fence();
        a.fence_i();
        a.ecall();
        a.ebreak();
        const auto w = a.assemble();

        const uint32_t exp[] = {
            enc::I(0x13, a0, 0x0, a1,  5),
            enc::R(0x33, t0, 0x0, t1, t2, 0x20),          // sub
            enc::R(0x33, s0, 0x1, s1, s2, 0x00),          // sll
            enc::R(0x13, s0, 0x1, s1, 5,  0x00),          // slli
            enc::I(0x13, a0, 0x2, a1, -1),
            enc::I(0x13, a0, 0x3, a1, 100),
            enc::I(0x13, a0, 0x4, a1, 0xFF),
            enc::I(0x13, a0, 0x6, a1, 0x1),
            enc::I(0x13, a0, 0x7, a1, 0xFF),
            enc::R(0x13, s0, 0x5, s1, 7, 0x00),           // srli
            enc::R(0x13, s0, 0x5, s1, 7, 0x20),           // srai
            enc::R(0x33, s0, 0x2, s1, s2, 0x00),          // slt
            enc::R(0x33, s0, 0x3, s1, s2, 0x00),          // sltu
            enc::R(0x33, s0, 0x4, s1, s2, 0x00),          // xor
            enc::R(0x33, s0, 0x5, s1, s2, 0x00),          // srl
            enc::R(0x33, s0, 0x5, s1, s2, 0x20),          // sra
            enc::R(0x33, s0, 0x6, s1, s2, 0x00),          // or
            enc::R(0x33, s0, 0x7, s1, s2, 0x00),          // and
            enc::U(0x37, a0, 0x12345),                    // lui
            enc::U(0x17, a0, 0x12345),                    // auipc
            enc::I(0x03, a0, 0x0, sp, -4),                // lb
            enc::I(0x03, a0, 0x1, sp,  0),
            enc::I(0x03, a0, 0x2, sp,  4),
            enc::I(0x03, a0, 0x4, sp,  8),
            enc::I(0x03, a0, 0x5, sp, 12),
            enc::S(0x0, sp, a0, -4),
            enc::S(0x1, sp, a0,  0),
            enc::S(0x2, sp, a0,  4),
            enc::I(0x67, ra, 0x0, a0, 4),                 // jalr
            enc::R(0x33, a0, 0x0, a1, a2, 0x01),          // mul
            enc::R(0x33, a0, 0x1, a1, a2, 0x01),
            enc::R(0x33, a0, 0x2, a1, a2, 0x01),
            enc::R(0x33, a0, 0x3, a1, a2, 0x01),
            enc::R(0x33, a0, 0x4, a1, a2, 0x01),
            enc::R(0x33, a0, 0x5, a1, a2, 0x01),
            enc::R(0x33, a0, 0x6, a1, a2, 0x01),
            enc::R(0x33, a0, 0x7, a1, a2, 0x01),
            0x0000000Fu, 0x0000100Fu, 0x00000073u, 0x00100073u,
        };
        REQUIRE(w.size() == sizeof(exp) / sizeof(exp[0]));
        for (std::size_t i = 0; i < w.size(); ++i) REQUIRE(w[i] == exp[i]);
    }

    // ---- Labels: forward + backward, both branches and jal --------------
    {
        Assembler a;
        a.label("start");             // pc = 0
        a.beq(a0, a1, "end");         // pc = 0, target = 12  → off = +12
        a.j("start");                 // pc = 4, target =  0  → off =  -4
        a.addi(a0, a0, 1);            // pc = 8
        a.label("end");               // pc = 12
        a.ret_();
        const auto w = a.assemble();
        REQUIRE(w.size() == 4);
        REQUIRE(w[0] == enc::B(0x0, a0, a1,  12));
        REQUIRE(w[1] == enc::J(0, -4));
        REQUIRE(w[2] == enc::I(0x13, a0, 0x0, a0, 1));
        REQUIRE(w[3] == enc::I(0x67, 0, 0x0, 1, 0));   // ret = jalr x0, x1, 0
    }

    // ---- Pseudoinstructions expand as documented ------------------------
    {
        Assembler a;
        a.nop();                                     // → addi x0, x0, 0
        a.mv(a0, a1);                                // → addi a0, a1, 0
        a.li(a0, 42);                                // small → single addi
        a.li(t0, -1);                                // fits in 12-bit imm
        a.jr(ra);                                    // → jalr x0, ra, 0
        const auto w = a.assemble();
        REQUIRE(w.size() == 5);
        REQUIRE(w[0] == 0x00000013u);                                // canonical NOP
        REQUIRE(w[1] == enc::I(0x13, a0, 0x0, a1, 0));
        REQUIRE(w[2] == enc::I(0x13, a0, 0x0, 0,   42));
        REQUIRE(w[3] == enc::I(0x13, t0, 0x0, 0,   -1));
        REQUIRE(w[4] == enc::I(0x67, 0,  0x0, ra,   0));
    }

    // ---- li with large immediate uses the lui + addi pair with the
    // ---- lower-12 sign-compensation trick -------------------------------
    {
        // 0x12345678: low 12 = 0x678 (positive) → hi20 rounds to 0x12345.
        Assembler a;
        a.li(a0, static_cast<int32_t>(0x12345678));
        const auto w = a.assemble();
        REQUIRE(w.size() == 2);
        REQUIRE(w[0] == enc::U(0x37, a0, 0x12345));
        REQUIRE(w[1] == enc::I(0x13, a0, 0x0, a0, 0x678));

        // 0x12345800: low 12 = 0x800 → high bit set → hi20 rounds up to
        // 0x12346, lo12 = -0x800. Then 0x12346000 + sext(-0x800) = 0x12345800.
        Assembler b;
        b.li(a0, static_cast<int32_t>(0x12345800));
        const auto wb = b.assemble();
        REQUIRE(wb.size() == 2);
        REQUIRE(wb[0] == enc::U(0x37, a0, 0x12346));
        REQUIRE(wb[1] == enc::I(0x13, a0, 0x0, a0, -0x800));
    }

    // ---- call target: auipc + jalr, sum lands on the label --------------
    {
        Assembler a;
        a.call("target");             // pc 0..7, ra <- pc + 8
        a.nop();                      // pc 8
        a.label("target");            // pc 12
        a.ecall();
        const auto w = a.assemble();
        REQUIRE(w.size() == 4);
        // Split (target - call_pc) = 12 through the (imm + 0x800) >> 12 trick.
        // For offset = 12, hi20 = 0, lo12 = 12.
        REQUIRE(w[0] == enc::U(0x17, ra, 0));
        REQUIRE(w[1] == enc::I(0x67, ra, 0x0, ra, 12));
    }

    // ---- The assembled program actually runs on the interpreter ---------
    // Sanity: sum 1..10 = 55, exit(55) via ECALL a7=93.
    {
        Assembler p;
        p.li(a0, 0);                          // sum = 0
        p.li(a1, 1);                          // i = 1
        p.li(a2, 11);                         // limit
        p.label("loop");
        p.beq(a1, a2, "done");
        p.add(a0, a0, a1);                    // sum += i
        p.addi(a1, a1, 1);
        p.j("loop");
        p.label("done");
        p.li(a7, 93);
        p.ecall();

        const ref::Result r = reftest::run(p.assemble(), 1000);
        REQUIRE(r.halted);
        REQUIRE(!r.trapped);
        REQUIRE(r.exit_code == 55);
        REQUIRE(r.regs[10]  == 55);
    }

    // ---- Error paths ----------------------------------------------------
    // Undefined label throws at assemble time, not silently.
    {
        Assembler a;
        a.j("nope");
        bool threw = false;
        try { (void)a.assemble(); } catch (const std::runtime_error&) { threw = true; }
        REQUIRE(threw);
    }
    // Branch offset out of range (>4094 bytes) throws.
    {
        Assembler a;
        a.beq(a0, a1, "far");
        for (int i = 0; i < 2050; ++i) a.nop();           // ~8 KiB gap
        a.label("far");
        bool threw = false;
        try { (void)a.assemble(); } catch (const std::runtime_error&) { threw = true; }
        REQUIRE(threw);
    }
}

// ------------------------------------------------------ @section("disasm") ---
SECTION("disasm") {
    // The text a trace carries is the only description of an instruction the
    // viewer ever gets, so it has to name the operands the encoding really has
    // — including the two places where the decoder deliberately forgets:
    // ADD/ADDI share an Op, and a store keeps its data register in rs2.
    asmc::Assembler a;
    a.addi(1, 1, -1);
    a.add(2, 2, 1);
    a.sub(3, 2, 1);
    a.slli(4, 1, 3);
    a.srai(5, 1, 2);
    a.lui(2, 0x8);
    a.auipc(6, 0x1);
    a.mul(5, 4, 4);
    a.divu(7, 5, 4);
    a.lw(8, 2, 8);
    a.lbu(9, 2, -3);
    a.sw(1, 2, 12);
    a.sb(9, 2, 0);
    a.jalr(1, 5, 16);
    a.andi(10, 10, 255);
    a.ecall();
    a.ebreak();
    const std::vector<uint32_t> words = a.assemble();

    const char* want[] = {
        "addi x1,x1,-1", "add x2,x2,x1", "sub x3,x2,x1", "slli x4,x1,3",
        "srai x5,x1,2", "lui x2,0x8", "auipc x6,0x1", "mul x5,x4,x4",
        "divu x7,x5,x4", "lw x8,8(x2)", "lbu x9,-3(x2)", "sw x1,12(x2)",
        "sb x9,0(x2)", "jalr x1,16(x5)", "andi x10,x10,255", "ecall", "ebreak",
    };
    REQUIRE(words.size() == sizeof(want) / sizeof(want[0]));
    for (std::size_t i = 0; i < words.size(); ++i) {
        const std::string got = disasm(words[i], wl::TEXT + static_cast<uint32_t>(i * 4));
        REQUIRE_MSG(got == want[i],
                    "    got \"" + got + "\", want \"" + std::string(want[i]) + "\"");
    }

    // ---- Control flow prints where it goes, not how far ------------------
    // A viewer compares the target against a PC; a displacement would make it
    // do the arithmetic the trace is supposed to have already done.
    {
        asmc::Assembler b;
        b.label("top");
        b.addi(1, 1, -1);
        b.bne(1, 0, "top");
        b.jal(1, "top");
        const std::vector<uint32_t> ws = b.assemble();
        REQUIRE(disasm(ws[1], 0x1004) == "bne x1,x0,0x1000");
        REQUIRE(disasm(ws[2], 0x1008) == "jal x1,0x1000");
    }

    // ---- An undecodable word says so rather than inventing an opcode -----
    REQUIRE(disasm(0xFFFFFFFFu, 0x1000).rfind("<invalid", 0) == 0);
    REQUIRE(disasm(0x00000000u, 0x1000).rfind("<invalid", 0) == 0);

    // ---- fence spellings and system ops -----------------------------------
    REQUIRE(disasm(enc::FENCE(), 0)   == "fence");
    REQUIRE(disasm(enc::FENCE_I(), 0) == "fence.i");

    // ---- reg_name (used by the differential reports, not by disasm) -------
    REQUIRE(std::string(reg_name(0))  == "zero");
    REQUIRE(std::string(reg_name(10)) == "a0");
    REQUIRE(std::string(reg_name(31)) == "t6");
    REQUIRE(std::string(reg_name(32)) == "x?");
}
