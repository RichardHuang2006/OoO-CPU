// Test driver. Each PLAN.md step appends one SECTION() block; the harness
// grows here rather than fanning out across files so the whole suite is one
// translation unit and `make test` is one binary.
//
// The diff_run() helper that compares the OoO simulator against ref.h is
// introduced in Step 2.3.

#include <cstdio>
#include <functional>
#include <initializer_list>
#include <optional>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "types.h"
#include "config.h"
#include "decoder.h"
#include "alu.h"

// ---------------------------------------------------------- harness plumbing ---
namespace test {

using Fn = std::function<void()>;

inline std::vector<std::pair<std::string, Fn>>& registry() {
    static std::vector<std::pair<std::string, Fn>> r;
    return r;
}

struct Register {
    Register(const char* name, Fn fn) { registry().emplace_back(name, std::move(fn)); }
};

inline int assertion_failures = 0;
inline void report_fail(const char* expr, const char* file, int line) {
    std::fprintf(stderr, "  FAIL: %s   at %s:%d\n", expr, file, line);
    ++assertion_failures;
}

}  // namespace test

#define MC_CAT_INNER(a, b) a##b
#define MC_CAT(a, b) MC_CAT_INNER(a, b)
#define SECTION(name)                                                          \
    static void MC_CAT(section_fn_, __LINE__)();                               \
    static const ::test::Register MC_CAT(section_reg_, __LINE__)(              \
        name, &MC_CAT(section_fn_, __LINE__));                                 \
    static void MC_CAT(section_fn_, __LINE__)()

#define REQUIRE(expr)                                                          \
    do {                                                                       \
        if (!(expr)) ::test::report_fail(#expr, __FILE__, __LINE__);           \
    } while (0)

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

    // round-trip through the std::optional<PhysReg> idiom used at Step 4.2
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

    // OpKind values are distinct — enough of a smoke test at this stage.
    static_assert(static_cast<int>(OpKind::ALU)  != static_cast<int>(OpKind::TRAP));
    static_assert(static_cast<int>(OpKind::LOAD) != static_cast<int>(OpKind::STORE));
    static_assert(static_cast<int>(OpKind::NOP)  != static_cast<int>(OpKind::ALU));
}

// ------------------------------------------------------ @section("config") ---
SECTION("config") {
    Config c;

    // defaults match DESIGN.md §9.1
    REQUIRE(c.width           == 2);
    REQUIRE(c.rob_size        == 32);
    REQUIRE(c.prf_size        == 64);
    REQUIRE(c.iq_size         == 16);
    REQUIRE(c.lq_size         == 8);
    REQUIRE(c.sq_size         == 8);
    REQUIRE(c.num_cdb         == 2);
    REQUIRE(c.num_alu         == 2);
    REQUIRE(c.num_branch      == 1);
    REQUIRE(c.num_mul         == 1);
    REQUIRE(c.num_mem         == 1);
    REQUIRE(c.alu_latency     == 1);
    REQUIRE(c.mem_latency     == 2);
    REQUIRE(c.mul_latency     == 3);
    REQUIRE(c.div_latency     == 20);
    REQUIRE(c.ghr_bits        == 12);
    REQUIRE(c.pht_size        == 4096);
    REQUIRE(c.btb_sets * c.btb_ways == 512);
    REQUIRE(c.ras_size        == 16);
    REQUIRE(c.num_checkpoints == 16);

    // §3.2 starvation-free rule at the default sizing
    REQUIRE(!c.prf_can_starve());

    // exact boundary: prf_size == rob_size + 32 is safe; one less starves
    Config edge = c;
    edge.prf_size = edge.rob_size + 32;
    REQUIRE(!edge.prf_can_starve());
    edge.prf_size = edge.rob_size + 32 - 1;
    REQUIRE(edge.prf_can_starve());

    // the specific stress config used later in the six-config sweep
    Config small_prf = c;
    small_prf.prf_size = 32;
    REQUIRE(small_prf.prf_can_starve());
}

// ------------------------------------------------------ @section("decode") ---
namespace enc {
    // File-local encoders — enough to build the decode-test cases without
    // duplicating raw hex constants. Step 2.1's asm.h supersedes these.
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

    // ---- M extension: DIV / REM edge cases (RISC-V §7.2) ------------------
    constexpr uint32_t INT_MIN_U = 0x80000000u;
    // The two cases the plan pins by name:
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

    // A few compile-time sanity checks — a regression in the pure primitives
    // fails to build rather than fails to run.
    static_assert(alu::add(3, 5) == 8u);
    static_assert(alu::div(0x80000000u, 0xFFFFFFFFu) == 0x80000000u);
    static_assert(alu::rem(0x80000000u, 0xFFFFFFFFu) == 0u);
    static_assert(alu::sra(0x80000000u, 4)           == 0xF8000000u);
}


// -------------------------------------------------------------------- main ---
int main() {
    int passes = 0, fails = 0;
    for (auto& [name, fn] : test::registry()) {
        int before = test::assertion_failures;
        std::printf("=== %s ===\n", name.c_str());
        fn();
        if (test::assertion_failures > before) ++fails;
        else                                   ++passes;
    }
    std::printf("\n%d section%s ok, %d failing (%d assertion failure%s)\n",
                passes, passes == 1 ? "" : "s",
                fails,
                test::assertion_failures, test::assertion_failures == 1 ? "" : "s");
    return test::assertion_failures ? 1 : 0;
}
