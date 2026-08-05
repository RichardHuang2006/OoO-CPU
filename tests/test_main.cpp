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
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "types.h"
#include "config.h"
#include "decoder.h"
#include "alu.h"
#include "memory.h"
#include "loader.h"
#include "asm.h"

// Include the driver TU so parse_args / print_help / run_program are
// unit-testable without shelling out. Its own int main() is guarded off.
#define MINI_CPU_NO_ENTRY
#include "main.cpp"

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

// ------------------------------------------------------ @section("memory") ---
SECTION("memory") {
    // ---- Fresh memory: no pages, loads return zero and don't allocate ----
    Memory m;
    REQUIRE(m.num_pages() == 0);
    REQUIRE(m.load_u8 (0x1000) == 0);
    REQUIRE(m.load_u16(0x1000) == 0);
    REQUIRE(m.load_u32(0x1000) == 0);
    REQUIRE(m.num_pages() == 0);

    // A single store lazily allocates one page.
    m.store_u8(0x1000, 0xAA);
    REQUIRE(m.num_pages() == 1);
    REQUIRE( m.has_page(0x1000));
    REQUIRE( m.has_page(0x1FFF));      // last byte of same page
    REQUIRE(!m.has_page(0x2000));      // next page not yet touched
    REQUIRE(m.load_u8(0x1000) == 0xAA);

    m.store_u8(0x2000, 0xBB);
    REQUIRE(m.num_pages() == 2);

    // ---- The plan's marquee test ------------------------------------------
    // Store 0xDEADBEEF at 0x1002 (misaligned within a page), then read the
    // same bytes back as one lw, two lh's, and four lb's — little-endian.
    Memory misa;
    misa.store_u32(0x1002, 0xDEADBEEFu);
    REQUIRE(misa.load_u32(0x1002) == 0xDEADBEEFu);
    REQUIRE(misa.load_u16(0x1002) == 0xBEEFu);
    REQUIRE(misa.load_u16(0x1004) == 0xDEADu);
    REQUIRE(misa.load_u8 (0x1002) == 0xEFu);
    REQUIRE(misa.load_u8 (0x1003) == 0xBEu);
    REQUIRE(misa.load_u8 (0x1004) == 0xADu);
    REQUIRE(misa.load_u8 (0x1005) == 0xDEu);
    // Bytes 0x1002..0x1005 all fall in the 0x1000 page.
    REQUIRE(misa.num_pages() == 1);

    // ---- Cross-page misaligned store --------------------------------------
    // 0x0FFE..0x1001 straddles the 0x0000 and 0x1000 pages.
    Memory cross;
    cross.store_u32(0x0FFE, 0xCAFEBABEu);
    REQUIRE(cross.num_pages() == 2);
    REQUIRE(cross.load_u32(0x0FFE) == 0xCAFEBABEu);
    REQUIRE(cross.load_u16(0x0FFE) == 0xBABEu);   // low half in page 0x0000
    REQUIRE(cross.load_u16(0x1000) == 0xCAFEu);   // high half in page 0x1000
    REQUIRE(cross.load_u8(0x0FFE) == 0xBEu);
    REQUIRE(cross.load_u8(0x0FFF) == 0xBAu);
    REQUIRE(cross.load_u8(0x1000) == 0xFEu);
    REQUIRE(cross.load_u8(0x1001) == 0xCAu);

    // ---- Speculative reads must not allocate ------------------------------
    Memory readonly;
    (void)readonly.load_u8 (0xDEAD);
    (void)readonly.load_u16(0xDEAD);
    (void)readonly.load_u32(0xDEAD);
    (void)readonly.load_u32(0x0FFE);   // even the straddling case
    REQUIRE(readonly.num_pages() == 0);

    // ---- write_bytes: loader.h's bulk-copy path ---------------------------
    Memory bulk;
    const uint8_t data[] = {0x11, 0x22, 0x33, 0x44, 0x55};
    bulk.write_bytes(0x2000, data, sizeof(data));
    REQUIRE(bulk.load_u32(0x2000) == 0x44332211u);
    REQUIRE(bulk.load_u8 (0x2004) == 0x55u);

    // A zero-length write is a no-op and does not allocate.
    Memory zero;
    zero.write_bytes(0x3000, data, 0);
    REQUIRE(zero.num_pages() == 0);
}

// ------------------------------------------------------ @section("loader") ---
namespace loadertest {
    // Build a minimal ELF32 little-endian executable with one PT_LOAD segment.
    inline std::vector<uint8_t> make_elf32(uint32_t entry, uint32_t vaddr,
                                           const uint8_t* payload,
                                           uint32_t size, uint32_t p_flags) {
        constexpr uint16_t ehdr_size = 52;
        constexpr uint16_t phdr_size = 32;
        const std::size_t payload_off = ehdr_size + phdr_size;
        std::vector<uint8_t> elf(payload_off + size, 0);

        auto wr8  = [&](std::size_t o, uint8_t v)  { elf[o] = v; };
        auto wr16 = [&](std::size_t o, uint16_t v) {
            elf[o]     = static_cast<uint8_t>(v);
            elf[o + 1] = static_cast<uint8_t>(v >> 8);
        };
        auto wr32 = [&](std::size_t o, uint32_t v) {
            elf[o]     = static_cast<uint8_t>(v);
            elf[o + 1] = static_cast<uint8_t>(v >>  8);
            elf[o + 2] = static_cast<uint8_t>(v >> 16);
            elf[o + 3] = static_cast<uint8_t>(v >> 24);
        };

        // Ehdr
        wr8(0, 0x7F); wr8(1, 'E'); wr8(2, 'L'); wr8(3, 'F');
        wr8(4, 1);            // ELFCLASS32
        wr8(5, 1);            // ELFDATA2LSB
        wr8(6, 1);            // EV_CURRENT
        wr16(16, 2);          // e_type = ET_EXEC
        wr16(18, 0xF3);       // e_machine = EM_RISCV
        wr32(20, 1);          // e_version
        wr32(24, entry);
        wr32(28, ehdr_size);  // e_phoff
        wr32(32, 0);          // e_shoff
        wr32(36, 0);          // e_flags
        wr16(40, ehdr_size);
        wr16(42, phdr_size);
        wr16(44, 1);          // e_phnum
        wr16(46, 40);         // e_shentsize
        wr16(48, 0);          // e_shnum
        wr16(50, 0);          // e_shstrndx

        // Phdr
        wr32(ehdr_size +  0, 1);                                             // PT_LOAD
        wr32(ehdr_size +  4, static_cast<uint32_t>(payload_off));            // p_offset
        wr32(ehdr_size +  8, vaddr);                                         // p_vaddr
        wr32(ehdr_size + 12, vaddr);                                         // p_paddr
        wr32(ehdr_size + 16, size);                                          // p_filesz
        wr32(ehdr_size + 20, size);                                          // p_memsz
        wr32(ehdr_size + 24, p_flags);                                       // p_flags
        wr32(ehdr_size + 28, 4);                                             // p_align

        for (uint32_t i = 0; i < size; ++i) elf[payload_off + i] = payload[i];
        return elf;
    }

    inline bool threw(std::function<void()> fn) {
        try { fn(); return false; } catch (const std::exception&) { return true; }
    }
}

SECTION("loader") {
    // Common payload: three 32-bit words at base 0x1000.
    const uint32_t words[] = {0xDEADBEEFu, 0xCAFEBABEu, 0x12345678u};
    const uint32_t base = 0x1000;

    uint8_t payload[sizeof(words)];
    for (std::size_t i = 0; i < 3; ++i) {
        payload[i*4 + 0] = static_cast<uint8_t>(words[i]);
        payload[i*4 + 1] = static_cast<uint8_t>(words[i] >>  8);
        payload[i*4 + 2] = static_cast<uint8_t>(words[i] >> 16);
        payload[i*4 + 3] = static_cast<uint8_t>(words[i] >> 24);
    }

    // ---- Hex ----------------------------------------------------------------
    Memory m_hex;
    {
        std::istringstream ss(
            "DEADBEEF\n"
            "# a comment on its own line\n"
            "\n"
            "  0xCAFEBABE  // trailing comment, leading spaces\n"
            "12345678\n");
        const auto r = load_hex(m_hex, ss, base);
        REQUIRE(r.entry == base);
        REQUIRE(r.ro_ranges.empty());
    }

    // ---- Raw ----------------------------------------------------------------
    Memory m_raw;
    {
        std::string bytes(reinterpret_cast<const char*>(payload), sizeof(payload));
        std::istringstream ss(bytes);
        const auto r = load_raw(m_raw, ss, base);
        REQUIRE(r.entry == base);
        REQUIRE(r.ro_ranges.empty());
    }

    // ---- ELF32 --------------------------------------------------------------
    Memory m_elf;
    {
        const auto blob = loadertest::make_elf32(base, base, payload,
                                                 sizeof(payload),
                                                 /*PF_R | PF_X =*/ 5);
        std::string s(reinterpret_cast<const char*>(blob.data()), blob.size());
        std::istringstream ss(s);
        const auto r = load_elf(m_elf, ss);
        REQUIRE(r.entry == base);
        REQUIRE(r.ro_ranges.size() == 1);
        REQUIRE(r.ro_ranges[0].first  == base);
        REQUIRE(r.ro_ranges[0].second == base + sizeof(payload));
    }

    // ---- The plan's marquee assertion: all three lands identical bytes ----
    for (std::size_t i = 0; i < 3; ++i) {
        REQUIRE(m_hex.load_u32(base + i * 4) == words[i]);
        REQUIRE(m_raw.load_u32(base + i * 4) == words[i]);
        REQUIRE(m_elf.load_u32(base + i * 4) == words[i]);
    }

    // ---- Writable ELF segment does NOT get flagged read-only --------------
    {
        Memory m;
        const auto blob = loadertest::make_elf32(base, base, payload,
                                                 sizeof(payload),
                                                 /*PF_R | PF_W =*/ 6);
        std::string s(reinterpret_cast<const char*>(blob.data()), blob.size());
        std::istringstream ss(s);
        const auto r = load_elf(m, ss);
        REQUIRE(r.ro_ranges.empty());
    }

    // ---- ELF BSS (p_memsz > p_filesz) reads back as zeros -----------------
    {
        Memory m;
        // Build ELF with p_filesz=size, p_memsz=size+8 (fake BSS tail).
        auto blob = loadertest::make_elf32(base, base, payload,
                                           sizeof(payload), 6);
        // Patch p_memsz at Phdr offset 20 → size + 8.
        const std::size_t phdr = 52;
        const uint32_t new_memsz = sizeof(payload) + 8;
        blob[phdr + 20] = static_cast<uint8_t>(new_memsz);
        blob[phdr + 21] = static_cast<uint8_t>(new_memsz >>  8);
        blob[phdr + 22] = static_cast<uint8_t>(new_memsz >> 16);
        blob[phdr + 23] = static_cast<uint8_t>(new_memsz >> 24);
        std::string s(reinterpret_cast<const char*>(blob.data()), blob.size());
        std::istringstream ss(s);
        (void)load_elf(m, ss);
        REQUIRE(m.load_u32(base) == words[0]);
        // Bytes beyond p_filesz read as zero (Memory returns 0 for unmapped).
        REQUIRE(m.load_u32(base + sizeof(payload))     == 0u);
        REQUIRE(m.load_u32(base + sizeof(payload) + 4) == 0u);
    }

    // ---- Error paths ------------------------------------------------------
    REQUIRE(loadertest::threw([]{
        Memory m; std::istringstream ss("NOTHEX\n"); load_hex(m, ss, 0);
    }));
    REQUIRE(loadertest::threw([]{
        Memory m; std::istringstream ss("not an elf at all"); load_elf(m, ss);
    }));
    REQUIRE(loadertest::threw([]{
        // Correct magic but ELFCLASS64
        Memory m;
        std::string s(52, '\0');
        s[0] = 0x7F; s[1] = 'E'; s[2] = 'L'; s[3] = 'F';
        s[4] = 2;    // ELFCLASS64
        s[5] = 1;
        std::istringstream ss(s);
        load_elf(m, ss);
    }));
}

// --------------------------------------------------------- @section("cli") ---
SECTION("cli") {
    // Small helper: argv from a vector<const char*> so string literals compose.
    auto parse = [](std::initializer_list<const char*> argv_in, CliOpts& out) {
        std::vector<char*> argv;
        std::vector<std::string> owned(argv_in.begin(), argv_in.end());
        for (auto& s : owned) argv.push_back(s.data());
        return parse_args(static_cast<int>(argv.size()), argv.data(), out);
    };

    // Every Config knob has a matching --flag, and setting it round-trips.
    {
        CliOpts o;
        REQUIRE(parse({"oooc", "--width", "4", "--rob", "128", "--prf=192",
                       "--iq", "32", "--cdb=3", "--mem-lat", "5",
                       "--ghr=8", "--pht", "1024", "--chkpt=8"}, o) == 0);
        REQUIRE(o.cfg.width == 4);
        REQUIRE(o.cfg.rob_size == 128);
        REQUIRE(o.cfg.prf_size == 192);        // via = syntax
        REQUIRE(o.cfg.iq_size == 32);
        REQUIRE(o.cfg.num_cdb == 3);
        REQUIRE(o.cfg.mem_latency == 5);
        REQUIRE(o.cfg.ghr_bits == 8);
        REQUIRE(o.cfg.pht_size == 1024);
        REQUIRE(o.cfg.num_checkpoints == 8);
    }

    // Every knob in KNOBS parses under both --flag=value and --flag value.
    for (const auto& k : KNOBS) {
        CliOpts o;
        std::string eq = std::string(k.flag) + "=17";
        REQUIRE(parse({"oooc", eq.c_str()}, o) == 0);
        REQUIRE(o.cfg.*(k.member) == 17u);

        CliOpts o2;
        REQUIRE(parse({"oooc", k.flag, "19"}, o2) == 0);
        REQUIRE(o2.cfg.*(k.member) == 19u);
    }

    // Boolean flags.
    { CliOpts o; REQUIRE(parse({"oooc", "--regs"},  o) == 0); REQUIRE(o.print_regs); }
    { CliOpts o; REQUIRE(parse({"oooc", "--trace"}, o) == 0); REQUIRE(o.trace); }
    { CliOpts o; REQUIRE(parse({"oooc", "--help"},  o) == 0); REQUIRE(o.show_help); }

    // Base with hex-prefixed and decimal values.
    { CliOpts o; REQUIRE(parse({"oooc", "--base", "0x1000"}, o) == 0);
      REQUIRE(o.has_base); REQUIRE(o.base == 0x1000); }
    { CliOpts o; REQUIRE(parse({"oooc", "--base=4096"},      o) == 0);
      REQUIRE(o.has_base); REQUIRE(o.base == 4096); }

    // Positional ELF path.
    { CliOpts o; REQUIRE(parse({"oooc", "prog.elf"}, o) == 0);
      REQUIRE(o.elf_path == "prog.elf"); }

    // Rejects: unknown flag, bad number, missing value, two positionals.
    { CliOpts o; REQUIRE(parse({"oooc", "--nope"},           o) != 0); }
    { CliOpts o; REQUIRE(parse({"oooc", "--width", "boom"},  o) != 0); }
    { CliOpts o; REQUIRE(parse({"oooc", "--width"},          o) != 0); }
    { CliOpts o; REQUIRE(parse({"oooc", "a.elf", "b.elf"},   o) != 0); }

    // ---- end-to-end interpreter drive ------------------------------------
    // Program (RV32I): 3 + 5 = 8, then ECALL with a7=93 → exit(a0).
    //   addi x17, x0, 93   ; 0x05D00893
    //   addi x10, x0, 3    ; 0x00300513
    //   addi x11, x0, 5    ; 0x00500593
    //   add  x10, x10, x11 ; 0x00B50533
    //   ecall              ; 0x00000073
    Memory mem;
    mem.store_u32(0x1000, 0x05D00893);
    mem.store_u32(0x1004, 0x00300513);
    mem.store_u32(0x1008, 0x00500593);
    mem.store_u32(0x100C, 0x00B50533);
    mem.store_u32(0x1010, 0x00000073);
    const RunResult r = run_program(mem, 0x1000, /*max_insts=*/16, /*trace=*/false);
    REQUIRE(r.halted);
    REQUIRE(!r.trapped);
    REQUIRE(r.retired == 5);
    REQUIRE(r.exit_code == 8);
    REQUIRE(r.regs[10] == 8);
    REQUIRE(r.regs[11] == 5);
    REQUIRE(r.regs[17] == 93);
    REQUIRE(r.regs[0]  == 0);

    // Illegal instruction traps at commit, doesn't spin.
    Memory bad;
    bad.store_u32(0x0, 0xFFFFFFFFu);   // opcode 0x7F → INVALID → TRAP
    const RunResult t = run_program(bad, 0, /*max_insts=*/8, /*trace=*/false);
    REQUIRE(t.trapped);
    REQUIRE(!t.halted);
    REQUIRE(t.retired == 1);
}

// --------------------------------------------------------- @section("asm") ---
SECTION("asm") {
    using namespace asmc;

    // ---- Every instruction encodes byte-exact vs. the enc:: reference ---
    // The enc:: helpers were pinned in @section("decode") against 57
    // instructions, so this is a genuine differential check.
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

        // Decode both halves and simulate: auipc ra, 0 → ra = pc0 + 0 = 0.
        // jalr ra, ra, 12 → next PC = (0 + 12) & ~1 = 12 → target ✓
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
        const auto words = p.assemble();

        Memory mem;
        for (std::size_t i = 0; i < words.size(); ++i) {
            mem.store_u32(0x1000 + static_cast<uint32_t>(i * 4), words[i]);
        }
        const RunResult r = run_program(mem, 0x1000, /*max_insts=*/1000, /*trace=*/false);
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
