// Test driver: one SECTION() block per group of assertions, all in one
// translation unit so `make test` builds a single binary.

#include <cstdint>

#include <algorithm>
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
#include "rob.h"
#include "cpu.h"
#include "asm.h"
#include "ref.h"

// Include the driver TU so parse_args / print_help are unit-testable
// without shelling out. Its own int main() is guarded off.
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

// For failures where the expression alone says nothing useful, such as a
// differential run that needs to name the register that diverged.
inline void report_fail_msg(const char* expr, const std::string& detail,
                            const char* file, int line) {
    std::fprintf(stderr, "  FAIL: %s   at %s:%d\n%s", expr, file, line, detail.c_str());
    if (!detail.empty() && detail.back() != '\n') std::fputc('\n', stderr);
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

#define REQUIRE_MSG(expr, detail)                                              \
    do {                                                                       \
        if (!(expr)) ::test::report_fail_msg(#expr, (detail), __FILE__, __LINE__); \
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

    // OpKind values are distinct — enough of a smoke test at this stage.
    static_assert(static_cast<int>(OpKind::ALU)  != static_cast<int>(OpKind::TRAP));
    static_assert(static_cast<int>(OpKind::LOAD) != static_cast<int>(OpKind::STORE));
    static_assert(static_cast<int>(OpKind::NOP)  != static_cast<int>(OpKind::ALU));
}

// ------------------------------------------------------ @section("config") ---
SECTION("config") {
    Config c;

    // documented defaults
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

    // the default sizing is starvation-free
    REQUIRE(!c.prf_can_starve());

    // exact boundary: prf_size == rob_size + 32 is safe; one less starves
    Config edge = c;
    edge.prf_size = edge.rob_size + 32;
    REQUIRE(!edge.prf_can_starve());
    edge.prf_size = edge.rob_size + 32 - 1;
    REQUIRE(edge.prf_can_starve());

    // the stress config used by the sweep
    Config small_prf = c;
    small_prf.prf_size = 32;
    REQUIRE(small_prf.prf_can_starve());
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

    // ---- Misaligned store, read back at three widths ----------------------
    // 0xDEADBEEF at 0x1002, read back as one lw, two lh's, four lb's.
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

    // ---- All three formats land identical bytes ---------------------------
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
    ref::Options o16;
    o16.max_insts = 16;
    const ref::Result r = ref::run(mem, 0x1000, o16);
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
    ref::Options o8;
    o8.max_insts = 8;
    const ref::Result t = ref::run(bad, 0, o8);
    REQUIRE(t.trapped);
    REQUIRE(!t.halted);
    REQUIRE(t.retired == 1);
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
        ref::Options o;
        o.max_insts = 1000;
        const ref::Result r = ref::run(mem, 0x1000, o);
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


// --------------------------------------------------------- @section("ref") ---
namespace reftest {

inline constexpr uint32_t TEXT = 0x1000;   // where every test program loads
inline constexpr uint32_t DATA = 0x2000;   // scratch area the programs write

inline void load_words(Memory& m, uint32_t base, const std::vector<uint32_t>& w) {
    for (std::size_t i = 0; i < w.size(); ++i) {
        m.store_u32(base + static_cast<uint32_t>(i * 4), w[i]);
    }
}

// Assemble-load-run in one call, for the programs whose memory image is not
// itself under test.
inline ref::Result run(const std::vector<uint32_t>& words, uint64_t budget = 100000) {
    Memory m;
    load_words(m, TEXT, words);
    ref::Options o;
    o.max_insts = budget;
    return ref::run(m, TEXT, o);
}

// Render a word stream as the .hex format load_hex() accepts.
inline std::string to_hex_text(const std::vector<uint32_t>& words) {
    std::ostringstream ss;
    for (uint32_t w : words) {
        char buf[16];
        std::snprintf(buf, sizeof(buf), "%08X\n", w);
        ss << buf;
    }
    return ss.str();
}

}  // namespace reftest

SECTION("ref") {
    using namespace asmc;
    using reftest::TEXT;
    using reftest::DATA;

    // ---- 1. ALU coverage --------------------------------------------------
    // Every RV32I integer operation once, results parked in callee-saved and
    // argument registers so the exit state pins all of them at once.
    {
        Assembler p;
        p.li(t0, 12);
        p.li(t1, 5);
        p.li(t2, -1);
        p.add  (a1, t0, t1);          // 17
        p.sub  (a2, t0, t1);          // 7
        p.sll  (a3, t0, t1);          // 12 << 5
        p.srl  (a4, t0, t1);          // 12 >> 5 → 0
        p.sra  (a5, t2, t1);          // -1 >>a 5 → -1 (sign-fill, not 0x07FFFFFF)
        p.and_ (s2, t0, t1);          // 4
        p.or_  (s3, t0, t1);          // 13
        p.xor_ (s4, t0, t1);          // 9
        p.slt  (s5, t2, t1);          // -1 <s 5 → 1
        p.sltu (s6, t2, t1);          // 0xFFFFFFFF <u 5 → 0
        p.lui  (s7, 0x12345);         // 0x12345000
        const uint32_t auipc_off = p.pc();
        p.auipc(s8, 0);               // its own PC
        p.addi (s9,  t0, -20);        // -8
        p.slti (s10, t2, 0);          // 1
        p.xori (s11, t0, 0xFF);       // 243
        p.srai (s1,  t2, 3);          // -1
        p.slli (s0,  t1, 2);          // 20
        p.add  (a0, a1, a2);          // exit code 24
        p.li(a7, 93);
        p.ecall();

        // 3 li + 17 ops + add + li + ecall, every li single-word.
        const ref::Result r = reftest::run(p.assemble());
        REQUIRE(r.halted);
        REQUIRE(!r.trapped);
        REQUIRE(!r.budget);
        REQUIRE(r.retired   == 23);
        REQUIRE(r.exit_code == 24);

        REQUIRE(r.regs[a1]  == 17u);
        REQUIRE(r.regs[a2]  == 7u);
        REQUIRE(r.regs[a3]  == 384u);
        REQUIRE(r.regs[a4]  == 0u);
        REQUIRE(r.regs[a5]  == 0xFFFFFFFFu);
        REQUIRE(r.regs[s2]  == 4u);
        REQUIRE(r.regs[s3]  == 13u);
        REQUIRE(r.regs[s4]  == 9u);
        REQUIRE(r.regs[s5]  == 1u);
        REQUIRE(r.regs[s6]  == 0u);
        REQUIRE(r.regs[s7]  == 0x12345000u);
        REQUIRE(r.regs[s8]  == TEXT + auipc_off);
        REQUIRE(r.regs[s9]  == static_cast<uint32_t>(-8));
        REQUIRE(r.regs[s10] == 1u);
        REQUIRE(r.regs[s11] == 243u);
        REQUIRE(r.regs[s1]  == 0xFFFFFFFFu);
        REQUIRE(r.regs[s0]  == 20u);
    }

    // ---- 2. x0 stays zero -------------------------------------------------
    {
        Assembler p;
        p.li(t0, 5);
        p.add (zero, t0, t0);         // discarded
        p.addi(zero, t0, 1);          // discarded
        p.li(a7, 93);
        p.li(a0, 0);
        p.ecall();

        const ref::Result r = reftest::run(p.assemble());
        REQUIRE(r.halted);
        REQUIRE(r.regs[0]   == 0u);
        REQUIRE(r.exit_code == 0u);
    }

    // ---- 3. Loop ----------------------------------------------------------
    // sum 1..100 = 5050. 100 iterations × 4 instructions, plus the exit-test
    // branch, a 3-instruction prologue, and a 2-instruction epilogue.
    {
        Assembler p;
        p.li(a0, 0);
        p.li(a1, 1);
        p.li(a2, 101);
        p.label("loop");
        p.beq(a1, a2, "done");
        p.add(a0, a0, a1);
        p.addi(a1, a1, 1);
        p.j("loop");
        p.label("done");
        p.li(a7, 93);
        p.ecall();

        const ref::Result r = reftest::run(p.assemble());
        REQUIRE(r.halted);
        REQUIRE(r.exit_code == 5050);
        REQUIRE(r.retired   == 3 + 100 * 4 + 1 + 2);
    }

    // ---- 4. Load / store, including sub-word and misaligned ---------------
    {
        Assembler p;
        p.li(s0, static_cast<int32_t>(DATA));
        p.li(t0, 0);                          // i
        p.li(t1, 8);                          // n
        p.li(a0, 0);                          // sum

        p.label("fill");                      // a[i] = i + 1
        p.beq(t0, t1, "fill_done");
        p.slli(t2, t0, 2);
        p.add(t3, s0, t2);
        p.addi(t4, t0, 1);
        p.sw(t4, t3, 0);
        p.addi(t0, t0, 1);
        p.j("fill");
        p.label("fill_done");

        p.li(t0, 0);
        p.label("sum");                       // sum += a[i]
        p.beq(t0, t1, "sum_done");
        p.slli(t2, t0, 2);
        p.add(t3, s0, t2);
        p.lw(t5, t3, 0);
        p.add(a0, a0, t5);
        p.addi(t0, t0, 1);
        p.j("sum");
        p.label("sum_done");

        p.li(t0, -3);                         // byte: sign vs. zero extension
        p.sb(t0, s0, 100);
        p.lb (a1, s0, 100);                   // -3
        p.lbu(a2, s0, 100);                   // 253

        p.li(t1, -300);                       // half: sign vs. zero extension
        p.sh(t1, s0, 104);
        p.lh (a3, s0, 104);                   // -300
        p.lhu(a4, s0, 104);                   // 65236

        p.li(t2, 0x12345678);                 // word at a misaligned address
        p.sw(t2, s0, 202);
        p.lw (a5, s0, 202);
        p.lhu(a6, s0, 202);                   // 0x5678

        p.li(a7, 93);
        p.ecall();

        Memory m;
        reftest::load_words(m, TEXT, p.assemble());
        ref::Options o;
        const ref::Result r = ref::run(m, TEXT, o);

        REQUIRE(r.halted);
        REQUIRE(r.exit_code == 36);           // 1 + 2 + ... + 8
        REQUIRE(r.regs[a1] == static_cast<uint32_t>(-3));
        REQUIRE(r.regs[a2] == 253u);
        REQUIRE(r.regs[a3] == static_cast<uint32_t>(-300));
        REQUIRE(r.regs[a4] == 65236u);
        REQUIRE(r.regs[a5] == 0x12345678u);
        REQUIRE(r.regs[a6] == 0x5678u);

        // The stores are visible in memory, not just in the loaded registers.
        for (uint32_t i = 0; i < 8; ++i) {
            REQUIRE(m.load_u32(DATA + i * 4) == i + 1);
        }
        REQUIRE(m.load_u8 (DATA + 100) == 0xFDu);
        REQUIRE(m.load_u16(DATA + 104) == 0xFED4u);
        REQUIRE(m.load_u32(DATA + 202) == 0x12345678u);
    }

    // ---- 5. Function calls: recursive fib(10) -----------------------------
    // Exercises call / ret, the RAS-relevant jal-jalr pairing, and a real
    // stack: 10 nested frames of saves and restores.
    {
        Assembler p;
        p.li(sp, 0x8000);
        p.li(a0, 10);
        p.call("fib");
        p.li(a7, 93);
        p.ecall();                            // exit(fib(10)) = 55

        p.label("fib");
        p.addi(sp, sp, -16);
        p.sw(ra, sp, 12);
        p.sw(s0, sp, 8);                      // s0 = n
        p.sw(s1, sp, 4);                      // s1 = fib(n-1)
        p.li(t0, 2);
        p.blt(a0, t0, "fib_done");            // n < 2 → return n unchanged
        p.mv(s0, a0);
        p.addi(a0, s0, -1);
        p.call("fib");
        p.mv(s1, a0);
        p.addi(a0, s0, -2);
        p.call("fib");
        p.add(a0, a0, s1);
        p.label("fib_done");
        p.lw(ra, sp, 12);
        p.lw(s0, sp, 8);
        p.lw(s1, sp, 4);
        p.addi(sp, sp, 16);
        p.ret_();

        const ref::Result r = reftest::run(p.assemble());
        REQUIRE(r.halted);
        REQUIRE(!r.trapped);
        REQUIRE(r.exit_code == 55);
        REQUIRE(r.regs[a0]  == 55u);
        REQUIRE(r.regs[sp]  == 0x8000u);      // every frame popped
        REQUIRE(r.regs[s0]  == 0u);           // callee-saved, restored
        REQUIRE(r.regs[s1]  == 0u);
    }

    // ---- 6. mul / div, including the edge cases that trap on real hardware -
    {
        Assembler p;
        p.li(a1, -1);
        p.li(a2, 10);
        p.li(a3, 0);
        p.li(a4, static_cast<int32_t>(0x80000000));   // INT_MIN

        p.div_(t0, a4, a1);           // INT_MIN / -1 → INT_MIN, no trap
        p.rem (t1, a4, a1);           // → 0
        p.div_(t2, a2, a3);           // x / 0 → -1
        p.divu(t3, a2, a3);           // → 0xFFFFFFFF
        p.rem (t4, a2, a3);           // x % 0 → x
        p.remu(t5, a2, a3);           // → 10
        p.div_(t6, a2, a1);           // 10 / -1 → -10

        p.mul   (s0, a2, a2);         // 100
        p.mulh  (s1, a1, a1);         // upper 32 of 1 → 0
        p.mulhu (s2, a1, a1);         // 0xFFFFFFFE
        p.mulhsu(s3, a1, a2);         // -10 >> 32 → 0xFFFFFFFF

        p.li(a7, 93);
        p.mv(a0, s0);
        p.ecall();

        const ref::Result r = reftest::run(p.assemble());
        REQUIRE(r.halted);
        REQUIRE(!r.trapped);                  // divide-by-zero is defined, not fatal
        REQUIRE(r.exit_code == 100);
        REQUIRE(r.regs[t0] == 0x80000000u);
        REQUIRE(r.regs[t1] == 0u);
        REQUIRE(r.regs[t2] == 0xFFFFFFFFu);
        REQUIRE(r.regs[t3] == 0xFFFFFFFFu);
        REQUIRE(r.regs[t4] == 10u);
        REQUIRE(r.regs[t5] == 10u);
        REQUIRE(r.regs[t6] == static_cast<uint32_t>(-10));
        REQUIRE(r.regs[s0] == 100u);
        REQUIRE(r.regs[s1] == 0u);
        REQUIRE(r.regs[s2] == 0xFFFFFFFEu);
        REQUIRE(r.regs[s3] == 0xFFFFFFFFu);
    }

    // ---- 7. Halt, trap, and budget are three distinct outcomes ------------
    {
        // ecall with a7 != 93 is not the exit syscall — it traps.
        Assembler p;
        p.li(a7, 42);
        p.ecall();
        const ref::Result r = reftest::run(p.assemble());
        REQUIRE(r.trapped);
        REQUIRE(!r.halted);
        REQUIRE(r.retired == 2);
    }
    {
        Assembler p;
        p.li(t0, 1);
        p.ebreak();
        p.li(t1, 2);                          // must never execute
        const ref::Result r = reftest::run(p.assemble());
        REQUIRE(r.trapped);
        REQUIRE(r.regs[t0] == 1u);
        REQUIRE(r.regs[t1] == 0u);
        REQUIRE(r.pc == TEXT + 4);            // the trap reports its own PC
    }
    {
        // An unbounded loop stops at the budget without halting or trapping.
        Assembler p;
        p.label("spin");
        p.j("spin");
        const ref::Result r = reftest::run(p.assemble(), /*budget=*/50);
        REQUIRE(r.budget);
        REQUIRE(!r.halted);
        REQUIRE(!r.trapped);
        REQUIRE(r.retired == 50);
    }
    {
        // Fetching from never-written memory reads zeros, which decode as
        // an illegal instruction rather than running off into the weeds.
        Memory empty;
        const ref::Result r = ref::run(empty, TEXT);
        REQUIRE(r.trapped);
        REQUIRE(r.retired == 1);
    }

    // ---- 8. The same program via the .hex loader --------------------------
    // Round-trips assembler → hex text → load_hex → interpreter, and pins
    // the result against the directly-loaded image: all 32 registers.
    {
        Assembler p;
        p.li(a0, 0);
        p.li(a1, 1);
        p.li(a2, 11);
        p.label("loop");
        p.beq(a1, a2, "done");
        p.add(a0, a0, a1);
        p.addi(a1, a1, 1);
        p.j("loop");
        p.label("done");
        p.li(a7, 93);
        p.ecall();
        const auto words = p.assemble();

        const ref::Result direct = reftest::run(words);

        Memory m;
        std::istringstream hex(reftest::to_hex_text(words));
        const LoadResult loaded = load_hex(m, hex, TEXT);
        REQUIRE(loaded.entry == TEXT);
        const ref::Result via_hex = ref::run(m, loaded.entry);

        REQUIRE(via_hex.halted);
        REQUIRE(via_hex.exit_code == 55);
        REQUIRE(via_hex.retired   == direct.retired);
        for (int i = 0; i < 32; ++i) REQUIRE(via_hex.regs[i] == direct.regs[i]);
    }

    // ---- 9. Tracing emits output and does not perturb the result ----------
    {
        Assembler p;
        p.li(a0, 7);
        p.li(a7, 93);
        p.ecall();
        const auto words = p.assemble();

        const ref::Result quiet = reftest::run(words);

        Memory m;
        reftest::load_words(m, TEXT, words);
        ref::Options o;
        o.trace = true;
        o.trace_out = std::tmpfile();
        const ref::Result traced = ref::run(m, TEXT, o);
        if (o.trace_out) {
            REQUIRE(std::ftell(o.trace_out) > 0);
            std::fclose(o.trace_out);
        }
        REQUIRE(traced.retired   == quiet.retired);
        REQUIRE(traced.exit_code == quiet.exit_code);
    }
}


// ================================================================ workloads ===
// The validation corpus: thirteen programs run through both the pipeline
// model and ref.h, covering ALU semantics, loops, arrays, store forwarding,
// sub-word and partial-overlap accesses, nested calls, recursive Fibonacci,
// an unpredictable branch, mul/div with divide-by-zero, a pointer chase, and
// a WAW/WAR renaming stress.
//
// Every expected exit code is derived independently of the simulator: by hand
// where the arithmetic is simple, by an equivalent C++ loop where it is not.
// A value read off a previous run would assert nothing.

namespace wl {

inline constexpr uint32_t TEXT  = 0x1000;    // program image
inline constexpr uint32_t DATA  = 0x4000;    // scratch the programs initialize
inline constexpr uint32_t STACK = 0x8000;    // grows down, away from DATA

struct Workload {
    std::string             name;
    std::vector<uint32_t>   words;
    std::optional<uint32_t> expect_exit;   // hand-computed, not observed
    uint64_t                budget = 200000;
};

using asmc::Assembler;
using namespace asmc;   // register aliases: a0, t0, s0, sp, ra, zero, ...

// exit(a0) — every workload leaves its result in a0 and ends here.
inline void exit_now(Assembler& p) {
    p.li(a7, 93);
    p.ecall();
}

// ---- 1. ALU coverage ------------------------------------------------------
inline Workload alu() {
    Assembler p;
    p.li(t0, 12);
    p.li(t1, 5);
    p.li(t2, -1);
    p.add  (a1, t0, t1);          // 17
    p.sub  (a2, t0, t1);          // 7
    p.sll  (a3, t0, t1);          // 384
    p.srl  (a4, t0, t1);          // 0
    p.sra  (a5, t2, t1);          // -1, sign-filled
    p.and_ (s2, t0, t1);          // 4
    p.or_  (s3, t0, t1);          // 13
    p.xor_ (s4, t0, t1);          // 9
    p.slt  (s5, t2, t1);          // 1
    p.sltu (s6, t2, t1);          // 0
    p.lui  (s7, 0x12345);
    p.auipc(s8, 0);
    p.addi (s9,  t0, -20);        // -8
    p.slti (s10, t2, 0);          // 1
    p.xori (s11, t0, 0xFF);       // 243
    p.srai (s1,  t2, 3);          // -1
    p.slli (s0,  t1, 2);          // 20
    p.add  (a0, a1, a2);          // 17 + 7
    exit_now(p);
    return {"alu", p.assemble(), 24, 200000};
}

// ---- 2. Loop --------------------------------------------------------------
inline Workload loop() {
    Assembler p;
    p.li(a0, 0);
    p.li(a1, 1);
    p.li(a2, 101);
    p.label("loop");
    p.beq(a1, a2, "done");
    p.add(a0, a0, a1);
    p.addi(a1, a1, 1);
    p.j("loop");
    p.label("done");
    exit_now(p);
    return {"loop", p.assemble(), 5050, 200000};   // sum 1..100
}

// ---- 3. Arrays: bubble sort -----------------------------------------------
// Sorts a permutation of 1..12, verifies the result is ordered, and exits
// with a[0] * 100 + a[11] — 112 when sorted, 999 if any pair is out of order.
inline Workload bubble_sort() {
    const int32_t input[12] = {9, 4, 7, 1, 12, 3, 8, 2, 11, 5, 10, 6};

    Assembler p;
    p.li(s0, static_cast<int32_t>(DATA));
    for (int i = 0; i < 12; ++i) {
        p.li(t0, input[i]);
        p.sw(t0, s0, i * 4);
    }
    p.li(s1, 12);
    p.li(t0, 0);                              // i
    p.label("outer");
    p.addi(t1, s1, -1);
    p.bge(t0, t1, "outer_done");
    p.li(t2, 0);                              // j
    p.sub(t3, t1, t0);                        // limit = n - 1 - i
    p.label("inner");
    p.bge(t2, t3, "inner_done");
    p.slli(t4, t2, 2);
    p.add(t4, s0, t4);                        // &a[j]
    p.lw(t5, t4, 0);
    p.lw(t6, t4, 4);
    p.bge(t6, t5, "no_swap");
    p.sw(t6, t4, 0);
    p.sw(t5, t4, 4);
    p.label("no_swap");
    p.addi(t2, t2, 1);
    p.j("inner");
    p.label("inner_done");
    p.addi(t0, t0, 1);
    p.j("outer");
    p.label("outer_done");

    p.li(a0, 0);
    p.li(t0, 0);
    p.label("check");
    p.addi(t1, s1, -1);
    p.bge(t0, t1, "check_done");
    p.slli(t2, t0, 2);
    p.add(t2, s0, t2);
    p.lw(t3, t2, 0);
    p.lw(t4, t2, 4);
    p.bge(t4, t3, "in_order");
    p.li(a0, 999);
    p.j("check_done");
    p.label("in_order");
    p.addi(t0, t0, 1);
    p.j("check");
    p.label("check_done");
    p.bne(a0, zero, "finish");
    p.lw(t3, s0, 0);                          // a[0]
    p.lw(t5, s0, 44);                         // a[11]
    p.li(t6, 100);
    p.mul(a0, t3, t6);
    p.add(a0, a0, t5);
    p.label("finish");
    exit_now(p);
    return {"bubble_sort", p.assemble(), 112, 200000};
}

// ---- 4. Arrays: 4x4 integer matmul ----------------------------------------
// A[i][j] = i + j, B[i][j] = 1 + (i == j), C = A·B, exit = sum of C.
// C[i][k] = Σ_j (i+j)(1 + [j==k]) = (4i + 6) + (i + k) = 5i + k + 6, so the
// total is Σ_i Σ_k (5i + k + 6) = 240.
inline Workload matmul() {
    Assembler p;
    p.li(s0, static_cast<int32_t>(DATA));         // A
    p.li(s1, static_cast<int32_t>(DATA + 64));    // B
    p.li(s2, static_cast<int32_t>(DATA + 128));   // C
    p.li(s3, 4);                                  // n

    p.li(t0, 0);                                  // i
    p.label("fill_i");
    p.bge(t0, s3, "fill_done");
    p.li(t1, 0);                                  // j
    p.label("fill_j");
    p.bge(t1, s3, "fill_i_next");
    p.mul(t2, t0, s3);
    p.add(t2, t2, t1);
    p.slli(t2, t2, 2);                            // (i*n + j) * 4
    p.add(t3, s0, t2);
    p.add(t4, t0, t1);
    p.sw(t4, t3, 0);                              // A[i][j] = i + j
    p.add(t3, s1, t2);
    p.li(t5, 1);
    p.bne(t0, t1, "store_b");
    p.li(t5, 2);
    p.label("store_b");
    p.sw(t5, t3, 0);                              // B[i][j] = 1 + (i == j)
    p.addi(t1, t1, 1);
    p.j("fill_j");
    p.label("fill_i_next");
    p.addi(t0, t0, 1);
    p.j("fill_i");
    p.label("fill_done");

    p.li(a0, 0);
    p.li(t0, 0);                                  // i
    p.label("mm_i");
    p.bge(t0, s3, "mm_done");
    p.li(t1, 0);                                  // k
    p.label("mm_k");
    p.bge(t1, s3, "mm_i_next");
    p.li(t2, 0);                                  // j
    p.li(s4, 0);                                  // accumulator
    p.label("mm_j");
    p.bge(t2, s3, "mm_store");
    p.mul(t3, t0, s3);
    p.add(t3, t3, t2);
    p.slli(t3, t3, 2);
    p.add(t3, s0, t3);
    p.lw(t4, t3, 0);                              // A[i][j]
    p.mul(t5, t2, s3);
    p.add(t5, t5, t1);
    p.slli(t5, t5, 2);
    p.add(t5, s1, t5);
    p.lw(t6, t5, 0);                              // B[j][k]
    p.mul(t4, t4, t6);
    p.add(s4, s4, t4);
    p.addi(t2, t2, 1);
    p.j("mm_j");
    p.label("mm_store");
    p.mul(t3, t0, s3);
    p.add(t3, t3, t1);
    p.slli(t3, t3, 2);
    p.add(t3, s2, t3);
    p.sw(s4, t3, 0);                              // C[i][k]
    p.add(a0, a0, s4);
    p.addi(t1, t1, 1);
    p.j("mm_k");
    p.label("mm_i_next");
    p.addi(t0, t0, 1);
    p.j("mm_i");
    p.label("mm_done");
    exit_now(p);
    return {"matmul", p.assemble(), 240, 200000};
}

// ---- 5. Arrays: sieve of Eratosthenes -------------------------------------
inline Workload sieve() {
    Assembler p;
    p.li(s0, static_cast<int32_t>(DATA));
    p.li(s1, 100);                            // n

    p.li(t0, 0);
    p.label("zero");
    p.bge(t0, s1, "zero_done");
    p.add(t1, s0, t0);
    p.sb(zero, t1, 0);
    p.addi(t0, t0, 1);
    p.j("zero");
    p.label("zero_done");

    p.li(t0, 2);
    p.label("sv_i");
    p.mul(t1, t0, t0);
    p.bge(t1, s1, "sv_done");                 // i*i >= n → every composite marked
    p.add(t2, s0, t0);
    p.lbu(t3, t2, 0);
    p.bne(t3, zero, "sv_next");               // i is composite; skip
    p.mv(t4, t1);
    p.label("mark");
    p.bge(t4, s1, "sv_next");
    p.add(t5, s0, t4);
    p.li(t6, 1);
    p.sb(t6, t5, 0);
    p.add(t4, t4, t0);
    p.j("mark");
    p.label("sv_next");
    p.addi(t0, t0, 1);
    p.j("sv_i");
    p.label("sv_done");

    p.li(a0, 0);
    p.li(t0, 2);
    p.label("count");
    p.bge(t0, s1, "count_done");
    p.add(t1, s0, t0);
    p.lbu(t2, t1, 0);
    p.bne(t2, zero, "count_next");
    p.addi(a0, a0, 1);
    p.label("count_next");
    p.addi(t0, t0, 1);
    p.j("count");
    p.label("count_done");
    exit_now(p);
    return {"sieve", p.assemble(), 25, 200000};   // 25 primes below 100
}

// ---- 6. Store-to-load forwarding ------------------------------------------
// Twenty dependent store→load pairs at one address, each feeding the next.
// The pipeline forwards them out of the store queue; the reference just sees
// memory, which is the disagreement worth testing for.
inline Workload store_forward() {
    Assembler p;
    p.li(s0, static_cast<int32_t>(DATA));
    p.li(s1, 20);
    p.li(t0, 0);                              // value
    p.li(t1, 0);                              // i
    p.label("sf");
    p.bge(t1, s1, "sf_done");
    p.sw(t0, s0, 0);
    p.lw(t2, s0, 0);                          // same address → forwards
    p.addi(t0, t2, 3);
    p.sw(t0, s0, 4);                          // different address
    p.lw(t3, s0, 0);                          // must not see the +4 store
    p.addi(t1, t1, 1);
    p.j("sf");
    p.label("sf_done");
    p.mv(a0, t0);
    exit_now(p);
    return {"store_forward", p.assemble(), 60, 200000};   // 3 per iteration
}

// ---- 7. Sub-word and partial-overlap accesses -----------------------------
// Each access partially overlaps an earlier store, the case a store queue
// has to answer with a replay rather than a forward.
inline Workload subword() {
    Assembler p;
    p.li(s0, static_cast<int32_t>(DATA));
    p.li(t0, 0x12345678);
    p.sw(t0, s0, 0);                          // bytes: 78 56 34 12
    p.lbu(a1, s0, 1);                         // 0x56
    p.lbu(a2, s0, 3);                         // 0x12
    p.li(t1, 0xABCD);
    p.sh(t1, s0, 2);                          // word becomes 0xABCD5678
    p.lw(a3, s0, 0);
    p.li(t2, -1);
    p.sb(t2, s0, 0);                          // word becomes 0xABCD56FF
    p.lw(a4, s0, 0);
    p.lb(a5, s0, 0);                          // -1, sign-extended
    p.lhu(a6, s0, 0);                         // 0x56FF
    p.li(t3, 0x0F0F0F0F);
    p.sw(t3, s0, 6);                          // misaligned, straddles the above
    p.lw(t4, s0, 4);                          // reads across two stores
    p.lhu(t5, s0, 6);                         // 0x0F0F
    p.andi(t6, a4, 0xFF);                     // 0xFF
    p.add(a0, a1, a2);
    p.add(a0, a0, t6);
    p.add(a0, a0, t5);
    exit_now(p);
    // 0x56 + 0x12 + 0xFF + 0x0F0F = 86 + 18 + 255 + 3855
    return {"subword", p.assemble(), 4214, 200000};
}

// ---- 8. Nested calls ------------------------------------------------------
// A four-deep call chain, ten times over: the return-address stack at
// several depths, plus ra save/restore.
inline Workload nested_calls() {
    Assembler p;
    p.li(sp, static_cast<int32_t>(STACK));
    p.li(a0, 0);
    p.li(s0, 10);
    p.li(s1, 0);
    p.label("nc_loop");
    p.bge(s1, s0, "nc_done");
    p.call("f1");
    p.addi(s1, s1, 1);
    p.j("nc_loop");
    p.label("nc_done");
    exit_now(p);

    // f1 → f2 → f3 → f4, adding 4, 3, 2, 1 on the way back out.
    const struct { const char* self; const char* callee; int32_t add; } frames[] = {
        {"f1", "f2", 4},
        {"f2", "f3", 3},
        {"f3", "f4", 2},
    };
    for (const auto& f : frames) {
        p.label(f.self);
        p.addi(sp, sp, -8);
        p.sw(ra, sp, 4);
        p.call(f.callee);
        p.addi(a0, a0, f.add);
        p.lw(ra, sp, 4);
        p.addi(sp, sp, 8);
        p.ret_();
    }
    p.label("f4");
    p.addi(a0, a0, 1);
    p.ret_();

    return {"nested_calls", p.assemble(), 100, 200000};   // 10 × (1+2+3+4)
}

// ---- 9. Recursive Fibonacci -----------------------------------------------
inline Workload fib() {
    Assembler p;
    p.li(sp, static_cast<int32_t>(STACK));
    p.li(a0, 12);
    p.call("fib");
    exit_now(p);

    p.label("fib");
    p.addi(sp, sp, -16);
    p.sw(ra, sp, 12);
    p.sw(s0, sp, 8);
    p.sw(s1, sp, 4);
    p.li(t0, 2);
    p.blt(a0, t0, "fib_done");                // fib(0) = 0, fib(1) = 1
    p.mv(s0, a0);
    p.addi(a0, s0, -1);
    p.call("fib");
    p.mv(s1, a0);
    p.addi(a0, s0, -2);
    p.call("fib");
    p.add(a0, a0, s1);
    p.label("fib_done");
    p.lw(ra, sp, 12);
    p.lw(s0, sp, 8);
    p.lw(s1, sp, 4);
    p.addi(sp, sp, 16);
    p.ret_();

    return {"fib", p.assemble(), 144, 2000000};   // fib(12)
}

// ---- 10. Unpredictable branch ---------------------------------------------
// A branch driven by a bit of an LCG stream, so no history pattern predicts
// it. The expected count comes from the same recurrence in C++.
inline Workload lcg_branch() {
    constexpr uint32_t SEED = 12345, MUL = 1103515245, INC = 12345, ITERS = 200;

    uint32_t x = SEED, taken = 0;
    for (uint32_t i = 0; i < ITERS; ++i) {
        x = x * MUL + INC;
        taken += (x >> 16) & 1u;
    }

    Assembler p;
    p.li(s0, static_cast<int32_t>(ITERS));
    p.li(s1, 0);                              // i
    p.li(s2, static_cast<int32_t>(SEED));     // x
    p.li(s3, static_cast<int32_t>(MUL));
    p.li(s4, static_cast<int32_t>(INC));
    p.li(a0, 0);                              // count
    p.label("lcg");
    p.bge(s1, s0, "lcg_done");
    p.mul(s2, s2, s3);
    p.add(s2, s2, s4);
    p.srli(t0, s2, 16);
    p.andi(t0, t0, 1);
    p.beq(t0, zero, "not_taken");
    p.addi(a0, a0, 1);
    p.label("not_taken");
    p.addi(s1, s1, 1);
    p.j("lcg");
    p.label("lcg_done");
    exit_now(p);
    return {"lcg_branch", p.assemble(), taken, 200000};
}

// ---- 11. mul / div, including divide-by-zero ------------------------------
inline Workload muldiv() {
    uint32_t acc = 0;
    for (uint32_t i = 1; i <= 20; ++i) acc += 1000u / i + 1000u % i;
    acc += 0xFFFFFFFFu;                       // the divide-by-zero result, -1

    Assembler p;
    p.li(a1, -1);
    p.li(a2, 10);
    p.li(a3, 0);
    p.li(a4, static_cast<int32_t>(0x80000000));

    p.div_(t0, a4, a1);                       // INT_MIN / -1 → INT_MIN, no trap
    p.rem (t1, a4, a1);                       // → 0
    p.div_(t2, a2, a3);                       // x / 0 → -1
    p.divu(t3, a2, a3);
    p.rem (t4, a2, a3);                       // x % 0 → x
    p.remu(t5, a2, a3);
    p.mulh  (a5, a1, a1);
    p.mulhu (a6, a1, a1);
    p.mulhsu(a7, a1, a2);

    p.li(s0, 1);                              // i
    p.li(s1, 21);
    p.li(s2, 0);                              // acc
    p.li(s3, 1000);
    p.label("md");
    p.bge(s0, s1, "md_done");
    p.div_(t0, s3, s0);
    p.rem (t1, s3, s0);
    p.add(s2, s2, t0);
    p.add(s2, s2, t1);
    p.addi(s0, s0, 1);
    p.j("md");
    p.label("md_done");
    p.div_(t2, s3, a3);                       // divide by zero once more
    p.add(s2, s2, t2);
    p.mv(a0, s2);
    exit_now(p);
    return {"muldiv", p.assemble(), acc, 200000};
}

// ---- 12. Pointer chase ----------------------------------------------------
// 32 two-word nodes in a stride-7 cycle (7 is coprime with 32, so the cycle
// covers every node). The traversal is a chain of dependent loads, which no
// amount of issue width can accelerate.
inline Workload pointer_chase() {
    Assembler p;
    p.li(s0, static_cast<int32_t>(DATA));
    p.li(s1, 32);

    p.li(t0, 0);
    p.label("build");
    p.bge(t0, s1, "build_done");
    p.slli(t1, t0, 3);
    p.add(t1, s0, t1);                        // &node[i]
    p.addi(t2, t0, 1);
    p.sw(t2, t1, 0);                          // node[i].value = i + 1
    p.addi(t3, t0, 7);
    p.andi(t3, t3, 31);
    p.slli(t3, t3, 3);
    p.add(t3, s0, t3);
    p.sw(t3, t1, 4);                          // node[i].next = &node[(i+7) % 32]
    p.addi(t0, t0, 1);
    p.j("build");
    p.label("build_done");

    p.li(a0, 0);
    p.mv(t0, s0);                             // p = &node[0]
    p.li(t1, 0);
    p.label("chase");
    p.bge(t1, s1, "chase_done");
    p.lw(t2, t0, 0);
    p.add(a0, a0, t2);
    p.lw(t0, t0, 4);                          // p = p->next
    p.addi(t1, t1, 1);
    p.j("chase");
    p.label("chase_done");
    exit_now(p);
    return {"pointer_chase", p.assemble(), 528, 200000};   // sum 1..32
}

// ---- 13. WAW / WAR renaming stress ----------------------------------------
// t0 is written five times per iteration and read in between. Each write
// must land in a distinct physical register or a later reader sees the
// wrong value.
inline Workload waw_war() {
    Assembler p;
    p.li(a0, 0);
    p.li(s0, 10);
    p.li(s1, 0);
    p.label("w_loop");
    p.bge(s1, s0, "w_done");
    p.li(t0, 1);
    p.add(a0, a0, t0);                        // +1
    p.li(t0, 2);                              // WAW
    p.add(a0, a0, t0);                        // +2
    p.li(t0, 3);                              // WAW
    p.add(a0, a0, t0);                        // +3
    p.mv(t1, t0);                             // read t0 ...
    p.li(t0, 4);                              // ... then WAR over it
    p.add(a0, a0, t1);                        // +3 (the pre-WAR value)
    p.add(a0, a0, t0);                        // +4
    p.addi(s1, s1, 1);
    p.j("w_loop");
    p.label("w_done");
    exit_now(p);
    return {"waw_war", p.assemble(), 130, 200000};   // 10 × 13
}

inline const std::vector<Workload>& corpus() {
    static const std::vector<Workload> c = [] {
        std::vector<Workload> v;
        v.push_back(alu());
        v.push_back(loop());
        v.push_back(bubble_sort());
        v.push_back(matmul());
        v.push_back(sieve());
        v.push_back(store_forward());
        v.push_back(subword());
        v.push_back(nested_calls());
        v.push_back(fib());
        v.push_back(lcg_branch());
        v.push_back(muldiv());
        v.push_back(pointer_chase());
        v.push_back(waw_war());
        return v;
    }();
    return c;
}

}  // namespace wl

// ========================================================= differential run ===
// `diff_run` runs a workload on the reference and on the model under test,
// then compares all 32 architectural registers, the exit code, and the
// retired-instruction count.
//
// The model is a swappable function defaulting to the reference itself, so
// the comparison can be tested against a known-equal pair and against
// deliberately corrupted ones.

namespace diff {

// Neutral result type, so the comparison does not care whether an outcome
// came from the interpreter or the pipeline. The interpreter has no cycles
// to report and leaves that field zero.
struct Outcome {
    uint32_t regs[32] = {};
    uint32_t exit_code = 0;
    uint64_t retired   = 0;
    uint64_t cycles    = 0;
    bool     halted    = false;
    bool     trapped   = false;
    bool     budget    = false;
};

inline Outcome from_ref(const ref::Result& r) {
    Outcome o;
    for (int i = 0; i < 32; ++i) o.regs[i] = r.regs[i];
    o.exit_code = r.exit_code;
    o.retired   = r.retired;
    o.halted    = r.halted;
    o.trapped   = r.trapped;
    o.budget    = r.budget;
    return o;
}

inline Outcome run_reference(const wl::Workload& w, const Config&) {
    Memory m;
    for (std::size_t i = 0; i < w.words.size(); ++i) {
        m.store_u32(wl::TEXT + static_cast<uint32_t>(i * 4), w.words[i]);
    }
    ref::Options o;
    o.max_insts = w.budget;
    return from_ref(ref::run(m, wl::TEXT, o));
}

using Runner = std::function<Outcome(const wl::Workload&, const Config&)>;

// Swap in a pipeline-backed runner to compare against the interpreter.
inline Runner& model() {
    static Runner r = &run_reference;
    return r;
}

struct ScopedModel {
    Runner saved;
    explicit ScopedModel(Runner r) : saved(model()) { model() = std::move(r); }
    ~ScopedModel() { model() = saved; }
};

inline const char* abi_name(int i) {
    static const char* names[32] = {
        "zero", "ra", "sp", "gp", "tp",  "t0", "t1", "t2",
        "s0",   "s1", "a0", "a1", "a2",  "a3", "a4", "a5",
        "a6",   "a7", "s2", "s3", "s4",  "s5", "s6", "s7",
        "s8",   "s9", "s10","s11","t3",  "t4", "t5", "t6",
    };
    return names[i];
}

struct Report {
    bool        ok = true;
    std::string detail;
};

inline Report compare(const std::string& name, const Outcome& want, const Outcome& got) {
    std::ostringstream d;
    bool ok = true;

    auto note = [&](const char* what, unsigned long long w, unsigned long long g) {
        ok = false;
        d << "    " << name << ": " << what << " reference=" << w << " model=" << g << '\n';
    };

    if (want.halted  != got.halted)  note("halted",  want.halted,  got.halted);
    if (want.trapped != got.trapped) note("trapped", want.trapped, got.trapped);
    if (want.budget  != got.budget)  note("budget",  want.budget,  got.budget);
    if (want.exit_code != got.exit_code) note("exit code", want.exit_code, got.exit_code);
    if (want.retired   != got.retired)   note("retired",   want.retired,   got.retired);

    for (int i = 0; i < 32; ++i) {
        if (want.regs[i] == got.regs[i]) continue;
        ok = false;
        char buf[128];
        std::snprintf(buf, sizeof(buf),
                      "    %s: x%d (%s) reference=0x%08X model=0x%08X\n",
                      name.c_str(), i, abi_name(i), want.regs[i], got.regs[i]);
        d << buf;
    }

    return {ok, d.str()};
}

// Run `w` on both layers and compare. A reference run that did not halt
// cleanly is reported as a broken workload; comparing two runs that both
// fell off the end proves nothing.
inline Report diff_run(const wl::Workload& w, const Config& cfg) {
    const Outcome want = run_reference(w, cfg);
    if (!want.halted || want.trapped || want.budget) {
        std::ostringstream d;
        d << "    " << w.name << ": reference did not halt cleanly"
          << " (trapped=" << want.trapped << " budget=" << want.budget
          << " retired=" << want.retired << ")\n";
        return {false, d.str()};
    }
    return compare(w.name, want, model()(w, cfg));
}

}  // namespace diff

// ----------------------------------------------- @section("diff_scaffold") ---
SECTION("diff_scaffold") {
    const Config cfg;
    const auto&  corpus = wl::corpus();

    // ---- The corpus is thirteen distinctly named programs ------------------
    REQUIRE(corpus.size() == 13);
    for (std::size_t i = 0; i < corpus.size(); ++i) {
        for (std::size_t j = i + 1; j < corpus.size(); ++j) {
            REQUIRE(corpus[i].name != corpus[j].name);
        }
    }

    // ---- Every workload halts, and lands on its independently known result -
    // The printed retired counts are the denominator of every IPC number, so
    // a change in one is a signal in its own right.
    for (const wl::Workload& w : corpus) {
        const diff::Outcome o = diff::run_reference(w, cfg);
        REQUIRE_MSG(o.halted && !o.trapped && !o.budget,
                    "    " + w.name + ": did not halt cleanly");
        REQUIRE_MSG(o.retired > 0, "    " + w.name + ": retired nothing");
        if (w.expect_exit) {
            char buf[128];
            std::snprintf(buf, sizeof(buf), "    %s: exit expected=%u actual=%u\n",
                          w.name.c_str(), *w.expect_exit, o.exit_code);
            REQUIRE_MSG(o.exit_code == *w.expect_exit, buf);
        }
        std::printf("    %-14s %5zu words  %7llu retired  exit=%u\n",
                    w.name.c_str(), w.words.size(),
                    static_cast<unsigned long long>(o.retired), o.exit_code);
    }

    // ---- Determinism: the same program twice is bit-identical -------------
    // Nothing reads the clock and every tie-break is age- or index-ordered,
    // so a failure has to reproduce exactly.
    for (const wl::Workload& w : corpus) {
        const diff::Outcome a = diff::run_reference(w, cfg);
        const diff::Outcome b = diff::run_reference(w, cfg);
        REQUIRE_MSG(diff::compare(w.name, a, b).ok,
                    "    " + w.name + ": two runs disagreed");
    }

    // ---- Reference against reference --------------------------------------
    // Trivially true, which is the point: it exercises diff_run's plumbing
    // with nothing else that could be at fault.
    for (const wl::Workload& w : corpus) {
        const diff::Report r = diff::diff_run(w, cfg);
        REQUIRE_MSG(r.ok, r.detail);
    }

    // ---- Negative controls ------------------------------------------------
    // Corrupt one comparand at a time; diff_run must catch each and name it.
    const wl::Workload& probe = corpus.front();
    {
        diff::ScopedModel swap([](const wl::Workload& w, const Config& c) {
            diff::Outcome o = diff::run_reference(w, c);
            o.regs[15] ^= 1u;                       // a5
            return o;
        });
        const diff::Report r = diff::diff_run(probe, cfg);
        REQUIRE(!r.ok);
        REQUIRE(r.detail.find("a5") != std::string::npos);
    }
    {
        diff::ScopedModel swap([](const wl::Workload& w, const Config& c) {
            diff::Outcome o = diff::run_reference(w, c);
            o.exit_code += 1;
            return o;
        });
        const diff::Report r = diff::diff_run(probe, cfg);
        REQUIRE(!r.ok);
        REQUIRE(r.detail.find("exit code") != std::string::npos);
    }
    {
        // The wrong-path-retire signal: right registers, too many commits.
        diff::ScopedModel swap([](const wl::Workload& w, const Config& c) {
            diff::Outcome o = diff::run_reference(w, c);
            o.retired += 1;
            return o;
        });
        const diff::Report r = diff::diff_run(probe, cfg);
        REQUIRE(!r.ok);
        REQUIRE(r.detail.find("retired") != std::string::npos);
    }
    {
        diff::ScopedModel swap([](const wl::Workload& w, const Config& c) {
            diff::Outcome o = diff::run_reference(w, c);
            o.halted  = false;
            o.trapped = true;
            return o;
        });
        const diff::Report r = diff::diff_run(probe, cfg);
        REQUIRE(!r.ok);
        REQUIRE(r.detail.find("trapped") != std::string::npos);
    }

    // The swap is scoped: the default runner is back.
    REQUIRE(diff::diff_run(probe, cfg).ok);

    // ---- A broken workload is reported as such, not silently passed -------
    {
        wl::Workload spinner;
        spinner.name   = "spinner";
        spinner.budget = 20;
        asmc::Assembler p;
        p.label("spin");
        p.j("spin");
        spinner.words = p.assemble();

        const diff::Report r = diff::diff_run(spinner, cfg);
        REQUIRE(!r.ok);
        REQUIRE(r.detail.find("did not halt cleanly") != std::string::npos);
    }
}


// ---------------------------------------------------- @section("cpu_tick") ---
namespace cputest {

using namespace asmc;

inline Memory image(const std::vector<uint32_t>& words) {
    Memory m;
    for (std::size_t i = 0; i < words.size(); ++i) {
        m.store_u32(wl::TEXT + static_cast<uint32_t>(i * 4), words[i]);
    }
    return m;
}

// The pipeline model dressed as a diff::Runner.
inline diff::Outcome run_cpu(const wl::Workload& w, const Config& cfg) {
    Memory m = image(w.words);
    Cpu cpu(m, cfg, wl::TEXT);
    cpu.run(w.budget * 8 + 1000);             // cycles, generously bounded

    diff::Outcome o;
    for (int i = 0; i < 32; ++i) o.regs[i] = cpu.reg(static_cast<ArchReg>(i));
    o.exit_code = cpu.exit_code();
    o.retired   = cpu.retired();
    o.cycles    = cpu.cycle();
    o.halted    = cpu.halted();
    o.trapped   = cpu.trapped();
    o.budget    = !cpu.done();
    return o;
}

}  // namespace cputest

SECTION("cpu_tick") {
    using namespace asmc;

    // Stage timing is clearest one instruction at a time; what width changes
    // is @section("frontend")'s subject.
    Config cfg;
    cfg.width = 1;

    // ---- A fresh machine has nothing in flight ----------------------------
    {
        Memory m;
        Cpu cpu(m, cfg, wl::TEXT);
        REQUIRE(cpu.cycle() == 0);
        REQUIRE(cpu.retired() == 0);
        REQUIRE(cpu.idle());
        REQUIRE(!cpu.done());
        REQUIRE(cpu.fetch_pc() == wl::TEXT);
        REQUIRE(cpu.arch_pc()  == wl::TEXT);
        for (int i = 0; i < 32; ++i) REQUIRE(cpu.reg(static_cast<ArchReg>(i)) == 0);
    }

    // ---- One addi, cycle by cycle -----------------------------------------
    // Fetched in cycle 1, decoded in 2, executed in 3, written back in 4,
    // retired in 5. Writeback and commit are separate stages, so the register
    // write lands one cycle before the retirement.
    {
        Assembler p;
        p.addi(a0, zero, 42);
        p.li(a7, 93);
        p.ecall();
        Memory m = cputest::image(p.assemble());
        Cpu cpu(m, cfg, wl::TEXT);

        cpu.tick();                                   // cycle 1: Fetch
        REQUIRE(!cpu.idle());
        REQUIRE(cpu.reg(a0) == 0);
        REQUIRE(cpu.fetch_pc() == wl::TEXT + 4);

        cpu.tick();                                   // cycle 2: Decode
        cpu.tick();                                   // cycle 3: Execute
        REQUIRE(cpu.reg(a0) == 0);                    // nothing written yet
        REQUIRE(cpu.retired() == 0);

        cpu.tick();                                   // cycle 4: Writeback
        REQUIRE(cpu.reg(a0) == 42);
        REQUIRE(cpu.retired() == 0);                  // written, not yet retired
        REQUIRE(cpu.arch_pc() == wl::TEXT);           // commit has not moved it

        cpu.tick();                                   // cycle 5: Commit
        REQUIRE(cpu.retired() == 1);
        REQUIRE(cpu.arch_pc() == wl::TEXT + 4);
        REQUIRE(cpu.commit_in_order());
        REQUIRE(!cpu.done());
    }

    // ---- Steady state: one instruction per cycle --------------------------
    // With no stalls, N instructions retire in N + 4 cycles; the pipeline
    // fill is the only overhead.
    {
        Assembler p;
        for (int i = 0; i < 100; ++i) p.nop();
        p.li(a0, 0);
        p.li(a7, 93);
        p.ecall();
        Memory m = cputest::image(p.assemble());
        Cpu cpu(m, cfg, wl::TEXT);
        REQUIRE(cpu.run(1000));

        REQUIRE(cpu.halted());
        REQUIRE(!cpu.trapped());
        REQUIRE(cpu.retired() == 103);
        REQUIRE(cpu.cycle()   == cpu.retired() + 4);
        REQUIRE(cpu.commit_in_order());

        // 100 nops changed no architectural state.
        REQUIRE(cpu.reg(a0) == 0);
        REQUIRE(cpu.reg(a7) == 93);
        for (int i = 0; i < 32; ++i) {
            if (i == a7) continue;
            REQUIRE(cpu.reg(static_cast<ArchReg>(i)) == 0);
        }
    }

    // ---- Nothing behind a halt may retire ---------------------------------
    // Both instructions after the ecall would trap if they reached commit,
    // so a clean halt means the squash worked.
    {
        Assembler p;
        p.addi(a0, zero, 42);
        p.li(a7, 93);
        p.ecall();
        p.sw(a0, zero, 0);                            // would corrupt memory
        p.ebreak();                                   // would trap
        Memory m = cputest::image(p.assemble());
        Cpu cpu(m, cfg, wl::TEXT);
        REQUIRE(cpu.run(1000));

        REQUIRE(cpu.halted());
        REQUIRE(!cpu.trapped());
        REQUIRE(cpu.exit_code() == 42);
        REQUIRE(cpu.retired() == 3);
        REQUIRE(cpu.cycle()   == 7);
        REQUIRE(cpu.idle());                          // the squash emptied the latches
    }

    // ---- A machine with no program traps, then makes no further progress --
    {
        Memory m;                                     // all zeros: illegal encoding
        Cpu cpu(m, cfg, wl::TEXT);
        REQUIRE(cpu.run(1000));

        REQUIRE(cpu.trapped());
        REQUIRE(!cpu.halted());
        REQUIRE(cpu.trap_cause() == TrapCause::ILLEGAL);
        REQUIRE(cpu.retired() == 1);
        REQUIRE(cpu.cycle()   == 5);

        // Ticking a finished machine is a no-op.
        const uint64_t cycle = cpu.cycle();
        const uint64_t retired = cpu.retired();
        for (int i = 0; i < 1000; ++i) cpu.tick();
        REQUIRE(cpu.cycle()   == cycle);
        REQUIRE(cpu.retired() == retired);
    }

    // ---- One instruction of every class, in one program -------------------
    {
        Assembler p;
        p.li(t0, 7);
        p.li(t1, 6);
        p.mul(t2, t0, t1);                            // 42, on the mul unit
        p.div_(t3, t2, t1);                           // 7, on the div unit
        p.li(t4, 0x2000);
        p.sw(t2, t4, 0);
        p.lw(a0, t4, 0);                              // reads the store back
        p.fence();
        p.beq(a0, t2, "ok");
        p.li(a0, 0);                                  // skipped if the branch works
        p.label("ok");
        p.li(a7, 93);
        p.ecall();
        Memory m = cputest::image(p.assemble());
        Cpu cpu(m, cfg, wl::TEXT);
        REQUIRE(cpu.run(10000));

        REQUIRE(cpu.halted());
        REQUIRE(!cpu.trapped());
        REQUIRE(cpu.exit_code() == 42);
        REQUIRE(cpu.reg(t3) == 7);
        REQUIRE(m.load_u32(0x2000) == 42);            // the store reached memory
        REQUIRE(cpu.commit_in_order());
    }
    {
        Assembler p;
        p.li(a7, 42);                                 // not the exit syscall
        p.ecall();
        Memory m = cputest::image(p.assemble());
        Cpu cpu(m, cfg, wl::TEXT);
        REQUIRE(cpu.run(1000));
        REQUIRE(cpu.trapped());
        REQUIRE(cpu.trap_cause() == TrapCause::ECALL_UNKNOWN);
    }

    // ---- First differential pass ------------------------------------------
    // The `alu` workload is pure ALU plus the exit ecall, the only one the
    // pipeline can run so far. Pins the stage plumbing, the x0 rule, and the
    // ecall path against ref.h on all 32 registers.
    {
        const wl::Workload* alu_wl = nullptr;
        for (const wl::Workload& w : wl::corpus()) {
            if (w.name == "alu") alu_wl = &w;
        }
        REQUIRE(alu_wl != nullptr);
        if (alu_wl) {
            diff::ScopedModel swap(&cputest::run_cpu);
            const diff::Report r = diff::diff_run(*alu_wl, cfg);
            REQUIRE_MSG(r.ok, r.detail);

            const diff::Outcome o = cputest::run_cpu(*alu_wl, cfg);
            REQUIRE(o.cycles == o.retired + 4);
        }
    }
}


// ---------------------------------------------------- @section("frontend") ---
namespace fetest {

// N back-to-back addis, then the exit sequence.
inline std::vector<uint32_t> addi_chain(int n) {
    asmc::Assembler p;
    for (int i = 0; i < n; ++i) p.addi(asmc::t0, asmc::zero, i);
    p.li(asmc::a7, 93);
    p.ecall();
    return p.assemble();
}

}  // namespace fetest

SECTION("frontend") {
    using namespace asmc;

    // ---- 20 addis decode in program order, `width` per cycle --------------
    // Nothing here stalls, so decode runs at full width from the cycle the
    // first bundle lands until the program runs out.
    for (uint32_t width : {1u, 2u, 4u}) {
        Config cfg;
        cfg.width   = width;
        cfg.num_alu = width;     // so the back end is never the bottleneck
        cfg.num_cdb = width;
        Memory m = cputest::image(fetest::addi_chain(20));
        Cpu cpu(m, cfg, wl::TEXT);
        cpu.record_decode(true);
        REQUIRE(cpu.run(2000));

        const std::vector<Cpu::DecodeRecord>& log = cpu.decode_log();
        const std::string tag = "    width=" + std::to_string(width) + ": ";

        // Program order, no gaps, no repeats: 20 addis plus li + ecall.
        REQUIRE_MSG(log.size() == 22, tag + "decoded " + std::to_string(log.size()));
        for (std::size_t i = 0; i < log.size(); ++i) {
            REQUIRE_MSG(log[i].seq == i, tag + "seq out of order at " + std::to_string(i));
            REQUIRE_MSG(log[i].pc == wl::TEXT + 4 * static_cast<uint32_t>(i),
                        tag + "pc out of order at " + std::to_string(i));
        }

        // Decode never exceeds width in a cycle, and reaches it while the
        // front end is running flat out.
        std::vector<uint32_t> per_cycle(cpu.cycle() + 2, 0);
        for (const Cpu::DecodeRecord& r : log) ++per_cycle[r.cycle];
        uint32_t peak = 0;
        for (uint32_t c : per_cycle) {
            REQUIRE_MSG(c <= width, tag + "decoded more than width in one cycle");
            peak = std::max(peak, c);
        }
        REQUIRE_MSG(peak == width, tag + "never reached full width");

        // The whole chain is decoded in the ceiling of 22/width cycles, plus
        // the cycle fetch spends filling the first bundle.
        const uint64_t span = log.back().cycle - log.front().cycle + 1;
        REQUIRE_MSG(span == (22 + width - 1) / width,
                    tag + "decode took " + std::to_string(span) + " cycles");
    }

    // ---- Fetch runs ahead of decode, then back-pressures ------------------
    // A 20-cycle divide jams issue, so the dispatch queue fills, then decode
    // stops, then the fetch queue fills, and only then does fetch stop. Every
    // stage stalls because its consumer did, not on a timer.
    {
        Config cfg;
        cfg.width = 1;
        Assembler p;
        p.li(t0, 1000);
        p.li(t1, 3);
        p.div_(t2, t0, t1);            // 20 cycles, blocking
        p.add(t3, t2, t2);             // waits on the divide
        for (int i = 0; i < 10; ++i) p.nop();
        p.li(a7, 93);
        p.ecall();
        Memory m = cputest::image(p.assemble());
        Cpu cpu(m, cfg, wl::TEXT);

        bool saw_full_dispatch = false;
        bool saw_full_fetch    = false;
        for (int i = 0; i < 40 && !cpu.done(); ++i) {
            cpu.tick();
            REQUIRE(cpu.dispatch_queue() <= cpu.dispatch_queue_capacity());
            REQUIRE(cpu.fetch_queue()    <= cpu.fetch_queue_capacity());
            if (cpu.dispatch_queue() == cpu.dispatch_queue_capacity()) {
                saw_full_dispatch = true;
                // Decode cannot have drained into a full queue, so the fetch
                // queue is what absorbs the next bundles.
                if (cpu.fetch_queue() == cpu.fetch_queue_capacity()) saw_full_fetch = true;
            }
        }
        REQUIRE(saw_full_dispatch);
        REQUIRE(saw_full_fetch);

        // Back-pressure only stalls; it never drops or duplicates work.
        REQUIRE(cpu.run(2000));
        REQUIRE(cpu.halted());
        REQUIRE(cpu.retired() == 16);
        REQUIRE(cpu.commit_in_order());
    }

    // ---- No wrong-path fetch: decode stops at a control transfer ----------
    {
        Config cfg;
        cfg.width = 2;
        Assembler p;
        p.j("target");
        for (int i = 0; i < 8; ++i) p.ebreak();        // never fetched
        p.label("target");
        p.li(a0, 5);
        p.li(a7, 93);
        p.ecall();
        Memory m = cputest::image(p.assemble());
        Cpu cpu(m, cfg, wl::TEXT);
        cpu.record_decode(true);
        REQUIRE(cpu.run(1000));

        REQUIRE(cpu.halted());
        REQUIRE(cpu.exit_code() == 5);
        for (const Cpu::DecodeRecord& r : cpu.decode_log()) {
            REQUIRE(r.raw != 0x00100073u);             // no ebreak ever decoded
        }
        REQUIRE(cpu.decode_log().size() == 4);         // jal, li, li, ecall
    }

    // ---- A taken branch costs one fetch bubble ----------------------------
    // Decode stalls the front end in the cycle it sees the jump; execute
    // redirects the PC and, running earlier in the same cycle than fetch,
    // lets fetch resume from the target immediately.
    {
        Config cfg;
        cfg.width = 1;
        Assembler p;
        p.j("target");
        p.label("target");
        p.li(a7, 93);
        p.ecall();
        Memory m = cputest::image(p.assemble());
        Cpu cpu(m, cfg, wl::TEXT);
        cpu.record_decode(true);
        REQUIRE(cpu.run(1000));

        const std::vector<Cpu::DecodeRecord>& log = cpu.decode_log();
        REQUIRE(log.size() == 3);
        REQUIRE(log[1].cycle == log[0].cycle + 2);     // one bubble after the jump
        REQUIRE(log[2].cycle == log[1].cycle + 1);     // and none after that
    }
}


// --------------------------------------------- @section("execute_inorder") ---
SECTION("execute_inorder") {
    using namespace asmc;

    // ---- Dependent ALU ops issue back to back -----------------------------
    // Writeback runs before execute in the same cycle, so a result frees its
    // register in time for the consumer to issue with no bubble between them.
    {
        Config cfg;
        cfg.width = 1;
        Assembler p;
        p.li(t0, 0);
        for (int i = 0; i < 200; ++i) p.addi(t0, t0, 1);
        p.mv(a0, t0);
        p.li(a7, 93);
        p.ecall();
        Memory m = cputest::image(p.assemble());
        Cpu cpu(m, cfg, wl::TEXT);
        REQUIRE(cpu.run(2000));

        REQUIRE(cpu.exit_code() == 200);
        REQUIRE(cpu.retired() == 204);
        REQUIRE(cpu.cycle() == cpu.retired() + 4);   // no stall anywhere
        REQUIRE(cpu.cycle() < 260);
    }

    // ---- Load-use latency tracks mem_latency exactly ----------------------
    {
        auto cycles_at = [](uint32_t lat) {
            Config cfg;
            cfg.width       = 1;
            cfg.mem_latency = lat;
            Assembler p;
            p.li(t0, 0x2000);
            p.li(t1, 7);
            p.sw(t1, t0, 0);
            p.lw(a0, t0, 0);
            p.add(a1, a0, a0);                       // consumes the loaded value
            p.li(a7, 93);
            p.ecall();
            Memory m = cputest::image(p.assemble());
            Cpu cpu(m, cfg, wl::TEXT);
            cpu.run(1000);
            return cpu.cycle();
        };
        REQUIRE(cycles_at(3) == cycles_at(2) + 1);
        REQUIRE(cycles_at(4) == cycles_at(2) + 2);
        REQUIRE(cycles_at(8) == cycles_at(2) + 6);
    }

    // ---- A pipelined unit overlaps its ops; a blocking one does not -------
    {
        const uint32_t dst[3] = {t2, t3, t4};
        auto cycles_for = [&dst](int n, bool use_div) {
            Config cfg;
            cfg.width = 1;
            Assembler p;
            p.li(t0, 100);
            p.li(t1, 7);
            for (int i = 0; i < n; ++i) {            // independent, same sources
                if (use_div) p.div_(dst[i], t0, t1);
                else         p.mul (dst[i], t0, t1);
            }
            p.li(a7, 93);
            p.ecall();
            Memory m = cputest::image(p.assemble());
            Cpu cpu(m, cfg, wl::TEXT);
            cpu.run(1000);
            return cpu.cycle();
        };
        const Config def;
        REQUIRE(cycles_for(3, false) == cycles_for(1, false) + 2);
        REQUIRE(cycles_for(3, true)  == cycles_for(1, true) + 2 * def.div_latency);
    }

    // ---- Results reach the register file in issue order -------------------
    // The add finishes 19 cycles before the divide it sits behind, and still
    // may not write first — the property that makes WAW safe unrenamed.
    {
        Config cfg;
        cfg.width = 1;
        Assembler p;
        p.li(t0, 100);
        p.li(t1, 7);
        p.div_(t2, t0, t1);                          // 14, after 20 cycles
        p.addi(t3, t1, 1);                           // 8, ready at once
        p.li(a7, 93);
        p.ecall();
        Memory m = cputest::image(p.assemble());
        Cpu cpu(m, cfg, wl::TEXT);

        bool add_wrote_first = false;
        while (!cpu.done() && cpu.cycle() < 200) {
            cpu.tick();
            if (cpu.reg(t3) != 0 && cpu.reg(t2) == 0) add_wrote_first = true;
        }
        REQUIRE(cpu.reg(t2) == 14);
        REQUIRE(cpu.reg(t3) == 8);
        REQUIRE(!add_wrote_first);
    }

    // ---- The whole corpus, against the interpreter, on every config -------
    // Same registers, same exit code, same retired count as ref.h. A machine
    // that is only correct at one width is not correct.
    {
        struct Named { const char* name; Config cfg; };
        std::vector<Named> configs;
        {
            Config c; c.width = 1; c.num_alu = 1; c.num_cdb = 1;
            configs.push_back({"1-wide", c});
        }
        configs.push_back({"default", Config{}});
        {
            Config c; c.width = 4; c.num_alu = 4; c.num_cdb = 4; c.rob_size = 64;
            configs.push_back({"4-wide", c});
        }
        {
            Config c; c.rob_size = 4; c.iq_size = 2; c.lq_size = 1; c.sq_size = 1;
            configs.push_back({"rob=4", c});
        }
        {
            Config c; c.mul_latency = 7; c.div_latency = 33; c.mem_latency = 5;
            configs.push_back({"slow-fu", c});
        }

        diff::ScopedModel swap(&cputest::run_cpu);
        for (const Named& n : configs) {
            for (const wl::Workload& w : wl::corpus()) {
                const diff::Report r = diff::diff_run(w, n.cfg);
                REQUIRE_MSG(r.ok, std::string("    config ") + n.name + "\n" + r.detail);
            }
        }
    }
}


// ---------------------------------------------- @section("commit_inorder") ---
SECTION("commit_inorder") {
    using namespace asmc;
    Config cfg;
    cfg.width = 1;

    // ---- A store reaches memory at commit, not at execute -----------------
    {
        Assembler p;
        p.li(t0, 0x400);                             // fits an addi, so one word
        p.li(t1, 0xAB);
        p.sw(t1, t0, 0);                             // third instruction
        p.li(a7, 93);
        p.ecall();
        Memory m = cputest::image(p.assemble());
        Cpu cpu(m, cfg, wl::TEXT);

        uint64_t wrote_at = 0;
        while (!cpu.done() && cpu.cycle() < 100) {
            cpu.tick();
            if (wrote_at == 0 && m.load_u32(0x400) != 0) wrote_at = cpu.cycle();
        }
        REQUIRE(m.load_u32(0x400) == 0xABu);
        REQUIRE(wrote_at == 7);                      // its commit cycle, not its 5th
        REQUIRE(cpu.retired() == 5);
    }

    // ---- Memory writes appear in program order ----------------------------
    // Checked every cycle, not just at the end: the set of written words must
    // always be a prefix of the program's stores.
    {
        Assembler p;
        p.li(t0, 0x3000);
        p.li(t1, 1);  p.sw(t1, t0, 0);
        p.li(t2, 2);  p.sw(t2, t0, 4);
        p.li(t3, 3);  p.sw(t3, t0, 8);
        p.li(a7, 93);
        p.ecall();
        Memory m = cputest::image(p.assemble());
        Cpu cpu(m, cfg, wl::TEXT);

        while (!cpu.done() && cpu.cycle() < 200) {
            cpu.tick();
            uint32_t written = 0;
            while (written < 3 && m.load_u32(0x3000 + 4 * written) != 0) ++written;
            for (uint32_t k = written; k < 3; ++k) {
                REQUIRE(m.load_u32(0x3000 + 4 * k) == 0);
            }
        }
        REQUIRE(m.load_u32(0x3000) == 1);
        REQUIRE(m.load_u32(0x3004) == 2);
        REQUIRE(m.load_u32(0x3008) == 3);
    }

    // ---- A load sees an older store -----------------------------------
    // Stores commit late, so the load has to wait for one; reading memory
    // early would return the stale word.
    {
        Assembler p;
        p.li(t0, 0x4000);
        p.li(t1, 0x11);
        p.sw(t1, t0, 0);
        p.lw(a0, t0, 0);
        p.li(a7, 93);
        p.ecall();
        Memory m = cputest::image(p.assemble());
        Cpu cpu(m, cfg, wl::TEXT);
        REQUIRE(cpu.run(1000));
        REQUIRE(cpu.halted());
        REQUIRE(cpu.exit_code() == 0x11u);
    }

    // ---- Nothing retires out of order, and the count matches ref.h --------
    // The store-heavy workloads are the ones where a commit-order bug would
    // hide, so they are worth naming rather than folding into the sweep.
    {
        diff::ScopedModel swap(&cputest::run_cpu);
        for (const char* name : {"store_forward", "subword", "bubble_sort", "nested_calls"}) {
            for (const wl::Workload& w : wl::corpus()) {
                if (w.name != name) continue;
                const diff::Report r = diff::diff_run(w, cfg);
                REQUIRE_MSG(r.ok, r.detail);

                const diff::Outcome o = cputest::run_cpu(w, cfg);
                const diff::Outcome ref = diff::run_reference(w, cfg);
                REQUIRE_MSG(o.retired == ref.retired, std::string("    ") + name +
                            ": retired " + std::to_string(o.retired) + " vs " +
                            std::to_string(ref.retired));
            }
        }
    }
}


// --------------------------------------------------------- @section("rob") ---
namespace robtest {

// An entry whose fields all derive from `pc`, so a test can tell which
// instruction it got back.
inline RobEntry entry(uint32_t pc, ArchReg dest = INVALID_ARCHREG) {
    RobEntry e;
    e.pc         = pc;
    e.next_pc    = pc + 4;
    e.dest_arch  = dest;
    e.dest_phys  = (dest == INVALID_ARCHREG) ? INVALID_PHYSREG : 100 + dest;
    e.stale_phys = (dest == INVALID_ARCHREG) ? INVALID_PHYSREG : 200 + dest;
    return e;
}

inline RobEntry done_entry(uint32_t pc) {
    RobEntry e = entry(pc);
    e.complete = true;
    return e;
}

}  // namespace robtest

SECTION("rob") {
    using robtest::done_entry;
    using robtest::entry;

    // ---- A fresh buffer is empty and sized from the config ----------------
    {
        Config cfg;
        Rob rob(cfg);
        REQUIRE(rob.capacity() == cfg.rob_size);
        REQUIRE(rob.size() == 0);
        REQUIRE(rob.empty());
        REQUIRE(!rob.full());
        REQUIRE(rob.next_seq() == 0);
        REQUIRE(rob.head() == INVALID_ROBINDEX);
        REQUIRE(rob.nth(0) == INVALID_ROBINDEX);
        REQUIRE(!rob.in_flight(0));
        REQUIRE(!rob.pop_head_if_complete().has_value());
        REQUIRE(rob.squash_all().empty());

        Config small = cfg;
        small.rob_size = 4;
        REQUIRE(Rob(small).capacity() == 4);
    }

    // ---- Filling: program order, monotonic seq, full at capacity ----------
    {
        Rob rob(8);
        for (uint32_t i = 0; i < 8; ++i) {
            const RobIndex idx = rob.allocate(entry(0x1000 + 4 * i, i + 1));
            REQUIRE(idx == i);
            REQUIRE(rob.at(idx).seq == i);
            REQUIRE(rob.size() == i + 1);
            REQUIRE(rob.head() == 0);
            REQUIRE(rob.in_flight(idx));
        }
        REQUIRE(rob.full());
        REQUIRE(rob.next_seq() == 8);
        REQUIRE(rob.tail() == rob.head());          // wrapped all the way round

        // The allocated payload survives untouched.
        REQUIRE(rob.at(3).pc == 0x100Cu);
        REQUIRE(rob.at(3).next_pc == 0x1010u);
        REQUIRE(rob.at(3).dest_arch == 4u);
        REQUIRE(rob.at(3).stale_phys == 204u);
        REQUIRE(!rob.at(3).complete);

        // nth() and age_of() are inverses over the live range.
        for (uint32_t k = 0; k < rob.size(); ++k) REQUIRE(rob.age_of(rob.nth(k)) == k);
    }

    // ---- Commit waits for the head, however the completions arrive --------
    {
        Rob rob(8);
        for (uint32_t i = 0; i < 4; ++i) rob.allocate(entry(0x1000 + 4 * i));

        rob.nth_entry(1).complete = true;
        rob.nth_entry(2).complete = true;
        REQUIRE(!rob.pop_head_if_complete().has_value());   // head still running
        REQUIRE(rob.size() == 4);

        rob.nth_entry(0).complete = true;
        const std::optional<RobEntry> first = rob.pop_head_if_complete();
        REQUIRE(first.has_value());
        REQUIRE(first->seq == 0);
        REQUIRE(rob.size() == 3);

        // The two already-complete entries now drain back to back, and the
        // fourth still blocks.
        REQUIRE(rob.pop_head_if_complete()->seq == 1);
        REQUIRE(rob.pop_head_if_complete()->seq == 2);
        REQUIRE(!rob.pop_head_if_complete().has_value());
        REQUIRE(rob.size() == 1);
    }

    // ---- FIFO order holds across every wrap boundary ----------------------
    {
        Rob rob(8);
        SeqNum   next_out  = 0;
        uint32_t allocated = 0;
        for (int i = 0; i < 200; ++i) {
            for (int k = 0; k < 2 && !rob.full(); ++k) {
                rob.allocate(done_entry(0x1000 + 4 * allocated++));
            }
            if (const std::optional<RobEntry> out = rob.pop_head_if_complete()) {
                REQUIRE(out->seq == next_out);
                REQUIRE(out->pc  == 0x1000u + 4 * next_out);
                ++next_out;
            }
        }
        while (const std::optional<RobEntry> out = rob.pop_head_if_complete()) {
            REQUIRE(out->seq == next_out++);
        }
        REQUIRE(rob.empty());
        REQUIRE(next_out == allocated);
        REQUIRE(allocated > 4 * rob.capacity());    // many laps, not one
    }

    // ---- truncate_to keeps its argument and everything older --------------
    {
        Rob rob(8);
        for (uint32_t i = 0; i < 8; ++i) rob.allocate(entry(0x1000 + 4 * i));

        const RobIndex branch = rob.nth(3);
        const std::vector<RobEntry> killed = rob.truncate_to(branch);

        // Youngest first: the order recovery frees physical registers in.
        REQUIRE(killed.size() == 4);
        REQUIRE(killed[0].seq == 7);
        REQUIRE(killed[1].seq == 6);
        REQUIRE(killed[2].seq == 5);
        REQUIRE(killed[3].seq == 4);

        REQUIRE(rob.size() == 4);
        REQUIRE(rob.head() == 0);
        REQUIRE(rob.at(branch).seq == 3);
        REQUIRE(rob.in_flight(branch));
        REQUIRE(!rob.in_flight(rob.capacity() - 1));

        // The tail is restored, so the next allocation lands right behind the
        // survivor — with a fresh sequence number, never a reused one.
        REQUIRE(rob.tail() == 4u);
        REQUIRE(!rob.full());
        const RobIndex refill = rob.allocate(entry(0x2000));
        REQUIRE(refill == 4u);
        REQUIRE(rob.at(refill).seq == 8);
        REQUIRE(rob.nth(4) == refill);
    }

    // ---- The same, with the live range straddling the end of storage ------
    {
        Rob rob(8);
        for (uint32_t i = 0; i < 5; ++i) rob.allocate(done_entry(0x1000 + 4 * i));
        for (uint32_t i = 0; i < 5; ++i) rob.pop_head_if_complete();
        REQUIRE(rob.empty());
        REQUIRE(rob.tail() == 5u);

        for (uint32_t i = 0; i < 6; ++i) rob.allocate(entry(0x2000 + 4 * i));
        REQUIRE(rob.head() == 5u);                  // slots 5,6,7,0,1,2 are live
        REQUIRE(rob.nth(3) == 0u);

        const std::vector<RobEntry> killed = rob.truncate_to(rob.nth(3));
        REQUIRE(killed.size() == 2);
        REQUIRE(killed[0].seq == 10);
        REQUIRE(killed[1].seq == 9);
        REQUIRE(rob.size() == 4);
        REQUIRE(rob.head() == 5u);
        REQUIRE(rob.tail() == 1u);
        REQUIRE(rob.nth_entry(3).seq == 8);
    }

    // ---- Truncation edge cases --------------------------------------------
    {
        Rob rob(8);
        for (uint32_t i = 0; i < 5; ++i) rob.allocate(entry(0x1000 + 4 * i));

        // Truncating to the youngest entry squashes nothing.
        REQUIRE(rob.truncate_to(rob.nth(4)).empty());
        REQUIRE(rob.size() == 5);

        // An index that is not live squashes nothing either: one past the
        // tail, and one out of range entirely.
        REQUIRE(rob.truncate_to(5).empty());
        REQUIRE(rob.truncate_to(rob.capacity()).empty());
        REQUIRE(rob.truncate_to(INVALID_ROBINDEX).empty());
        REQUIRE(rob.size() == 5);

        // Truncating to the head leaves exactly the head.
        const std::vector<RobEntry> killed = rob.truncate_to(rob.head());
        REQUIRE(killed.size() == 4);
        REQUIRE(killed.front().seq == 4);
        REQUIRE(killed.back().seq == 1);
        REQUIRE(rob.size() == 1);
        REQUIRE(rob.nth_entry(0).seq == 0);
    }

    // ---- squash_all empties the buffer, youngest first --------------------
    {
        Rob rob(4);
        for (uint32_t i = 0; i < 4; ++i) rob.allocate(entry(0x1000 + 4 * i));
        REQUIRE(rob.full());

        const std::vector<RobEntry> killed = rob.squash_all();
        REQUIRE(killed.size() == 4);
        for (uint32_t i = 0; i < 4; ++i) REQUIRE(killed[i].seq == 3 - i);
        REQUIRE(rob.empty());
        REQUIRE(rob.head() == INVALID_ROBINDEX);

        // Sequence numbers carry on past the squash.
        const RobIndex idx = rob.allocate(entry(0x2000));
        REQUIRE(rob.head() == idx);
        REQUIRE(rob.at(idx).seq == 4);
    }

    // ---- A 4-entry ROB is the structural-hazard config ---------------------
    {
        Config cfg;
        cfg.rob_size = 4;
        Rob rob(cfg);
        for (uint32_t i = 0; i < 4; ++i) {
            REQUIRE(!rob.full());
            rob.allocate(entry(0x1000 + 4 * i));
        }
        REQUIRE(rob.full());                        // dispatch stalls here
        rob.nth_entry(0).complete = true;
        REQUIRE(rob.pop_head_if_complete().has_value());
        REQUIRE(!rob.full());                       // and unstalls on one commit
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
