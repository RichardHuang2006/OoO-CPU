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
#include "prf.h"
#include "freelist.h"
#include "rat.h"
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
    { CliOpts o; REQUIRE(parse({"oooc", "--ref"},   o) == 0); REQUIRE(o.use_ref); }
    { CliOpts o; REQUIRE(parse({"oooc", "--stats"}, o) == 0); REQUIRE(o.show_stats); }
    { CliOpts o; REQUIRE(parse({"oooc", "--ipc-table"}, o) == 0); REQUIRE(o.ipc_table); }

    // The pipeline is what runs unless the interpreter is asked for by name.
    { CliOpts o; REQUIRE(parse({"oooc", "prog.elf"}, o) == 0); REQUIRE(!o.use_ref); }

    // Budgets, in both units.
    { CliOpts o; REQUIRE(parse({"oooc", "--max-insts", "500"}, o) == 0);
      REQUIRE(o.max_insts == 500); }
    { CliOpts o; REQUIRE(parse({"oooc", "--max-cycles=900"}, o) == 0);
      REQUIRE(o.max_cycles == 900); }
    { CliOpts o; REQUIRE(parse({"oooc", "--max-cycles", "nope"}, o) != 0); }

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


#include "workloads.h"   // the validation corpus, shared with tools/


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

    // ---- The corpus is fourteen distinctly named programs ------------------
    REQUIRE(corpus.size() == 14);
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
    // Fetch, decode, rename, dispatch, issue, writeback, commit — one cycle
    // each. The value exists in the physical register file after writeback,
    // but reading a0 only follows the committed mapping, so it appears at
    // commit and not a cycle earlier.
    {
        Assembler p;
        p.addi(a0, zero, 42);
        p.li(a7, 93);
        p.ecall();
        Memory m = cputest::image(p.assemble());
        Cpu cpu(m, cfg, wl::TEXT);
        cpu.record_rename(true);

        cpu.tick();                                   // cycle 1: Fetch
        REQUIRE(!cpu.idle());
        REQUIRE(cpu.reg(a0) == 0);
        REQUIRE(cpu.fetch_pc() == wl::TEXT + 4);

        cpu.tick();                                   // cycle 2: Decode
        REQUIRE(cpu.rename_log().empty());

        cpu.tick();                                   // cycle 3: Rename
        REQUIRE(cpu.rename_log().size() == 1);
        const PhysReg dest = cpu.rename_log()[0].dest;
        REQUIRE(dest >= 32);                          // off the free list
        REQUIRE(!cpu.prf().is_ready(dest));           // owes a value

        cpu.tick();                                   // cycle 4: Dispatch
        REQUIRE(cpu.iq().size() == 1);

        cpu.tick();                                   // cycle 5: Issue
        REQUIRE(cpu.iq().size() == 1);                // the addi left, li took its place
        REQUIRE(cpu.iq().entries()[0].seq == 1);
        REQUIRE(!cpu.prf().is_ready(dest));           // still in the unit

        cpu.tick();                                   // cycle 6: Writeback
        REQUIRE(cpu.prf().is_ready(dest));
        REQUIRE(cpu.prf().read(dest) == 42);
        REQUIRE(cpu.reg(a0) == 0);                    // not architectural yet
        REQUIRE(cpu.retired() == 0);
        REQUIRE(cpu.arch_pc() == wl::TEXT);           // commit has not moved it

        cpu.tick();                                   // cycle 7: Commit
        REQUIRE(cpu.reg(a0) == 42);
        REQUIRE(cpu.committed_map(a0) == dest);
        REQUIRE(cpu.retired() == 1);
        REQUIRE(cpu.arch_pc() == wl::TEXT + 4);
        REQUIRE(cpu.commit_in_order());
        REQUIRE(!cpu.done());
    }

    // ---- Steady state: one instruction per cycle --------------------------
    // With no stalls, N instructions retire in N + 6 cycles; the pipeline
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
        REQUIRE(cpu.cycle()   == cpu.retired() + 6);
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
        REQUIRE(cpu.cycle()   == 9);
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
        REQUIRE(cpu.cycle()   == 7);

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
            REQUIRE(o.cycles == o.retired + 6);
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

    // ---- Back-pressure travels up one stage at a time ---------------------
    // Twenty addis all waiting on one 20-cycle divide cannot leave the issue
    // queue, so it fills, then dispatch stops and the rename queue fills, then
    // the decode queue, then the fetch queue, and only then does fetch stop.
    // Every stage stalls because its consumer did, not on a timer.
    {
        Config cfg;
        cfg.width = 1;
        Assembler p;
        p.li(t0, 1000);
        p.li(t1, 3);
        p.div_(t2, t0, t1);            // 20 cycles, blocking
        for (int i = 0; i < 20; ++i) p.addi(t3, t2, 1);   // every one waits on it
        p.li(a7, 93);
        p.ecall();
        Memory m = cputest::image(p.assemble());
        Cpu cpu(m, cfg, wl::TEXT);

        bool saw_full_iq = false, saw_full_rename = false;
        bool saw_full_decode = false, saw_full_fetch = false;
        for (int i = 0; i < 40 && !cpu.done(); ++i) {
            cpu.tick();
            REQUIRE(cpu.iq().size()      <= cpu.iq().capacity());
            REQUIRE(cpu.rename_queue()   <= cpu.queue_capacity());
            REQUIRE(cpu.decode_queue()   <= cpu.queue_capacity());
            REQUIRE(cpu.fetch_queue()    <= cpu.queue_capacity());
            if (cpu.iq().full())                              saw_full_iq = true;
            if (cpu.rename_queue() == cpu.queue_capacity())   saw_full_rename = true;
            if (cpu.decode_queue() == cpu.queue_capacity())   saw_full_decode = true;
            if (cpu.fetch_queue()  == cpu.queue_capacity())   saw_full_fetch = true;
        }
        REQUIRE(saw_full_iq);
        REQUIRE(saw_full_rename);
        REQUIRE(saw_full_decode);
        REQUIRE(saw_full_fetch);
        REQUIRE(cpu.stats().stall_count(Stall::IQ_FULL) > 0);

        // Back-pressure only stalls; it never drops or duplicates work.
        REQUIRE(cpu.run(2000));
        REQUIRE(cpu.halted());
        REQUIRE(cpu.retired() == 25);
        REQUIRE(cpu.commit_in_order());
    }

    // ---- The wrong path is fetched, and none of it retires ----------------
    // A jump the target cache has never seen falls through, so the ebreaks
    // behind it really are decoded. Every one of them is thrown away when the
    // jump resolves, which is the difference between speculating and being
    // wrong about the answer.
    {
        Config cfg;
        cfg.width = 2;
        Assembler p;
        p.j("target");
        for (int i = 0; i < 8; ++i) p.ebreak();        // fetched, never retired
        p.label("target");
        p.li(a0, 5);
        p.li(a7, 93);
        p.ecall();
        Memory m = cputest::image(p.assemble());
        Cpu cpu(m, cfg, wl::TEXT);
        cpu.record_decode(true);
        REQUIRE(cpu.run(1000));

        REQUIRE(cpu.halted());
        REQUIRE(!cpu.trapped());                       // no ebreak reached commit
        REQUIRE(cpu.exit_code() == 5);
        REQUIRE(cpu.retired() == 4);                   // jal, li, li, ecall
        REQUIRE(cpu.stats().mispredicts == 1);
        REQUIRE(cpu.stats().squashed > 0);

        bool decoded_wrong_path = false;
        for (const Cpu::DecodeRecord& r : cpu.decode_log()) {
            if (r.raw == 0x00100073u) decoded_wrong_path = true;
        }
        REQUIRE(decoded_wrong_path);
    }

    // ---- A branch costs a redirect once, and then stops costing ----------
    // The first pass through the loop has nothing in the target cache, so it
    // pays. Once the branch has committed taken, fetch follows it, and a
    // hundred iterations cost far fewer than a hundred redirects.
    {
        Config cfg;
        cfg.width = 1;
        Assembler p;
        p.li(t0, 0);
        p.li(t1, 100);
        p.label("loop");
        p.addi(t0, t0, 1);
        p.blt(t0, t1, "loop");
        p.mv(a0, t0);
        p.li(a7, 93);
        p.ecall();
        Memory m = cputest::image(p.assemble());
        Cpu cpu(m, cfg, wl::TEXT);
        REQUIRE(cpu.run(5000));

        REQUIRE(cpu.exit_code() == 100);
        REQUIRE(cpu.stats().branches == 100);
        REQUIRE(cpu.stats().mispredicts < 20);
        REQUIRE(cpu.stats().btb_hits > 90);
    }
}


// ------------------------------------------------- @section("execute_fu") ---
SECTION("execute_fu") {
    using namespace asmc;

    // ---- Load-use latency tracks mem_latency exactly ----------------------
    // Nothing stored to this address, so the load has to go to memory and
    // pays the full latency.
    {
        auto cycles_at = [](uint32_t lat) {
            Config cfg;
            cfg.width       = 1;
            cfg.mem_latency = lat;
            Assembler p;
            p.li(t0, 0x2000);
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

    // ---- A younger op overtakes a divide, and commit still does not -------
    // The add is nineteen cycles quicker than the divide it sits behind and
    // writes its physical register first. Renaming is what makes that safe;
    // in-order commit is what keeps it invisible from the outside.
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
        cpu.record_issue(true);

        bool add_visible_first = false;
        while (!cpu.done() && cpu.cycle() < 200) {
            cpu.tick();
            if (cpu.reg(t3) != 0 && cpu.reg(t2) == 0) add_visible_first = true;
        }
        REQUIRE(cpu.reg(t2) == 14);
        REQUIRE(cpu.reg(t3) == 8);
        REQUIRE(!add_visible_first);                 // commit is still in order
        REQUIRE(cpu.commit_in_order());

        // The divide is seq 2 and the add seq 3, and the younger one finishes
        // eighteen cycles earlier.
        uint64_t div_wb = 0, add_wb = 0;
        for (const Cpu::IssueRecord& r : cpu.issue_log()) {
            if (r.seq == 2) div_wb = r.wb_cycle;
            if (r.seq == 3) add_wb = r.wb_cycle;
        }
        REQUIRE(div_wb > 0);
        REQUIRE(add_wb > 0);
        REQUIRE(add_wb < div_wb);
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


// -------------------------------------------------- @section("commit_ooo") ---
SECTION("commit_ooo") {
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
        REQUIRE(wrote_at == 9);                      // its commit cycle, not its 7th
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


// ------------------------------------------------------ @section("rename") ---
SECTION("rename") {
    using namespace asmc;

    // ---- Reusing an architectural register allocates a new physical one ---
    // Three writes to t0 with nothing in between: unrenamed they would be a
    // WAW chain, renamed they are three unrelated registers.
    {
        Config cfg;
        cfg.width = 1;
        Assembler p;
        p.addi(t0, zero, 1);
        p.addi(t0, zero, 2);
        p.addi(t0, zero, 3);
        p.li(a7, 93);
        p.ecall();
        Memory m = cputest::image(p.assemble());
        Cpu cpu(m, cfg, wl::TEXT);
        cpu.record_rename(true);
        REQUIRE(cpu.run(1000));

        const std::vector<Cpu::RenameRecord>& log = cpu.rename_log();
        REQUIRE(log.size() == 5);

        std::vector<PhysReg> dests;
        for (std::size_t i = 0; i < 3; ++i) {
            REQUIRE(log[i].rd == t0);
            REQUIRE(log[i].dest != INVALID_PHYSREG);
            dests.push_back(log[i].dest);
        }
        std::sort(dests.begin(), dests.end());
        REQUIRE(std::adjacent_find(dests.begin(), dests.end()) == dests.end());

        // Each one displaces the previous mapping, which is what its ROB entry
        // carries to commit.
        REQUIRE(log[1].stale == log[0].dest);
        REQUIRE(log[2].stale == log[1].dest);
        REQUIRE(cpu.reg(t0) == 3);
    }

    // ---- A source reads the physical register its producer allocated ------
    {
        Config cfg;
        cfg.width = 1;
        Assembler p;
        p.addi(t0, zero, 1);
        p.addi(t1, t0, 1);
        p.addi(t2, t1, 1);
        p.li(a7, 93);
        p.ecall();
        Memory m = cputest::image(p.assemble());
        Cpu cpu(m, cfg, wl::TEXT);
        cpu.record_rename(true);
        REQUIRE(cpu.run(1000));

        const std::vector<Cpu::RenameRecord>& log = cpu.rename_log();
        REQUIRE(log[1].src1 == log[0].dest);
        REQUIRE(log[2].src1 == log[1].dest);
        REQUIRE(cpu.reg(t2) == 3);
    }

    // ---- Writing x0 allocates nothing -------------------------------------
    {
        Config cfg;
        cfg.width = 1;
        Assembler p;
        for (int i = 0; i < 8; ++i) p.addi(zero, zero, 1);   // discarded writes
        p.li(a7, 93);
        p.ecall();
        Memory m = cputest::image(p.assemble());
        Cpu cpu(m, cfg, wl::TEXT);
        cpu.record_rename(true);

        const uint32_t before = cpu.free_list().num_free();
        REQUIRE(cpu.run(1000));

        uint32_t allocated = 0;
        for (const Cpu::RenameRecord& r : cpu.rename_log()) {
            if (r.rd == static_cast<ArchReg>(zero) || r.rd == INVALID_ARCHREG) {
                REQUIRE(r.dest == INVALID_PHYSREG);
            } else {
                ++allocated;
            }
        }
        REQUIRE(allocated == 1);                    // only li a7
        REQUIRE(cpu.free_list().num_free() == before);
        REQUIRE(cpu.reg(zero) == 0);
        REQUIRE(cpu.rat().map(0) == 0);
    }

    // ---- Rename stalls on an empty free list, and still finishes ----------
    {
        Config cfg;
        cfg.width    = 1;
        cfg.rob_size = 32;
        cfg.prf_size = 34;                          // two spare registers
        REQUIRE(cfg.prf_can_starve());
        Memory m = cputest::image(fetest::addi_chain(40));
        Cpu cpu(m, cfg, wl::TEXT);
        REQUIRE(cpu.run(5000));

        REQUIRE(cpu.halted());
        REQUIRE(cpu.retired() == 42);
        REQUIRE(cpu.stats().stall_count(Stall::PHYSREG) > 0);
    }

    // ---- Rename stalls on a full ROB --------------------------------------
    {
        Config cfg;
        cfg.width    = 1;
        cfg.rob_size = 2;   // shallower than the rename-to-commit distance
        cfg.prf_size = 64;
        Memory m = cputest::image(fetest::addi_chain(40));
        Cpu cpu(m, cfg, wl::TEXT);
        REQUIRE(cpu.run(5000));

        REQUIRE(cpu.halted());
        REQUIRE(cpu.retired() == 42);
        REQUIRE(cpu.stats().stall_count(Stall::ROB_FULL) > 0);
    }
}


// ----------------------------------------------------- @section("reclaim") ---
namespace rectest {

// Every physical register is in exactly one of three places: the free list,
// the current RAT, or an in-flight ROB entry as the mapping it displaced.
// Nothing may be lost and nothing may be counted twice.
inline bool conserved(const Cpu& cpu) {
    const uint32_t size = cpu.config().prf_size;
    std::vector<bool> seen(size, false);
    uint32_t total = 0;

    auto take = [&](PhysReg p) {
        if (p == INVALID_PHYSREG || p >= size) return true;
        if (seen[p]) return false;                  // two owners is a leak
        seen[p] = true;
        ++total;
        return true;
    };

    for (PhysReg p = 0; p < size; ++p) {
        if (cpu.free_list().contains(p) && !take(p)) return false;
    }
    for (PhysReg p : cpu.rat().mapping()) {
        if (!take(p)) return false;
    }
    for (uint32_t k = 0; k < cpu.rob().size(); ++k) {
        if (!take(cpu.rob().nth_entry(k).stale_phys)) return false;
    }
    return total == size;
}

}  // namespace rectest

SECTION("reclaim") {
    using namespace asmc;

    // ---- The count is conserved every single cycle ------------------------
    {
        Config cfg;
        Assembler p;
        p.li(t0, 0);
        for (int i = 0; i < 30; ++i) {              // heavy destination reuse
            p.addi(t0, t0, 1);
            p.addi(t1, t0, 2);
            p.addi(t0, t1, 3);
        }
        p.mv(a0, t0);
        p.li(a7, 93);
        p.ecall();
        Memory m = cputest::image(p.assemble());
        Cpu cpu(m, cfg, wl::TEXT);

        REQUIRE(rectest::conserved(cpu));
        while (!cpu.done() && cpu.cycle() < 2000) {
            cpu.tick();
            REQUIRE(rectest::conserved(cpu));
        }
        REQUIRE(cpu.halted());
        REQUIRE(cpu.exit_code() == 180);
    }

    // ---- The pool is whole again at exit, on every workload ---------------
    // A register leaked on a squash or freed twice on commit shows up here as
    // a count that no longer matches the empty machine.
    {
        Config cfg;
        for (const wl::Workload& w : wl::corpus()) {
            Memory m = cputest::image(w.words);
            Cpu cpu(m, cfg, wl::TEXT);
            REQUIRE_MSG(cpu.run(w.budget * 8 + 1000),
                        std::string("    ") + w.name + " did not finish");
            REQUIRE_MSG(cpu.free_list().num_free() == cfg.prf_size - 32,
                        std::string("    ") + w.name + ": free list holds " +
                        std::to_string(cpu.free_list().num_free()));
            REQUIRE_MSG(rectest::conserved(cpu), std::string("    ") + w.name + " leaked");
        }
    }

    // ---- WAR and WAW disappear: the reordered run matches the oracle ------
    {
        Config cfg;
        diff::ScopedModel swap(&cputest::run_cpu);
        for (const char* name : {"alu", "shift_edge", "mul_div", "sieve"}) {
            for (const wl::Workload& w : wl::corpus()) {
                if (w.name != name) continue;
                const diff::Report r = diff::diff_run(w, cfg);
                REQUIRE_MSG(r.ok, r.detail);
            }
        }
    }
}


// ---------------------------------------------------------- @section("iq") ---
namespace iqtest {

inline IssueQueue::Entry entry(SeqNum seq, PhysReg dest, PhysReg s1, PhysReg s2,
                               bool s1_ready = true, bool s2_ready = true,
                               OpKind kind = OpKind::ALU) {
    IssueQueue::Entry e;
    e.seq        = seq;
    e.rob        = seq;
    e.kind       = kind;
    e.dest       = dest;
    e.src1       = s1;
    e.src2       = s2;
    e.src1_ready = s1_ready;
    e.src2_ready = s2_ready;
    return e;
}

}  // namespace iqtest

SECTION("iq") {
    using iqtest::entry;

    // ---- A waiter joins the ready set the moment its tag is broadcast -----
    {
        IssueQueue iq(8);
        REQUIRE(iq.empty());
        REQUIRE(iq.capacity() == 8);

        iq.insert(entry(0, 5, 1, 2));                     // produces p5
        iq.insert(entry(1, 6, 5, 2, /*s1_ready=*/false)); // waits on p5
        REQUIRE(iq.size() == 2);

        std::vector<IssueQueue::Entry> ready = iq.select(8);
        REQUIRE(ready.size() == 1);
        REQUIRE(ready[0].seq == 0);

        iq.wakeup(5);
        ready = iq.select(8);
        REQUIRE(ready.size() == 2);
        REQUIRE(ready[1].seq == 1);

        // A broadcast nobody is waiting on changes nothing.
        iq.wakeup(99);
        REQUIRE(iq.select(8).size() == 2);
    }

    // ---- Both operands have to arrive -------------------------------------
    {
        IssueQueue iq(8);
        iq.insert(entry(0, 7, 5, 6, false, false));
        REQUIRE(iq.select(8).empty());
        iq.wakeup(5);
        REQUIRE(iq.select(8).empty());
        iq.wakeup(6);
        REQUIRE(iq.select(8).size() == 1);
    }

    // ---- Select is oldest first and bounded by the limit ------------------
    {
        IssueQueue iq(8);
        iq.insert(entry(10, 40, 1, 2));
        iq.insert(entry(11, 41, 1, 2, false));            // not ready
        iq.insert(entry(12, 42, 1, 2));
        iq.insert(entry(13, 43, 1, 2));

        const std::vector<IssueQueue::Entry> ready = iq.select(2);
        REQUIRE(ready.size() == 2);
        REQUIRE(ready[0].seq == 10);
        REQUIRE(ready[1].seq == 12);                      // 11 was skipped, not blocking

        const std::vector<IssueQueue::Entry> all = iq.select(8);
        REQUIRE(all.size() == 3);
        REQUIRE(all[2].seq == 13);
    }

    // ---- Erase removes exactly one entry, and full() is honest ------------
    {
        IssueQueue iq(3);
        iq.insert(entry(0, 40, 1, 2));
        iq.insert(entry(1, 41, 1, 2));
        iq.insert(entry(2, 42, 1, 2));
        REQUIRE(iq.full());

        iq.erase(1);
        REQUIRE(iq.size() == 2);
        REQUIRE(!iq.full());
        REQUIRE(iq.select(8)[0].seq == 0);
        REQUIRE(iq.select(8)[1].seq == 2);

        iq.erase(99);                                     // absent: no-op
        REQUIRE(iq.size() == 2);
    }

    // ---- Recovery drops everything younger than the surviving branch ------
    {
        IssueQueue iq(8);
        for (SeqNum s = 0; s < 6; ++s) iq.insert(entry(s, 40 + s, 1, 2));
        iq.squash_after(2);
        REQUIRE(iq.size() == 3);
        for (const IssueQueue::Entry& e : iq.entries()) REQUIRE(e.seq <= 2);

        iq.clear();
        REQUIRE(iq.empty());
    }
}


// ---------------------------------------------------- @section("dispatch") ---
SECTION("dispatch") {
    using namespace asmc;

    // ---- Occupancy follows the trace one cycle at a time ------------------
    // Four dependent addis behind a divide: dispatch fills the queue at one
    // per cycle, and nothing leaves until the divide delivers.
    {
        Config cfg;
        cfg.width = 1;
        Assembler p;
        p.li(t0, 100);
        p.li(t1, 7);
        p.div_(t2, t0, t1);
        for (int i = 0; i < 4; ++i) p.addi(t3, t2, 1);
        p.li(a7, 93);
        p.ecall();
        Memory m = cputest::image(p.assemble());
        Cpu cpu(m, cfg, wl::TEXT);

        // Cycle 4 is when the first uop reaches the queue, and the two li's
        // issue straight back out, so occupancy only starts climbing once the
        // divide has taken its seat and the dependents pile up behind it.
        uint32_t peak = 0;
        for (int i = 0; i < 12; ++i) {
            cpu.tick();
            peak = std::max(peak, cpu.iq().size());
        }
        REQUIRE(peak >= 4);
        REQUIRE(cpu.iq().size() <= cpu.iq().capacity());

        REQUIRE(cpu.run(1000));
        REQUIRE(cpu.halted());
        REQUIRE(cpu.reg(t3) == 15);
    }

    // ---- A full queue stalls dispatch and is reported as such -------------
    {
        Config cfg;
        cfg.width   = 1;
        cfg.iq_size = 2;
        Assembler p;
        p.li(t0, 100);
        p.li(t1, 7);
        p.div_(t2, t0, t1);
        for (int i = 0; i < 8; ++i) p.addi(t3, t2, 1);
        p.li(a7, 93);
        p.ecall();
        Memory m = cputest::image(p.assemble());
        Cpu cpu(m, cfg, wl::TEXT);
        REQUIRE(cpu.run(2000));

        REQUIRE(cpu.halted());
        REQUIRE(cpu.stats().stall_count(Stall::IQ_FULL) > 0);
        REQUIRE(cpu.stats().dominant_stall() == Stall::IQ_FULL);
        REQUIRE(cpu.reg(t3) == 15);
    }

    // ---- Ready bits are sampled from the register file on the way in ------
    {
        Config cfg;
        cfg.width = 1;
        Assembler p;
        p.li(t0, 5);
        p.li(t1, 6);
        p.add(t2, t0, t1);
        p.li(a7, 93);
        p.ecall();
        Memory m = cputest::image(p.assemble());
        Cpu cpu(m, cfg, wl::TEXT);

        // The add dispatches in cycle 6, by which point li t0 has written back
        // but li t1 has not, so it arrives with one operand outstanding.
        for (int i = 0; i < 6; ++i) cpu.tick();
        REQUIRE(cpu.iq().size() == 1);
        const IssueQueue::Entry& e = cpu.iq().entries()[0];
        REQUIRE(e.src1_ready);
        REQUIRE(!e.src2_ready);

        REQUIRE(cpu.run(1000));
        REQUIRE(cpu.reg(t2) == 11);
    }
}


// ------------------------------------------------------- @section("issue") ---
SECTION("issue") {
    using namespace asmc;

    // ---- Independent work issues together -------------------------------
    {
        Config cfg;
        cfg.width   = 2;
        cfg.num_alu = 2;
        cfg.num_cdb = 2;
        Assembler p;
        p.addi(t0, zero, 1);
        p.addi(t1, zero, 2);
        p.li(a7, 93);
        p.ecall();
        Memory m = cputest::image(p.assemble());
        Cpu cpu(m, cfg, wl::TEXT);
        cpu.record_issue(true);
        REQUIRE(cpu.run(1000));

        const std::vector<Cpu::IssueRecord>& log = cpu.issue_log();
        REQUIRE(log.size() >= 2);
        REQUIRE(log[0].seq == 0);
        REQUIRE(log[1].seq == 1);
        REQUIRE(log[0].cycle == log[1].cycle);       // same cycle, two units
    }

    // ---- One too many for the units, and the oldest go first --------------
    // Three independent ALU ops with two units: two issue, the third waits a
    // cycle, and the reason is recorded rather than lost.
    {
        Config cfg;
        cfg.width   = 4;
        cfg.num_alu = 2;
        cfg.num_cdb = 4;
        Assembler p;
        p.addi(t0, zero, 1);
        p.addi(t1, zero, 2);
        p.addi(t2, zero, 3);
        p.fence();                                   // needs no unit, so it is
        p.li(a7, 93);                                // not a fourth contender
        p.ecall();
        Memory m = cputest::image(p.assemble());
        Cpu cpu(m, cfg, wl::TEXT);
        cpu.record_issue(true);
        REQUIRE(cpu.run(1000));

        // Looked up by sequence number, because the order they issued in is
        // exactly what is under test.
        std::vector<uint64_t> at(3, 0);
        for (const Cpu::IssueRecord& r : cpu.issue_log()) {
            if (r.seq < 3) at[r.seq] = r.cycle;
        }
        REQUIRE(at[0] > 0);
        REQUIRE(at[1] == at[0]);
        REQUIRE(at[2] == at[0] + 1);                 // deferred exactly one cycle
        REQUIRE(cpu.stats().stall_count(Stall::ALU_PORT) == 1);
    }

    // ---- A blocked op only blocks itself ----------------------------------
    // The multiply cannot go while its operand is missing; the independent
    // addi behind it does not have to wait for it.
    {
        Config cfg;
        cfg.width   = 2;
        cfg.num_alu = 2;
        Assembler p;
        p.li(t0, 4);
        p.li(t1, 100);
        p.div_(t2, t1, t0);          // 20 cycles
        p.mul(t3, t2, t2);           // waits on the divide
        p.addi(t4, zero, 9);         // independent, younger
        p.li(a7, 93);
        p.ecall();
        Memory m = cputest::image(p.assemble());
        Cpu cpu(m, cfg, wl::TEXT);
        cpu.record_issue(true);
        REQUIRE(cpu.run(1000));

        uint64_t mul_cycle = 0, addi_cycle = 0;
        for (const Cpu::IssueRecord& r : cpu.issue_log()) {
            if (r.seq == 3) mul_cycle  = r.cycle;
            if (r.seq == 4) addi_cycle = r.cycle;
        }
        REQUIRE(addi_cycle > 0);
        REQUIRE(mul_cycle > addi_cycle);             // program order did not apply
        REQUIRE(cpu.reg(t4) == 9);
        REQUIRE(cpu.reg(t3) == 625);
        REQUIRE(cpu.commit_in_order());
    }
}


// ------------------------------------------------- @section("execute_ooo") ---
SECTION("execute_ooo") {
    using namespace asmc;

    // ---- A result with nowhere to land does not issue ---------------------
    // Two independent addis on a single writeback port would both finish next
    // cycle. One books the port at issue; the other has to wait a cycle for
    // its own, rather than piling up at writeback.
    {
        Config cfg;
        cfg.width   = 2;
        cfg.num_alu = 2;
        cfg.num_cdb = 1;
        Assembler p;
        p.addi(t0, zero, 1);
        p.addi(t1, zero, 2);
        p.li(a7, 93);
        p.ecall();
        Memory m = cputest::image(p.assemble());
        Cpu cpu(m, cfg, wl::TEXT);
        cpu.record_issue(true);
        REQUIRE(cpu.run(1000));

        const std::vector<Cpu::IssueRecord>& log = cpu.issue_log();
        REQUIRE(log.size() >= 2);
        REQUIRE(log[1].cycle == log[0].cycle + 1);
        REQUIRE(log[1].wb_cycle == log[0].wb_cycle + 1);
        REQUIRE(cpu.stats().stall_count(Stall::CDB) >= 1);

        // Two ports and the same program: no delay at all.
        Config wide = cfg;
        wide.num_cdb = 2;
        Memory m2 = cputest::image(p.assemble());
        Cpu cpu2(m2, wide, wl::TEXT);
        cpu2.record_issue(true);
        REQUIRE(cpu2.run(1000));
        REQUIRE(cpu2.issue_log()[1].cycle == cpu2.issue_log()[0].cycle);
        REQUIRE(cpu2.stats().stall_count(Stall::CDB) == 0);
    }

    // ---- Ops route to their own class of unit -----------------------------
    // Saturating the multiplier does not slow the adds down, because they
    // never contended for it.
    {
        Config cfg;
        cfg.width   = 2;
        cfg.num_alu = 2;
        cfg.num_mul = 1;
        cfg.num_cdb = 4;
        Assembler p;
        p.li(t0, 3);
        p.li(t1, 4);
        p.mul(t2, t0, t1);
        p.addi(t3, t0, 1);
        p.addi(t4, t1, 1);
        p.li(a7, 93);
        p.ecall();
        Memory m = cputest::image(p.assemble());
        Cpu cpu(m, cfg, wl::TEXT);
        cpu.record_issue(true);
        REQUIRE(cpu.run(1000));

        uint64_t mul_cycle = 0, add_cycle = 0;
        for (const Cpu::IssueRecord& r : cpu.issue_log()) {
            if (r.seq == 2) mul_cycle = r.cycle;
            if (r.seq == 3) add_cycle = r.cycle;
        }
        REQUIRE(mul_cycle == add_cycle);             // different units, one cycle
        REQUIRE(cpu.reg(t2) == 12);
        REQUIRE(cpu.reg(t3) == 4);
        REQUIRE(cpu.reg(t4) == 5);
    }
}


// -------------------------------------------------- @section("wakeup_fast") ---
SECTION("wakeup_fast") {
    using namespace asmc;

    // ---- Dependent single-cycle ops issue back to back --------------------
    // Writeback broadcasts the tag before select runs in the same cycle, so a
    // 200-long dependence chain costs one cycle per link and not two. Without
    // that path the same program takes about twice as long, which is what the
    // bound is drawn to separate.
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
        cpu.record_issue(true);
        REQUIRE(cpu.run(2000));

        REQUIRE(cpu.exit_code() == 200);
        REQUIRE(cpu.retired() == 204);
        REQUIRE(cpu.cycle() == cpu.retired() + 6);   // no stall anywhere
        REQUIRE(cpu.cycle() < 260);

        // Link by link: every consumer issues the cycle after its producer.
        const std::vector<Cpu::IssueRecord>& log = cpu.issue_log();
        for (std::size_t i = 2; i < 200; ++i) {
            REQUIRE(log[i].cycle == log[i - 1].cycle + 1);
        }
    }

    // ---- The chain is one cycle per link at every latency -----------------
    // Slowing the ALU down moves the whole chain in lockstep, which is the
    // shape a real dependence chain has and a bubble-per-link does not.
    {
        auto cycles_at = [](uint32_t alu_latency) {
            Config cfg;
            cfg.width       = 1;
            cfg.alu_latency = alu_latency;
            Assembler p;
            p.li(t0, 0);
            for (int i = 0; i < 20; ++i) p.addi(t0, t0, 1);
            p.li(a7, 93);
            p.ecall();
            Memory m = cputest::image(p.assemble());
            Cpu cpu(m, cfg, wl::TEXT);
            cpu.run(2000);
            return cpu.cycle();
        };
        // Twenty-one ops in the chain counting the li that starts it, so each
        // extra cycle of latency costs twenty-one.
        REQUIRE(cycles_at(2) == cycles_at(1) + 21);
        REQUIRE(cycles_at(3) == cycles_at(1) + 42);
    }
}


// --------------------------------------------------------- @section("lsq") ---
SECTION("lsq") {
    // ---- Seats are handed out in order, and given back in order -----------
    {
        Lsq lsq(2, 2);
        REQUIRE(lsq.loads().empty());
        REQUIRE(lsq.stores().capacity() == 2);

        const std::optional<uint32_t> a = lsq.alloc_store(0, 4);
        const std::optional<uint32_t> b = lsq.alloc_store(1, 4);
        REQUIRE(a.has_value());
        REQUIRE(b.has_value());
        REQUIRE(lsq.stores().full());
        REQUIRE(!lsq.alloc_store(2, 4).has_value());   // dispatch stalls here

        lsq.stores().pop_head();
        REQUIRE(lsq.stores().size() == 1);
        REQUIRE(lsq.stores().nth(0).seq == 1);
        REQUIRE(lsq.alloc_store(2, 4).has_value());
    }

    // ---- A resolved store that covers the load forwards it ----------------
    {
        Lsq lsq(4, 4);
        const uint32_t s = *lsq.alloc_store(0, 4);
        lsq.resolve_addr(s, 0x100);
        lsq.resolve_data(s, 0xDEADBEEF);

        const uint32_t l = *lsq.alloc_load(1, 4);
        lsq.resolve_load_addr(l, 0x100);

        const ForwardResult r = lsq.search_older_stores(l);
        REQUIRE(r.kind == Forward::FORWARD);
        REQUIRE(r.data == 0xDEADBEEFu);
    }

    // ---- An unresolved older store forces a replay ------------------------
    // The address is unknown, so it might be this one. Guessing is the bug
    // this rule exists to prevent.
    {
        Lsq lsq(4, 4);
        const uint32_t s = *lsq.alloc_store(0, 4);     // address never resolved
        const uint32_t l = *lsq.alloc_load(1, 4);
        lsq.resolve_load_addr(l, 0x200);
        REQUIRE(lsq.search_older_stores(l).kind == Forward::REPLAY);

        // Resolving it somewhere else clears the ambiguity.
        lsq.resolve_addr(s, 0x300);
        lsq.resolve_data(s, 1);
        REQUIRE(lsq.search_older_stores(l).kind == Forward::NO_MATCH);
    }

    // ---- A store with a known address but no data yet also replays --------
    {
        Lsq lsq(4, 4);
        const uint32_t s = *lsq.alloc_store(0, 4);
        lsq.resolve_addr(s, 0x100);
        const uint32_t l = *lsq.alloc_load(1, 4);
        lsq.resolve_load_addr(l, 0x100);
        REQUIRE(lsq.search_older_stores(l).kind == Forward::REPLAY);

        lsq.resolve_data(s, 77);
        const ForwardResult r = lsq.search_older_stores(l);
        REQUIRE(r.kind == Forward::FORWARD);
        REQUIRE(r.data == 77u);
    }

    // ---- No older store aliases the address -------------------------------
    {
        Lsq lsq(4, 4);
        const uint32_t s = *lsq.alloc_store(0, 4);
        lsq.resolve_addr(s, 0x100);
        lsq.resolve_data(s, 5);
        const uint32_t l = *lsq.alloc_load(1, 4);
        lsq.resolve_load_addr(l, 0x104);               // adjacent, not overlapping
        REQUIRE(lsq.search_older_stores(l).kind == Forward::NO_MATCH);
    }

    // ---- A younger store is not this load's problem -----------------------
    {
        Lsq lsq(4, 4);
        const uint32_t l = *lsq.alloc_load(0, 4);
        lsq.resolve_load_addr(l, 0x100);
        const uint32_t s = *lsq.alloc_store(1, 4);     // younger
        lsq.resolve_addr(s, 0x100);
        lsq.resolve_data(s, 9);
        REQUIRE(lsq.search_older_stores(l).kind == Forward::NO_MATCH);
    }

    // ---- Sub-word: a covered byte forwards, a straddling half replays -----
    {
        Lsq lsq(4, 4);
        const uint32_t s = *lsq.alloc_store(0, 4);     // sw 0x100 = 0x44332211
        lsq.resolve_addr(s, 0x100);
        lsq.resolve_data(s, 0x44332211);

        const uint32_t b0 = *lsq.alloc_load(1, 1);     // lb 0x100
        lsq.resolve_load_addr(b0, 0x100);
        REQUIRE(lsq.search_older_stores(b0).data == 0x11u);

        const uint32_t b2 = *lsq.alloc_load(2, 1);     // lb 0x102
        lsq.resolve_load_addr(b2, 0x102);
        const ForwardResult r2 = lsq.search_older_stores(b2);
        REQUIRE(r2.kind == Forward::FORWARD);
        REQUIRE(r2.data == 0x33u);

        const uint32_t h = *lsq.alloc_load(3, 2);      // lh 0x102, still covered
        lsq.resolve_load_addr(h, 0x102);
        REQUIRE(lsq.search_older_stores(h).data == 0x4433u);
    }

    // ---- Partial overlap is never stitched together -----------------------
    // A one-byte store under a four-byte load covers some of it, so the load
    // waits for the store to commit rather than guessing at the rest.
    {
        Lsq lsq(4, 4);
        const uint32_t s = *lsq.alloc_store(0, 1);
        lsq.resolve_addr(s, 0x102);
        lsq.resolve_data(s, 0xFF);

        const uint32_t l = *lsq.alloc_load(1, 4);
        lsq.resolve_load_addr(l, 0x100);
        REQUIRE(lsq.search_older_stores(l).kind == Forward::REPLAY);
    }

    // ---- The youngest covering store wins, and shadows older ambiguity ----
    {
        Lsq lsq(4, 4);
        const uint32_t s0 = *lsq.alloc_store(0, 4);    // address unknown
        const uint32_t s1 = *lsq.alloc_store(1, 4);
        lsq.resolve_addr(s1, 0x100);
        lsq.resolve_data(s1, 0xAAAA);

        const uint32_t l = *lsq.alloc_load(2, 4);
        lsq.resolve_load_addr(l, 0x100);

        // s1 defines every byte the load wants, whatever s0 turns out to be.
        const ForwardResult r = lsq.search_older_stores(l);
        REQUIRE(r.kind == Forward::FORWARD);
        REQUIRE(r.data == 0xAAAAu);

        // Reversed: the unknown one is younger, so it could still land on top.
        Lsq other(4, 4);
        const uint32_t t0 = *other.alloc_store(0, 4);
        other.resolve_addr(t0, 0x100);
        other.resolve_data(t0, 0xBBBB);
        other.alloc_store(1, 4);                       // unresolved, younger
        const uint32_t l2 = *other.alloc_load(2, 4);
        other.resolve_load_addr(l2, 0x100);
        REQUIRE(other.search_older_stores(l2).kind == Forward::REPLAY);
        (void)s0;
    }

    // ---- Recovery drops the entries that never happened -------------------
    {
        Lsq lsq(8, 8);
        for (SeqNum s = 0; s < 5; ++s) lsq.alloc_load(s, 4);
        for (SeqNum s = 0; s < 5; ++s) lsq.alloc_store(s + 10, 4);
        lsq.squash_after(2);
        REQUIRE(lsq.loads().size() == 3);
        REQUIRE(lsq.stores().empty());                 // every store was younger

        lsq.clear();
        REQUIRE(lsq.loads().empty());
    }
}


// ------------------------------------------------ @section("load_forward") ---
SECTION("load_forward") {
    using namespace asmc;

    // ---- A load reads a store that has not committed yet ------------------
    // Memory still holds zero when the load produces its value, so the only
    // place the answer could have come from is the store queue.
    {
        Config cfg;
        cfg.width = 1;
        Assembler p;
        p.li(t0, 0x400);
        p.li(t1, 0x123);
        p.sw(t1, t0, 0);
        p.lw(a0, t0, 0);
        p.li(a7, 93);
        p.ecall();
        Memory m = cputest::image(p.assemble());
        Cpu cpu(m, cfg, wl::TEXT);
        REQUIRE(cpu.run(1000));

        REQUIRE(cpu.exit_code() == 0x123u);
        REQUIRE(cpu.stats().load_forwards == 1);
        REQUIRE(cpu.stats().load_memory == 0);         // memory never consulted
    }

    // ---- Forwarding does not pay memory latency ---------------------------
    // The same program at four different memory latencies takes exactly as
    // long, because the load never goes there.
    {
        auto cycles_at = [](uint32_t lat) {
            Config cfg;
            cfg.width       = 1;
            cfg.mem_latency = lat;
            Assembler p;
            p.li(t0, 0x400);
            p.li(t1, 7);
            p.sw(t1, t0, 0);
            p.lw(a0, t0, 0);
            p.add(a1, a0, a0);
            p.li(a7, 93);
            p.ecall();
            Memory m = cputest::image(p.assemble());
            Cpu cpu(m, cfg, wl::TEXT);
            cpu.run(1000);
            return cpu.cycle();
        };
        REQUIRE(cycles_at(2) == cycles_at(8));
    }

    // ---- An unresolved older store makes the load wait, then finish -------
    // The store's address depends on a 20-cycle divide, so the load has to
    // replay until the ambiguity clears — and then reads the right bytes.
    {
        Config cfg;
        cfg.width = 1;
        Assembler p;
        p.li(t0, 0x400);
        p.li(t1, 8);
        p.li(t2, 2);
        p.div_(t3, t1, t2);          // 4, after 20 cycles
        p.add(t4, t0, t3);           // address 0x404, unknown until then
        p.li(t5, 0x99);
        p.sw(t5, t4, 0);             // store to it
        p.lw(a0, t0, 4);             // same address, younger
        p.li(a7, 93);
        p.ecall();
        Memory m = cputest::image(p.assemble());
        Cpu cpu(m, cfg, wl::TEXT);
        REQUIRE(cpu.run(1000));

        REQUIRE(cpu.exit_code() == 0x99u);
        REQUIRE(cpu.stats().load_replays > 0);
        REQUIRE(cpu.stats().load_forwards == 1);
    }

    // ---- A load that no store covers goes to memory -----------------------
    {
        Config cfg;
        cfg.width = 1;
        Assembler p;
        p.li(t0, 0x400);
        p.li(t1, 0x55);
        p.sw(t1, t0, 0);
        p.lw(a0, t0, 16);            // a different word entirely
        p.li(a7, 93);
        p.ecall();
        Memory m = cputest::image(p.assemble());
        Cpu cpu(m, cfg, wl::TEXT);
        REQUIRE(cpu.run(1000));

        REQUIRE(cpu.exit_code() == 0);
        REQUIRE(cpu.stats().load_memory == 1);
        REQUIRE(cpu.stats().load_forwards == 0);
    }

    // ---- Sub-word forwarding through the pipeline -------------------------
    {
        Config cfg;
        cfg.width = 1;
        Assembler p;
        p.li(t0, 0x400);
        p.li(t1, 0x44332211);
        p.sw(t1, t0, 0);
        p.lbu(a0, t0, 2);            // the third byte of the pending store
        p.li(a7, 93);
        p.ecall();
        Memory m = cputest::image(p.assemble());
        Cpu cpu(m, cfg, wl::TEXT);
        REQUIRE(cpu.run(1000));

        REQUIRE(cpu.exit_code() == 0x33u);
        REQUIRE(cpu.stats().load_forwards == 1);
    }

    // ---- A full load queue stalls dispatch --------------------------------
    {
        Config cfg;
        cfg.width       = 1;
        cfg.lq_size     = 1;
        cfg.mem_latency = 6;
        Assembler p;
        p.li(t0, 0x400);
        for (int i = 0; i < 8; ++i) p.lw(t1, t0, 4 * i);
        p.li(a7, 93);
        p.ecall();
        Memory m = cputest::image(p.assemble());
        Cpu cpu(m, cfg, wl::TEXT);
        REQUIRE(cpu.run(2000));

        REQUIRE(cpu.halted());
        REQUIRE(cpu.stats().stall_count(Stall::LQ_FULL) > 0);
    }
}


// ------------------------------------------------- @section("store_commit") ---
SECTION("store_commit") {
    using namespace asmc;

    // ---- Nothing reaches memory before its store commits ------------------
    {
        Config cfg;
        cfg.width = 1;
        Assembler p;
        p.li(t0, 0x400);
        p.li(t1, 1);  p.sw(t1, t0, 0);
        p.li(t2, 2);  p.sw(t2, t0, 4);
        p.li(t3, 3);  p.sw(t3, t0, 8);
        p.li(a7, 93);
        p.ecall();
        Memory m = cputest::image(p.assemble());
        Cpu cpu(m, cfg, wl::TEXT);

        // The set of written words is a prefix of the program's stores at
        // every point, never just at the end.
        while (!cpu.done() && cpu.cycle() < 200) {
            cpu.tick();
            uint32_t written = 0;
            while (written < 3 && m.load_u32(0x400 + 4 * written) != 0) ++written;
            for (uint32_t k = written; k < 3; ++k) {
                REQUIRE(m.load_u32(0x400 + 4 * k) == 0);
            }
            // A store is either still queued or already in memory, never both.
            REQUIRE(written + cpu.lsq().stores().size() <= 3);
        }
        REQUIRE(m.load_u32(0x400) == 1);
        REQUIRE(m.load_u32(0x404) == 2);
        REQUIRE(m.load_u32(0x408) == 3);
        REQUIRE(cpu.lsq().stores().empty());
    }

    // ---- A store under a branch that is never taken leaves memory alone ---
    {
        Config cfg;
        cfg.width = 1;
        Assembler p;
        p.li(t0, 0x400);
        p.li(t1, 0xFF);
        p.li(t2, 1);
        p.beq(t2, zero, "skip");     // not taken
        p.j("done");
        p.label("skip");
        p.sw(t1, t0, 0);             // only on the untaken path
        p.label("done");
        p.li(a7, 93);
        p.ecall();
        Memory m = cputest::image(p.assemble());
        Cpu cpu(m, cfg, wl::TEXT);
        REQUIRE(cpu.run(1000));

        REQUIRE(cpu.halted());
        REQUIRE(m.load_u32(0x400) == 0);
        REQUIRE(cpu.stats().stores == 0);
    }

    // ---- Store-heavy workloads still match the interpreter ----------------
    {
        Config cfg;
        diff::ScopedModel swap(&cputest::run_cpu);
        for (const char* name : {"store_forward", "subword", "bubble_sort",
                                 "pointer_chase", "matmul"}) {
            for (const wl::Workload& w : wl::corpus()) {
                if (w.name != name) continue;
                const diff::Report r = diff::diff_run(w, cfg);
                REQUIRE_MSG(r.ok, r.detail);
            }
        }
    }

    // ---- Forwarding is doing real work on the corpus ----------------------
    // If the count were zero the whole path would be untested by the sweep.
    {
        Config cfg;
        for (const wl::Workload& w : wl::corpus()) {
            if (w.name != "store_forward") continue;
            Memory m = cputest::image(w.words);
            Cpu cpu(m, cfg, wl::TEXT);
            REQUIRE(cpu.run(w.budget * 8 + 1000));
            REQUIRE(cpu.stats().load_forwards > 0);
        }
    }
}


// ------------------------------------------------------ @section("gshare") ---
SECTION("gshare") {
    using namespace asmc;

    // ---- Counters saturate, and the top bit is the prediction -------------
    {
        Gshare g(4, 16);
        const uint32_t i = g.index(0x1000);
        REQUIRE(g.counter(i) == 1);                  // weakly not taken
        REQUIRE(!g.predict_at(i));

        g.update(i, true);
        REQUIRE(g.predict_at(i));                    // 1 -> 2 flips it
        g.update(i, true);
        g.update(i, true);
        REQUIRE(g.counter(i) == 3);                  // and stops there
        REQUIRE(g.predict_at(i));

        g.update(i, false);
        REQUIRE(g.predict_at(i));                    // one wrong outcome is absorbed
        g.update(i, false);
        REQUIRE(!g.predict_at(i));
        for (int k = 0; k < 5; ++k) g.update(i, false);
        REQUIRE(g.counter(i) == 0);
    }

    // ---- History and PC both pick the counter -----------------------------
    {
        Gshare g(4, 16);
        REQUIRE(g.index(0x1000) == ((0x1000u >> 2) & 15u));
        REQUIRE(g.index(0x1000) != g.index(0x1004));  // different branches differ

        const uint32_t before = g.index(0x1000);
        g.shift(true);
        REQUIRE(g.ghr() == 1);
        REQUIRE(g.index(0x1000) == (before ^ 1u));    // same branch, new context

        for (int k = 0; k < 8; ++k) g.shift(true);
        REQUIRE(g.ghr() == 15);                       // four bits, and no more
    }

    // ---- A regular loop converges, and stays converged --------------------
    // Five hundred iterations of a branch that is taken every time but the
    // last: the history fills, the counter saturates, and the rest are free.
    {
        Config cfg;
        Assembler p;
        p.li(t0, 0);
        p.li(t1, 500);
        p.label("loop");
        p.addi(t0, t0, 1);
        p.blt(t0, t1, "loop");
        p.mv(a0, t0);
        p.li(a7, 93);
        p.ecall();
        Memory m = cputest::image(p.assemble());
        Cpu cpu(m, cfg, wl::TEXT);
        REQUIRE(cpu.run(20000));

        REQUIRE(cpu.exit_code() == 500);
        REQUIRE(cpu.stats().branches == 500);
        REQUIRE(cpu.stats().mispredict_rate() < 0.05);
        REQUIRE(cpu.stats().mpki() < 40.0);
    }

    // ---- History is what makes the hard case predictable ------------------
    // A branch alternating taken and not taken is a coin flip to anything
    // without context and perfectly predictable with it. A single-counter
    // table has no context, so it does measurably worse.
    {
        auto mispredicts = [](uint32_t pht_size) {
            Config cfg;
            cfg.pht_size = pht_size;
            Assembler p;
            p.li(t0, 0);
            p.li(t1, 200);
            p.li(t2, 0);
            p.label("loop");
            p.andi(t3, t0, 1);                       // alternates every iteration
            p.beq(t3, zero, "even");
            p.addi(t2, t2, 1);
            p.label("even");
            p.addi(t0, t0, 1);
            p.blt(t0, t1, "loop");
            p.mv(a0, t2);
            p.li(a7, 93);
            p.ecall();
            Memory m = cputest::image(p.assemble());
            Cpu cpu(m, cfg, wl::TEXT);
            cpu.run(50000);
            REQUIRE(cpu.exit_code() == 100);
            return cpu.stats().mispredicts;
        };
        const uint64_t with_history = mispredicts(4096);
        const uint64_t one_counter  = mispredicts(1);
        REQUIRE(with_history * 2 < one_counter);
    }
}


// --------------------------------------------------------- @section("btb") ---
SECTION("btb") {
    using namespace asmc;

    // ---- A cold entry misses, and a PC-tagged one does not collide --------
    {
        Btb btb(4, 2);
        REQUIRE(!btb.lookup(0x1000).valid);

        btb.update(0x1000, 0x2000, BranchKind::JUMP);
        const Btb::Hit h = btb.lookup(0x1000);
        REQUIRE(h.valid);
        REQUIRE(h.target == 0x2000u);
        REQUIRE(h.kind == BranchKind::JUMP);

        // Same set, different PC: the tag keeps them apart.
        REQUIRE(!btb.lookup(0x1010).valid);
        btb.update(0x1010, 0x3000, BranchKind::CONDITIONAL);
        REQUIRE(btb.lookup(0x1000).target == 0x2000u);
        REQUIRE(btb.lookup(0x1010).target == 0x3000u);

        // Retargeting an existing entry updates it in place.
        btb.update(0x1000, 0x4000, BranchKind::JUMP);
        REQUIRE(btb.lookup(0x1000).target == 0x4000u);
    }

    // ---- A full set evicts the least recently used ------------------------
    {
        Btb btb(1, 2);                                // one set, two ways
        btb.update(0x1000, 0xA, BranchKind::JUMP);
        btb.update(0x1004, 0xB, BranchKind::JUMP);
        REQUIRE(btb.lookup(0x1000).valid);
        REQUIRE(btb.lookup(0x1004).valid);

        btb.update(0x1008, 0xC, BranchKind::JUMP);    // evicts the oldest
        REQUIRE(!btb.lookup(0x1000).valid);
        REQUIRE(btb.lookup(0x1004).valid);
        REQUIRE(btb.lookup(0x1008).valid);
    }

    // ---- A branch is predicted only after it has committed taken ----------
    // The first pass has nothing cached, so fetch falls through and the jump
    // is corrected when it executes. The second pass is free.
    {
        Config cfg;
        cfg.width = 1;
        Assembler p;
        p.li(t0, 0);
        p.li(t1, 2);
        p.label("loop");
        p.addi(t0, t0, 1);
        p.j("bottom");                                // an unconditional hop
        p.ebreak();                                   // only on the wrong path
        p.label("bottom");
        p.blt(t0, t1, "loop");
        p.mv(a0, t0);
        p.li(a7, 93);
        p.ecall();
        Memory m = cputest::image(p.assemble());
        Cpu cpu(m, cfg, wl::TEXT);
        REQUIRE(cpu.run(1000));

        REQUIRE(cpu.halted());
        REQUIRE(!cpu.trapped());
        REQUIRE(cpu.exit_code() == 2);
        // Two passes over the jump: the first misses the cache, the second hits.
        REQUIRE(cpu.stats().btb_lookups == 4);        // two jumps, two branches
        REQUIRE(cpu.stats().btb_hits == 2);
    }

    // ---- An indirect jump is corrected the first time, cached after -------
    {
        Config cfg;
        cfg.width = 1;
        Assembler p;
        p.li(t0, 0);
        p.li(t1, 3);
        p.label("loop");
        p.auipc(t2, 0);
        p.addi(t2, t2, 16);                           // four instructions ahead
        p.jalr(zero, t2, 0);                          // indirect, same target
        p.ebreak();
        p.label("landing");
        p.addi(t0, t0, 1);
        p.blt(t0, t1, "loop");
        p.mv(a0, t0);
        p.li(a7, 93);
        p.ecall();
        Memory m = cputest::image(p.assemble());
        Cpu cpu(m, cfg, wl::TEXT);
        REQUIRE(cpu.run(2000));

        REQUIRE(cpu.halted());
        REQUIRE(!cpu.trapped());
        REQUIRE(cpu.exit_code() == 3);
        REQUIRE(cpu.stats().mispredicts >= 1);        // the cold indirect jump
        REQUIRE(cpu.stats().btb_hits >= 2);           // and it is cached after
    }
}


// --------------------------------------------------------- @section("ras") ---
SECTION("ras") {
    using namespace asmc;

    // ---- Last in, first out, and an empty stack is a legal state ----------
    {
        Ras ras(4);
        REQUIRE(ras.empty());
        REQUIRE(ras.pop() == 0);                      // falls back, does not fault

        ras.push(0x100);
        ras.push(0x200);
        REQUIRE(ras.depth() == 2);
        REQUIRE(ras.peek() == 0x200u);
        REQUIRE(ras.pop() == 0x200u);
        REQUIRE(ras.pop() == 0x100u);
        REQUIRE(ras.empty());
    }

    // ---- Overflowing costs the outermost frames and nothing else ----------
    {
        Ras ras(2);
        ras.push(0x100);
        ras.push(0x200);
        ras.push(0x300);                              // 0x100 is gone
        REQUIRE(ras.depth() == 2);
        REQUIRE(ras.pop() == 0x300u);
        REQUIRE(ras.pop() == 0x200u);
    }

    // ---- Returns from a nested call chain are predicted -------------------
    {
        Config cfg;
        for (const wl::Workload& w : wl::corpus()) {
            if (w.name != "nested_calls") continue;
            Memory m = cputest::image(w.words);
            Cpu cpu(m, cfg, wl::TEXT);
            REQUIRE(cpu.run(w.budget * 8 + 1000));
            REQUIRE(cpu.halted());
            REQUIRE(cpu.stats().ras_pops > 0);
            REQUIRE(cpu.stats().ras_accuracy() > 0.8);
        }
    }

    // ---- Recursion is where the stack earns its keep ----------------------
    // A one-deep stack cannot hold a recursive call chain, so its returns go
    // to the target cache instead — which always names the last caller.
    {
        auto mispredicts_with = [](uint32_t ras_size) {
            Config cfg;
            cfg.ras_size = ras_size;
            for (const wl::Workload& w : wl::corpus()) {
                if (w.name != "fib") continue;
                Memory m = cputest::image(w.words);
                Cpu cpu(m, cfg, wl::TEXT);
                cpu.run(w.budget * 8 + 1000);
                REQUIRE(cpu.halted());
                REQUIRE(cpu.exit_code() == 144);
                return cpu.stats().mispredicts;
            }
            return uint64_t{0};
        };
        REQUIRE(mispredicts_with(16) * 2 < mispredicts_with(1));
    }
}


// -------------------------------------------------- @section("checkpoint") ---
SECTION("checkpoint") {
    using namespace asmc;

    // ---- Every branch takes a slot, and gives it back -------------------
    {
        Config cfg;
        Memory m = cputest::image(fetest::addi_chain(20));
        Cpu cpu(m, cfg, wl::TEXT);
        const uint32_t all = cpu.rat().num_checkpoints();
        REQUIRE(cpu.rat().num_free_checkpoints() == all);

        while (!cpu.done() && cpu.cycle() < 2000) {
            cpu.tick();
            REQUIRE(cpu.rat().num_free_checkpoints() <= all);
        }
        REQUIRE(cpu.rat().num_free_checkpoints() == all);
    }

    // ---- Running out stalls rename instead of overwriting a snapshot ------
    {
        Config cfg;
        cfg.num_checkpoints = 1;                     // one branch in flight
        Assembler p;
        p.li(t0, 0);
        p.li(t1, 30);
        p.label("loop");
        p.addi(t0, t0, 1);
        p.blt(t0, t1, "loop");
        p.mv(a0, t0);
        p.li(a7, 93);
        p.ecall();
        Memory m = cputest::image(p.assemble());
        Cpu cpu(m, cfg, wl::TEXT);
        REQUIRE(cpu.run(5000));

        REQUIRE(cpu.exit_code() == 30);              // still correct, just slower
        REQUIRE(cpu.stats().stall_count(Stall::CHECKPOINT) > 0);
        REQUIRE(cpu.rat().num_free_checkpoints() == 1);
    }

    // ---- A branch-dense workload never loses a slot -----------------------
    {
        Config cfg;
        cfg.num_checkpoints = 2;
        diff::ScopedModel swap(&cputest::run_cpu);
        for (const char* name : {"lcg_branch", "fib", "sieve"}) {
            for (const wl::Workload& w : wl::corpus()) {
                if (w.name != name) continue;
                const diff::Report r = diff::diff_run(w, cfg);
                REQUIRE_MSG(r.ok, r.detail);

                Memory m = cputest::image(w.words);
                Cpu cpu(m, cfg, wl::TEXT);
                REQUIRE(cpu.run(w.budget * 20 + 1000));
                REQUIRE(cpu.rat().num_free_checkpoints() == 2);
            }
        }
    }
}


// ----------------------------------------------------- @section("recover") ---
SECTION("recover") {
    using namespace asmc;

    // ---- A mispredicted branch leaves no trace ----------------------------
    // Everything on the wrong path allocated registers, queue seats and
    // checkpoints. After recovery the machine is indistinguishable from one
    // that never guessed.
    {
        Config cfg;
        Assembler p;
        p.li(t0, 0);
        p.li(t1, 50);
        p.li(t2, 0x400);
        p.label("loop");
        p.addi(t0, t0, 1);
        p.andi(t3, t0, 3);
        p.bne(t3, zero, "skip");                     // taken three times in four
        p.sw(t0, t2, 0);
        p.label("skip");
        p.blt(t0, t1, "loop");
        p.mv(a0, t0);
        p.li(a7, 93);
        p.ecall();
        Memory m = cputest::image(p.assemble());
        Cpu cpu(m, cfg, wl::TEXT);

        while (!cpu.done() && cpu.cycle() < 5000) {
            cpu.tick();
            REQUIRE(rectest::conserved(cpu));        // no register lost, ever
        }
        REQUIRE(cpu.halted());
        REQUIRE(cpu.exit_code() == 50);
        REQUIRE(m.load_u32(0x400) == 48);            // the last multiple of four
        REQUIRE(cpu.stats().mispredicts > 0);
        REQUIRE(cpu.stats().squashed > 0);
        REQUIRE(cpu.free_list().num_free() == cfg.prf_size - 32);
        REQUIRE(cpu.rat().num_free_checkpoints() == cpu.rat().num_checkpoints());
        REQUIRE(cpu.commit_in_order());
    }

    // ---- An unpredictable branch mispredicts constantly and leaks nothing --
    {
        Config cfg;
        for (const wl::Workload& w : wl::corpus()) {
            if (w.name != "lcg_branch") continue;
            Memory m = cputest::image(w.words);
            Cpu cpu(m, cfg, wl::TEXT);
            REQUIRE(cpu.run(w.budget * 8 + 1000));

            REQUIRE(cpu.halted());
            REQUIRE(cpu.stats().mispredicts > 50);   // genuinely unpredictable
            REQUIRE(cpu.free_list().num_free() == cfg.prf_size - 32);
            REQUIRE(cpu.rat().num_free_checkpoints() == cpu.rat().num_checkpoints());
            REQUIRE(rectest::conserved(cpu));
            REQUIRE(cpu.lsq().loads().empty());
            REQUIRE(cpu.lsq().stores().empty());
        }
    }

    // ---- The retired count matches the oracle exactly ---------------------
    // One wrong-path instruction reaching commit would show up here and
    // nowhere else, since its architectural effect might well be invisible.
    {
        Config cfg;
        diff::ScopedModel swap(&cputest::run_cpu);
        for (const wl::Workload& w : wl::corpus()) {
            const diff::Outcome model = cputest::run_cpu(w, cfg);
            const diff::Outcome ref   = diff::run_reference(w, cfg);
            REQUIRE_MSG(model.retired == ref.retired,
                        "    " + w.name + ": retired " + std::to_string(model.retired) +
                        " vs " + std::to_string(ref.retired));
        }
    }
}


// ------------------------------------------------------- @section("stats") ---
namespace stattest {

// Slots the issue stage could have used and did not. Each cycle offers
// `width` of them, so this is what the issue-side breakdown must fit inside.
inline const Stall ISSUE_STALLS[] = {
    Stall::ALU_PORT, Stall::BRANCH_PORT, Stall::MUL_PORT, Stall::DIV_PORT,
    Stall::MEM_PORT, Stall::CDB, Stall::STORE_ORDER, Stall::OPERANDS,
};
inline const Stall RENAME_STALLS[]   = {Stall::ROB_FULL, Stall::PHYSREG, Stall::CHECKPOINT};
inline const Stall DISPATCH_STALLS[] = {Stall::IQ_FULL, Stall::LQ_FULL, Stall::SQ_FULL};

inline Stats run(const wl::Workload& w, const Config& cfg) {
    Memory m = cputest::image(w.words);
    Cpu cpu(m, cfg, wl::TEXT);
    cpu.run(w.budget * 40 + 10000);
    REQUIRE_MSG(cpu.done(), "    " + w.name + " did not finish");
    return cpu.stats();
}

inline const wl::Workload& named(const std::string& name) {
    for (const wl::Workload& w : wl::corpus()) {
        if (w.name == name) return w;
    }
    static const wl::Workload none;
    REQUIRE_MSG(false, "    no workload named " + name);
    return none;
}

}  // namespace stattest

SECTION("stats") {
    using namespace stattest;

    // ---- Every renamed uop either retires or is squashed ------------------
    // The stage counters are a funnel, and the two ends have to agree: a uop
    // that reached rename took a ROB entry, and a ROB entry leaves exactly one
    // way.
    {
        const Config cfg;
        for (const wl::Workload& w : wl::corpus()) {
            const Stats s = run(w, cfg);
            REQUIRE_MSG(s.renamed == s.retired + s.squashed,
                        "    " + w.name + ": renamed " + std::to_string(s.renamed) +
                        " != retired " + std::to_string(s.retired) +
                        " + squashed " + std::to_string(s.squashed));
            REQUIRE(s.fetched    >= s.decoded);
            REQUIRE(s.decoded    >= s.renamed);
            REQUIRE(s.renamed    >= s.dispatched);
            REQUIRE(s.dispatched >= s.issued);
            REQUIRE(s.issued     >= s.wrote_back);
            REQUIRE(s.retired    >= 1);
        }
    }

    // ---- A load is served from exactly one place --------------------------
    {
        const Config cfg;
        for (const wl::Workload& w : wl::corpus()) {
            const Stats s = run(w, cfg);
            REQUIRE_MSG(s.loads == s.load_forwards + s.load_memory,
                        "    " + w.name + ": loads " + std::to_string(s.loads) +
                        " != forwards " + std::to_string(s.load_forwards) +
                        " + memory " + std::to_string(s.load_memory));
        }
        REQUIRE(run(named("store_forward"), cfg).load_forwards > 0);
        REQUIRE(run(named("pointer_chase"), cfg).load_forwards == 0);
    }

    // ---- The rates are the counters, divided ------------------------------
    {
        const Stats s = run(named("crc32"), Config{});
        REQUIRE(std::abs(s.ipc() * s.cpi() - 1.0) < 1e-9);
        REQUIRE(std::abs(s.ipc() - double(s.retired) / double(s.cycles)) < 1e-9);
        REQUIRE(std::abs(s.mpki() - 1000.0 * s.mispredict_rate() * s.branches / s.retired) < 1e-6);
        REQUIRE(s.btb_hit_rate() <= 1.0);
        REQUIRE(s.ras_accuracy() <= 1.0);

        const Stats empty;
        REQUIRE(empty.ipc() == 0.0);            // no cycles, no division
        REQUIRE(empty.cpi() == 0.0);
        REQUIRE(empty.mispredict_rate() == 0.0);
        REQUIRE(empty.dominant_stall() == Stall::COUNT);
    }

    // ---- The breakdown fits in the slots the machine actually had ---------
    // Lost issue slots are bounded by width per cycle, and each front-end
    // stage can only lose one bundle per cycle. A breakdown that overflows
    // those bounds is double-counting and cannot be read as a fraction.
    {
        for (const uint32_t width : {1u, 2u, 4u}) {
            Config cfg;
            cfg.width = width;
            for (const wl::Workload& w : wl::corpus()) {
                const Stats s = run(w, cfg);
                uint64_t issue_side = 0, rename_side = 0, dispatch_side = 0;
                for (const Stall st : ISSUE_STALLS)    issue_side    += s.stall_count(st);
                for (const Stall st : RENAME_STALLS)   rename_side   += s.stall_count(st);
                for (const Stall st : DISPATCH_STALLS) dispatch_side += s.stall_count(st);
                REQUIRE_MSG(issue_side <= s.cycles * width,
                            "    " + w.name + ": issue-side stalls exceed the slots");
                REQUIRE_MSG(rename_side <= s.cycles, "    " + w.name + ": rename stalls exceed cycles");
                REQUIRE_MSG(dispatch_side <= s.cycles, "    " + w.name + ": dispatch stalls exceed cycles");
            }
        }
    }

    // ---- An unstressed run blames nothing ---------------------------------
    {
        const Stats s = run(named("alu"), Config{});
        REQUIRE(s.dominant_stall() == Stall::COUNT);
        REQUIRE(std::string(stall_name(Stall::COUNT)) == "none");
    }

    // ---- Starve one resource and the breakdown names that resource --------
    // This is the whole point of the stall accounting. Each row takes the
    // default machine, removes exactly one thing, and asks what hurt.
    {
        struct Case {
            const char* workload;
            Stall       expect;
            void      (*starve)(Config&);
        };
        static const Case cases[] = {
            {"matmul",        Stall::ROB_FULL,    [](Config& c) { c.rob_size = 4; c.prf_size = 40; }},
            {"sieve",         Stall::ROB_FULL,    [](Config& c) { c.rob_size = 4; c.prf_size = 40; }},
            {"matmul",        Stall::IQ_FULL,     [](Config& c) { c.iq_size = 2; }},
            {"sieve",         Stall::CHECKPOINT,  [](Config& c) { c.num_checkpoints = 1; }},
            {"crc32",         Stall::CHECKPOINT,  [](Config& c) { c.num_checkpoints = 1; }},
            {"matmul",        Stall::PHYSREG,     [](Config& c) { c.prf_size = 36; }},
            {"waw_war",       Stall::PHYSREG,     [](Config& c) { c.prf_size = 36; }},
            {"pointer_chase", Stall::LQ_FULL,     [](Config& c) { c.lq_size = 1; }},
            {"bubble_sort",   Stall::SQ_FULL,     [](Config& c) { c.sq_size = 1; }},
            {"matmul",        Stall::CDB,         [](Config& c) { c.num_cdb = 1; }},
            {"matmul",        Stall::ALU_PORT,    [](Config& c) { c.num_alu = 1; c.width = 4; }},
            {"muldiv",        Stall::DIV_PORT,    [](Config&)   {}},
        };
        for (const Case& c : cases) {
            Config cfg;
            c.starve(cfg);
            const Stats s = run(named(c.workload), cfg);
            REQUIRE_MSG(s.dominant_stall() == c.expect,
                        std::string("    ") + c.workload + ": blamed " +
                        stall_name(s.dominant_stall()) + ", expected " + stall_name(c.expect));
        }
    }

    // ---- Giving the resource back is what proves the diagnosis ------------
    // A cause that does not go away when the resource does was a symptom.
    {
        const wl::Workload& w = named("matmul");
        Config tight;
        tight.rob_size = 4;
        tight.prf_size = 40;
        const Stats starved = run(w, tight);

        Config roomy = tight;
        roomy.rob_size = 32;
        roomy.prf_size = 64;
        const Stats relieved = run(w, roomy);

        REQUIRE(relieved.stall_count(Stall::ROB_FULL) * 4 < starved.stall_count(Stall::ROB_FULL));
        REQUIRE(relieved.cycles < starved.cycles);
        REQUIRE(relieved.retired == starved.retired);   // same program, either way
    }

    // ---- crc32 is not short of a resource; it is short of a prediction ----
    // Its inner branch turns on one bit of a CRC, which nothing can predict,
    // so the breakdown correctly refuses to blame any structure and the cost
    // shows up as work thrown away instead.
    {
        const Config cfg;
        const Stats crc = run(named("crc32"), cfg);
        REQUIRE(crc.dominant_stall() == Stall::COUNT);
        REQUIRE(crc.mispredict_rate() > 0.15);
        REQUIRE(double(crc.squashed) / crc.retired > 0.15);

        // No other program in the corpus throws away as much.
        for (const wl::Workload& w : wl::corpus()) {
            if (w.name == "crc32") continue;
            REQUIRE(run(w, cfg).squashed < crc.squashed);
        }
    }
}


// ------------------------------------------------ @section("config_sweep") ---
namespace sweep {

struct Machine {
    const char* name;
    Config      cfg;
};

// Six machines that stress different parts of the same design. Correctness is
// supposed to be identical on all of them; a renaming, wakeup or recovery bug
// usually is not, which is what makes running the corpus six times worth more
// than running it once.
inline const std::vector<Machine>& machines() {
    static const std::vector<Machine> m = [] {
        std::vector<Machine> v;

        v.push_back({"default", Config{}});

        Config narrow;                        // nothing overlaps; latency is exposed
        narrow.width   = 1;
        narrow.num_cdb = 1;
        narrow.num_alu = 1;
        v.push_back({"1-wide/1-CDB", narrow});

        Config wide;                          // deep window, plenty of ports
        wide.width    = 4;
        wide.rob_size = 128;
        wide.prf_size = 160;
        wide.iq_size  = 32;
        wide.num_alu  = 4;
        wide.num_cdb  = 4;
        v.push_back({"4-wide/ROB=128", wide});

        Config starved;                       // every structure is a bottleneck
        starved.rob_size        = 4;
        starved.prf_size        = 40;
        starved.iq_size         = 2;
        starved.lq_size         = 1;
        starved.sq_size         = 1;
        starved.num_checkpoints = 1;
        v.push_back({"starved", starved});

        Config slow;                          // wakeup has to track real latencies
        slow.alu_latency = 3;
        slow.mem_latency = 8;
        slow.mul_latency = 8;
        slow.div_latency = 40;
        v.push_back({"long-latency", slow});

        Config blind;                         // recovery on almost every branch
        blind.ghr_bits = 0;
        blind.pht_size = 1;
        blind.btb_sets = 1;
        blind.btb_ways = 1;
        blind.ras_size = 1;
        v.push_back({"1-entry predictors", blind});

        return v;
    }();
    return m;
}

}  // namespace sweep

SECTION("config_sweep") {
    diff::ScopedModel swap(&cputest::run_cpu);

    // ---- Every program, on every machine, gets the same answer ------------
    for (const sweep::Machine& m : sweep::machines()) {
        for (const wl::Workload& w : wl::corpus()) {
            const diff::Report r = diff::diff_run(w, m.cfg);
            REQUIRE_MSG(r.ok, std::string("  [") + m.name + "]\n" + r.detail);
        }
    }

    // ---- And leaves no resource behind on any of them ---------------------
    for (const sweep::Machine& m : sweep::machines()) {
        for (const wl::Workload& w : wl::corpus()) {
            Memory mem = cputest::image(w.words);
            Cpu cpu(mem, m.cfg, wl::TEXT);
            REQUIRE(cpu.run(w.budget * 40 + 10000));
            REQUIRE_MSG(cpu.free_list().num_free() == m.cfg.prf_size - 32,
                        std::string("    ") + w.name + " on " + m.name + ": leaked a physreg");
            REQUIRE(cpu.rat().num_free_checkpoints() == cpu.rat().num_checkpoints());
            REQUIRE(cpu.lsq().loads().empty());
            REQUIRE(cpu.lsq().stores().empty());
            REQUIRE(cpu.commit_in_order());
        }
    }

    // ---- The six machines really are six machines -------------------------
    // A sweep whose configurations all behave alike would pass whatever it was
    // pointed at, so the timings have to actually diverge.
    {
        const wl::Workload& w = stattest::named("matmul");
        std::vector<uint64_t> cycles;
        for (const sweep::Machine& m : sweep::machines()) {
            cycles.push_back(stattest::run(w, m.cfg).cycles);
        }
        for (std::size_t i = 0; i < cycles.size(); ++i) {
            for (std::size_t j = i + 1; j < cycles.size(); ++j) {
                REQUIRE(cycles[i] != cycles[j]);
            }
        }
        REQUIRE(cycles[2] < cycles[0]);     // wide is faster than default
        REQUIRE(cycles[1] > cycles[0]);     // narrow is slower
        REQUIRE(cycles[3] > cycles[1]);     // starved is worse than merely narrow
    }
}


// -------------------------------------------------- @section("properties") ---
namespace props {

// Table-driven CRC-32, written a different way from the bitwise loop the
// workload runs, so agreeing with it means something.
inline uint32_t crc32_table_driven(const std::vector<uint8_t>& bytes) {
    static const std::array<uint32_t, 256> table = [] {
        std::array<uint32_t, 256> t{};
        for (uint32_t i = 0; i < 256; ++i) {
            uint32_t c = i;
            for (int k = 0; k < 8; ++k) c = (c >> 1) ^ (0xEDB88320u & (~(c & 1u) + 1u));
            t[i] = c;
        }
        return t;
    }();

    uint32_t crc = 0xFFFFFFFFu;
    for (const uint8_t b : bytes) crc = table[(crc ^ b) & 0xFFu] ^ (crc >> 8);
    return crc ^ 0xFFFFFFFFu;
}

inline uint64_t cycles_of(const std::vector<uint32_t>& words, const Config& cfg) {
    Memory m = cputest::image(words);
    Cpu cpu(m, cfg, wl::TEXT);
    REQUIRE(cpu.run(200000));
    return cpu.cycle();
}

}  // namespace props

SECTION("properties") {
    using namespace asmc;

    // ---- Dependent single-cycle ops issue back to back --------------------
    // Two hundred addis that each need the one before it, on a machine that
    // can only start one op per cycle. Anything above ~1.3 cycles apiece means
    // wakeup is not reaching select in the same cycle the value lands.
    {
        Config cfg;
        cfg.width = 1;
        Assembler p;
        p.li(a0, 0);
        for (int i = 0; i < 200; ++i) p.addi(a0, a0, 1);
        p.li(a7, 93);
        p.ecall();
        Memory m = cputest::image(p.assemble());
        Cpu cpu(m, cfg, wl::TEXT);
        REQUIRE(cpu.run(1000));
        REQUIRE(cpu.exit_code() == 200);
        REQUIRE(cpu.cycle() < 260);
    }

    // ---- Load-use latency is the configured number, exactly ---------------
    {
        Assembler p;
        p.li(t0, 0x400);
        p.lw(t1, t0, 0);                      // nothing wrote it, so memory answers
        p.addi(a0, t1, 7);
        p.li(a7, 93);
        p.ecall();
        const std::vector<uint32_t> code = p.assemble();

        Config base;
        const uint64_t at2 = props::cycles_of(code, base);
        for (const uint32_t lat : {3u, 5u, 9u, 20u}) {
            Config cfg;
            cfg.mem_latency = lat;
            REQUIRE(props::cycles_of(code, cfg) == at2 + (lat - base.mem_latency));
        }
    }

    // ---- A forwarded load pays none of it ---------------------------------
    {
        Assembler p;
        p.li(t0, 0x400);
        p.li(t1, 77);
        p.sw(t1, t0, 0);
        p.lw(a0, t0, 0);                      // served by the store queue
        p.li(a7, 93);
        p.ecall();
        const std::vector<uint32_t> code = p.assemble();

        const uint64_t at2 = props::cycles_of(code, Config{});
        for (const uint32_t lat : {8u, 20u}) {
            Config cfg;
            cfg.mem_latency = lat;
            REQUIRE(props::cycles_of(code, cfg) == at2);
        }

        Memory m = cputest::image(code);
        Cpu cpu(m, Config{}, wl::TEXT);
        REQUIRE(cpu.run(1000));
        REQUIRE(cpu.exit_code() == 77);
        REQUIRE(cpu.stats().load_forwards == 1);
        REQUIRE(cpu.stats().load_memory == 0);
    }

    // ---- A load behind an unresolved store waits instead of guessing ------
    {
        Assembler p;
        p.li(t0, 0x400);
        p.li(t1, 5);
        p.li(t2, 41);
        p.div_(t3, t1, t1);                   // slow, and the store needs it
        p.slli(t3, t3, 10);                   // address = 0x400 only once it lands
        p.sw(t2, t3, 0);
        p.lw(a0, t0, 0);                      // may not pass the store
        p.li(a7, 93);
        p.ecall();
        Memory m = cputest::image(p.assemble());
        Cpu cpu(m, Config{}, wl::TEXT);
        REQUIRE(cpu.run(1000));

        REQUIRE(cpu.exit_code() == 41);       // it waited, and then forwarded
        REQUIRE(cpu.stats().load_replays > 0);
        REQUIRE(cpu.stats().stall_count(Stall::STORE_ORDER) > 0);
    }

    // ---- A starved machine blames one of the things it was starved of -----
    {
        Config cfg;
        cfg.rob_size        = 4;
        cfg.prf_size        = 40;
        cfg.iq_size         = 2;
        cfg.lq_size         = 1;
        cfg.sq_size         = 1;
        cfg.num_checkpoints = 1;

        for (const wl::Workload& w : wl::corpus()) {
            const Stall dominant = stattest::run(w, cfg).dominant_stall();
            const bool starved_resource =
                dominant == Stall::ROB_FULL   || dominant == Stall::IQ_FULL    ||
                dominant == Stall::LQ_FULL    || dominant == Stall::SQ_FULL    ||
                dominant == Stall::PHYSREG    || dominant == Stall::CHECKPOINT ||
                dominant == Stall::DIV_PORT;   // muldiv is slower than any queue
            REQUIRE_MSG(starved_resource,
                        "    " + w.name + ": blamed " + stall_name(dominant));
        }
    }

    // ---- The predictor learns, and then the loop is free ------------------
    // The honest statement of "it learned" is not a rate — it is that ten
    // times the iterations cost the same number of mispredicts. What the
    // machine pays for is filling the history once; a coin flip would pay
    // half of every trip forever.
    {
        auto spin = [](int32_t trips) {
            Assembler p;
            p.li(t0, 0);
            p.li(t1, trips);
            p.label("spin");
            p.addi(t0, t0, 1);
            p.blt(t0, t1, "spin");
            p.mv(a0, t0);
            p.li(a7, 93);
            p.ecall();
            Memory m = cputest::image(p.assemble());
            Cpu cpu(m, Config{}, wl::TEXT);
            REQUIRE(cpu.run(50000));
            REQUIRE(cpu.exit_code() == static_cast<uint32_t>(trips));
            return cpu.stats();
        };
        const Stats few  = spin(300);
        const Stats many = spin(3000);

        REQUIRE(many.mispredicts == few.mispredicts);
        REQUIRE(few.mispredicts <= Config{}.ghr_bits + 4);   // history fill, and no more
        REQUIRE(many.mispredict_rate() < 0.01);
    }

    // ---- crc32 agrees with zlib, byte for byte ----------------------------
    // The only check in the suite that does not appeal to ref.h: an outside
    // authority fixed this number long before this simulator existed.
    {
        std::vector<uint8_t> bytes(256);
        for (int i = 0; i < 256; ++i) bytes[i] = static_cast<uint8_t>(i);
        REQUIRE(props::crc32_table_driven(bytes) == 0x29058C73u);   // zlib's answer

        Memory m = cputest::image(stattest::named("crc32").words);
        Cpu cpu(m, Config{}, wl::TEXT);
        REQUIRE(cpu.run(200000));
        REQUIRE(cpu.halted());
        REQUIRE(cpu.exit_code() == props::crc32_table_driven(bytes));
    }
}


// ---------------------------------------------------- @section("examples") ---
namespace extest {

// The four machines the CLI's --ipc-table prints, so the numbers in the
// documentation and the numbers asserted here come from the same place.
inline Config narrow()  { Config c; c.width = 1; return c; }
inline Config wide() {
    Config c;
    c.width = 4; c.rob_size = 128; c.prf_size = 160; c.iq_size = 32;
    c.num_alu = 4; c.num_cdb = 4;
    return c;
}
inline Config tiny() {
    Config c;
    c.rob_size = 4; c.prf_size = 40; c.iq_size = 2;
    return c;
}

// Loads a generated example the way the CLI would, and hands back exactly the
// words the file held — the loader places them in memory, and the file's own
// data lines say how many there are.
inline std::optional<std::vector<uint32_t>> read_hex(const std::string& path) {
    std::ifstream count_pass(path);
    if (!count_pass) return std::nullopt;
    std::size_t n = 0;
    for (std::string line; std::getline(count_pass, line); ) {
        if (line.find_first_of("#/") == 0) continue;
        if (line.find_first_not_of(" \t\r") != std::string::npos) ++n;
    }

    std::ifstream in(path);
    Memory m;
    const LoadResult r = load_hex(m, in, wl::TEXT);
    std::vector<uint32_t> words;
    for (std::size_t i = 0; i < n; ++i) {
        words.push_back(m.load_u32(r.entry + static_cast<uint32_t>(i * 4)));
    }
    return words;
}

inline double ipc_of(const std::vector<uint32_t>& words, const Config& cfg) {
    Memory m = cputest::image(words);
    Cpu cpu(m, cfg, wl::TEXT);
    REQUIRE(cpu.run(2'000'000));
    return cpu.stats().ipc();
}

}  // namespace extest

SECTION("examples") {
    using namespace extest;

    // ---- What the generator wrote is what the corpus assembled ------------
    // The examples are the shipped copy of programs the rest of the suite
    // validates; if they can drift, running them proves nothing.
    for (const char* name : {"sieve", "matmul", "bubble_sort", "fib", "crc32"}) {
        const std::string path = std::string("examples/") + name + ".hex";
        const std::optional<std::vector<uint32_t>> loaded = read_hex(path);
        REQUIRE_MSG(loaded.has_value(), "    missing " + path + " (run: make examples)");

        const wl::Workload& w = stattest::named(name);
        REQUIRE(loaded->size() == w.words.size());
        for (std::size_t i = 0; i < w.words.size(); ++i) {
            REQUIRE_MSG((*loaded)[i] == w.words[i],
                        std::string("    ") + name + ".hex differs from the corpus");
        }

        Memory m = cputest::image(*loaded);
        Cpu cpu(m, Config{}, wl::TEXT);
        REQUIRE(cpu.run(2'000'000));
        REQUIRE(cpu.halted());
        REQUIRE(cpu.exit_code() == *w.expect_exit);
    }

    // ---- The IPC table says what the documentation says --------------------
    {
        const std::vector<uint32_t>& matmul = stattest::named("matmul").words;
        const std::vector<uint32_t>& fib    = stattest::named("fib").words;
        const std::vector<uint32_t>& crc    = stattest::named("crc32").words;

        // matmul has independent work to find, so width pays twice over.
        REQUIRE(ipc_of(matmul, Config{}) > 1.6 * ipc_of(matmul, narrow()));
        REQUIRE(ipc_of(matmul, wide())   > 1.5 * ipc_of(matmul, Config{}));

        // fib runs out of memory ports long before it runs out of width.
        REQUIRE(ipc_of(fib, wide()) < 1.1 * ipc_of(fib, Config{}));

        // crc32 is a dependent bit-serial loop; width barely helps it.
        REQUIRE(ipc_of(crc, wide()) < 1.3 * ipc_of(crc, Config{}));

        // A four-entry window erases the benefit of width entirely.
        REQUIRE(ipc_of(matmul, tiny()) < ipc_of(matmul, narrow()));
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


// --------------------------------------------------------- @section("rat") ---
SECTION("rat") {
    Config cfg;                    // num_checkpoints = 16
    Rat rat(cfg);

    // ---- Reset state: identity mapping and full pool --------------------
    for (ArchReg a = 0; a < Rat::ARCH_REGS; ++a) REQUIRE(rat.map(a) == a);
    REQUIRE(rat.num_checkpoints()      == cfg.num_checkpoints);
    REQUIRE(rat.num_free_checkpoints() == cfg.num_checkpoints);

    // ---- set() writes; set(0, _) is a no-op ----------------------------
    rat.set(5, 40);
    REQUIRE(rat.map(5) == 40u);
    rat.set(0, 99);
    REQUIRE(rat.map(0) == 0u);
    rat.set(31, 63);
    REQUIRE(rat.map(31) == 63u);

    // ---- alloc_checkpoint snapshots the current state -------------------
    const auto cp = rat.alloc_checkpoint();
    REQUIRE(cp.has_value());
    REQUIRE(rat.num_free_checkpoints() == cfg.num_checkpoints - 1);

    // ---- Restore reproduces the RAT bit-identical -----------------------
    // Snapshot the pre-restore mapping so we can compare byte-for-byte.
    const auto expected = rat.mapping();
    // Arbitrary writes on top of the snapshot.
    for (ArchReg a = 1; a < Rat::ARCH_REGS; ++a) rat.set(a, 100 + a);
    // The RAT is now different.
    bool differ = false;
    for (ArchReg a = 0; a < Rat::ARCH_REGS; ++a)
        if (rat.mapping()[a] != expected[a]) differ = true;
    REQUIRE(differ);
    // Restore.
    rat.restore_checkpoint(*cp);
    for (ArchReg a = 0; a < Rat::ARCH_REGS; ++a)
        REQUIRE(rat.mapping()[a] == expected[a]);

    // ---- Restore does not release the slot ------------------------------
    REQUIRE(rat.num_free_checkpoints() == cfg.num_checkpoints - 1);
    // Restoring twice is idempotent.
    rat.set(5, 999);
    rat.restore_checkpoint(*cp);
    REQUIRE(rat.map(5) == expected[5]);

    // ---- free_checkpoint returns the slot -------------------------------
    rat.free_checkpoint(*cp);
    REQUIRE(rat.num_free_checkpoints() == cfg.num_checkpoints);

    // ---- Pool refuses allocation when full instead of overwriting -------
    std::vector<CheckpointId> held;
    while (auto id = rat.alloc_checkpoint()) held.push_back(*id);
    REQUIRE(held.size() == cfg.num_checkpoints);
    REQUIRE(!rat.alloc_checkpoint().has_value());
    // Every id is distinct.
    for (std::size_t i = 0; i < held.size(); ++i)
        for (std::size_t j = i + 1; j < held.size(); ++j)
            REQUIRE(held[i] != held[j]);
    // Free one, alloc succeeds.
    rat.free_checkpoint(held[0]);
    const auto again = rat.alloc_checkpoint();
    REQUIRE(again.has_value());
    REQUIRE(!rat.alloc_checkpoint().has_value());

    // ---- free_checkpoint tolerates the sentinels commit paths pass ------
    rat.free_checkpoint(INVALID_CHECKPOINT);
    rat.free_checkpoint(cfg.num_checkpoints);
    rat.free_checkpoint(cfg.num_checkpoints + 10);

    // ---- Independent snapshots survive intervening writes ---------------
    // Take two snapshots of different states, restore both, verify each
    // reproduces its own state and does not leak into the other.
    Rat r2(4);
    r2.set(1, 100);
    const auto snap_a = r2.alloc_checkpoint();
    r2.set(1, 200);
    const auto snap_b = r2.alloc_checkpoint();
    r2.set(1, 300);              // now RAT[1] = 300, neither snapshot
    r2.restore_checkpoint(*snap_a);
    REQUIRE(r2.map(1) == 100u);
    r2.restore_checkpoint(*snap_b);
    REQUIRE(r2.map(1) == 200u);
    r2.restore_checkpoint(*snap_a);
    REQUIRE(r2.map(1) == 100u);

    // ---- reset() restores identity mapping and full pool ----------------
    rat.reset();
    for (ArchReg a = 0; a < Rat::ARCH_REGS; ++a) REQUIRE(rat.map(a) == a);
    REQUIRE(rat.num_free_checkpoints() == cfg.num_checkpoints);
}


// ---------------------------------------------------- @section("freelist") ---
SECTION("freelist") {
    // ---- Default sizing: |free| = prf_size - 32 -------------------------
    Config cfg;                     // prf_size = 64
    FreeList fl(cfg);
    REQUIRE(fl.capacity() == cfg.prf_size);
    REQUIRE(fl.num_free() == cfg.prf_size - 32);

    // ---- Every alloc returns a register in [32, prf_size) ---------------
    // p0..p31 are never handed out; the initial RAT holds them.
    std::vector<PhysReg> seen;
    while (auto r = fl.alloc()) seen.push_back(*r);
    REQUIRE(seen.size() == cfg.prf_size - 32);
    for (PhysReg r : seen) {
        REQUIRE(r >= 32);
        REQUIRE(r <  cfg.prf_size);
    }
    // And every one is distinct.
    for (std::size_t i = 0; i < seen.size(); ++i)
        for (std::size_t j = i + 1; j < seen.size(); ++j)
            REQUIRE(seen[i] != seen[j]);

    // ---- Starvation reports itself; no phantom p0 ----------------------
    REQUIRE(fl.empty());
    REQUIRE(!fl.alloc().has_value());

    // ---- free() pushes back, alloc() sees them again --------------------
    fl.free(40);
    fl.free(41);
    REQUIRE(fl.num_free() == 2);
    const PhysReg first  = fl.alloc().value();
    const PhysReg second = fl.alloc().value();
    REQUIRE(first  == 40);          // FIFO order
    REQUIRE(second == 41);
    REQUIRE(fl.empty());

    // ---- free() silently ignores the values commit is allowed to pass ---
    // The commit path calls free(stale_phys) unconditionally; stale_phys is
    // INVALID_PHYSREG when the retiring uop had writes_rd == false, and
    // could conceivably be 0 through some future path. Both must be safe.
    fl.free(0);
    fl.free(INVALID_PHYSREG);
    fl.free(cfg.prf_size);          // one past the end
    fl.free(cfg.prf_size + 5);
    REQUIRE(fl.empty());
    REQUIRE(!fl.alloc().has_value());

    // ---- reset() restores the initial state -----------------------------
    for (uint32_t r = 32; r < cfg.prf_size; ++r) fl.free(r);
    (void)fl.alloc();               // remove one
    fl.reset();
    REQUIRE(fl.num_free() == cfg.prf_size - 32);

    // ---- Alloc / free invariant under a random-ish sequence -------------
    // |free| + |allocated| == prf_size - 32 after every step. Reset first
    // so the checked size is known.
    fl.reset();
    std::vector<PhysReg> live;
    const uint32_t init_free = fl.num_free();
    // Simple LCG so the sequence is deterministic and the failure is
    // reproducible.
    uint32_t s = 0x1234;
    for (int step = 0; step < 500; ++step) {
        s = s * 1103515245u + 12345u;
        const bool do_alloc = live.empty() || (s & 1);
        if (do_alloc) {
            auto r = fl.alloc();
            if (r) live.push_back(*r);
        } else {
            const uint32_t idx = (s >> 1) % live.size();
            fl.free(live[idx]);
            live[idx] = live.back();
            live.pop_back();
        }
        REQUIRE(fl.num_free() + live.size() == init_free);
    }

    // ---- Starvation-config: prf_size < rob_size + 32 ---------------------
    // The design invariant is that this can actually run out mid-run rather
    // than merely stalling occasionally. Drain it and observe.
    Config stress;                  // rob = 32, prf = 64 default → starvation-free
    stress.prf_size = stress.rob_size + 4;   // deep enough to trip
    REQUIRE(stress.prf_can_starve());
    FreeList tight(stress);
    // The initial free set is prf_size - 32 = rob + 4 - 32 = 4 entries. So
    // any workload with 5+ in-flight dest-writing uops on top of the 32
    // arch-visible mappings starves.
    REQUIRE(tight.num_free() == stress.prf_size - 32);
    uint32_t handed_out = 0;
    while (tight.alloc()) ++handed_out;
    REQUIRE(handed_out == stress.prf_size - 32);
    REQUIRE(!tight.alloc().has_value());   // starved, reported as such
}


// --------------------------------------------------------- @section("prf") ---
SECTION("prf") {
    Config cfg;
    Prf prf(cfg);
    REQUIRE(prf.capacity() == cfg.prf_size);

    // ---- reset state: every register zero and ready ---------------------
    for (uint32_t r = 0; r < prf.capacity(); ++r) {
        REQUIRE(prf.read(r) == 0u);
        REQUIRE(prf.is_ready(r));
    }

    // ---- basic write / read / ready round-trip --------------------------
    prf.write(5, 0xDEADBEEF);
    REQUIRE(prf.read(5) == 0xDEADBEEFu);
    REQUIRE(prf.is_ready(5));

    prf.write(cfg.prf_size - 1, 0x12345678);
    REQUIRE(prf.read(cfg.prf_size - 1) == 0x12345678u);
    REQUIRE(prf.is_ready(cfg.prf_size - 1));

    // ---- mark_pending clears ready without touching the value -----------
    // The stale value survives; the IQ is what prevents a stale read, not
    // the PRF. Verifying the value here pins that PRF is storage, not
    // policy.
    prf.mark_pending(5);
    REQUIRE(!prf.is_ready(5));
    REQUIRE(prf.read(5) == 0xDEADBEEFu);
    prf.write(5, 42);
    REQUIRE(prf.is_ready(5));
    REQUIRE(prf.read(5) == 42u);

    // ---- p0 is hard-wired to zero and always ready ----------------------
    REQUIRE(prf.read(0) == 0u);
    REQUIRE(prf.is_ready(0));
    prf.write(0, 0xFFFFFFFFu);
    REQUIRE(prf.read(0) == 0u);
    REQUIRE(prf.is_ready(0));
    prf.mark_pending(0);
    REQUIRE(prf.is_ready(0));
    REQUIRE(prf.read(0) == 0u);

    // ---- reset() returns to the initial state ---------------------------
    prf.mark_pending(1);
    prf.mark_pending(2);
    prf.write(10, 0xABCDABCD);
    prf.reset();
    for (uint32_t r = 0; r < prf.capacity(); ++r) {
        REQUIRE(prf.read(r) == 0u);
        REQUIRE(prf.is_ready(r));
    }

    // ---- Raw-size constructor for tests that need an atypical PRF -------
    Prf tiny(4);
    REQUIRE(tiny.capacity() == 4);
    tiny.write(3, 99);
    REQUIRE(tiny.read(3) == 99u);
    REQUIRE(tiny.is_ready(3));
    tiny.mark_pending(3);
    REQUIRE(!tiny.is_ready(3));
    // p0 hard-wired even in a starved PRF where p0 would otherwise be free.
    tiny.mark_pending(0);
    REQUIRE(tiny.is_ready(0));
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
