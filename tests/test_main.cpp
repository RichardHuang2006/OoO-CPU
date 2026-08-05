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
