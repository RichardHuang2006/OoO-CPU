#pragma once

// Shared test infrastructure. Each tests/test_*.cpp is a standalone binary:
// it includes this header exactly once, registers its SECTION() blocks, and
// gets the main() defined at the bottom. The helpers here are the plumbing
// every suite needs — the assertion macros, the differential-run scaffolding
// against the reference interpreter, and the common ways of building and
// running a program on the pipeline.

#include <cstdint>
#include <cstdio>

#include <algorithm>
#include <functional>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "config.h"
#include "cpu.h"
#include "instruction.h"
#include "memory.h"

#include "asm.h"
#include "ref.h"
#include "workloads.h"

// ---------------------------------------------------------------- harness ---
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

// ---------------------------------------------- reference-interpreter runs ---
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

// ========================================================= differential run ===
// `diff_run` runs a workload on the reference and on the model under test, then
// compares all 32 architectural registers, the exit code, and the
// retired-instruction count.
//
// The model is a swappable function defaulting to the reference itself, so the
// comparison can be exercised against a known-equal pair and against
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
                      name.c_str(), i, reg_name(static_cast<ArchReg>(i)),
                      want.regs[i], got.regs[i]);
        d << buf;
    }

    return {ok, d.str()};
}

// Run `w` on both layers and compare. A reference run that did not halt cleanly
// is reported as a broken workload rather than a model mismatch.
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

// ------------------------------------------------------------ pipeline runs ---
namespace cputest {

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

// -------------------------------------------------- conservation invariant ---
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

// ------------------------------------------------------- program templates ---
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

// --------------------------------------------------------- stats-run helpers ---
namespace stattest {

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

// ------------------------------------------------------------------- main ---
// Defined here because every test binary is exactly one translation unit;
// linking two test files into one binary would double-define it on purpose,
// since that is a build-system mistake worth catching.
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
