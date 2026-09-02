// Differential validation: the in-order reference interpreter itself, the
// scaffolding that compares it against the pipeline, the full workload corpus
// on a matrix of machine configurations, microarchitectural property
// assertions, the CRC-32 external reference, and the shipped examples.

#include <cstdio>
#include <fstream>

#include "loader.h"
#include "test_support.h"

// --------------------------------------------------------- @section("ref") ---
SECTION("ref") {
    using namespace asmc;
    using reftest::DATA;
    using reftest::TEXT;

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
        // ecall with a7 != 93 is not the exit syscall, so it traps.
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

// ----------------------------------------------- @section("config_matrix") ---
// The whole corpus, against the interpreter, on every configuration in a
// hand-picked matrix. Same registers, same exit code, same retired count as
// ref.h. A machine that is only correct at one width is not correct.
SECTION("config_matrix") {
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

// ------------------------------------------------ @section("config_sweep") ---
namespace sweep {

struct Machine {
    const char* name;
    Config      cfg;
};

// Six machines that stress different parts of the same design. Correctness must
// be identical on all of them, whereas renaming, wakeup and recovery bugs are
// usually configuration-dependent.
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
            REQUIRE(cpu.checkpoints().num_free() == cpu.checkpoints().capacity());
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
    // Learning shows up as a constant, not a rate: ten times the iterations
    // cost the same number of mispredicts, because the only cost is filling the
    // history once. An unpredicted branch would pay on half of every trip.
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
    // The only check in the suite that does not appeal to ref.h: the expected
    // value is fixed by an external reference implementation.
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

// Loads a generated example the way the CLI would and returns exactly the words
// the file held: the loader places them in memory, and the file's own data lines
// give the count.
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
