// Validation suite: every program is run on the out-of-order core and on the
// in-order reference interpreter, and the committed architectural state must
// match. Programs are then re-run across a range of machine configurations --
// widths, ROB/PRF/IQ/LSQ sizes, CDB counts -- since any renaming, wakeup, or
// recovery bug shows up as a configuration-dependent result.
#include <cstdio>
#include <functional>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "../src/cpu.h"
#include "../src/loader.h"
#include "../src/memory.h"
#include "asm.h"
#include "ref.h"

using namespace rvasm;

namespace {

int g_failures = 0;
int g_checks = 0;

constexpr uint32_t kBase = 0x1000;
constexpr uint32_t kSp   = 0x80000;

struct Program {
    std::string name;
    std::vector<uint32_t> code;
};

void fail(const std::string& what) {
    std::cout << "  FAIL: " << what << "\n";
    g_failures++;
}

void check(bool cond, const std::string& what) {
    g_checks++;
    if (!cond) fail(what);
}

Memory make_memory(const Program& p) {
    Memory m;
    for (size_t i = 0; i < p.code.size(); i++)
        m.write32(kBase + (uint32_t)i * 4, p.code[i]);
    return m;
}

// Run one program on one configuration and compare against the reference.
Stats run_and_compare(const Program& p, const Config& cfg, const std::string& tag) {
    Memory ref_mem = make_memory(p);
    const RefResult ref = run_reference(ref_mem, kBase, kSp);

    Memory mem = make_memory(p);
    Config c = cfg;
    c.normalize();
    Cpu cpu(c, mem, kBase);
    cpu.set_arch_reg(2, kSp);
    cpu.run();

    const std::string where = p.name + " [" + tag + "]";

    g_checks++;
    if (!cpu.halted() || cpu.halt_reason() == "cycle limit reached" ||
        cpu.halt_reason().rfind("deadlock", 0) == 0) {
        fail(where + ": did not halt cleanly (" + cpu.halt_reason() + ")");
        return cpu.stats();
    }

    for (int r = 0; r < 32; r++) {
        g_checks++;
        if (cpu.arch_reg(r) != ref.regs[r]) {
            char buf[256];
            std::snprintf(buf, sizeof buf,
                          "%s: x%d = 0x%08x, reference 0x%08x",
                          where.c_str(), r, cpu.arch_reg(r), ref.regs[r]);
            fail(buf);
        }
    }

    g_checks++;
    if (cpu.exit_code() != ref.exit_code)
        fail(where + ": exit code mismatch");

    // Committed instruction count must equal the reference's dynamic count:
    // no wrong-path instruction may ever retire.
    g_checks++;
    if (cpu.stats().committed != ref.instructions) {
        char buf[256];
        std::snprintf(buf, sizeof buf, "%s: committed %llu, reference executed %llu",
                      where.c_str(), (unsigned long long)cpu.stats().committed,
                      (unsigned long long)ref.instructions);
        fail(buf);
    }

    return cpu.stats();
}

std::vector<std::pair<std::string, Config>> configurations() {
    std::vector<std::pair<std::string, Config>> v;

    Config def;                              // the documented default machine
    v.emplace_back("default 2-wide", def);

    Config scalar = def;
    scalar.fetch_width = scalar.decode_width = scalar.rename_width =
        scalar.dispatch_width = scalar.issue_width = scalar.commit_width = 1;
    scalar.num_alu = 1; scalar.num_cdbs = 1;
    v.emplace_back("1-wide, 1 CDB", scalar);

    Config wide = def;
    wide.fetch_width = wide.decode_width = wide.rename_width =
        wide.dispatch_width = wide.issue_width = wide.commit_width = 4;
    wide.rob_entries = 128; wide.phys_regs = 160; wide.iq_entries = 32;
    wide.lq_entries = 16; wide.sq_entries = 16; wide.num_cdbs = 4;
    wide.num_alu = 4; wide.num_mem = 2;
    v.emplace_back("4-wide, rob=128", wide);

    Config tiny = def;                        // maximal structural pressure
    tiny.rob_entries = 4; tiny.phys_regs = 36; tiny.iq_entries = 2;
    tiny.lq_entries = 1; tiny.sq_entries = 1; tiny.num_cdbs = 1;
    tiny.num_alu = 1; tiny.num_mem = 1; tiny.max_checkpoints = 1;
    tiny.fetch_queue = 2; tiny.decode_queue = 2; tiny.dispatch_queue = 2;
    v.emplace_back("tiny rob=4 iq=2 chkpt=1", tiny);

    Config slow = def;                        // stress wakeup at long latencies
    slow.mem_latency = 5; slow.mul_latency = 8; slow.alu_latency = 2;
    v.emplace_back("long latencies", slow);

    Config nopred = def;                      // defeat the predictors
    nopred.pht_entries = 2; nopred.ghr_bits = 1;
    nopred.btb_sets = 1; nopred.btb_ways = 1; nopred.ras_entries = 1;
    v.emplace_back("degenerate predictors", nopred);

    return v;
}

// ---------------------------------------------------------------------------
// Test programs
// ---------------------------------------------------------------------------

Program prog_alu() {
    Asm a(kBase);
    a.li(t0, 0x12345678);
    a.li(t1, -19088744);            // 0xfedcba98
    a.add(s0, t0, t1);
    a.sub(s1, t0, t1);
    a.and_(s2, t0, t1);
    a.or_(s3, t0, t1);
    a.xor_(s4, t0, t1);
    a.sll(s5, t0, t1);              // shift amount is t1 & 31
    a.srl(s6, t0, t1);
    a.sra(s7, t1, t0);
    a.slt(s8, t0, t1);
    a.sltu(s9, t0, t1);
    a.addi(s10, t0, -2048);
    a.slti(s11, t1, 5);
    a.sltiu(t2, t1, 5);
    a.xori(t3, t0, -1);
    a.ori(t4, t0, 0x7ff);
    a.andi(t5, t0, 0x0f0);
    a.slli(t6, t0, 13);
    a.srli(a1, t0, 7);
    a.srai(a2, t1, 3);
    a.lui(a3, 0xdeadb000u);
    a.auipc(a4, 0x1000);
    a.add(a5, x0, x0);              // writes to x0-adjacent regs
    a.addi(x0, t0, 5);              // must not change x0
    a.exit_(0);
    return {"alu", a.code()};
}

Program prog_loop_sum() {
    Asm a(kBase);
    a.li(t0, 0);                    // accumulator
    a.li(t1, 1);                    // i
    a.li(t2, 101);                  // bound
    a.label("loop");
    a.add(t0, t0, t1);
    a.addi(t1, t1, 1);
    a.blt(t1, t2, "loop");          // highly predictable backward branch
    a.mv(s0, t0);                   // 5050
    a.exit_(0);
    return {"loop_sum", a.code()};
}

Program prog_data_hazards() {
    // A long dependent chain interleaved with an independent one: exercises
    // back-to-back wakeup and out-of-order completion in the same window.
    Asm a(kBase);
    a.li(t0, 1);
    a.li(t1, 1);
    a.li(t2, 40);
    a.label("loop");
    a.add(t0, t0, t0);              // dependent chain
    a.addi(t0, t0, 3);
    a.xori(t0, t0, 0x5a);
    a.add(t1, t1, t1);              // independent chain
    a.addi(t1, t1, 1);
    a.addi(t2, t2, -1);
    a.bne(t2, x0, "loop");
    a.exit_(0);
    return {"data_hazards", a.code()};
}

Program prog_mem() {
    Asm a(kBase);
    a.li(s0, 0x40000);              // array base
    a.li(t0, 0);                    // i
    a.li(t1, 32);
    a.label("fill");
    a.slli(t2, t0, 2);
    a.add(t2, s0, t2);
    a.addi(t3, t0, 100);
    a.sw(t3, t2, 0);
    a.addi(t0, t0, 1);
    a.blt(t0, t1, "fill");

    a.li(t0, 0);
    a.li(s1, 0);                    // sum
    a.label("read");
    a.slli(t2, t0, 2);
    a.add(t2, s0, t2);
    a.lw(t3, t2, 0);
    a.add(s1, s1, t3);
    a.addi(t0, t0, 1);
    a.blt(t0, t1, "read");
    a.exit_(0);
    return {"mem_array", a.code()};
}

Program prog_store_forward() {
    // Every load hits a store still sitting in the store queue.
    Asm a(kBase);
    a.li(s0, 0x40000);
    a.li(t0, 0);
    a.li(t1, 24);
    a.li(s1, 0);
    a.label("loop");
    a.addi(t2, t0, 7);
    a.sw(t2, s0, 0);                // same address every iteration
    a.lw(t3, s0, 0);                // forwarded from the store queue
    a.add(s1, s1, t3);
    a.sw(t3, s0, 4);
    a.lw(t4, s0, 4);
    a.add(s1, s1, t4);
    a.addi(t0, t0, 1);
    a.blt(t0, t1, "loop");
    a.exit_(0);
    return {"store_forward", a.code()};
}

Program prog_partial_mem() {
    // Byte / halfword accesses, sign and zero extension, and partial overlap
    // between a wide store and narrow loads (which must not be forwarded).
    Asm a(kBase);
    a.li(s0, 0x40000);
    a.li(t0, 0xdeadbeefu);
    a.sw(t0, s0, 0);
    a.lb(s1, s0, 0);
    a.lbu(s2, s0, 0);
    a.lh(s3, s0, 2);
    a.lhu(s4, s0, 2);
    a.li(t1, -3);
    a.sb(t1, s0, 8);
    a.lb(s5, s0, 8);
    a.lbu(s6, s0, 8);
    a.li(t2, -300);
    a.sh(t2, s0, 12);
    a.lh(s7, s0, 12);
    a.lhu(s8, s0, 12);
    a.lw(s9, s0, 0);
    a.exit_(0);
    return {"partial_mem", a.code()};
}

Program prog_calls() {
    // Nested calls and returns: drives the RAS and its checkpoint/restore.
    Asm a(kBase);
    a.li(s0, 0);
    a.li(t0, 0);
    a.li(t1, 20);
    a.label("loop");
    a.mv(a0, t0);
    a.call("triple");
    a.add(s0, s0, a0);
    a.addi(t0, t0, 1);
    a.blt(t0, t1, "loop");
    a.exit_(0);

    a.label("triple");
    a.addi(sp, sp, -8);
    a.sw(ra, sp, 0);
    a.sw(a0, sp, 4);
    a.call("dbl");
    a.lw(t2, sp, 4);
    a.add(a0, a0, t2);              // 3x
    a.lw(ra, sp, 0);
    a.addi(sp, sp, 8);
    a.ret();

    a.label("dbl");
    a.add(a0, a0, a0);
    a.ret();
    return {"calls", a.code()};
}

Program prog_fib_recursive() {
    // fib(14) recursively: deep call nesting, many returns, unpredictable
    // branch behaviour near the base cases.
    Asm a(kBase);
    a.li(a0, 14);
    a.call("fib");
    a.mv(s0, a0);
    a.exit_(0);

    a.label("fib");
    a.li(t0, 2);
    a.bge(a0, t0, "recurse");
    a.ret();                        // fib(0)=0, fib(1)=1
    a.label("recurse");
    a.addi(sp, sp, -12);
    a.sw(ra, sp, 0);
    a.sw(a0, sp, 4);
    a.addi(a0, a0, -1);
    a.call("fib");
    a.sw(a0, sp, 8);                // fib(n-1)
    a.lw(a0, sp, 4);
    a.addi(a0, a0, -2);
    a.call("fib");
    a.lw(t1, sp, 8);
    a.add(a0, a0, t1);
    a.lw(ra, sp, 0);
    a.addi(sp, sp, 12);
    a.ret();
    return {"fib_recursive", a.code()};
}

Program prog_mispredict() {
    // Data-dependent, deliberately hard-to-predict branch driven by a simple
    // LCG, so recovery runs constantly.
    Asm a(kBase);
    a.li(s0, 12345);                // rng state
    a.li(s1, 0);                    // taken count
    a.li(t0, 0);
    a.li(t1, 200);
    a.li(s2, 1103515245);
    a.li(s3, 12345);
    a.label("loop");
    a.mul(s0, s0, s2);
    a.add(s0, s0, s3);
    a.srli(t2, s0, 16);
    a.andi(t2, t2, 1);
    a.beq(t2, x0, "skip");
    a.addi(s1, s1, 1);
    a.addi(s4, s1, 3);
    a.label("skip");
    a.addi(t0, t0, 1);
    a.blt(t0, t1, "loop");
    a.exit_(0);
    return {"mispredict_lcg", a.code()};
}

Program prog_muldiv() {
    Asm a(kBase);
    a.li(t0, 123456);
    a.li(t1, -789);
    a.mul(s0, t0, t1);
    a.mulh(s1, t0, t1);
    a.mulhu(s2, t0, t1);
    a.div(s3, t0, t1);
    a.divu(s4, t0, t1);
    a.rem(s5, t0, t1);
    a.li(t2, 0);
    a.div(s6, t0, t2);              // division by zero is defined, not a trap
    a.rem(s7, t0, t2);
    a.mul(s8, s0, s1);              // dependent on a multi-cycle producer
    a.add(s9, s8, s0);
    a.exit_(0);
    return {"muldiv", a.code()};
}

Program prog_load_use_chain() {
    // Pointer chase: each load depends on the previous load's result, so the
    // machine is limited by load use latency, not by width.
    Asm a(kBase);
    a.li(s0, 0x40000);
    a.li(t0, 0);
    a.li(t1, 16);
    a.label("build");               // node[i] -> node[i+1]
    a.slli(t2, t0, 2);
    a.add(t2, s0, t2);
    a.addi(t3, t2, 4);
    a.sw(t3, t2, 0);
    a.addi(t0, t0, 1);
    a.blt(t0, t1, "build");

    a.sw(x0, s0, 64);               // terminate the list
    a.mv(t4, s0);
    a.li(s1, 0);
    a.label("chase");
    a.beq(t4, x0, "done");
    a.lw(t4, t4, 0);
    a.addi(s1, s1, 1);
    a.j("chase");
    a.label("done");
    a.exit_(0);
    return {"load_use_chain", a.code()};
}

Program prog_waw_war() {
    // Repeated writes to the same architectural register from independent
    // producers: correct only if renaming removes the WAW/WAR dependences.
    Asm a(kBase);
    a.li(t0, 5);
    a.li(t1, 7);
    a.mul(s0, t0, t1);              // slow producer of s0
    a.li(s0, 1);                    // WAW with the multiply
    a.addi(t0, s0, 1);              // must read the *new* s0
    a.mul(s1, t0, t1);
    a.li(s1, 2);
    a.addi(t1, s1, 1);
    a.li(s0, 3);
    a.add(s2, s0, s1);
    a.li(s0, 4);                    // WAR against the add above
    a.add(s3, s2, s0);
    a.exit_(0);
    return {"waw_war", a.code()};
}

Program prog_ecall_exit_code() {
    Asm a(kBase);
    a.li(t0, 20);
    a.li(t1, 22);
    a.add(a0, t0, t1);
    a.li(a7, 93);
    a.ecall();
    return {"exit_code", a.code()};
}

std::vector<Program> all_programs() {
    return {prog_alu(),           prog_loop_sum(),      prog_data_hazards(),
            prog_mem(),           prog_store_forward(), prog_partial_mem(),
            prog_calls(),         prog_fib_recursive(), prog_mispredict(),
            prog_muldiv(),        prog_load_use_chain(), prog_waw_war(),
            prog_ecall_exit_code()};
}

// ---------------------------------------------------------------------------
// Targeted checks beyond "same architectural state"
// ---------------------------------------------------------------------------

void test_expected_values() {
    std::cout << "known-value checks\n";

    struct Case { Program p; int reg; uint32_t want; const char* what; };
    std::vector<Case> cases = {
        {prog_loop_sum(), s0, 5050, "sum 1..100"},
        {prog_fib_recursive(), s0, 377, "fib(14)"},
        {prog_calls(), s0, 3 * (0 + 19) * 20 / 2, "3 * sum(0..19)"},
        {prog_mem(), s1, 32 * 100 + (31 * 32 / 2), "array sum"},
        {prog_load_use_chain(), s1, 17, "pointer chase length"},
    };

    for (auto& c : cases) {
        Memory mem = make_memory(c.p);
        Config cfg;
        cfg.normalize();
        Cpu cpu(cfg, mem, kBase);
        cpu.set_arch_reg(2, kSp);
        cpu.run();
        g_checks++;
        if (cpu.arch_reg(c.reg) != c.want) {
            char buf[256];
            std::snprintf(buf, sizeof buf, "%s: got %u, want %u", c.what,
                          cpu.arch_reg(c.reg), c.want);
            fail(buf);
        }
    }

    // ECALL must deliver a0 as the exit status.
    {
        Program p = prog_ecall_exit_code();
        Memory mem = make_memory(p);
        Config cfg; cfg.normalize();
        Cpu cpu(cfg, mem, kBase);
        cpu.run();
        check(cpu.exit_code() == 42, "ecall exit code == 42");
        check(cpu.halt_reason() == "exit syscall", "halt reason is exit syscall");
    }
}

void test_x0_is_hardwired() {
    std::cout << "x0 semantics\n";
    Asm a(kBase);
    a.li(t0, 1234);
    a.add(x0, t0, t0);
    a.addi(x0, t0, 1);
    a.lui(x0, 0x40000000);
    a.add(s0, x0, x0);
    a.exit_(0);
    Program p{"x0", a.code()};

    Memory mem = make_memory(p);
    Config cfg; cfg.normalize();
    Cpu cpu(cfg, mem, kBase);
    cpu.run();
    check(cpu.arch_reg(0) == 0, "x0 stays zero");
    check(cpu.arch_reg(s0) == 0, "reads of x0 return zero");
}

void test_no_physreg_leak() {
    // Run a long program, then confirm the machine still retires at full speed
    // on a second identical run -- a leaked physical register or ROB entry
    // would show up as a deadlock or a collapsed IPC.
    std::cout << "resource reclamation\n";
    Program p = prog_fib_recursive();
    Config cfg; cfg.normalize();

    Memory mem = make_memory(p);
    Cpu cpu(cfg, mem, kBase);
    cpu.set_arch_reg(2, kSp);
    cpu.run();
    check(cpu.halt_reason() == "exit syscall", "fib run terminates via ecall");
    const double ipc = (double)cpu.stats().committed / (double)cpu.stats().cycles;
    check(ipc > 0.2, "IPC is not degenerate (got " + std::to_string(ipc) + ")");
}

void test_back_to_back_alu() {
    // A pure dependent ALU chain on a 1-wide machine with 1-cycle ALU ops must
    // sustain ~1 IPC: this is the property atomic wakeup+select buys.
    std::cout << "back-to-back dependent ALU issue\n";
    Asm a(kBase);
    a.li(t0, 0);
    for (int i = 0; i < 200; i++) a.addi(t0, t0, 1);
    a.exit_(0);
    Program p{"alu_chain", a.code()};

    Config cfg;
    cfg.fetch_width = cfg.decode_width = cfg.rename_width = cfg.dispatch_width =
        cfg.issue_width = cfg.commit_width = 1;
    cfg.num_alu = 1;
    cfg.normalize();

    Memory mem = make_memory(p);
    Cpu cpu(cfg, mem, kBase);
    cpu.run();
    check(cpu.arch_reg(t0) == 200, "dependent chain result");

    // 200 dependent 1-cycle ops on a 1-wide machine: ~1 cycle each plus the
    // pipeline fill. Anything much worse means wakeup is losing a cycle.
    const uint64_t cycles = cpu.stats().cycles;
    check(cycles < 260,
          "dependent ALU chain issues back-to-back (took " +
              std::to_string(cycles) + " cycles for 200 ops)");
}

void test_load_use_latency() {
    // An isolated load followed by its consumer should cost exactly
    // mem_latency cycles of separation.
    std::cout << "load use latency\n";
    uint64_t prev_cycles = 0;
    for (int lat : {2, 3, 5}) {
        Asm a(kBase);
        a.li(s0, 0x40000);
        a.li(t1, 77);
        a.sw(t1, s0, 0);
        a.li(t0, 0);
        a.li(t2, 50);
        a.label("loop");
        a.lw(t3, s0, 0);
        a.addi(t3, t3, 1);          // consumer, must wait `lat` cycles
        a.sub(s0, s0, t3);          // feed the result back into the address so
        a.add(s0, s0, t3);          // the next load truly depends on this one
        a.addi(t0, t0, 1);
        a.blt(t0, t2, "loop");
        a.exit_(0);
        Program p{"loaduse", a.code()};

        Config cfg;
        cfg.mem_latency = lat;
        cfg.normalize();
        Memory mem = make_memory(p);
        Cpu cpu(cfg, mem, kBase);
        cpu.run();
        check(cpu.arch_reg(t3) == 78,
              "load-use result with mem_latency=" + std::to_string(lat));
        // The loop is load-use bound, so its length must track the latency.
        check(cpu.stats().cycles > prev_cycles,
              "raising mem_latency to " + std::to_string(lat) +
                  " lengthens a load-use bound loop");
        prev_cycles = cpu.stats().cycles;
    }
}

void test_store_to_load_forwarding() {
    // Loads that hit an uncommitted store must read the store queue, and loads
    // that reach a store with an unresolved address must replay.
    std::cout << "store queue forwarding\n";
    Program p = prog_store_forward();
    Config cfg; cfg.normalize();
    Memory mem = make_memory(p);
    Cpu cpu(cfg, mem, kBase);
    cpu.set_arch_reg(2, kSp);
    cpu.run();

    const Stats& s = cpu.stats();
    check(cpu.arch_reg(s1) == 888, "forwarded values are correct");
    check(s.store_forwards > 0, "loads forward from the store queue");
    check(s.store_forwards <= s.loads, "forwards never exceed committed loads");
    check(s.load_replays > 0, "loads replay behind an unresolved store address");

    // Byte-granular partial overlap must NOT be forwarded from a wider store.
    Program q = prog_partial_mem();
    Memory mem2 = make_memory(q);
    Cpu cpu2(cfg, mem2, kBase);
    cpu2.set_arch_reg(2, kSp);
    cpu2.run();
    check(cpu2.arch_reg(s1) == 0xffffffefu, "lb sign-extends a forwarded byte");
    check(cpu2.arch_reg(s2) == 0xef, "lbu zero-extends");
    check(cpu2.arch_reg(s3) == 0xffffdeadu, "lh sign-extends the upper half");
    check(cpu2.arch_reg(s4) == 0xdead, "lhu zero-extends the upper half");
}

void test_structural_hazards_are_counted() {
    // A deliberately starved machine must report the resources it ran out of.
    std::cout << "structural hazard accounting\n";

    Config tight;
    tight.rob_entries = 4; tight.phys_regs = 36; tight.iq_entries = 2;
    tight.num_cdbs = 1; tight.num_alu = 1;
    tight.fetch_width = tight.decode_width = tight.rename_width =
        tight.dispatch_width = tight.issue_width = tight.commit_width = 4;
    tight.normalize();

    Program p = prog_data_hazards();
    Memory mem = make_memory(p);
    Cpu cpu(tight, mem, kBase);
    cpu.run();
    const Stats& s = cpu.stats();
    check(s.stall_rob_full > 0, "a 4-entry ROB fills up");
    check(s.stall_iq_full > 0, "a 2-entry issue queue fills up");
    check(s.stall_fu_busy > 0, "a single ALU is a structural bottleneck");

    // A PRF smaller than ROB+32 makes physical registers the binding resource.
    Config starved;
    starved.rob_entries = 64; starved.phys_regs = 34;   // only 2 spare mappings
    starved.normalize();
    check(starved.prf_can_starve(), "undersized PRF is reported as starvable");
    Memory mem2 = make_memory(p);
    Cpu cpu2(starved, mem2, kBase);
    cpu2.run();
    check(cpu2.stats().stall_no_phys > 0, "an undersized PRF starves rename");
    check(cpu2.arch_reg(t2) == 0, "starved machine still computes correctly");

    // One CDB with several ALUs guarantees writeback contention.
    Config narrow_cdb;
    narrow_cdb.num_cdbs = 1; narrow_cdb.num_alu = 4;
    narrow_cdb.fetch_width = narrow_cdb.decode_width = narrow_cdb.rename_width =
        narrow_cdb.dispatch_width = narrow_cdb.issue_width =
        narrow_cdb.commit_width = 4;
    narrow_cdb.normalize();
    Memory mem3 = make_memory(prog_mem());
    Cpu cpu3(narrow_cdb, mem3, kBase);
    cpu3.set_arch_reg(2, kSp);
    cpu3.run();
    check(cpu3.stats().stall_cdb > 0, "a single CDB creates writeback contention");
}

void test_branch_checkpoint_pressure() {
    // With one checkpoint, only one branch can be in flight at a time; the
    // machine must still be correct, just slower.
    std::cout << "branch checkpoint pressure\n";
    Program p = prog_mispredict();

    Config many; many.max_checkpoints = 16; many.normalize();
    Config one;  one.max_checkpoints = 1;   one.normalize();

    Memory m1 = make_memory(p);
    Cpu c1(many, m1, kBase); c1.set_arch_reg(2, kSp); c1.run();
    Memory m2 = make_memory(p);
    Cpu c2(one, m2, kBase);  c2.set_arch_reg(2, kSp); c2.run();

    for (int r = 0; r < 32; r++)
        check(c1.arch_reg(r) == c2.arch_reg(r),
              "checkpoint count does not change architectural state");
    check(c2.stats().stall_no_chkpt > 0, "a single checkpoint starves rename");
    check(c2.stats().cycles > c1.stats().cycles,
          "checkpoint starvation costs cycles");
}

void test_misprediction_recovery_costs_cycles() {
    std::cout << "misprediction accounting\n";
    Program p = prog_mispredict();
    Config cfg; cfg.normalize();
    Memory mem = make_memory(p);
    Cpu cpu(cfg, mem, kBase);
    cpu.set_arch_reg(2, kSp);
    cpu.run();

    const Stats& s = cpu.stats();
    check(s.mispredicts > 0, "unpredictable branches do mispredict");
    check(s.squashed > 0, "wrong-path instructions are squashed");
    check(s.committed < s.fetched, "fetched exceeds committed under mispredicts");

    // A predictable loop must do far better than the random branch.
    Program q = prog_loop_sum();
    Memory mem2 = make_memory(q);
    Cpu cpu2(cfg, mem2, kBase);
    cpu2.run();
    const double mpki_rand =
        1000.0 * (double)s.mispredicts / (double)s.committed;
    const double mpki_loop =
        1000.0 * (double)cpu2.stats().mispredicts / (double)cpu2.stats().committed;
    check(mpki_loop < mpki_rand,
          "gshare predicts the regular loop better than the LCG branch");
}

void test_stat_consistency() {
    std::cout << "statistics consistency\n";
    for (const Program& p : all_programs()) {
        Config cfg; cfg.normalize();
        Memory mem = make_memory(p);
        Cpu cpu(cfg, mem, kBase);
        cpu.set_arch_reg(2, kSp);
        cpu.run();
        const Stats& s = cpu.stats();
        check(s.fetched >= s.decoded, p.name + ": fetched >= decoded");
        check(s.decoded >= s.renamed, p.name + ": decoded >= renamed");
        check(s.renamed >= s.dispatched, p.name + ": renamed >= dispatched");
        check(s.committed <= s.renamed, p.name + ": committed <= renamed");
        check(s.committed > 0, p.name + ": something committed");
    }
}

void test_hex_loader() {
    std::cout << "hex loader\n";
    Program p = prog_loop_sum();
    const std::string path = "/tmp/oooc_test_prog.hex";
    {
        FILE* f = std::fopen(path.c_str(), "w");
        if (!f) { fail("cannot write " + path); return; }
        std::fprintf(f, "# generated by the test suite\n");
        for (uint32_t w : p.code) std::fprintf(f, "%08x\n", w);
        std::fclose(f);
    }
    Memory mem;
    LoadResult lr = load_hex(path, mem, kBase);
    check(lr.ok, "hex load succeeds");
    check(lr.entry == kBase, "hex entry is the base address");

    Config cfg; cfg.normalize();
    Cpu cpu(cfg, mem, kBase);
    cpu.run();
    check(cpu.arch_reg(s0) == 5050, "program loaded from hex runs correctly");
    std::remove(path.c_str());
}

} // namespace

int main() {
    std::cout << "=== OoO RV32I core validation ===\n\n";

    const auto configs = configurations();
    const auto programs = all_programs();

    for (const auto& cp : configs) {
        std::cout << "config: " << cp.first << "\n";
        for (const Program& p : programs) run_and_compare(p, cp.second, cp.first);
    }
    std::cout << "\n";

    test_expected_values();
    test_x0_is_hardwired();
    test_no_physreg_leak();
    test_back_to_back_alu();
    test_load_use_latency();
    test_store_to_load_forwarding();
    test_structural_hazards_are_counted();
    test_branch_checkpoint_pressure();
    test_misprediction_recovery_costs_cycles();
    test_stat_consistency();
    test_hex_loader();

    std::cout << "\n" << (g_checks - g_failures) << "/" << g_checks
              << " checks passed\n";
    if (g_failures) {
        std::cout << g_failures << " FAILURES\n";
        return 1;
    }
    std::cout << "OK\n";
    return 0;
}
