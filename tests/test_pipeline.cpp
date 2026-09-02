// Whole-pipeline behavior: configuration, the CLI driver, cycle-by-cycle
// stage movement, width and back-pressure, function-unit latency (pipelined
// multiply vs. blocking divide), in-order commit, the statistics invariants,
// and cycle tracing (which must never perturb the machine it observes).

#include <cmath>
#include <cstdio>

#include "test_support.h"
#include "trace.h"

// Include the driver TU so parse_args / print_help are unit-testable
// without shelling out. Its own int main() is guarded off.
#define MINI_CPU_NO_ENTRY
#include "main.cpp"

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
    { CliOpts o; REQUIRE(parse({"oooc", "--help"},  o) == 0); REQUIRE(o.show_help); }
    { CliOpts o; REQUIRE(parse({"oooc", "--ref"},   o) == 0); REQUIRE(o.use_ref); }
    { CliOpts o; REQUIRE(parse({"oooc", "--stats"}, o) == 0); REQUIRE(o.show_stats); }
    { CliOpts o; REQUIRE(parse({"oooc", "--ipc-table"}, o) == 0); REQUIRE(o.ipc_table); }

    // Trace flags: a path plus the window controls.
    { CliOpts o; REQUIRE(parse({"oooc", "--trace", "out.ndjson"}, o) == 0);
      REQUIRE(o.trace_path == "out.ndjson"); }
    { CliOpts o; REQUIRE(parse({"oooc", "--trace=t.json", "--trace-start", "100",
                                "--trace-cycles=250"}, o) == 0);
      REQUIRE(o.trace_path == "t.json");
      REQUIRE(o.trace_start == 100);
      REQUIRE(o.trace_cycles == 250); }
    { CliOpts o; REQUIRE(parse({"oooc", "--trace"}, o) != 0); }          // path required
    { CliOpts o; REQUIRE(parse({"oooc", "--trace-start", "x"}, o) != 0); }

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

// ---------------------------------------------------- @section("cpu_tick") ---
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
    // Fetch, decode, rename, dispatch, issue, writeback, commit: one cycle
    // each — the stages run in reverse order per tick, so the uop advances
    // exactly one latch per cycle. The value is in the physical register file
    // after writeback, but reading a0 follows the committed mapping, so it
    // appears at commit.
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
    // The `alu` workload is pure ALU plus the exit ecall. Pins the stage
    // plumbing, the x0 rule, and the ecall path against ref.h on all 32
    // registers.
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
    // writes its physical register first. Renaming makes that safe; in-order
    // commit keeps it externally invisible.
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

// ------------------------------------------------------- @section("stats") ---
namespace stallsets {

// Slots the issue stage could have used and did not. Each cycle offers
// `width` of them, so this is what the issue-side breakdown must fit inside.
inline const Stall ISSUE_STALLS[] = {
    Stall::ALU_PORT, Stall::BRANCH_PORT, Stall::MUL_PORT, Stall::DIV_PORT,
    Stall::MEM_PORT, Stall::CDB, Stall::STORE_ORDER, Stall::OPERANDS,
};
inline const Stall RENAME_STALLS[]   = {Stall::ROB_FULL, Stall::PHYSREG, Stall::CHECKPOINT};
inline const Stall DISPATCH_STALLS[] = {Stall::IQ_FULL, Stall::LQ_FULL, Stall::SQ_FULL};

}  // namespace stallsets

SECTION("stats") {
    using namespace stallsets;
    using stattest::named;
    using stattest::run;

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
        const Config dc;
        const Stats s = run(named("crc32"), dc);
        REQUIRE(std::abs(s.ipc() * s.cpi() - 1.0) < 1e-9);
        REQUIRE(std::abs(s.ipc() - double(s.retired) / double(s.cycles)) < 1e-9);
        REQUIRE(std::abs(s.mpki() - 1000.0 * s.mispredict_rate() * s.branches / s.retired) < 1e-6);
        REQUIRE(s.btb_hit_rate() <= 1.0);
        REQUIRE(s.ras_accuracy() <= 1.0);

        // Issue-slot utilization is issued over offered, and offered is
        // width slots per cycle.
        const double util = s.issue_utilization(dc.width);
        REQUIRE(util > 0.0);
        REQUIRE(util <= 1.0);
        REQUIRE(std::abs(util - double(s.issued) / (double(s.cycles) * dc.width)) < 1e-12);

        const Stats empty;
        REQUIRE(empty.ipc() == 0.0);            // no cycles, no division
        REQUIRE(empty.cpi() == 0.0);
        REQUIRE(empty.mispredict_rate() == 0.0);
        REQUIRE(empty.issue_utilization(2) == 0.0);
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

// ------------------------------------------------------- @section("trace") ---
// The cycle tracer is an observer: attaching it must change nothing, and what
// it writes must be well-formed NDJSON that honors the window controls.
SECTION("trace") {
    using namespace asmc;

    // ---- Trace invariance ---------------------------------------------------
    // The same workload, traced and untraced: identical architectural results,
    // identical cycle count, identical statistics, on a program with
    // mispredicts, forwarding, replays, and squashes in play.
    {
        const wl::Workload& w = stattest::named("lcg_branch");
        const Config cfg;

        Memory m_plain = cputest::image(w.words);
        Cpu plain(m_plain, cfg, wl::TEXT);
        REQUIRE(plain.run(w.budget * 8 + 1000));

        std::ostringstream sink;
        Trace::Options topt;
        topt.max_cycles = 1u << 30;                   // trace everything
        Memory m_traced = cputest::image(w.words);
        Cpu traced(m_traced, cfg, wl::TEXT);
        Trace trace(sink, cfg, wl::TEXT, topt);
        traced.attach_trace(&trace);
        REQUIRE(traced.run(w.budget * 8 + 1000));

        REQUIRE(traced.cycle()     == plain.cycle());
        REQUIRE(traced.retired()   == plain.retired());
        REQUIRE(traced.exit_code() == plain.exit_code());
        REQUIRE(traced.halted()    == plain.halted());
        for (int i = 0; i < 32; ++i) {
            REQUIRE(traced.reg(static_cast<ArchReg>(i)) ==
                    plain.reg(static_cast<ArchReg>(i)));
        }
        // The whole Stats block, field by field and stall by stall.
        const Stats& a = plain.stats();
        const Stats& b = traced.stats();
        REQUIRE(a.cycles == b.cycles);
        REQUIRE(a.fetched == b.fetched);
        REQUIRE(a.decoded == b.decoded);
        REQUIRE(a.renamed == b.renamed);
        REQUIRE(a.dispatched == b.dispatched);
        REQUIRE(a.issued == b.issued);
        REQUIRE(a.wrote_back == b.wrote_back);
        REQUIRE(a.retired == b.retired);
        REQUIRE(a.squashed == b.squashed);
        REQUIRE(a.branches == b.branches);
        REQUIRE(a.mispredicts == b.mispredicts);
        REQUIRE(a.btb_lookups == b.btb_lookups);
        REQUIRE(a.btb_hits == b.btb_hits);
        REQUIRE(a.ras_pops == b.ras_pops);
        REQUIRE(a.ras_hits == b.ras_hits);
        REQUIRE(a.loads == b.loads);
        REQUIRE(a.stores == b.stores);
        REQUIRE(a.load_forwards == b.load_forwards);
        REQUIRE(a.load_replays == b.load_replays);
        REQUIRE(a.load_memory == b.load_memory);
        for (std::size_t i = 0; i < a.stalls.size(); ++i) {
            REQUIRE(a.stalls[i] == b.stalls[i]);
        }

        // One header plus one record per cycle, all shaped like JSON objects.
        const std::string text = sink.str();
        std::istringstream lines(text);
        std::string line;
        uint64_t total = 0, cycles = 0;
        bool header_first = false;
        while (std::getline(lines, line)) {
            REQUIRE(!line.empty());
            REQUIRE(line.front() == '{');
            REQUIRE(line.back()  == '}');
            if (total == 0) {
                header_first = line.find("\"type\":\"header\"") != std::string::npos;
            } else {
                REQUIRE(line.find("\"type\":\"cycle\"") != std::string::npos);
                ++cycles;
            }
            ++total;
        }
        REQUIRE(header_first);
        REQUIRE(cycles == plain.cycle());
        REQUIRE(trace.cycles_recorded() == plain.cycle());

        // The header carries the configuration; the body carries the events
        // the run demonstrably contained.
        REQUIRE(text.find("\"config\":{") != std::string::npos);
        REQUIRE(text.find("\"width\":2")  != std::string::npos);
        REQUIRE(text.find("\"e\":\"F\"")  != std::string::npos);   // fetch
        REQUIRE(text.find("\"e\":\"R\"")  != std::string::npos);   // rename
        REQUIRE(text.find("\"e\":\"I\"")  != std::string::npos);   // issue
        REQUIRE(text.find("\"e\":\"C\"")  != std::string::npos);   // complete
        REQUIRE(text.find("\"e\":\"CM\"") != std::string::npos);   // commit
        REQUIRE(text.find("\"e\":\"SQ\"") != std::string::npos);   // squash
        REQUIRE(text.find("\"e\":\"MP\"") != std::string::npos);   // mispredict
        REQUIRE(text.find("\"rob\":[")    != std::string::npos);
        REQUIRE(text.find("\"iq\":[")     != std::string::npos);
        REQUIRE(text.find("\"rat\":[")    != std::string::npos);
        REQUIRE(text.find("\"asm\":\"")   != std::string::npos);
    }

    // ---- Replay events appear when replays happen ---------------------------
    {
        const wl::Workload& w = stattest::named("store_forward");
        std::ostringstream sink;
        Trace::Options topt;
        topt.max_cycles = 1u << 30;
        Memory m = cputest::image(w.words);
        Cpu cpu(m, Config{}, wl::TEXT);
        Trace trace(sink, Config{}, wl::TEXT, topt);
        cpu.attach_trace(&trace);
        REQUIRE(cpu.run(w.budget * 8 + 1000));
        if (cpu.stats().load_replays > 0) {
            REQUIRE(sink.str().find("\"e\":\"RP\"") != std::string::npos);
        }
        REQUIRE(sink.str().find("\"lq\":[") != std::string::npos);
        REQUIRE(sink.str().find("\"sq\":[") != std::string::npos);
    }

    // ---- The window: start and maximum-cycle controls -----------------------
    {
        std::ostringstream sink;
        Trace::Options topt;
        topt.start      = 10;
        topt.max_cycles = 5;
        Memory m = cputest::image(fetest::addi_chain(60));
        Cpu cpu(m, Config{}, wl::TEXT);
        Trace trace(sink, Config{}, wl::TEXT, topt);
        cpu.attach_trace(&trace);
        REQUIRE(cpu.run(4000));
        REQUIRE(cpu.cycle() > 20);                    // the run outlives the window

        std::istringstream lines(sink.str());
        std::string line;
        std::vector<uint64_t> recorded;
        while (std::getline(lines, line)) {
            const auto pos = line.find("\"cycle\":");
            if (line.find("\"type\":\"cycle\"") == std::string::npos) continue;
            recorded.push_back(std::stoull(line.substr(pos + 8)));
        }
        REQUIRE(recorded.size() == 5);                // max_cycles honored
        REQUIRE(recorded.front() == 10);              // start honored
        for (std::size_t i = 1; i < recorded.size(); ++i) {
            REQUIRE(recorded[i] == recorded[i - 1] + 1);
        }
        REQUIRE(trace.cycles_recorded() == 5);
    }
}
