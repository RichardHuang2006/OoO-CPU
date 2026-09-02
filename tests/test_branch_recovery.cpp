// Branch prediction and misprediction recovery: gshare counters and history,
// the set-associative BTB with LRU replacement, the return address stack, the
// per-branch checkpoint pool, and full pipeline recovery — including nested
// in-flight branches and cancellation of younger CDB reservations.

#include "rename.h"
#include "test_support.h"

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

    // ---- Branch-kind classification, straight off the decoder -------------
    {
        using namespace asmc;
        Assembler p;
        p.jal(ra, 8);                 // call: link register rd
        p.jal(zero, 8);               // plain jump
        p.jalr(ra, a0, 0);            // call via register
        p.jalr(zero, ra, 0);          // return: link register rs1
        p.jalr(zero, a0, 0);          // indirect jump, no link involved
        p.beq(a0, a1, "l");
        p.label("l");
        p.addi(a0, a0, 1);            // not a branch at all
        const std::vector<uint32_t> w = p.assemble();

        REQUIRE(classify(decode(w[0])) == BranchKind::CALL);
        REQUIRE(classify(decode(w[1])) == BranchKind::JUMP);
        REQUIRE(classify(decode(w[2])) == BranchKind::CALL);
        REQUIRE(classify(decode(w[3])) == BranchKind::RETURN);
        REQUIRE(classify(decode(w[4])) == BranchKind::JUMP);
        REQUIRE(classify(decode(w[5])) == BranchKind::CONDITIONAL);
        REQUIRE(classify(decode(w[6])) == BranchKind::NONE);
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
    // A one-deep stack cannot hold a recursive call chain, so its returns fall
    // back to the BTB, which always names the last caller.
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

// ---------------------------------------------------- @section("snapshot") ---
// The speculative front-end state a checkpoint holds — global history and the
// return stack — restores bit-exact. This is the unit-level half of recovery;
// the pipeline-level half is @section("recover").
SECTION("snapshot") {
    Config cfg;
    cfg.ghr_bits = 8;
    cfg.ras_size = 4;
    BranchPredictor bp(cfg);

    // Advance both speculative structures, snapshot, then keep going.
    bp.shift_history(true);
    bp.shift_history(false);
    bp.shift_history(true);
    // predict() pushes/pops the RAS only on BTB hits, so drive the tables to
    // set up a call the front end will see.
    bp.btb().update(0x1000, 0x2000, BranchKind::CALL);
    (void)bp.predict(0x1000);                    // pushes 0x1004
    REQUIRE(bp.ras().depth() == 1);
    REQUIRE(bp.ras().peek() == 0x1004u);

    const uint32_t ghr_at_snap = bp.ghr();
    const BranchPredictor::Snapshot snap = bp.snapshot();

    // Wander off: more history, deeper and then popped RAS.
    bp.shift_history(true);
    bp.shift_history(true);
    bp.btb().update(0x3000, 0x4000, BranchKind::CALL);
    (void)bp.predict(0x3000);                    // pushes 0x3004
    bp.btb().update(0x5000, 0, BranchKind::RETURN);
    (void)bp.predict(0x5000);                    // pops it again
    (void)bp.predict(0x5000);                    // pops the original 0x1004 too
    REQUIRE(bp.ghr() != ghr_at_snap);
    REQUIRE(bp.ras().depth() == 0);

    // Restore: both structures return to the snapshot exactly.
    bp.restore(snap);
    REQUIRE(bp.ghr() == ghr_at_snap);
    REQUIRE(bp.ras().depth() == 1);
    REQUIRE(bp.ras().peek() == 0x1004u);
    const std::vector<uint32_t> entries = bp.ras().entries();
    REQUIRE(entries.size() == 1);
    REQUIRE(entries[0] == 0x1004u);
}

// ----------------------------------------------- @section("checkpoint_pool") ---
// The bounded pool in rename.h: allocation, restoration through the RAT,
// exhaustion, sentinel-tolerant free, and slot independence.
SECTION("checkpoint_pool") {
    Config cfg;                    // num_checkpoints = 16
    CheckpointPool pool(cfg);
    RegisterAliasTable rat;

    // ---- Reset state: full pool ------------------------------------------
    REQUIRE(pool.capacity() == cfg.num_checkpoints);
    REQUIRE(pool.num_free() == cfg.num_checkpoints);

    // ---- alloc + fill snapshots the current RAT ---------------------------
    rat.set(5, 40);
    rat.set(31, 63);
    const std::optional<CheckpointId> cp = pool.alloc();
    REQUIRE(cp.has_value());
    pool.at(*cp) = Checkpoint{rat.mapping(), {}};
    REQUIRE(pool.num_free() == cfg.num_checkpoints - 1);

    // ---- Restore reproduces the RAT bit-identical -------------------------
    const std::array<PhysReg, 32> expected = rat.mapping();
    for (ArchReg a = 1; a < RegisterAliasTable::ARCH_REGS; ++a) rat.set(a, 100 + a);
    bool differ = false;
    for (ArchReg a = 0; a < RegisterAliasTable::ARCH_REGS; ++a) {
        if (rat.mapping()[a] != expected[a]) differ = true;
    }
    REQUIRE(differ);
    rat.adopt(pool.at(*cp).rat);
    for (ArchReg a = 0; a < RegisterAliasTable::ARCH_REGS; ++a) {
        REQUIRE(rat.mapping()[a] == expected[a]);
    }

    // ---- Restore does not release the slot; restoring twice is idempotent -
    REQUIRE(pool.num_free() == cfg.num_checkpoints - 1);
    rat.set(5, 999);
    rat.adopt(pool.at(*cp).rat);
    REQUIRE(rat.map(5) == expected[5]);

    // ---- free() returns the slot ------------------------------------------
    pool.free(*cp);
    REQUIRE(pool.num_free() == cfg.num_checkpoints);

    // ---- Pool refuses allocation when full instead of overwriting ---------
    std::vector<CheckpointId> held;
    while (auto id = pool.alloc()) held.push_back(*id);
    REQUIRE(held.size() == cfg.num_checkpoints);
    REQUIRE(!pool.alloc().has_value());
    // Every id is distinct.
    for (std::size_t i = 0; i < held.size(); ++i)
        for (std::size_t j = i + 1; j < held.size(); ++j)
            REQUIRE(held[i] != held[j]);
    // Free one, alloc succeeds exactly once.
    pool.free(held[0]);
    const std::optional<CheckpointId> again = pool.alloc();
    REQUIRE(again.has_value());
    REQUIRE(!pool.alloc().has_value());

    // ---- free() tolerates the sentinels commit paths pass ------------------
    pool.free(INVALID_CHECKPOINT);
    pool.free(cfg.num_checkpoints);
    pool.free(cfg.num_checkpoints + 10);

    // ---- Independent snapshots survive intervening writes ------------------
    CheckpointPool p2(4);
    RegisterAliasTable r2;
    r2.set(1, 100);
    const CheckpointId snap_a = *p2.alloc();
    p2.at(snap_a) = Checkpoint{r2.mapping(), {}};
    r2.set(1, 200);
    const CheckpointId snap_b = *p2.alloc();
    p2.at(snap_b) = Checkpoint{r2.mapping(), {}};
    r2.set(1, 300);              // now RAT[1] = 300, neither snapshot
    r2.adopt(p2.at(snap_a).rat);
    REQUIRE(r2.map(1) == 100u);
    r2.adopt(p2.at(snap_b).rat);
    REQUIRE(r2.map(1) == 200u);
    r2.adopt(p2.at(snap_a).rat);
    REQUIRE(r2.map(1) == 100u);

    // ---- A checkpoint carries front-end state too ---------------------------
    Config bpc;
    bpc.ghr_bits = 8;
    BranchPredictor bp(bpc);
    bp.shift_history(true);
    bp.shift_history(true);
    CheckpointPool p3(2);
    const CheckpointId id3 = *p3.alloc();
    p3.at(id3) = Checkpoint{r2.mapping(), bp.snapshot()};
    bp.shift_history(false);
    bp.shift_history(false);
    REQUIRE(bp.ghr() == 0xCu);   // 0b1100
    bp.restore(p3.at(id3).front_end);
    REQUIRE(bp.ghr() == 0x3u);   // the checkpointed 0b11

    // ---- reset() restores the full pool -------------------------------------
    pool.reset();
    REQUIRE(pool.num_free() == cfg.num_checkpoints);
}

// -------------------------------------------------- @section("checkpoint") ---
SECTION("checkpoint") {
    using namespace asmc;

    // ---- Every branch takes a slot, and gives it back -------------------
    {
        Config cfg;
        Memory m = cputest::image(fetest::addi_chain(20));
        Cpu cpu(m, cfg, wl::TEXT);
        const uint32_t all = cpu.checkpoints().capacity();
        REQUIRE(cpu.checkpoints().num_free() == all);

        while (!cpu.done() && cpu.cycle() < 2000) {
            cpu.tick();
            REQUIRE(cpu.checkpoints().num_free() <= all);
        }
        REQUIRE(cpu.checkpoints().num_free() == all);
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
        REQUIRE(cpu.checkpoints().num_free() == 1);
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
                REQUIRE(cpu.checkpoints().num_free() == 2);
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
        REQUIRE(cpu.checkpoints().num_free() == cpu.checkpoints().capacity());
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
            REQUIRE(cpu.checkpoints().num_free() == cpu.checkpoints().capacity());
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

// ---------------------------------------------- @section("nested_branches") ---
// Two unresolved branches in flight, and the *older* one is the one that
// mispredicted: recovery must squash the younger branch too, free its
// checkpoint, and unwind to the older branch's snapshot — not to the
// younger's, and not to a reset machine.
SECTION("nested_branches") {
    using namespace asmc;

    Config cfg;
    Assembler p;
    p.li(t0, 100);
    p.li(t1, 7);
    p.div_(t2, t0, t1);              // 20 cycles: holds the older branch open
    p.bne(t2, zero, "good");         // older branch: taken, cold-predicted NT
    // -- wrong path, fetched while the divide runs --------------------------
    p.beq(t0, t0, "wp2");            // younger branch: also mispredicts, and
    p.ebreak();                      //   recovers first, within the wrong path
    p.label("wp2");
    p.mul(t3, t0, t1);               // wrong-path work holding real resources
    p.addi(t4, t3, 1);
    p.ebreak();                      // never commits: the older branch wins
    // -- correct path --------------------------------------------------------
    p.label("good");
    p.li(a0, 77);
    p.li(a7, 93);
    p.ecall();

    Memory m = cputest::image(p.assemble());
    Cpu cpu(m, cfg, wl::TEXT);

    uint32_t max_ckpts_out = 0;
    while (!cpu.done() && cpu.cycle() < 1000) {
        cpu.tick();
        max_ckpts_out = std::max(
            max_ckpts_out, cpu.checkpoints().capacity() - cpu.checkpoints().num_free());
        REQUIRE(rectest::conserved(cpu));
    }

    REQUIRE(cpu.halted());
    REQUIRE(!cpu.trapped());                         // no wrong-path ebreak escaped
    REQUIRE(cpu.exit_code() == 77);
    REQUIRE(max_ckpts_out >= 2);                     // both branches held slots at once
    REQUIRE(cpu.stats().mispredicts == 2);           // younger first, then the older
    REQUIRE(cpu.stats().squashed > 0);
    REQUIRE(cpu.checkpoints().num_free() == cpu.checkpoints().capacity());
    REQUIRE(cpu.free_list().num_free() == cfg.prf_size - 32);
    REQUIRE(cpu.commit_in_order());
}

// --------------------------------------------------- @section("cdb_cancel") ---
// A squashed uop that booked a writeback port for a future cycle must give it
// back, or the phantom reservation would throttle correct-path issue forever.
SECTION("cdb_cancel") {
    using namespace asmc;

    Config cfg;                       // width 2, mul_latency 3
    Assembler p;
    p.li(t0, 100);
    p.li(t1, 7);
    p.div_(t2, t0, t1);              // 20 cycles
    p.beq(t2, t2, "good");           // always taken; cold-predicted NT; waits
                                     //   on the divide, so it resolves late
    // -- wrong path: wakes on the same divide broadcast, so it issues in the
    // -- same cycle the branch does, booking a CDB slot 3 cycles out ---------
    p.mul(t3, t2, t2);
    p.mul(t4, t2, t2);
    p.ebreak();
    p.label("good");
    p.li(a0, 9);
    p.li(a7, 93);
    p.ecall();

    Memory m = cputest::image(p.assemble());
    Cpu cpu(m, cfg, wl::TEXT);

    // Run to the recovery cycle.
    while (!cpu.done() && cpu.stats().mispredicts == 0 && cpu.cycle() < 500) {
        cpu.tick();
    }
    REQUIRE(cpu.stats().mispredicts == 1);
    REQUIRE(cpu.stats().squashed >= 2);              // the muls existed and died

    // Immediately after recovery nothing surviving has a result in flight:
    // the divide broadcast this cycle, the branch writes no register, and the
    // correct path has not issued yet. Every future CDB slot must be clean —
    // a leftover booking here is exactly the leak this test exists to catch.
    const uint64_t longest = std::max({cfg.alu_latency, cfg.branch_latency,
                                       cfg.mul_latency, cfg.div_latency,
                                       cfg.mem_latency, 1u});
    for (uint64_t k = 1; k <= longest + 1; ++k) {
        REQUIRE(cpu.cdb_reserved_at(cpu.cycle() + k) == 0);
    }

    REQUIRE(cpu.run(1000));
    REQUIRE(cpu.halted());
    REQUIRE(!cpu.trapped());
    REQUIRE(cpu.exit_code() == 9);
    REQUIRE(cpu.checkpoints().num_free() == cpu.checkpoints().capacity());
    REQUIRE(rectest::conserved(cpu));
}
