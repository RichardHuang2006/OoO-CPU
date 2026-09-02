// Register renaming and out-of-order issue: the physical register file, free
// list, register alias table, ROB bookkeeping, the non-data-capture issue
// queue (tag wakeup, oldest-ready select), function-unit selection, CDB
// reservation and contention, and the no-leak conservation invariant.

#include "test_support.h"

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
    // Speculative completion (the `complete` flag) is not commit: entries 1
    // and 2 are done executing but stay put until the head completes too.
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

        // The tail is restored, so the next allocation lands directly behind the
        // survivor, with a fresh sequence number rather than a reused one.
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
// The pure mapping table. Its per-branch checkpointing lives in the
// CheckpointPool, tested with the rest of recovery in test_branch_recovery.
SECTION("rat") {
    RegisterAliasTable rat;

    // ---- Reset state: identity mapping ----------------------------------
    for (ArchReg a = 0; a < RegisterAliasTable::ARCH_REGS; ++a) {
        REQUIRE(rat.map(a) == a);
    }

    // ---- set() writes; set(0, _) is a no-op ----------------------------
    rat.set(5, 40);
    REQUIRE(rat.map(5) == 40u);
    rat.set(0, 99);
    REQUIRE(rat.map(0) == 0u);
    rat.set(31, 63);
    REQUIRE(rat.map(31) == 63u);

    // ---- mapping() exposes the whole array; adopt() replaces it ---------
    std::array<PhysReg, RegisterAliasTable::ARCH_REGS> saved = rat.mapping();
    for (ArchReg a = 1; a < RegisterAliasTable::ARCH_REGS; ++a) rat.set(a, 100 + a);
    REQUIRE(rat.map(5) == 105u);
    rat.adopt(saved);
    for (ArchReg a = 0; a < RegisterAliasTable::ARCH_REGS; ++a) {
        REQUIRE(rat.mapping()[a] == saved[a]);
    }
    REQUIRE(rat.map(5) == 40u);

    // adopt() cannot smuggle a mapping into x0's slot via set(), and adopt
    // itself is wholesale, so slot 0 comes from the adopted array.
    REQUIRE(rat.map(0) == 0u);

    // ---- reset() restores identity mapping ------------------------------
    rat.reset();
    for (ArchReg a = 0; a < RegisterAliasTable::ARCH_REGS; ++a) {
        REQUIRE(rat.map(a) == a);
    }
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
    PhysicalRegisterFile prf(cfg);
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
    PhysicalRegisterFile tiny(4);
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
        for (const char* name : {"alu", "waw_war", "muldiv", "sieve"}) {
            for (const wl::Workload& w : wl::corpus()) {
                if (w.name != name) continue;
                const diff::Report r = diff::diff_run(w, cfg);
                REQUIRE_MSG(r.ok, r.detail);
            }
        }
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
