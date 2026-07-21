#include "cpu.h"

#include <algorithm>
#include <cstdio>
#include <iomanip>
#include <ostream>

Cpu::Cpu(const Config& cfg, Memory& mem, uint32_t entry_pc)
    : cfg_(cfg), mem_(mem),
      prf_(cfg.phys_regs),
      fl_(cfg.phys_regs),
      rob_(cfg.rob_entries),
      iq_(cfg.iq_entries),
      lsq_(cfg.lq_entries, cfg.sq_entries),
      ckpt_(cfg.max_checkpoints),
      gshare_(cfg.ghr_bits, cfg.pht_entries),
      btb_(cfg.btb_sets, cfg.btb_ways),
      ras_(cfg.ras_entries),
      cdb_ring_((size_t)kCdbRing, 0),
      pc_(entry_pc) {}

// ---------------------------------------------------------------------------
// Cycle
// ---------------------------------------------------------------------------

bool Cpu::tick() {
    if (halted_) return false;

    // Reverse pipeline order: every stage reads what its producer latched in
    // the previous cycle. The one exception is Writeback -> Issue, which is an
    // intentional same-cycle path (atomic wakeup + select).
    commit_stage();
    if (halted_) { cycle_++; st_.cycles = cycle_; return false; }

    writeback_stage();
    execute_stage();
    issue_stage();
    dispatch_stage();
    rename_stage();
    decode_stage();
    fetch_stage();

    // A misprediction detected in Execute redirects the front end for the next
    // cycle; recovery is applied once, for the oldest offending branch.
    handle_recovery();

    cycle_++;
    st_.cycles = cycle_;
    return !halted_;
}

void Cpu::run() {
    while (!halted_ && cycle_ < cfg_.max_cycles) {
        tick();
        // Watchdog: a correct machine always retires something eventually.
        if (!halted_ && cycle_ - last_commit_cycle_ > 100000) {
            halted_ = true;
            halt_reason_ = "deadlock: no instruction committed for 100000 cycles";
            exit_code_ = -1;
        }
    }
    if (!halted_ && cycle_ >= cfg_.max_cycles) {
        halted_ = true;
        halt_reason_ = "cycle limit reached";
        exit_code_ = -1;
    }
}

// ---------------------------------------------------------------------------
// Commit (in order, off the ROB head)
// ---------------------------------------------------------------------------

void Cpu::commit_stage() {
    for (int n = 0; n < cfg_.commit_width; n++) {
        if (rob_.empty()) break;
        RobEntry& e = rob_.head_entry();
        if (!e.done) {
            if (n == 0) st_.commit_blocked_cycles++;
            break;
        }

        if (e.illegal) {
            char buf[64];
            std::snprintf(buf, sizeof buf, "illegal instruction at pc=0x%08x", e.pc);
            halt_reason_ = buf;
            exit_code_ = -1;
            halted_ = true;
            return;
        }

        // Stores become architecturally visible here, in program order.
        if (e.is_store) {
            SQEntry& s = lsq_.sq().head_entry();
            mem_.write(s.addr, s.data, s.bytes);
            lsq_.sq().pop();
            st_.stores++;
        }
        if (e.is_load) {
            lsq_.lq().pop();
            st_.loads++;
        }

        // Reclaim the *stale* mapping: the value it holds is now dead.
        if (e.pdst >= 0) {
            arat_.set(e.arch_dst, e.pdst);
            fl_.free(e.pstale);
        }

        if (e.is_branch) st_.branches++;
        if (e.is_jump) st_.jumps++;
        if (e.is_branch || e.is_jump) update_predictors(e);
        if (e.mispredicted) {
            st_.mispredicts++;
            if (e.is_branch) st_.cond_mispredicts++;
        }
        if (e.chkpt >= 0) ckpt_.free(e.chkpt);

        st_.committed++;
        last_commit_cycle_ = cycle_;

        if (e.is_system) {
            // Minimal environment call: a7 == 93 is exit(a0), anything else
            // simply stops the machine.
            const bool ebreak = (e.op == Op::EBREAK);
            uint32_t a7 = prf_.read(arat_.lookup(17));
            uint32_t a0 = prf_.read(arat_.lookup(10));
            if (!ebreak && a7 == 93) {
                exit_code_ = (int)a0;
                halt_reason_ = "exit syscall";
            } else {
                exit_code_ = 0;
                halt_reason_ = ebreak ? "ebreak" : "environment call";
            }
            halted_ = true;
            rob_.pop();
            return;
        }

        rob_.pop();
    }
}

void Cpu::update_predictors(const RobEntry& e) {
    if (e.is_branch) gshare_.update(e.pc, e.ghr_snapshot, e.actual_taken);
    if (e.actual_taken) btb_.update(e.pc, e.actual_target, e.is_ret);
    if (e.used_ras) {
        st_.ras_predictions++;
        if (!e.mispredicted) st_.ras_correct++;
    }
}

// ---------------------------------------------------------------------------
// Writeback (CDB arbitration; broadcasts wake the issue queue this same cycle)
// ---------------------------------------------------------------------------

void Cpu::writeback_stage() {
    if (wb_.empty()) { cdb_slot(cycle_) = 0; return; }

    // Ops holding a reservation are guaranteed a port (their slot was booked
    // at select). Everything else -- loads, whose latency is not known early
    // enough to reserve -- arbitrates oldest-first for what is left.
    std::stable_sort(wb_.begin(), wb_.end(), [](const WbOp& a, const WbOp& b) {
        if (a.reserved != b.reserved) return a.reserved;
        return a.uop.seq < b.uop.seq;
    });

    int free_ports = cfg_.num_cdbs;
    std::vector<WbOp> deferred;

    for (WbOp& w : wb_) {
        if (w.needs_cdb) {
            if (!w.reserved && free_ports <= 0) {
                st_.stall_cdb++;
                deferred.push_back(w);      // retry next cycle
                continue;
            }
            free_ports--;
        }

        Uop& u = w.uop;
        if (u.pdst >= 0) {
            prf_.write(u.pdst, u.result);   // value + ready bit
            iq_.wakeup(u.pdst);             // tag broadcast -> select sees it now
        }

        if (u.rob_idx >= 0) {
            RobEntry& e = rob_.at(u.rob_idx);
            if (e.valid && e.seq == u.seq) {
                e.done = true;
                e.actual_taken = u.actual_taken;
                e.actual_target = u.actual_target;
                e.mispredicted = u.mispredicted;
                e.npc = u.actual_taken ? u.actual_target : u.d.pc + 4;
            }
        }
        trace("writeback", u);
    }

    wb_.swap(deferred);
    cdb_slot(cycle_) = 0;   // this cycle's reservations have been consumed
}

// ---------------------------------------------------------------------------
// Execute
// ---------------------------------------------------------------------------

void Cpu::execute_stage() {
    std::vector<FuOp> still_running;
    still_running.reserve(fus_.size());

    for (FuOp& f : fus_) {
        if (--f.remaining > 0) { still_running.push_back(f); continue; }

        bool replay = false;
        Uop u = f.uop;
        if (complete_op(u, replay)) {
            push_wb(u, f.reserved);
        } else {
            // Load blocked on an older store with an unresolved address.
            f.remaining = 1;
            still_running.push_back(f);
        }
    }
    fus_.swap(still_running);
}

bool Cpu::complete_op(Uop& u, bool& replay) {
    replay = false;
    const DecodedInst& d = u.d;

    if (d.is_load) {
        u.addr = u.val1 + (uint32_t)d.imm;
        if (u.lq_idx >= 0) {
            LQEntry& l = lsq_.lq().at(u.lq_idx);
            l.addr = u.addr;
            l.addr_valid = true;
        }
        uint32_t raw = 0;
        FwdResult r = lsq_.disambiguate(u.seq, u.addr, d.mem_bytes(), mem_, raw);
        if (r == FwdResult::Wait) {
            replay = true;
            st_.load_replays++;
            return false;
        }
        if (r == FwdResult::Forwarded) st_.store_forwards++;
        u.result = load_extend(d.op, raw);
        return true;
    }

    if (d.is_store) {
        u.addr = u.val1 + (uint32_t)d.imm;
        if (u.sq_idx >= 0) {
            SQEntry& s = lsq_.sq().at(u.sq_idx);
            s.addr = u.addr;   s.addr_valid = true;
            s.data = u.val2;   s.data_valid = true;
        }
        return true;
    }

    if (d.is_branch || d.is_jump) {
        bool taken;
        uint32_t target;
        if (d.is_branch) {
            taken = branch_taken(d.op, u.val1, u.val2);
            target = taken ? d.pc + (uint32_t)d.imm : d.pc + 4;
        } else if (d.op == Op::JAL) {
            taken = true;
            target = d.pc + (uint32_t)d.imm;
        } else {  // JALR
            taken = true;
            target = (u.val1 + (uint32_t)d.imm) & ~1u;
        }
        u.actual_taken = taken;
        u.actual_target = target;
        u.result = d.pc + 4;     // link value for JAL/JALR
        u.mispredicted = (taken != u.pred_taken) ||
                         (taken && target != u.pred_target);
        if (u.mispredicted) record_misprediction(u);
        return true;
    }

    u.result = alu_execute(d, u.val1, u.val2);
    return true;
}

void Cpu::push_wb(Uop& u, bool reserved) {
    WbOp w;
    w.uop = u;
    w.needs_cdb = (u.pdst >= 0);
    w.reserved = reserved;
    wb_.push_back(w);
    trace("execute", u);
}

// ---------------------------------------------------------------------------
// Issue: atomic wakeup + select
// ---------------------------------------------------------------------------

void Cpu::issue_stage() {
    // Ready bits were already updated by writeback_stage() earlier in this same
    // cycle -- that is the wakeup half. Select now runs over the result.
    alu_avail_ = cfg_.num_alu;
    mul_avail_ = cfg_.num_mul;
    mem_avail_ = cfg_.num_mem;
    br_avail_  = cfg_.num_branch;

    const std::vector<int> candidates = iq_.ready_oldest_first();
    int issued = 0;

    for (int idx : candidates) {
        if (issued >= cfg_.issue_width) break;

        Uop u = iq_.at(idx).uop;
        const FU fu = u.d.fu;

        if (fu == FU::DIV && cycle_ < div_busy_until_) { st_.stall_fu_busy++; continue; }
        if (!fu_port_available(fu)) { st_.stall_fu_busy++; continue; }

        const int lat = fu_latency(u.d);

        // Deterministic-latency producers book their writeback port up front,
        // which is what makes speculative (issue-time) wakeup safe. Loads
        // cannot, so they use slow wakeup and contend at writeback.
        bool reserved = false;
        if (u.pdst >= 0 && !u.d.is_load) {
            if (!reserve_cdb(cycle_ + (uint64_t)lat)) { st_.stall_cdb++; continue; }
            reserved = true;
        }

        // Operand read: the PRF write for a producer completing this cycle
        // already happened in writeback_stage(), which models the bypass path.
        u.val1 = u.d.use_rs1 ? prf_.read(u.psrc1) : 0;
        u.val2 = u.d.use_rs2 ? prf_.read(u.psrc2) : 0;

        consume_fu_port(fu);
        if (fu == FU::DIV) div_busy_until_ = cycle_ + (uint64_t)lat;
        iq_.remove(idx);
        issued++;
        st_.issued++;
        trace("issue", u);

        if (lat <= 1) {
            // Single-cycle op: result is latched immediately and broadcast in
            // the next cycle's writeback, so a dependent op issues back-to-back.
            bool replay = false;
            if (complete_op(u, replay)) {
                push_wb(u, reserved);
            } else {
                fus_.push_back(FuOp{u, 1, cycle_ + 1, reserved});
            }
        } else {
            fus_.push_back(FuOp{u, lat - 1, cycle_ + (uint64_t)lat, reserved});
        }
    }
}

int Cpu::fu_latency(const DecodedInst& d) const {
    switch (d.fu) {
        case FU::ALU:    return cfg_.alu_latency;
        case FU::BRANCH: return cfg_.alu_latency;
        case FU::MUL:    return cfg_.mul_latency;
        case FU::DIV:    return cfg_.div_latency;
        case FU::MEM:    return d.is_load ? cfg_.mem_latency : 1;
        default:         return 1;
    }
}

bool Cpu::fu_port_available(FU fu) {
    switch (fu) {
        case FU::ALU:    return alu_avail_ > 0;
        case FU::BRANCH: return br_avail_ > 0;
        case FU::MUL:
        case FU::DIV:    return mul_avail_ > 0;
        case FU::MEM:    return mem_avail_ > 0;
        default:         return true;
    }
}

void Cpu::consume_fu_port(FU fu) {
    switch (fu) {
        case FU::ALU:    alu_avail_--; break;
        case FU::BRANCH: br_avail_--;  break;
        case FU::MUL:
        case FU::DIV:    mul_avail_--; break;
        case FU::MEM:    mem_avail_--; break;
        default: break;
    }
}

int& Cpu::cdb_slot(uint64_t cycle) { return cdb_ring_[(size_t)(cycle % kCdbRing)]; }

bool Cpu::reserve_cdb(uint64_t cycle) {
    int& s = cdb_slot(cycle);
    if (s >= cfg_.num_cdbs) return false;
    s++;
    return true;
}

void Cpu::release_cdb(uint64_t cycle) {
    int& s = cdb_slot(cycle);
    if (s > 0) s--;
}

// ---------------------------------------------------------------------------
// Dispatch: issue queue + LSQ allocation
// ---------------------------------------------------------------------------

void Cpu::dispatch_stage() {
    for (int n = 0; n < cfg_.dispatch_width; n++) {
        if (rq_.empty()) break;
        Uop u = rq_.front();

        // Traps and illegal instructions never execute; they retire (and stop
        // the machine) straight from the ROB.
        if (u.d.is_system || u.d.illegal) {
            rob_.at(u.rob_idx).done = true;
            rq_.pop_front();
            st_.dispatched++;
            continue;
        }

        if (iq_.full()) { st_.stall_iq_full += cfg_.dispatch_width - n; break; }
        if (u.d.is_load && lsq_.lq().full()) {
            st_.stall_lq_full += cfg_.dispatch_width - n; break;
        }
        if (u.d.is_store && lsq_.sq().full()) {
            st_.stall_sq_full += cfg_.dispatch_width - n; break;
        }

        if (u.d.is_load) {
            u.lq_idx = lsq_.lq().alloc();
            LQEntry& l = lsq_.lq().at(u.lq_idx);
            l.seq = u.seq; l.rob_idx = u.rob_idx; l.bytes = u.d.mem_bytes();
            rob_.at(u.rob_idx).lq_idx = u.lq_idx;
        }
        if (u.d.is_store) {
            u.sq_idx = lsq_.sq().alloc();
            SQEntry& s = lsq_.sq().at(u.sq_idx);
            s.seq = u.seq; s.rob_idx = u.rob_idx; s.bytes = u.d.mem_bytes();
            rob_.at(u.rob_idx).sq_idx = u.sq_idx;
        }

        const bool r1 = !u.d.use_rs1 || prf_.ready(u.psrc1);
        const bool r2 = !u.d.use_rs2 || prf_.ready(u.psrc2);
        iq_.insert(u, r1, r2);

        rq_.pop_front();
        st_.dispatched++;
        trace("dispatch", u);
    }
}

// ---------------------------------------------------------------------------
// Rename: RAT lookup/update, physreg + ROB allocation, branch checkpoint
// ---------------------------------------------------------------------------

void Cpu::rename_stage() {
    for (int n = 0; n < cfg_.rename_width; n++) {
        if (dq_.empty()) break;
        if ((int)rq_.size() >= cfg_.dispatch_queue) {
            st_.stall_dispatch_queue += cfg_.rename_width - n; break;
        }

        Uop u = dq_.front();

        if (rob_.full())                     { st_.stall_rob_full += cfg_.rename_width - n; break; }
        if (u.d.has_rd && fl_.empty())       { st_.stall_no_phys  += cfg_.rename_width - n; break; }
        const bool needs_ckpt = u.d.is_ctrl();
        if (needs_ckpt && !ckpt_.available()) { st_.stall_no_chkpt += cfg_.rename_width - n; break; }

        // Sources are read before the destination is written so that an
        // instruction reading its own architectural destination is correct.
        u.psrc1 = rat_.lookup(u.d.rs1);
        u.psrc2 = rat_.lookup(u.d.rs2);

        if (u.d.has_rd) {
            u.pstale = rat_.lookup(u.d.rd);
            u.pdst = fl_.alloc();
            prf_.clear(u.pdst);
            rat_.set(u.d.rd, u.pdst);
        }

        u.rob_idx = rob_.alloc();
        RobEntry& e = rob_.at(u.rob_idx);
        e.seq = u.seq;         e.pc = u.d.pc;          e.op = u.d.op;
        e.arch_dst = u.d.rd;   e.pdst = u.pdst;        e.pstale = u.pstale;
        e.is_branch = u.d.is_branch;  e.is_jump = u.d.is_jump;
        e.is_load = u.d.is_load;      e.is_store = u.d.is_store;
        e.is_system = u.d.is_system;  e.illegal = u.d.illegal;
        e.is_ret = u.d.is_ret;
        e.pred_taken = u.pred_taken;  e.ghr_snapshot = u.ghr_snapshot;
        e.used_ras = u.used_ras;      e.npc = u.d.pc + 4;

        // A checkpoint of the *post-rename* RAT: the branch itself survives
        // recovery, only younger instructions are undone.
        if (needs_ckpt) {
            u.chkpt = ckpt_.alloc();
            Checkpoint& c = ckpt_.at(u.chkpt);
            c.rat = rat_.raw();
            c.ghr = u.ghr_snapshot;        // pre-branch history
            c.ras = u.ras_snapshot;
            c.ras_top = u.ras_top;
            c.ras_depth = u.ras_depth;
            c.seq = u.seq;
            e.chkpt = u.chkpt;
        }
        u.ras_snapshot.clear();   // owned by the checkpoint from here on

        rq_.push_back(u);
        dq_.pop_front();
        st_.renamed++;
        trace("rename", u);
    }
}

// ---------------------------------------------------------------------------
// Decode
// ---------------------------------------------------------------------------

void Cpu::decode_stage() {
    for (int n = 0; n < cfg_.decode_width; n++) {
        if (fq_.empty()) break;
        if ((int)dq_.size() >= cfg_.decode_queue) {
            st_.stall_decode_queue += cfg_.decode_width - n; break;
        }
        Uop u = fq_.front();
        fq_.pop_front();
        u.d = decode(u.d.raw, u.d.pc);
        dq_.push_back(u);
        st_.decoded++;
    }
}

// ---------------------------------------------------------------------------
// Fetch
// ---------------------------------------------------------------------------

void Cpu::fetch_stage() {
    int n = 0;
    for (; n < cfg_.fetch_width; n++) {
        if ((int)fq_.size() >= cfg_.fetch_queue) {
            st_.stall_fetch_queue += cfg_.fetch_width - n;
            break;
        }

        Uop u;
        u.d.pc = pc_;
        u.d.raw = mem_.read32(pc_);
        u.seq = seq_++;

        // Predecode is enough to steer the front end (direction, call/return).
        const DecodedInst pre = decode(u.d.raw, pc_);
        predict(pc_, pre, u);

        fq_.push_back(u);
        st_.fetched++;

        pc_ = u.pred_taken ? u.pred_target : pc_ + 4;
        if (u.pred_taken) { n++; break; }   // fetch block ends at a taken branch
    }
    if (n == 0) st_.idle_fetch_cycles++;
}

void Cpu::predict(uint32_t pc, const DecodedInst& pre, Uop& u) {
    u.ghr_snapshot = gshare_.ghr();

    uint32_t btb_target = 0;
    bool btb_is_ret = false;
    const bool hit = btb_.lookup(pc, btb_target, btb_is_ret);
    u.btb_hit = hit;

    bool taken = false;
    uint32_t target = pc + 4;

    if (pre.is_ret) {
        if (ras_.valid()) {
            target = ras_.pop();
            taken = true;
            u.used_ras = true;
        } else if (hit) {
            target = btb_target;
            taken = true;
        }
    } else if (pre.is_jump) {
        // Unconditional, but the target is only known once the BTB has been
        // trained; a miss simply falls through and is corrected at Execute.
        if (hit) { target = btb_target; taken = true; }
        else st_.btb_misses++;
    } else if (pre.is_branch) {
        const bool dir = gshare_.predict(pc);
        if (dir && hit) { target = btb_target; taken = true; }
        else if (dir)   { st_.btb_misses++; }
        gshare_.shift(taken);   // speculative history update
    }

    if (pre.is_call) ras_.push(pc + 4);

    // Snapshot the (speculative) RAS after this instruction's own push/pop so
    // recovery restores exactly the state the branch left behind.
    if (pre.is_ctrl()) {
        u.ras_snapshot = ras_.snapshot();
        u.ras_top = ras_.top();
        u.ras_depth = ras_.depth();
    }
    if (hit) btb_.touch(pc);

    u.pred_taken = taken;
    u.pred_target = target;
}

// ---------------------------------------------------------------------------
// Misprediction recovery
// ---------------------------------------------------------------------------

void Cpu::record_misprediction(const Uop& u) {
    // Recover from the oldest offending branch; younger ones are squashed by it.
    if (!redirect_pending_ || u.seq < redirect_uop_.seq) {
        redirect_pending_ = true;
        redirect_uop_ = u;
    }
}

void Cpu::handle_recovery() {
    if (!redirect_pending_) return;
    redirect_pending_ = false;

    const Uop b = redirect_uop_;
    RobEntry& be = rob_.at(b.rob_idx);
    if (!be.valid || be.seq != b.seq) return;   // already squashed by an older branch

    // 1. Return the destinations of every younger instruction to the free list,
    //    youngest first, and drop their ROB entries.
    const int younger = rob_.younger_count(b.rob_idx);
    for (int k = younger; k >= 1; k--) {
        RobEntry& y = rob_.at((b.rob_idx + k) % rob_.capacity());
        if (y.pdst >= 0) fl_.free_front(y.pdst);
        if (y.chkpt >= 0) { ckpt_.free(y.chkpt); y.chkpt = -1; }
    }
    st_.squashed += (uint64_t)younger + fq_.size() + dq_.size();
    rob_.truncate_after(b.rob_idx);

    // 2. Purge the out-of-order structures, releasing booked writeback ports.
    iq_.squash_after(b.seq);
    {
        std::vector<FuOp> keep;
        for (FuOp& f : fus_) {
            if (f.uop.seq > b.seq) { if (f.reserved) release_cdb(f.wb_cycle); }
            else keep.push_back(f);
        }
        fus_.swap(keep);
    }
    {
        std::vector<WbOp> keep;
        for (WbOp& w : wb_)
            if (w.uop.seq <= b.seq) keep.push_back(w);
        wb_.swap(keep);
    }
    lsq_.squash_after(b.seq);

    // 3. Restore the front-end state from this branch's checkpoint.
    ckpt_.free_younger_than(b.seq);
    if (b.chkpt >= 0) {
        Checkpoint& c = ckpt_.at(b.chkpt);
        rat_.restore(c.rat);
        gshare_.set_ghr(c.ghr);
        if (b.d.is_branch) gshare_.shift(b.actual_taken);
        ras_.restore(c.ras, c.ras_top, c.ras_depth);
        ckpt_.free(b.chkpt);
        be.chkpt = -1;   // do not free it again at commit
    }

    flush_frontend();
    pc_ = b.actual_taken ? b.actual_target : b.d.pc + 4;
}

void Cpu::flush_frontend() {
    fq_.clear();
    dq_.clear();
    rq_.clear();
}

// ---------------------------------------------------------------------------
// Reporting
// ---------------------------------------------------------------------------

void Cpu::print_stats(std::ostream& os) const {
    const double cyc = (double)(st_.cycles ? st_.cycles : 1);
    const double ipc = (double)st_.committed / cyc;
    const uint64_t ctrl = st_.branches + st_.jumps;

    auto pct = [&](uint64_t a, uint64_t b) {
        return b ? (100.0 * (double)a / (double)b) : 0.0;
    };
    auto row = [&](const char* name, uint64_t v) {
        os << "  " << std::left << std::setw(26) << name << std::right
           << std::setw(12) << v << "   " << std::fixed << std::setprecision(2)
           << std::setw(6) << (double)v / cyc << " /cycle\n";
    };

    os << "\n=== Simulation ===\n"
       << "  halt reason               " << (halt_reason_.empty() ? "-" : halt_reason_)
       << "\n  exit code                 " << exit_code_ << "\n";

    os << "\n=== Performance ===\n";
    os << "  cycles                    " << std::setw(12) << st_.cycles << "\n";
    os << "  instructions committed    " << std::setw(12) << st_.committed << "\n";
    os << "  IPC                       " << std::setw(12) << std::fixed
       << std::setprecision(3) << ipc << "\n";
    os << "  CPI                       " << std::setw(12)
       << (st_.committed ? cyc / (double)st_.committed : 0.0) << "\n";

    os << "\n=== Pipeline flow (uops) ===\n";
    row("fetched", st_.fetched);
    row("decoded", st_.decoded);
    row("renamed", st_.renamed);
    row("dispatched", st_.dispatched);
    row("issued", st_.issued);
    row("committed", st_.committed);
    os << "  " << std::left << std::setw(26) << "squashed (wrong path)"
       << std::right << std::setw(12) << st_.squashed << "   "
       << std::setprecision(2) << pct(st_.squashed, st_.fetched) << "% of fetched\n";

    os << "\n=== Branch prediction ===\n";
    os << "  conditional branches      " << std::setw(12) << st_.branches << "\n";
    os << "  jumps (jal/jalr)          " << std::setw(12) << st_.jumps << "\n";
    os << "  mispredicts               " << std::setw(12) << st_.mispredicts
       << "   " << std::setprecision(2) << pct(st_.mispredicts, ctrl) << "% of control\n";
    os << "  cond. mispredicts         " << std::setw(12) << st_.cond_mispredicts
       << "   " << pct(st_.cond_mispredicts, st_.branches) << "% of branches\n";
    os << "  BTB misses (pred taken)   " << std::setw(12) << st_.btb_misses << "\n";
    os << "  RAS predictions / correct " << std::setw(12) << st_.ras_predictions
       << " / " << st_.ras_correct << "\n";
    os << "  MPKI                      " << std::setw(12) << std::setprecision(2)
       << (st_.committed ? 1000.0 * (double)st_.mispredicts / (double)st_.committed : 0.0)
       << "\n";

    os << "\n=== Memory ===\n";
    os << "  loads committed           " << std::setw(12) << st_.loads << "\n";
    os << "  stores committed          " << std::setw(12) << st_.stores << "\n";
    os << "  store->load forwards      " << std::setw(12) << st_.store_forwards << "\n";
    os << "  load replays (unknown addr)" << std::setw(11) << st_.load_replays << "\n";

    os << "\n=== Stall causes (lost issue slots) ===\n";
    row("ROB full", st_.stall_rob_full);
    row("physreg starvation", st_.stall_no_phys);
    row("branch chkpt starvation", st_.stall_no_chkpt);
    row("issue queue full", st_.stall_iq_full);
    row("load queue full", st_.stall_lq_full);
    row("store queue full", st_.stall_sq_full);
    row("dispatch queue full", st_.stall_dispatch_queue);
    row("decode queue full", st_.stall_decode_queue);
    row("fetch queue full", st_.stall_fetch_queue);
    row("FU structural hazard", st_.stall_fu_busy);
    row("CDB contention", st_.stall_cdb);
    os << "  " << std::left << std::setw(26) << "cycles w/ ROB head stalled"
       << std::right << std::setw(12) << st_.commit_blocked_cycles << "   "
       << std::setprecision(2) << pct(st_.commit_blocked_cycles, st_.cycles) << "% of cycles\n";
    os << std::defaultfloat;
}

void Cpu::trace(const char* stage, const Uop& u) const {
    if (!cfg_.trace) return;
    std::printf("[%6llu] %-9s seq=%-5llu pc=0x%08x %-7s p%d<-p%d,p%d\n",
                (unsigned long long)cycle_, stage, (unsigned long long)u.seq,
                u.d.pc, op_name(u.d.op), u.pdst, u.psrc1, u.psrc2);
}
