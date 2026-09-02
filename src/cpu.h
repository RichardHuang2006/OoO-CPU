#pragma once

#include <array>
#include <cstdint>
#include <deque>
#include <string>
#include <vector>

#include "branch_predictor.h"
#include "config.h"
#include "instruction.h"
#include "issue_queue.h"
#include "lsq.h"
#include "memory.h"
#include "rename.h"
#include "rob.h"
#include "stats.h"

// The seven-stage out-of-order pipeline, with in-order commit from the ROB
// head and `Config::width` uops per stage per cycle.
//
// The seven modeled stages, and the internal functions that implement them:
//
//   1. fetch              fetch()                predict and read one bundle
//   2. decode             decode_stage()         raw word → Decoded
//   3. rename/dispatch    rename() + dispatch()  map registers, take ROB / IQ /
//                                                LSQ seats (split in two
//                                                functions, one queue between
//                                                them, so each is one idea)
//   4. issue/reg-read     issue()                oldest-ready select, then read
//                                                operand values from the PRF
//   5. execute            execute_uop() /        compute results, targets, and
//                         execute()              store data; multi-cycle ops
//                                                wait here for their latency
//   6. memory/writeback   execute_load() +       loads search the store queue
//                         writeback()            or read memory; results land
//                                                in the PRF and broadcast tags
//   7. commit             commit()               architectural state advances,
//                                                in program order only
//
// tick() runs the stages in reverse order (commit first, fetch last) so the
// inter-stage queues behave as latches: each stage drains what its producer
// left last cycle before the producer refills it. Writeback is the exception:
// it broadcasts before issue in the same cycle, so a dependent single-cycle op
// can issue back to back with its producer.
//
// Rename maps sources to the RAT's current physical tags and destinations to
// free-list registers, holding the displaced mapping in the ROB entry until
// commit returns it (the full transaction is documented in rename.h). WAW and
// WAR hazards disappear, so only true dependences reach the issue queue, which
// selects the oldest ready ops the function units and writeback ports can
// accept. Loads consult the store queue for forwarding instead of waiting for
// older stores to commit.
//
// Branches checkpoint the register mapping, global history, and return stack
// at rename, and stall if the pool is exhausted. On misprediction, recovery
// reclaims younger physical registers youngest first, flushes younger entries
// from every structure, cancels their CDB reservations, restores the
// checkpoint, and redirects fetch in the same cycle.

enum class TrapCause : uint8_t {
    NONE,
    ILLEGAL,          // undecodable instruction
    EBREAK,
    ECALL_UNKNOWN,    // ecall with a7 != 93
};

// The cycle each stage handled a uop, 0 meaning "has not happened". Written
// once per stage and never read by the pipeline itself: an instruction's
// timing is something an observer wants and the machine does not, so nothing
// here can change how the machine behaves.
struct StageCycles {
    uint64_t fetch    = 0;
    uint64_t decode   = 0;
    uint64_t rename   = 0;
    uint64_t dispatch = 0;
    uint64_t issue    = 0;
    uint64_t complete = 0;
};

// One in-flight instruction, gaining fields as it moves down the pipeline:
// fetch fills pc/raw, decode fills `dec`, rename fills the tags and the ROB
// index, issue fills the operand values and the result, store address or
// branch target.
struct Uop {
    SeqNum    seq        = INVALID_SEQNUM;
    RobIndex  rob        = INVALID_ROBINDEX;
    uint32_t  pc         = 0;
    uint32_t  raw        = 0;
    Decoded   dec        {};

    // Identity from fetch onward. Sequence numbers are only stamped at rename,
    // so this is the only way to follow a uop through the front end — and the
    // only way a trace can say that the thing in the decode queue this cycle is
    // the thing that was in the fetch queue last cycle.
    uint32_t  uid        = 0;
    StageCycles at       {};

    PhysReg   src1       = 0;
    PhysReg   src2       = 0;
    PhysReg   dest       = INVALID_PHYSREG;   // no destination when absent
    PhysReg   stale      = INVALID_PHYSREG;   // freed when this uop commits
    uint32_t  val1       = 0;                 // read from the PRF at issue
    uint32_t  val2       = 0;

    uint32_t  result     = 0;
    bool      has_result = false;
    uint32_t  next_pc    = 0;
    uint32_t  mem_addr   = 0;    // access address, computed at issue
    uint32_t  store_data = 0;    // store data, written to memory at commit
    uint32_t  lsq_idx    = 0;    // load- or store-queue seat, taken at dispatch
    uint64_t  wb_cycle   = 0;    // writeback port booked at issue
    TrapCause trap       = TrapCause::NONE;

    // ---- front-end prediction state, and what recovery must undo ---------
    bool       pred_taken  = false;
    uint32_t   pred_target = 0;
    bool       btb_hit     = false;
    bool       from_ras    = false;
    uint32_t   pht_index   = 0;
    BranchKind bkind       = BranchKind::NONE;
    CheckpointId ckpt      = INVALID_CHECKPOINT;
    BranchPredictor::Snapshot fe_snap{};   // history and return stack at fetch
};

class Cpu {
public:
    Cpu(Memory& mem, const Config& cfg, uint32_t entry_pc);

    // Advance one cycle. Ticking a finished machine is a no-op.
    void tick();

    // Tick until finished or out of cycles; true if it finished.
    bool run(uint64_t max_cycles);

    const Config& config() const { return cfg_; }

    // ---- architectural state ---------------------------------------------
    // Reads go through the committed mapping, so they never observe a
    // speculative value.
    uint32_t reg(ArchReg r) const { return prf_.read(arch_rat_[r]); }
    std::array<uint32_t, 32> regs() const;
    uint32_t fetch_pc() const { return pc_; }        // where fetch looks next
    uint32_t arch_pc()  const { return arch_pc_; }   // next uncommitted PC

    // ---- run status -------------------------------------------------------
    uint64_t  cycle()      const { return cycle_; }
    uint64_t  retired()    const { return stats_.retired; }
    uint64_t  issued()     const { return stats_.issued; }
    bool      halted()     const { return halted_; }
    bool      trapped()    const { return trapped_; }
    bool      done()       const { return halted_ || trapped_; }
    uint32_t  exit_code()  const { return exit_code_; }
    TrapCause trap_cause() const { return trap_cause_; }
    const Stats& stats()   const { return stats_; }

    // No instruction anywhere in the pipeline.
    bool idle() const {
        return fetch_q_.empty() && decode_q_.empty() && rename_q_.empty() &&
               iq_.empty() && executing_.empty() && wb_fast_.empty() &&
               wb_slow_.empty() && rob_.empty() && lsq_.loads().empty() &&
               lsq_.stores().empty();
    }

    // False if anything ever committed out of program order.
    bool commit_in_order() const { return commit_in_order_; }

    // ---- pipeline observability -------------------------------------------
    const Rob&                  rob()         const { return rob_; }
    const PhysicalRegisterFile& prf()         const { return prf_; }
    const RegisterAliasTable&   rat()         const { return rat_; }
    const FreeList&             free_list()   const { return free_list_; }
    const CheckpointPool&       checkpoints() const { return ckpts_; }
    const IssueQueue&           iq()          const { return iq_; }
    const Lsq&                  lsq()         const { return lsq_; }
    const BranchPredictor&      bpred()       const { return bpred_; }

    uint32_t fetch_queue()   const { return static_cast<uint32_t>(fetch_q_.size()); }
    uint32_t decode_queue()  const { return static_cast<uint32_t>(decode_q_.size()); }
    uint32_t rename_queue()  const { return static_cast<uint32_t>(rename_q_.size()); }
    uint32_t executing()     const { return static_cast<uint32_t>(executing_.size()); }
    uint32_t queue_capacity() const { return queue_cap_; }
    bool     fetch_stalled() const { return fetch_stalled_; }

    // Committed mapping of an architectural register.
    PhysReg committed_map(ArchReg r) const { return arch_rat_[r]; }

    // ---- cycle-level observability ----------------------------------------
    // The whole state a photograph of one cycle needs. Everything here is a
    // const view of storage the pipeline already keeps, so an external tracer
    // (trace.h) can write a snapshot without the machine knowing what a trace
    // is.

    // Function-unit classes, in the order fu_free_at() indexes them.
    static constexpr int FU_CLASSES = 5;
    static const char* fu_class_name(int cls);

    const std::deque<Uop>&  fetch_uops()     const { return fetch_q_; }
    const std::deque<Uop>&  decode_uops()    const { return decode_q_; }
    const std::deque<Uop>&  rename_uops()    const { return rename_q_; }
    const std::vector<Uop>& executing_uops() const { return executing_; }
    const std::deque<Uop>&  wb_fast_uops()   const { return wb_fast_; }
    const std::deque<Uop>&  wb_slow_uops()   const { return wb_slow_; }

    // Per-instruction payload behind a live ROB slot. Only meaningful for a
    // slot the ROB reports as in flight.
    const Uop& inflight(RobIndex idx) const { return inflight_[idx]; }

    // Cycle each unit becomes free again, one vector per class.
    const std::array<std::vector<uint64_t>, FU_CLASSES>& fu_free_at() const {
        return fu_free_at_;
    }

    // Writeback ports booked, as a ring indexed by cycle % cdb_window().
    const std::vector<uint32_t>& cdb_bookings() const { return cdb_booked_; }
    uint64_t cdb_window() const { return cdb_window_; }

    // Writeback ports already booked for a (current or future) cycle.
    uint32_t cdb_reserved_at(uint64_t cycle) const {
        return cdb_booked_[static_cast<std::size_t>(cycle % cdb_window_)];
    }

    // ---- what happened during the current cycle ----------------------------
    // Recorded only while observing, and cleared at the top of every tick, so
    // these describe the cycle that just ran and nothing else. Off by default:
    // an untraced run pays one branch per would-be event.
    void observe(bool on) { observing_ = on; }
    bool observing() const { return observing_; }
    const std::vector<std::string>& events()        const { return events_; }
    const std::vector<SeqNum>&      squashed_seqs() const { return squashed_seqs_; }
    const std::vector<SeqNum>&      retired_seqs()  const { return retired_seqs_; }

    // ---- per-stage logs, recorded only when a test asks for them -----------
    struct DecodeRecord {
        uint64_t cycle;
        uint32_t pc;
        uint32_t raw;      // sequence numbers are assigned later, at rename
    };
    struct RenameRecord {
        uint64_t cycle;
        SeqNum   seq;
        uint32_t pc;
        ArchReg  rd;
        PhysReg  dest;
        PhysReg  stale;
        PhysReg  src1;
        PhysReg  src2;
    };
    struct IssueRecord {
        uint64_t cycle;
        SeqNum   seq;
        uint32_t pc;
        PhysReg  dest;
        uint64_t wb_cycle;
    };
    void record_decode(bool on) { record_decode_ = on; }
    void record_rename(bool on) { record_rename_ = on; }
    void record_issue (bool on) { record_issue_  = on; }
    const std::vector<DecodeRecord>& decode_log() const { return decode_log_; }
    const std::vector<RenameRecord>& rename_log() const { return rename_log_; }
    const std::vector<IssueRecord>&  issue_log()  const { return issue_log_; }

private:
    // Function-unit pools, in FU_CLASSES order. NONE covers the ops that
    // occupy no unit and is not a pool.
    enum class Fu : uint8_t { ALU, BRANCH, MUL, DIV, MEM, NONE };

    void commit();
    void writeback();
    void execute();
    void issue();
    void dispatch();
    void rename();
    void decode_stage();
    void fetch();

    // Compute the result, the store address and data, or the branch target.
    void execute_uop(Uop& u);

    // Resolve a load's address and search the store queue. False means an older
    // store may own these bytes but cannot yet say, so the load keeps its
    // issue-queue seat and retries next cycle.
    bool execute_load(Uop& u, uint32_t& latency);

    // Land a result: value into the PRF, tag onto the queue, entry marked done.
    void complete(const Uop& u);

    static Fu unit_of(const Decoded& d);
    uint32_t latency_of(const Decoded& d) const;
    bool     pipelined(Fu f) const { return f == Fu::MUL; }
    static Stall port_stall(Fu f);

    // Index of a unit free this cycle, or -1. Fu::NONE always "has" one.
    int free_unit(Fu f) const;

    // Book a writeback port for the cycle a result will land. Ops with a known
    // latency do this at issue and cannot issue without it, so CDB bandwidth is
    // modelled rather than assumed.
    bool reserve_cdb(uint64_t at_cycle);

    // Drop everything in flight and undo its renaming, so nothing behind a
    // halting instruction can reach commit or leak a physical register.
    void squash_in_flight();

    // Unwind to just after `br`, which mispredicted. Every younger uop returns
    // the register it allocated, never the stale one it displaced, which an
    // older entry still owns and returns itself.
    void recover(const Uop& br);

    // Give back a writeback port booked by a uop that is being squashed.
    void release_cdb(const Uop& u);

    // Append a line to this cycle's event log, printf-style. Does nothing
    // unless the machine is being observed. Checked like printf where the
    // compiler can, since a wrong specifier here is undefined behaviour that
    // only a traced run would ever hit.
#if defined(__GNUC__) || defined(__clang__)
    __attribute__((format(printf, 2, 3)))
#endif
    void event(const char* fmt, ...);

    // Drop the uops younger than `seq` from a queue of in-flight work.
    template <typename Q>
    void squash_queue(Q& q, SeqNum seq) {
        Q kept;
        for (const Uop& u : q) {
            if (u.seq <= seq) kept.push_back(u);
            else              release_cdb(u);
        }
        q.swap(kept);
    }

    Memory&              mem_;
    Config               cfg_;
    Rob                  rob_;
    PhysicalRegisterFile prf_;
    FreeList             free_list_;
    RegisterAliasTable   rat_;
    CheckpointPool       ckpts_;
    IssueQueue           iq_;
    Lsq                  lsq_;
    BranchPredictor      bpred_;

    // Per-instruction payload, indexed by ROB slot, read back at commit.
    std::vector<Uop> inflight_;

    // The mapping as of the last commit. Speculation cannot reach it, so it
    // defines what an architectural register read returns.
    std::array<PhysReg, 32> arch_rat_{};

    uint32_t pc_      = 0;
    uint32_t arch_pc_ = 0;

    // Inter-stage queues, each `queue_cap_` uops deep.
    std::deque<Uop> fetch_q_;     // fetch -> decode
    std::deque<Uop> decode_q_;    // decode -> rename
    std::deque<Uop> rename_q_;    // rename -> dispatch

    // Ops holding a function unit. Their `wb_cycle` says when the result is
    // due, so the unit needs no separate bookkeeping.
    std::vector<Uop> executing_;

    // Writeback candidates. Fast ops booked their port at issue and always
    // land; loads did not and take whatever bandwidth is left over.
    std::deque<Uop> wb_fast_;
    std::deque<Uop> wb_slow_;

    // Cycle each unit of each class becomes free again.
    std::array<std::vector<uint64_t>, FU_CLASSES> fu_free_at_;

    // Writeback ports booked per cycle, as a ring indexed by cycle. The window
    // is wider than the longest latency, so no two bookings can collide.
    std::vector<uint32_t> cdb_booked_;
    uint64_t              cdb_window_ = 1;

    uint32_t queue_cap_     = 0;
    bool     fetch_stalled_ = false;   // waiting on a control transfer to resolve

    uint64_t  cycle_      = 0;
    bool      halted_     = false;
    bool      trapped_    = false;
    uint32_t  exit_code_  = 0;
    TrapCause trap_cause_ = TrapCause::NONE;
    bool      commit_in_order_ = true;
    SeqNum    last_committed_seq_ = INVALID_SEQNUM;
    Stats     stats_{};

    uint32_t next_uid_ = 0;   // fetch order, never reused, squashes included

    // What the current cycle did, kept only while observing (see observe()).
    bool                     observing_ = false;
    std::vector<std::string> events_;
    std::vector<SeqNum>      squashed_seqs_;
    std::vector<SeqNum>      retired_seqs_;

    bool record_decode_ = false;
    bool record_rename_ = false;
    bool record_issue_  = false;
    std::vector<DecodeRecord> decode_log_;
    std::vector<RenameRecord> rename_log_;
    std::vector<IssueRecord>  issue_log_;
};
