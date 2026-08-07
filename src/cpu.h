#pragma once

#include <array>
#include <cstdint>
#include <deque>
#include <vector>

#include "config.h"
#include "decoder.h"
#include "freelist.h"
#include "issue_queue.h"
#include "lsq.h"
#include "memory.h"
#include "prf.h"
#include "rat.h"
#include "rob.h"
#include "stats.h"
#include "types.h"

// Seven stages — fetch, decode, rename, dispatch, issue, execute, writeback —
// feeding in-order commit off the reorder-buffer head, `Config::width` uops per
// stage per cycle.
//
// tick() runs the stages in reverse order, so each drains what its producer
// left behind last cycle and the queues between them behave like latches. The
// one deliberate exception is writeback, which broadcasts a result before issue
// runs in the same cycle: that is what lets a dependent single-cycle op issue
// back to back with its producer instead of losing a cycle to the round trip.
//
// Rename is what makes the back end out of order. Sources become physical tags
// from the RAT, destinations come off the free list, and the displaced mapping
// rides in the ROB entry until commit returns it — so two writers to the same
// architectural register never share storage and only true dependences remain.
// Issue then picks the oldest ready ops the function units and writeback ports
// can accept, in whatever order that turns out to be.
//
// Two things are still simpler than hardware. A load waits for every older
// store to commit, because there is no store queue to forward from yet, and the
// front end has no predictor: decode stops fetching at a control transfer and
// issue redirects the PC, so no wrong-path instruction is ever fetched.

enum class TrapCause : uint8_t {
    NONE,
    ILLEGAL,          // undecodable instruction
    EBREAK,
    ECALL_UNKNOWN,    // ecall with a7 != 93
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
    // What a register holds is whatever its committed mapping points at, so
    // reads here never see a speculative value.
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
    const Rob&        rob()       const { return rob_; }
    const Prf&        prf()       const { return prf_; }
    const Rat&        rat()       const { return rat_; }
    const FreeList&   free_list() const { return free_list_; }
    const IssueQueue& iq()        const { return iq_; }
    const Lsq&        lsq()       const { return lsq_; }

    uint32_t fetch_queue()   const { return static_cast<uint32_t>(fetch_q_.size()); }
    uint32_t decode_queue()  const { return static_cast<uint32_t>(decode_q_.size()); }
    uint32_t rename_queue()  const { return static_cast<uint32_t>(rename_q_.size()); }
    uint32_t executing()     const { return static_cast<uint32_t>(executing_.size()); }
    uint32_t queue_capacity() const { return queue_cap_; }
    bool     fetch_stalled() const { return fetch_stalled_; }

    // Committed mapping of an architectural register.
    PhysReg committed_map(ArchReg r) const { return arch_rat_[r]; }

    // ---- traces, recorded only when a test asks for them -------------------
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
    // Function-unit pools. NONE covers the ops that occupy no unit.
    enum class Fu : uint8_t { ALU, BRANCH, MUL, DIV, MEM, NONE };
    static constexpr int FU_COUNT = 5;

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

    // Resolve a load's address and ask the store queue about it. False means
    // an older store might own these bytes but cannot say yet, so the load
    // stays in the issue queue and tries again next cycle.
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
    // latency do this at issue and cannot issue without it, which keeps the
    // port count honest instead of letting results pile up at writeback.
    bool reserve_cdb(uint64_t at_cycle);

    // Drop everything in flight and undo its renaming, so nothing behind a
    // halting instruction can reach commit or leak a physical register.
    void squash_in_flight();

    Memory&     mem_;
    Config      cfg_;
    Rob         rob_;
    Prf         prf_;
    FreeList    free_list_;
    Rat         rat_;
    IssueQueue  iq_;
    Lsq         lsq_;

    // Per-instruction payload, indexed by ROB slot, read back at commit.
    std::vector<Uop> inflight_;

    // The mapping as of the last commit. Speculation cannot reach it, so it is
    // what an architectural register read means.
    std::array<PhysReg, 32> arch_rat_{};

    uint32_t pc_      = 0;
    uint32_t arch_pc_ = 0;

    // Inter-stage queues, each `queue_cap_` uops deep.
    std::deque<Uop> fetch_q_;     // fetch -> decode
    std::deque<Uop> decode_q_;    // decode -> rename
    std::deque<Uop> rename_q_;    // rename -> dispatch

    // Ops holding a function unit. `finish` is the cycle their result appears.
    struct FuOp {
        Uop      uop;
        uint64_t finish;
    };
    std::vector<FuOp> executing_;

    // Writeback candidates. Fast ops booked their port at issue and always
    // land; loads did not and take whatever bandwidth is left over.
    std::deque<Uop> wb_fast_;
    std::deque<Uop> wb_slow_;

    // Cycle each unit of each class becomes free again.
    std::array<std::vector<uint64_t>, FU_COUNT> fu_free_at_;

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
    Stats     stats_{};

    bool record_decode_ = false;
    bool record_rename_ = false;
    bool record_issue_  = false;
    std::vector<DecodeRecord> decode_log_;
    std::vector<RenameRecord> rename_log_;
    std::vector<IssueRecord>  issue_log_;
};
