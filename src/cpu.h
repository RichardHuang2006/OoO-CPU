// The out-of-order core.
//
// Stages: Fetch -> Decode -> Rename -> Dispatch -> Issue -> Execute -> Writeback,
// with in-order Commit driven off the ROB head. tick() evaluates the stages in
// reverse pipeline order so that each stage observes the previous cycle's
// output of its producer -- i.e. the inter-stage queues behave as latches.
//
// The one deliberate intra-cycle path is Writeback -> Issue: a tag broadcast in
// cycle T clears ready bits before select runs in the same cycle T, which is
// what lets a dependent 1-cycle ALU op issue back-to-back with its producer.
#pragma once
#include <deque>
#include <string>
#include <vector>

#include "alu.h"
#include "bpred.h"
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

class Cpu {
public:
    Cpu(const Config& cfg, Memory& mem, uint32_t entry_pc);

    // Advance one cycle. Returns false once the core has halted.
    bool tick();

    // Run until halt, max_cycles, or a deadlock watchdog fires.
    void run();

    bool halted() const { return halted_; }
    int  exit_code() const { return exit_code_; }
    const Stats& stats() const { return st_; }
    const Config& config() const { return cfg_; }

    // Committed architectural register value (via the retirement RAT).
    uint32_t arch_reg(int r) const { return prf_.read(arat_.lookup(r)); }

    // Seed architectural state before the first tick (e.g. the stack pointer).
    void set_arch_reg(int r, uint32_t v) { if (r) prf_.write(arat_.lookup(r), v); }
    uint32_t pc() const { return pc_; }

    void print_stats(std::ostream& os) const;

private:
    // ---- pipeline stages, in reverse evaluation order --------------------
    void commit_stage();
    void writeback_stage();
    void execute_stage();
    void issue_stage();
    void dispatch_stage();
    void rename_stage();
    void decode_stage();
    void fetch_stage();

    // ---- helpers ---------------------------------------------------------
    struct FuOp {
        Uop uop;
        int remaining = 0;      // cycles left in the function unit
        uint64_t wb_cycle = 0;  // cycle its reserved CDB slot belongs to
        bool reserved = false;  // holds a CDB reservation
    };
    struct WbOp {
        Uop uop;
        bool needs_cdb = false;
        bool reserved = false;
    };

    // Compute the result of `u`. Returns false if a load must replay because
    // an older store's address is still unknown.
    bool complete_op(Uop& u, bool& replay);

    void push_wb(Uop& u, bool reserved);    // hand a completed uop to writeback
    int  fu_latency(const DecodedInst& d) const;
    bool fu_port_available(FU fu);
    void consume_fu_port(FU fu);

    bool reserve_cdb(uint64_t cycle);
    void release_cdb(uint64_t cycle);
    int& cdb_slot(uint64_t cycle);

    void record_misprediction(const Uop& u);
    void handle_recovery();
    void flush_frontend();

    void update_predictors(const RobEntry& e);
    void predict(uint32_t pc, const DecodedInst& pre, Uop& u);

    void trace(const char* stage, const Uop& u) const;

    // ---- state -----------------------------------------------------------
    Config cfg_;
    Memory& mem_;
    Stats st_;

    RAT rat_;      // speculative, checkpointed at every branch
    RAT arat_;     // retirement RAT: committed architectural mapping
    PRF prf_;
    FreeList fl_;
    ROB rob_;
    IssueQueue iq_;
    LSQ lsq_;
    CheckpointPool ckpt_;

    Gshare gshare_;
    BTB btb_;
    RAS ras_;

    // Inter-stage latches.
    std::deque<Uop> fq_;   // fetch  -> decode (raw word + prediction)
    std::deque<Uop> dq_;   // decode -> rename
    std::deque<Uop> rq_;   // rename -> dispatch

    std::vector<FuOp> fus_;    // in-flight execution
    std::vector<WbOp> wb_;     // awaiting a writeback port

    std::vector<int> cdb_ring_;   // reserved CDB count per future cycle
    static constexpr int kCdbRing = 64;

    // Per-cycle function unit port budget.
    int alu_avail_ = 0, mul_avail_ = 0, mem_avail_ = 0, br_avail_ = 0;
    uint64_t div_busy_until_ = 0;   // the divider is not pipelined

    uint32_t pc_ = 0;
    uint64_t seq_ = 0;
    uint64_t cycle_ = 0;

    // Pending misprediction recovery, applied at the end of the tick.
    bool     redirect_pending_ = false;
    Uop      redirect_uop_;

    bool halted_ = false;
    int  exit_code_ = 0;
    std::string halt_reason_;
    uint64_t last_commit_cycle_ = 0;

public:
    const std::string& halt_reason() const { return halt_reason_; }
};
