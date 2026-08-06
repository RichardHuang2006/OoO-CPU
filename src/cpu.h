#pragma once

#include <array>
#include <cstdint>
#include <deque>
#include <vector>

#include "config.h"
#include "decoder.h"
#include "memory.h"
#include "rob.h"
#include "types.h"

// In-order pipeline: fetch, decode, execute, writeback, commit, with a reorder
// buffer holding each instruction from dispatch to commit and `Config::width`
// uops moving through every stage per cycle.
//
// tick() runs the stages in reverse order, so each sees what its producer left
// behind last cycle and the queues between them behave like hardware latches.
// That already buys one real property: a result written back in cycle T frees
// its register before issue runs in cycle T, so a dependent op issues back to
// back with its producer instead of losing a cycle.
//
// Still strictly in order and still unrenamed, which shows up as three stalls:
// issue stops at the first uop whose operands are not ready, results write back
// in issue order, and a load waits for every older store to commit because
// there is no store queue to forward from yet.
//
// The front end has no branch predictor either. Decode stops fetching when it
// sees a control transfer and execute redirects the PC, so no wrong-path
// instruction is ever fetched.

enum class TrapCause : uint8_t {
    NONE,
    ILLEGAL,          // undecodable instruction
    EBREAK,
    ECALL_UNKNOWN,    // ecall with a7 != 93
};

// One in-flight instruction, gaining fields as it moves down the pipeline:
// fetch fills pc/raw, decode fills `dec` and the ROB index, execute fills the
// result or the store address and data.
struct Uop {
    SeqNum    seq        = INVALID_SEQNUM;
    RobIndex  rob        = INVALID_ROBINDEX;
    uint32_t  pc         = 0;
    uint32_t  raw        = 0;
    Decoded   dec        {};
    uint32_t  result     = 0;
    bool      has_result = false;
    uint32_t  next_pc    = 0;
    uint32_t  mem_addr   = 0;    // store address, computed at execute
    uint32_t  store_data = 0;    // store data, written to memory at commit
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
    uint32_t reg(ArchReg r) const { return regs_[r]; }
    const std::array<uint32_t, 32>& regs() const { return regs_; }
    uint32_t fetch_pc() const { return pc_; }        // where fetch looks next
    uint32_t arch_pc()  const { return arch_pc_; }   // next uncommitted PC

    // ---- run status -------------------------------------------------------
    uint64_t  cycle()      const { return cycle_; }
    uint64_t  retired()    const { return retired_; }
    uint64_t  issued()     const { return issued_; }
    bool      halted()     const { return halted_; }
    bool      trapped()    const { return trapped_; }
    bool      done()       const { return halted_ || trapped_; }
    uint32_t  exit_code()  const { return exit_code_; }
    TrapCause trap_cause() const { return trap_cause_; }

    // No instruction anywhere in the pipeline.
    bool idle() const {
        return fetch_q_.empty() && dispatch_q_.empty() && executing_.empty() &&
               wb_q_.empty() && rob_.empty();
    }

    // False if anything ever committed out of program order.
    bool commit_in_order() const { return commit_in_order_; }

    // ---- pipeline observability -------------------------------------------
    const Rob& rob() const { return rob_; }
    uint32_t fetch_queue()    const { return static_cast<uint32_t>(fetch_q_.size()); }
    uint32_t dispatch_queue() const { return static_cast<uint32_t>(dispatch_q_.size()); }
    uint32_t executing()      const { return static_cast<uint32_t>(executing_.size()); }
    uint32_t fetch_queue_capacity()    const { return queue_cap_; }
    uint32_t dispatch_queue_capacity() const { return queue_cap_; }
    bool     fetch_stalled() const { return fetch_stalled_; }

    // Decode output, recorded only when a test asks for it.
    struct DecodeRecord {
        uint64_t cycle;
        SeqNum   seq;
        uint32_t pc;
        uint32_t raw;
    };
    void record_decode(bool on) { record_decode_ = on; }
    const std::vector<DecodeRecord>& decode_log() const { return decode_log_; }

private:
    // Function-unit pools. NONE covers the ops that occupy no unit.
    enum class Fu : uint8_t { ALU, BRANCH, MUL, DIV, MEM, NONE };
    static constexpr int FU_COUNT = 5;

    void commit();
    void writeback();
    void execute();
    void decode_stage();
    void fetch();

    // Compute the result, the store address and data, or the branch target.
    void execute_uop(Uop& u);

    static Fu unit_of(const Decoded& d);
    uint32_t latency_of(const Decoded& d) const;
    bool     pipelined(Fu f) const { return f == Fu::MUL; }

    // Index of a unit free this cycle, or -1. Fu::NONE always "has" one.
    int free_unit(Fu f) const;

    // An operand is not ready while an already-issued op still owes it.
    bool operands_ready(const Decoded& d) const {
        return reg_busy_[d.rs1] == 0 && reg_busy_[d.rs2] == 0;
    }

    // True while any store older than `seq` is still in the ROB. Stores write
    // memory at commit, so a load issued before then would read stale bytes.
    bool older_store_pending(SeqNum seq) const;

    // Drop everything in flight, so nothing behind a halting instruction can
    // reach commit.
    void squash_in_flight();

    Memory& mem_;
    Config  cfg_;
    Rob     rob_;

    // Per-instruction payload, indexed by ROB slot, read back at commit.
    std::vector<Uop> inflight_;

    std::array<uint32_t, 32> regs_{};
    uint32_t pc_      = 0;
    uint32_t arch_pc_ = 0;

    // Outstanding writes per architectural register, the in-order machine's
    // stand-in for renaming.
    std::array<uint32_t, 32> reg_busy_{};

    // Inter-stage queues, each `queue_cap_` uops deep.
    std::deque<Uop> fetch_q_;       // fetch -> decode
    std::deque<Uop> dispatch_q_;    // decode -> execute
    std::deque<Uop> wb_q_;          // execute -> writeback

    // Ops occupying a function unit, in issue order. `finish` is the cycle
    // their result appears; draining from the front keeps writeback in order.
    struct FuOp {
        Uop      uop;
        uint64_t finish;
    };
    std::deque<FuOp> executing_;

    // Cycle each unit of each class becomes free again.
    std::array<std::vector<uint64_t>, FU_COUNT> fu_free_at_;

    uint32_t queue_cap_     = 0;
    bool     fetch_stalled_ = false;   // waiting on a control transfer to resolve

    uint64_t  cycle_      = 0;
    uint64_t  retired_    = 0;
    uint64_t  issued_     = 0;
    bool      halted_     = false;
    bool      trapped_    = false;
    uint32_t  exit_code_  = 0;
    TrapCause trap_cause_ = TrapCause::NONE;
    bool      commit_in_order_ = true;

    bool                       record_decode_ = false;
    std::vector<DecodeRecord>  decode_log_;
};
