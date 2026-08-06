#pragma once

#include <array>
#include <cstdint>
#include <optional>

#include "config.h"
#include "decoder.h"
#include "memory.h"
#include "types.h"

// The pipeline model. This is the shell every Phase 4-7 mechanism is grafted
// into, so it starts deliberately small: five stages, one instruction per
// latch, architectural registers only.
//
//     Fetch → Decode → Execute → Writeback → Commit
//
// Stages are evaluated in *reverse* order inside tick() — commit before
// writeback before execute before decode before fetch (DESIGN.md §2). Each
// stage therefore observes the state its producer left behind last cycle,
// which is what makes the single-slot latches behave like hardware latches
// rather than a same-cycle conveyor belt.
//
// That ordering is not a stylistic choice. It is the reason a tag broadcast
// at Writeback in cycle T can clear an issue-queue ready bit *before* select
// runs in cycle T (§4.2), which is what Step 5.5's back-to-back dependent
// issue depends on. Getting it right here means that property costs nothing
// later.
//
// What this step does NOT do, by design: no width (one uop per stage per
// cycle regardless of Config::width), no reorder buffer, no renaming, no
// branches, no memory operations, no function-unit latencies. Every
// unimplemented op class raises TrapCause::UNIMPLEMENTED rather than
// silently producing nothing — a silent no-op here would surface in Phase 5
// as an inexplicable register mismatch.

enum class TrapCause : uint8_t {
    NONE,
    ILLEGAL,          // undecodable instruction
    EBREAK,
    ECALL_UNKNOWN,    // ecall with a7 != 93
    UNIMPLEMENTED,    // an op class this phase does not execute yet
};

// One in-flight instruction. A single record travels the whole pipeline,
// gaining fields as it goes: Fetch fills seq/pc/raw, Decode fills `dec`,
// Execute fills the result. Later phases add rename and ROB fields here.
struct Uop {
    SeqNum    seq        = INVALID_SEQNUM;
    uint32_t  pc         = 0;
    uint32_t  raw        = 0;
    Decoded   dec        {};
    uint32_t  result     = 0;
    bool      has_result = false;
    uint32_t  next_pc    = 0;
    TrapCause trap       = TrapCause::NONE;
};

class Cpu {
public:
    Cpu(Memory& mem, const Config& cfg, uint32_t entry_pc);

    // Advance one cycle. A machine that has halted or trapped is frozen:
    // ticking it again is a no-op, so a driver loop cannot accidentally
    // manufacture cycles after the program is over.
    void tick();

    // Tick until the machine finishes or `max_cycles` is reached. Returns
    // true if it finished (halted or trapped) rather than running out.
    bool run(uint64_t max_cycles);

    const Config& config() const { return cfg_; }

    // ---- architectural state ---------------------------------------------
    uint32_t reg(ArchReg r) const { return regs_[r]; }
    const std::array<uint32_t, 32>& regs() const { return regs_; }
    uint32_t fetch_pc() const { return pc_; }        // where Fetch will look next
    uint32_t arch_pc()  const { return arch_pc_; }   // PC of the next uncommitted insn

    // ---- run status -------------------------------------------------------
    uint64_t  cycle()      const { return cycle_; }
    uint64_t  retired()    const { return retired_; }
    bool      halted()     const { return halted_; }
    bool      trapped()    const { return trapped_; }
    bool      done()       const { return halted_ || trapped_; }
    uint32_t  exit_code()  const { return exit_code_; }
    TrapCause trap_cause() const { return trap_cause_; }

    // No instruction anywhere in the pipeline.
    bool idle() const { return !if_id_ && !id_ex_ && !ex_wb_ && !wb_cm_; }

    // Commit must be strictly in program order — trivially true with one
    // in-order path, but Phase 5 keeps commit in order while everything
    // upstream reorders, and this flag is what notices if it stops being.
    bool commit_in_order() const { return commit_in_order_; }

private:
    void commit();
    void writeback();
    void execute();
    void decode_stage();
    void fetch();

    // Drop everything in flight. On a halt or trap the younger instructions
    // already fetched behind it must never reach commit; this is also the
    // shape Step 7.5's misprediction recovery takes.
    void squash_in_flight();

    Memory&  mem_;
    Config   cfg_;

    std::array<uint32_t, 32> regs_{};
    uint32_t pc_      = 0;
    uint32_t arch_pc_ = 0;

    // Inter-stage latches, one uop deep. Step 3.3 widens the front-end pair
    // into queues; the back end stays latch-shaped until the issue queue
    // replaces it in Phase 5.
    std::optional<Uop> if_id_;
    std::optional<Uop> id_ex_;
    std::optional<Uop> ex_wb_;
    std::optional<Uop> wb_cm_;

    uint64_t  cycle_      = 0;
    uint64_t  retired_    = 0;
    SeqNum    next_seq_   = 0;
    bool      halted_     = false;
    bool      trapped_    = false;
    uint32_t  exit_code_  = 0;
    TrapCause trap_cause_ = TrapCause::NONE;
    bool      commit_in_order_ = true;
};
