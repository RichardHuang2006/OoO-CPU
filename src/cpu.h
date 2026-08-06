#pragma once

#include <array>
#include <cstdint>
#include <optional>

#include "config.h"
#include "decoder.h"
#include "memory.h"
#include "types.h"

// Five-stage in-order pipeline: fetch, decode, execute, writeback, commit.
// One uop per latch, architectural registers only.
//
// tick() runs the stages in reverse order, so each one sees what its producer
// left behind last cycle and the latches behave like hardware latches.
//
// Not here yet: width, reorder buffer, renaming, branches, memory ops,
// function-unit latencies. Op classes that are not executed raise
// TrapCause::UNIMPLEMENTED rather than silently producing no result.

enum class TrapCause : uint8_t {
    NONE,
    ILLEGAL,          // undecodable instruction
    EBREAK,
    ECALL_UNKNOWN,    // ecall with a7 != 93
    UNIMPLEMENTED,    // op class not executed yet
};

// One in-flight instruction, gaining fields as it moves down the pipeline:
// fetch fills seq/pc/raw, decode fills `dec`, execute fills the result.
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
    bool      halted()     const { return halted_; }
    bool      trapped()    const { return trapped_; }
    bool      done()       const { return halted_ || trapped_; }
    uint32_t  exit_code()  const { return exit_code_; }
    TrapCause trap_cause() const { return trap_cause_; }

    // No instruction anywhere in the pipeline.
    bool idle() const { return !if_id_ && !id_ex_ && !ex_wb_ && !wb_cm_; }

    // False if anything ever committed out of program order.
    bool commit_in_order() const { return commit_in_order_; }

private:
    void commit();
    void writeback();
    void execute();
    void decode_stage();
    void fetch();

    // Drop everything in flight, so nothing fetched behind a halting
    // instruction can reach commit.
    void squash_in_flight();

    Memory&  mem_;
    Config   cfg_;

    std::array<uint32_t, 32> regs_{};
    uint32_t pc_      = 0;
    uint32_t arch_pc_ = 0;

    // Inter-stage latches, one uop deep.
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
