#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <deque>
#include <optional>
#include <vector>

#include "branch_predictor.h"   // BranchPredictor::Snapshot, checkpointed with the RAT
#include "config.h"
#include "instruction.h"

// R10000-style register renaming: every structure that maps architectural
// registers onto the unified physical register file lives in this file.
//
//   PhysicalRegisterFile   the values, one ready bit per register
//   FreeList               physical registers not owned by anyone
//   RegisterAliasTable     the current (speculative) arch → phys mapping
//   Checkpoint             what one branch must be able to restore
//   CheckpointPool         a bounded pool of those, one per in-flight branch
//
// The complete renaming transaction, performed by Cpu::rename() and undone by
// Cpu::recover() / finished by Cpu::commit():
//
//   1. Read the source mappings:      src1 = rat.map(rs1), src2 = rat.map(rs2)
//   2. Allocate a destination:        dest = free_list.alloc()   (stall if empty)
//   3. Remember the displaced map:    stale = rat.map(rd)        (kept in the ROB)
//   4. Install the new mapping:       rat.set(rd, dest)
//   5. Mark the value outstanding:    prf.mark_pending(dest)
//   6. At commit:                     arch mapping := dest; free_list.free(stale)
//   7. After a squash, youngest first: free_list.free(dest of each squashed uop),
//      then restore the checkpointed RAT (or adopt the committed mapping).
//
// Two writers of one architectural register never share physical storage, so
// WAW and WAR hazards disappear at rename and only true dependences (RAW,
// carried by the physical tags) reach the issue queue. The stale mapping is
// freed only at commit — freeing it earlier could hand the register to a new
// writer while an older in-flight reader still holds its tag.
//
// x0 discipline: p0 is permanently mapped to x0, always ready, and never
// allocated or freed, so no caller guards on rd == x0 or rs == x0.

// ---------------------------------------------------------------------------
// Unified physical register file: one value and one ready bit per register.
// The issue queue tracks the ready bit; the value is read after select.
// ---------------------------------------------------------------------------
class PhysicalRegisterFile {
public:
    explicit PhysicalRegisterFile(uint32_t capacity)
        : values_(capacity, 0), ready_(capacity, 1) {}
    explicit PhysicalRegisterFile(const Config& cfg)
        : PhysicalRegisterFile(cfg.prf_size) {}

    uint32_t capacity() const { return static_cast<uint32_t>(values_.size()); }

    uint32_t read(PhysReg r) const {
        return r == 0 ? 0u : values_[r];
    }

    // Writes the value and flips ready on. No-op for p0.
    void write(PhysReg r, uint32_t value) {
        if (r == 0) return;
        values_[r] = value;
        ready_[r]  = 1;
    }

    bool is_ready(PhysReg r) const {
        return r == 0 || ready_[r] != 0;
    }

    // Clears ready after a fresh allocation (step 5). p0 stays ready, so no
    // in-flight uop ever waits on x0 to produce a value.
    void mark_pending(PhysReg r) {
        if (r == 0) return;
        ready_[r] = 0;
    }

    void reset() {
        std::fill(values_.begin(), values_.end(), 0u);
        std::fill(ready_.begin(),  ready_.end(),  uint8_t{1});
    }

private:
    std::vector<uint32_t> values_;
    std::vector<uint8_t>  ready_;   // 1 = ready, 0 = pending
};

// ---------------------------------------------------------------------------
// Physical registers neither mapped by the RAT nor held in the ROB as a stale
// mapping. FIFO order is arbitrary but keeps runs deterministic and spreads
// reuse out in time.
//
// p0 is reserved for x0 forever; p1..p31 hold the reset RAT; p32 upward start
// free. free() silently ignores p0, INVALID_PHYSREG, and out-of-range indices
// so commit-time reclamation can be a single unconditional call.
// ---------------------------------------------------------------------------
class FreeList {
public:
    explicit FreeList(uint32_t prf_size) : capacity_(prf_size) {
        reset();
    }
    explicit FreeList(const Config& cfg) : FreeList(cfg.prf_size) {}

    uint32_t capacity() const { return capacity_; }
    uint32_t num_free() const { return static_cast<uint32_t>(free_.size()); }
    bool     empty()    const { return free_.empty(); }

    // Pop the next free register (step 2), nullopt when starved. Callers
    // stall rename on nullopt.
    std::optional<PhysReg> alloc() {
        if (free_.empty()) return std::nullopt;
        const PhysReg r = free_.front();
        free_.pop_front();
        return r;
    }

    // Push a register back (steps 6 and 7). See header comment for the
    // ignored values.
    void free(PhysReg r) {
        if (r == 0 || r >= capacity_ || r == INVALID_PHYSREG) return;
        free_.push_back(r);
    }

    // Linear, but only tests ask; the pipeline never looks.
    bool contains(PhysReg r) const {
        return std::find(free_.begin(), free_.end(), r) != free_.end();
    }

    void reset() {
        free_.clear();
        for (uint32_t r = 32; r < capacity_; ++r) free_.push_back(r);
    }

private:
    uint32_t             capacity_;
    std::deque<PhysReg>  free_;
};

// ---------------------------------------------------------------------------
// Maps 32 architectural registers to their current speculative physical
// mapping. set(0, _) is a no-op so callers do not guard on x0. The committed
// counterpart lives in the Cpu (arch_rat_) and advances only at commit.
// ---------------------------------------------------------------------------
class RegisterAliasTable {
public:
    static constexpr uint32_t ARCH_REGS = 32;

    RegisterAliasTable() { reset(); }

    PhysReg map(ArchReg a) const { return rat_[a]; }

    // Rewrite the mapping for `a` (step 4). No-op on x0.
    void set(ArchReg a, PhysReg p) {
        if (a == 0) return;
        rat_[a] = p;
    }

    // The whole array: snapshotted into a checkpoint, and walked by tests.
    const std::array<PhysReg, ARCH_REGS>& mapping() const { return rat_; }

    // Adopt a mapping wholesale: checkpoint restore after a misprediction, or
    // the committed mapping when everything in flight is squashed (step 7).
    void adopt(const std::array<PhysReg, ARCH_REGS>& m) { rat_ = m; }

    void reset() {
        for (uint32_t i = 0; i < ARCH_REGS; ++i) rat_[i] = i;
    }

private:
    std::array<PhysReg, ARCH_REGS> rat_{};
};

// ---------------------------------------------------------------------------
// What one in-flight branch must be able to restore: the speculative RAT as of
// the branch (its own destination included, because the branch itself survives
// recovery), and the front end's speculative state — global history and the
// return address stack — as of its fetch.
// ---------------------------------------------------------------------------
struct Checkpoint {
    std::array<PhysReg, RegisterAliasTable::ARCH_REGS> rat{};
    BranchPredictor::Snapshot front_end{};
};

// A bounded pool of checkpoints, one slot per in-flight branch. The pool
// refuses allocation when full; the caller stalls the branch that would have
// taken the slot, because a branch that cannot be checkpointed cannot be
// recovered from. Slots are released at the branch's commit, or during a
// squash when the branch itself is on the wrong path.
class CheckpointPool {
public:
    explicit CheckpointPool(uint32_t capacity)
        : slots_(capacity), capacity_(capacity) {
        reset();
    }
    explicit CheckpointPool(const Config& cfg) : CheckpointPool(cfg.num_checkpoints) {}

    uint32_t capacity() const { return capacity_; }
    uint32_t num_free() const { return static_cast<uint32_t>(free_slots_.size()); }

    // Take a slot; nullopt when exhausted. The caller fills at(id) itself, so
    // what a checkpoint captures is visible at the call site in Cpu::rename().
    std::optional<CheckpointId> alloc() {
        if (free_slots_.empty()) return std::nullopt;
        const CheckpointId id = free_slots_.front();
        free_slots_.pop_front();
        return id;
    }

    Checkpoint&       at(CheckpointId id)       { return slots_[id]; }
    const Checkpoint& at(CheckpointId id) const { return slots_[id]; }

    // Push a slot back. No-op on INVALID_CHECKPOINT or out-of-range so commit
    // and recovery can call unconditionally for every squashed entry.
    void free(CheckpointId id) {
        if (id >= capacity_) return;
        free_slots_.push_back(id);
    }

    void reset() {
        free_slots_.clear();
        for (CheckpointId i = 0; i < capacity_; ++i) free_slots_.push_back(i);
    }

private:
    std::vector<Checkpoint>  slots_;
    std::deque<CheckpointId> free_slots_;
    uint32_t                 capacity_;
};
