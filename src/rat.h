#pragma once

#include <array>
#include <cstdint>
#include <deque>
#include <optional>
#include <vector>

#include "config.h"
#include "types.h"

// Maps 32 architectural registers to their current physical mapping.
// set(0, _) is a no-op so callers do not guard on x0.
//
// A bounded pool holds full RAT snapshots, one per in-flight branch.
// alloc_checkpoint() copies the RAT at call time; restore_checkpoint()
// copies it back; free_checkpoint() releases the slot. The pool refuses
// allocation when full — the caller stalls the branch that would take it.

class Rat {
public:
    static constexpr uint32_t ARCH_REGS = 32;

    explicit Rat(uint32_t num_checkpoints)
        : snapshots_(num_checkpoints), num_checkpoints_(num_checkpoints) {
        reset();
    }
    explicit Rat(const Config& cfg) : Rat(cfg.num_checkpoints) {}

    PhysReg map(ArchReg a) const { return rat_[a]; }

    // Rewrite the mapping for `a`. No-op on x0.
    void set(ArchReg a, PhysReg p) {
        if (a == 0) return;
        rat_[a] = p;
    }

    // The whole array, for tests and for anything that walks all 32 slots.
    const std::array<PhysReg, ARCH_REGS>& mapping() const { return rat_; }

    // Adopt a mapping wholesale. Squashing everything in flight restores the
    // committed one this way, without going through a checkpoint.
    void adopt(const std::array<PhysReg, ARCH_REGS>& m) { rat_ = m; }

    uint32_t num_checkpoints()      const { return num_checkpoints_; }
    uint32_t num_free_checkpoints() const {
        return static_cast<uint32_t>(free_slots_.size());
    }

    // Snapshot the RAT into a fresh slot. nullopt when the pool is full.
    std::optional<CheckpointId> alloc_checkpoint() {
        if (free_slots_.empty()) return std::nullopt;
        const CheckpointId id = free_slots_.front();
        free_slots_.pop_front();
        snapshots_[id] = rat_;
        return id;
    }

    // Copy the snapshot back into the RAT. Does not release the slot;
    // recovery frees it separately so age-ordered squash logic stays in
    // one place.
    void restore_checkpoint(CheckpointId id) {
        rat_ = snapshots_[id];
    }

    // Push a slot back to the pool. No-op on INVALID_CHECKPOINT or
    // out-of-range so commit / recovery can call unconditionally.
    void free_checkpoint(CheckpointId id) {
        if (id >= num_checkpoints_) return;
        free_slots_.push_back(id);
    }

    void reset() {
        for (uint32_t i = 0; i < ARCH_REGS; ++i) rat_[i] = i;
        free_slots_.clear();
        for (CheckpointId i = 0; i < num_checkpoints_; ++i) free_slots_.push_back(i);
    }

private:
    std::array<PhysReg, ARCH_REGS>              rat_{};
    std::vector<std::array<PhysReg, ARCH_REGS>> snapshots_;
    std::deque<CheckpointId>                    free_slots_;
    uint32_t                                    num_checkpoints_;
};
