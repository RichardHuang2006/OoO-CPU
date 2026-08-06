#pragma once

#include <cstdint>
#include <deque>
#include <optional>

#include "config.h"
#include "types.h"

// The set of physical registers not currently mapped to any architectural
// register and not currently sitting in the ROB as a stale mapping. Rename
// pops from the front, commit pushes to the back. FIFO is arbitrary — the
// ordering has no correctness meaning — but it makes traces deterministic
// and keeps the fresh mapping "far" from a just-freed one in time, which
// tends to spread out reuse in a way that helps a human reader.
//
// Initial contents follow DESIGN.md section 3: p0 is reserved for x0
// forever, p1..p31 are the reset RAT mappings, and everything from p32
// upward starts free.
//
// The class enforces its own invariants: free() ignores p0, INVALID_PHYSREG,
// and out-of-range indices, so commit can call free(stale_phys)
// unconditionally even for the writes_rd == false path where stale_phys
// is INVALID_PHYSREG.
class FreeList {
public:
    explicit FreeList(uint32_t prf_size) : capacity_(prf_size) {
        reset();
    }
    explicit FreeList(const Config& cfg) : FreeList(cfg.prf_size) {}

    uint32_t capacity() const { return capacity_; }
    uint32_t num_free() const { return static_cast<uint32_t>(free_.size()); }
    bool     empty()    const { return free_.empty(); }

    // Pop the next free register, or nullopt when starved. Callers stall
    // rename on nullopt.
    std::optional<PhysReg> alloc() {
        if (free_.empty()) return std::nullopt;
        const PhysReg r = free_.front();
        free_.pop_front();
        return r;
    }

    // Push a register back. Silently no-ops on p0 / INVALID_PHYSREG /
    // out-of-range so commit-time reclamation can be a single call.
    void free(PhysReg r) {
        if (r == 0 || r >= capacity_ || r == INVALID_PHYSREG) return;
        free_.push_back(r);
    }

    // Restore the reset state: everything from p32 upward is free.
    void reset() {
        free_.clear();
        for (uint32_t r = 32; r < capacity_; ++r) free_.push_back(r);
    }

private:
    uint32_t             capacity_;
    std::deque<PhysReg>  free_;
};
