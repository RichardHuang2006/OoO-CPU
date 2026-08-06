#pragma once

#include <cstdint>
#include <deque>
#include <optional>

#include "config.h"
#include "types.h"

// Physical registers not currently mapped to an architectural register and
// not sitting in the ROB as a stale mapping. FIFO order is arbitrary but
// keeps traces deterministic and spreads reuse out in time.
//
// p0 is reserved for x0 forever; p1..p31 hold the reset RAT; p32 upward
// start free. free() silently ignores p0, INVALID_PHYSREG, and out-of-range
// indices so commit-time reclamation can be a single unconditional call.

class FreeList {
public:
    explicit FreeList(uint32_t prf_size) : capacity_(prf_size) {
        reset();
    }
    explicit FreeList(const Config& cfg) : FreeList(cfg.prf_size) {}

    uint32_t capacity() const { return capacity_; }
    uint32_t num_free() const { return static_cast<uint32_t>(free_.size()); }
    bool     empty()    const { return free_.empty(); }

    // Pop the next free register, nullopt when starved. Callers stall rename
    // on nullopt.
    std::optional<PhysReg> alloc() {
        if (free_.empty()) return std::nullopt;
        const PhysReg r = free_.front();
        free_.pop_front();
        return r;
    }

    // Push a register back. See header comment for the ignored values.
    void free(PhysReg r) {
        if (r == 0 || r >= capacity_ || r == INVALID_PHYSREG) return;
        free_.push_back(r);
    }

    void reset() {
        free_.clear();
        for (uint32_t r = 32; r < capacity_; ++r) free_.push_back(r);
    }

private:
    uint32_t             capacity_;
    std::deque<PhysReg>  free_;
};
