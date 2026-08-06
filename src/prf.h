#pragma once

#include <algorithm>
#include <cstdint>
#include <vector>

#include "config.h"
#include "types.h"

// Unified physical register file: one value and one ready bit per register.
// The ready bit is what the issue queue subscribes to; the value is what
// execute reads after select. p0 is hard-wired to zero and always ready, so
// callers never guard rd == x0 before writing.

class Prf {
public:
    explicit Prf(uint32_t capacity)
        : values_(capacity, 0), ready_(capacity, 1) {}
    explicit Prf(const Config& cfg) : Prf(cfg.prf_size) {}

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

    // Flips ready off after a fresh allocation. p0 stays ready; no in-flight
    // uop is ever allowed to depend on x0 producing a value.
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
