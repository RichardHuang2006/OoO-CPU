// Unified physical register file holding both committed and speculative
// values (explicit renaming / R10000 style -- no data in the ROB), plus the
// per-register ready bit consulted when renaming a consumer's sources.
#pragma once
#include <cstdint>
#include <vector>

class PRF {
public:
    explicit PRF(int n) : val_((size_t)n, 0), ready_((size_t)n, false) {
        // p0..p31 hold the initial architectural state and are ready at reset.
        for (int i = 0; i < 32 && i < n; i++) ready_[(size_t)i] = true;
    }

    uint32_t read(int p) const { return val_[(size_t)p]; }
    void write(int p, uint32_t v) { val_[(size_t)p] = v; ready_[(size_t)p] = true; }

    bool ready(int p) const { return ready_[(size_t)p]; }
    void set_ready(int p, bool r) { ready_[(size_t)p] = r; }

    // Called at allocation: the new mapping has no value yet.
    void clear(int p) { ready_[(size_t)p] = false; val_[(size_t)p] = 0; }

    size_t size() const { return val_.size(); }

private:
    std::vector<uint32_t> val_;
    std::vector<bool> ready_;
};
