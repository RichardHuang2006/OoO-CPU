// Register alias table: 32 architectural registers -> physical registers.
// The speculative RAT is checkpointed per in-flight branch; a separate
// architectural RAT tracks committed state for the final register dump.
#pragma once
#include <array>
#include <cstdint>
#include <vector>

class RAT {
public:
    RAT() { for (int i = 0; i < 32; i++) map_[i] = i; }

    int  lookup(int areg) const { return map_[areg]; }
    void set(int areg, int preg) { if (areg != 0) map_[areg] = preg; }

    const std::array<int, 32>& raw() const { return map_; }
    void restore(const std::array<int, 32>& snap) { map_ = snap; }

private:
    std::array<int, 32> map_{};
};

// Pool of branch checkpoints. Renaming a branch takes one; committing or
// squashing the branch returns it. Exhausting the pool stalls rename.
struct Checkpoint {
    std::array<int, 32> rat{};
    uint32_t ghr = 0;
    std::vector<uint32_t> ras;
    int ras_top = 0, ras_depth = 0;
    uint64_t seq = 0;
    bool in_use = false;
};

class CheckpointPool {
public:
    explicit CheckpointPool(int n) : slots_((size_t)n) {}

    bool available() const {
        for (const auto& c : slots_) if (!c.in_use) return true;
        return false;
    }

    int alloc() {
        for (size_t i = 0; i < slots_.size(); i++)
            if (!slots_[i].in_use) { slots_[i].in_use = true; return (int)i; }
        return -1;
    }

    void free(int id) { if (id >= 0) slots_[(size_t)id].in_use = false; }

    void free_all() { for (auto& c : slots_) c.in_use = false; }

    Checkpoint& at(int id) { return slots_[(size_t)id]; }

    // Release every checkpoint younger than `seq` (they belong to squashed ops).
    void free_younger_than(uint64_t seq) {
        for (auto& c : slots_)
            if (c.in_use && c.seq > seq) c.in_use = false;
    }

private:
    std::vector<Checkpoint> slots_;
};
