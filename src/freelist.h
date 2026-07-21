// FIFO of unmapped physical registers. Registers enter at commit (the stale
// mapping of the retiring instruction) or on a squash (the destination of a
// wrong-path instruction).
#pragma once
#include <cstddef>
#include <deque>

class FreeList {
public:
    // p0..p31 are the initial architectural mappings; the rest start free.
    FreeList(int phys_regs, int reserved = 32) {
        for (int p = reserved; p < phys_regs; p++) free_.push_back(p);
    }

    bool empty() const { return free_.empty(); }
    size_t size() const { return free_.size(); }

    int alloc() {
        if (free_.empty()) return -1;
        int p = free_.front();
        free_.pop_front();
        return p;
    }

    void free(int p) { if (p >= 0) free_.push_back(p); }

    // Squash recovery returns registers youngest-first; push them to the front
    // so allocation order stays stable and easy to follow in traces.
    void free_front(int p) { if (p >= 0) free_.push_front(p); }

private:
    std::deque<int> free_;
};
