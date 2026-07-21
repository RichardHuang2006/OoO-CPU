// Reorder buffer: circular queue giving in-order commit and a precise
// recovery point. Entries carry bookkeeping only -- values live in the PRF.
#pragma once
#include <cstdint>
#include <vector>
#include "types.h"

struct RobEntry {
    bool valid = false;
    bool done  = false;         // result written back
    uint64_t seq = 0;
    uint32_t pc  = 0;

    uint8_t arch_dst = 0;
    int pdst   = -1;            // new physical destination (-1 if none)
    int pstale = -1;            // previous mapping, reclaimed at commit

    bool is_branch = false, is_jump = false, is_load = false, is_store = false;
    bool is_system = false, illegal = false, is_ret = false;

    int lq_idx = -1, sq_idx = -1, chkpt = -1;

    // Branch bookkeeping, filled at rename and completed at writeback; the
    // predictors are updated non-speculatively at commit.
    bool     pred_taken = false, actual_taken = false, mispredicted = false;
    bool     used_ras = false;
    uint32_t actual_target = 0;
    uint32_t ghr_snapshot = 0;

    uint32_t npc = 0;           // architecturally correct next PC (for redirect)
    Op op = Op::ILLEGAL;
};

class ROB {
public:
    explicit ROB(int n) : e_((size_t)n), n_(n) {}

    bool full() const  { return count_ == n_; }
    bool empty() const { return count_ == 0; }
    int  size() const  { return count_; }
    int  capacity() const { return n_; }

    int head() const { return head_; }
    int tail() const { return (head_ + count_) % n_; }   // next free slot

    RobEntry& at(int i) { return e_[(size_t)i]; }
    const RobEntry& at(int i) const { return e_[(size_t)i]; }

    RobEntry& head_entry() { return e_[(size_t)head_]; }

    // Returns the allocated index, or -1 when full.
    int alloc() {
        if (full()) return -1;
        int idx = tail();
        e_[(size_t)idx] = RobEntry{};
        e_[(size_t)idx].valid = true;
        count_++;
        return idx;
    }

    void pop() {
        e_[(size_t)head_].valid = false;
        head_ = (head_ + 1) % n_;
        count_--;
    }

    // Number of entries strictly younger than `idx` (used by squash walks).
    int younger_count(int idx) const {
        int off = (idx - head_ + n_) % n_;   // age position of idx
        return count_ - off - 1;
    }

    int next(int idx) const { return (idx + 1) % n_; }

    // Drop everything younger than `idx`, keeping `idx` itself.
    void truncate_after(int idx) {
        int off = (idx - head_ + n_) % n_;
        for (int k = off + 1; k < count_; k++)
            e_[(size_t)((head_ + k) % n_)].valid = false;
        count_ = off + 1;
    }

    void clear() {
        for (auto& x : e_) x = RobEntry{};
        head_ = 0; count_ = 0;
    }

private:
    std::vector<RobEntry> e_;
    int n_;
    int head_ = 0;
    int count_ = 0;
};
