// Non-data-capture issue queue: entries hold physical source *tags* and ready
// bits, never values. Operands are read from the PRF after select.
//
// Wakeup and select happen in one atomic cycle (see Cpu::issue_stage): tags
// broadcast by producers selected in cycle T clear ready bits at the start of
// cycle T+1, so a dependent 1-cycle ALU op is selected in T+1 and executes
// back-to-back with its producer.
#pragma once
#include <algorithm>
#include <cstdint>
#include <vector>
#include "types.h"

struct IQEntry {
    bool valid = false;
    Uop  uop;
    bool r1 = true, r2 = true;   // source ready bits
    bool ready() const { return r1 && r2; }
};

class IssueQueue {
public:
    explicit IssueQueue(int n) : e_((size_t)n), n_(n) {}

    bool full() const { return count_ == n_; }
    int  size() const { return count_; }
    int  capacity() const { return n_; }

    int insert(const Uop& u, bool r1, bool r2) {
        for (int i = 0; i < n_; i++) {
            if (e_[(size_t)i].valid) continue;
            e_[(size_t)i] = IQEntry{true, u, r1, r2};
            count_++;
            return i;
        }
        return -1;
    }

    // Broadcast one physical tag; every waiting consumer marks it ready.
    void wakeup(int ptag) {
        if (ptag < 0) return;
        for (auto& x : e_) {
            if (!x.valid) continue;
            if (x.uop.d.use_rs1 && x.uop.psrc1 == ptag) x.r1 = true;
            if (x.uop.d.use_rs2 && x.uop.psrc2 == ptag) x.r2 = true;
        }
    }

    IQEntry& at(int i) { return e_[(size_t)i]; }
    const IQEntry& at(int i) const { return e_[(size_t)i]; }

    void remove(int i) {
        if (!e_[(size_t)i].valid) return;
        e_[(size_t)i].valid = false;
        count_--;
    }

    // Age-ordered arbitration: indices of ready entries, oldest first.
    std::vector<int> ready_oldest_first() const {
        std::vector<int> r;
        r.reserve((size_t)count_);
        for (int i = 0; i < n_; i++)
            if (e_[(size_t)i].valid && e_[(size_t)i].ready()) r.push_back(i);
        std::sort(r.begin(), r.end(), [&](int a, int b) {
            return e_[(size_t)a].uop.seq < e_[(size_t)b].uop.seq;
        });
        return r;
    }

    void squash_after(uint64_t seq) {
        for (int i = 0; i < n_; i++) {
            if (e_[(size_t)i].valid && e_[(size_t)i].uop.seq > seq) {
                e_[(size_t)i].valid = false;
                count_--;
            }
        }
    }

    void clear() { for (auto& x : e_) x.valid = false; count_ = 0; }

private:
    std::vector<IQEntry> e_;
    int n_;
    int count_ = 0;
};
