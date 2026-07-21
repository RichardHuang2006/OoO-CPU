// Load queue + store queue. Both are circular and allocated in program order
// at Dispatch, so "older than me" is a sequence-number comparison. Stores
// write memory only at commit; loads forward from the store queue.
#pragma once
#include <cstdint>
#include <vector>
#include "memory.h"

struct SQEntry {
    bool valid = false;
    uint64_t seq = 0;
    int rob_idx = -1;
    uint32_t addr = 0;
    bool addr_valid = false;
    uint32_t data = 0;
    bool data_valid = false;
    int bytes = 0;
};

struct LQEntry {
    bool valid = false;
    uint64_t seq = 0;
    int rob_idx = -1;
    uint32_t addr = 0;
    bool addr_valid = false;
    int bytes = 0;
};

enum class FwdResult { FromMemory, Forwarded, Wait };

template <typename E>
class CircQueue {
public:
    explicit CircQueue(int n) : e_((size_t)n), n_(n) {}

    bool full() const { return count_ == n_; }
    bool empty() const { return count_ == 0; }
    int  size() const { return count_; }

    int alloc() {
        if (full()) return -1;
        int idx = (head_ + count_) % n_;
        e_[(size_t)idx] = E{};
        e_[(size_t)idx].valid = true;
        count_++;
        return idx;
    }

    E& at(int i) { return e_[(size_t)i]; }
    const E& at(int i) const { return e_[(size_t)i]; }

    E& head_entry() { return e_[(size_t)head_]; }
    int head() const { return head_; }

    void pop() {
        e_[(size_t)head_].valid = false;
        head_ = (head_ + 1) % n_;
        count_--;
    }

    // Youngest-first removal of wrong-path entries.
    void squash_after(uint64_t seq) {
        while (count_ > 0) {
            int last = (head_ + count_ - 1) % n_;
            if (e_[(size_t)last].seq <= seq) break;
            e_[(size_t)last].valid = false;
            count_--;
        }
    }

    void clear() { for (auto& x : e_) x = E{}; head_ = 0; count_ = 0; }

    int capacity() const { return n_; }

    // Iterate oldest -> youngest.
    template <typename F>
    void for_each(F f) {
        for (int k = 0; k < count_; k++) f(e_[(size_t)((head_ + k) % n_)]);
    }
    template <typename F>
    void for_each(F f) const {
        for (int k = 0; k < count_; k++) f(e_[(size_t)((head_ + k) % n_)]);
    }

private:
    std::vector<E> e_;
    int n_;
    int head_ = 0;
    int count_ = 0;
};

class LSQ {
public:
    LSQ(int lq, int sq) : lq_(lq), sq_(sq) {}

    CircQueue<LQEntry>& lq() { return lq_; }
    CircQueue<SQEntry>& sq() { return sq_; }

    // Resolve a load against older stores. `out` is the raw (unextended) data.
    FwdResult disambiguate(uint64_t load_seq, uint32_t addr, int bytes,
                           const Memory& mem, uint32_t& out) const {
        const SQEntry* match = nullptr;
        bool unresolved_older = false;

        // Walk oldest -> youngest so the newest overlapping store wins.
        sq_.for_each([&](const SQEntry& s) {
            if (!s.valid || s.seq >= load_seq) return;

            if (!s.addr_valid) { unresolved_older = true; return; }

            const bool overlap = (s.addr < addr + (uint32_t)bytes) &&
                                 (addr < s.addr + (uint32_t)s.bytes);
            if (!overlap) return;

            // Only a store that fully covers the load can be forwarded.
            const bool covers = s.addr <= addr &&
                                (s.addr + (uint32_t)s.bytes) >= (addr + (uint32_t)bytes);
            if (!covers || !s.data_valid) { unresolved_older = true; return; }
            match = &s;
        });

        // A store whose address is still unknown may alias: replay the load.
        if (unresolved_older) return FwdResult::Wait;

        if (match) {
            const uint32_t shift = (addr - match->addr) * 8;
            uint32_t v = match->data >> shift;
            if (bytes < 4) v &= (1u << (bytes * 8)) - 1;
            out = v;
            return FwdResult::Forwarded;
        }

        out = mem.read(addr, bytes);
        return FwdResult::FromMemory;
    }

    void squash_after(uint64_t seq) { lq_.squash_after(seq); sq_.squash_after(seq); }
    void clear() { lq_.clear(); sq_.clear(); }

private:
    CircQueue<LQEntry> lq_;
    CircQueue<SQEntry> sq_;
};
