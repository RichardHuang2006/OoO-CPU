#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include "config.h"
#include "types.h"

// Load and store queues. Entries are allocated at dispatch, in program order,
// so "older than me" is a comparison and never a search.
//
// A store does not touch memory until it commits, which makes the store queue
// a speculative write buffer: memory holds the committed past, the queue holds
// everything a younger load might still need to see. That is why a load
// consults the queue first and only falls through to memory when no older
// store can possibly cover it.

struct LsqEntry {
    SeqNum   seq        = INVALID_SEQNUM;
    uint32_t addr       = 0;
    uint32_t data       = 0;    // store data, in the low `size` bytes
    uint8_t  size       = 4;    // 1, 2 or 4
    bool     addr_ready = false;
    bool     data_ready = false;
};

// What a load learned by searching the older stores.
enum class Forward : uint8_t {
    NO_MATCH,   // nothing older can cover these bytes; go to memory
    FORWARD,    // an older store has them, and they are known
    REPLAY,     // an older store might have them; ask again next cycle
};

struct ForwardResult {
    Forward  kind = Forward::NO_MATCH;
    uint32_t data = 0;
};

// A circular queue of in-flight accesses, filled at dispatch and drained in
// order at commit.
class LsqQueue {
public:
    explicit LsqQueue(uint32_t capacity)
        : slots_(capacity == 0 ? 1 : capacity), capacity_(capacity) {}

    uint32_t capacity() const { return capacity_; }
    uint32_t size()     const { return count_; }
    bool     empty()    const { return count_ == 0; }
    bool     full()     const { return count_ >= capacity_; }

    std::optional<uint32_t> alloc(SeqNum seq, uint8_t size) {
        if (full()) return std::nullopt;
        const uint32_t idx = wrap(head_ + count_);
        slots_[idx] = LsqEntry{seq, 0, 0, size, false, false};
        ++count_;
        return idx;
    }

    LsqEntry&       at(uint32_t idx)       { return slots_[idx]; }
    const LsqEntry& at(uint32_t idx) const { return slots_[idx]; }

    // The k-th oldest entry, k = 0 being the head.
    LsqEntry&       nth(uint32_t k)       { return slots_[wrap(head_ + k)]; }
    const LsqEntry& nth(uint32_t k) const { return slots_[wrap(head_ + k)]; }

    void pop_head() {
        if (count_ == 0) return;
        head_ = wrap(head_ + 1);
        --count_;
    }

    // Recovery: entries younger than the surviving branch never happened.
    void squash_after(SeqNum seq) {
        while (count_ > 0 && nth(count_ - 1).seq > seq) --count_;
    }

    void clear() { head_ = 0; count_ = 0; }

private:
    uint32_t wrap(uint32_t v) const {
        return v % static_cast<uint32_t>(slots_.size());
    }

    std::vector<LsqEntry> slots_;
    uint32_t capacity_ = 0;
    uint32_t head_     = 0;
    uint32_t count_    = 0;
};

class Lsq {
public:
    explicit Lsq(uint32_t lq_size, uint32_t sq_size) : lq_(lq_size), sq_(sq_size) {}
    explicit Lsq(const Config& cfg) : Lsq(cfg.lq_size, cfg.sq_size) {}

    LsqQueue&       loads()        { return lq_; }
    const LsqQueue& loads()  const { return lq_; }
    LsqQueue&       stores()       { return sq_; }
    const LsqQueue& stores() const { return sq_; }

    std::optional<uint32_t> alloc_load (SeqNum seq, uint8_t size) { return lq_.alloc(seq, size); }
    std::optional<uint32_t> alloc_store(SeqNum seq, uint8_t size) { return sq_.alloc(seq, size); }

    void resolve_load_addr(uint32_t idx, uint32_t addr) {
        lq_.at(idx).addr = addr;
        lq_.at(idx).addr_ready = true;
    }
    void resolve_addr(uint32_t idx, uint32_t addr) {
        sq_.at(idx).addr = addr;
        sq_.at(idx).addr_ready = true;
    }
    void resolve_data(uint32_t idx, uint32_t data) {
        sq_.at(idx).data = data;
        sq_.at(idx).data_ready = true;
    }

    // Walk the older stores oldest to youngest. A store that fully covers the
    // load with known data settles the question and shadows everything before
    // it; anything that might cover it but cannot say so yet leaves the load
    // to ask again. Partial overlap is treated as unknown rather than stitched
    // together — rare enough that the simplicity is worth the replay.
    ForwardResult search_older_stores(uint32_t load_idx) const {
        const LsqEntry& ld = lq_.at(load_idx);
        ForwardResult out;
        if (!ld.addr_ready) {
            out.kind = Forward::REPLAY;
            return out;
        }

        const uint64_t lo = ld.addr;
        const uint64_t hi = lo + ld.size;
        bool ambiguous = false;

        for (uint32_t k = 0; k < sq_.size(); ++k) {
            const LsqEntry& st = sq_.nth(k);
            if (st.seq > ld.seq) break;            // younger; not this load's problem

            if (!st.addr_ready) { ambiguous = true; continue; }

            const uint64_t slo = st.addr;
            const uint64_t shi = slo + st.size;
            if (shi <= lo || slo >= hi) continue;  // disjoint

            const bool covers = slo <= lo && shi >= hi;
            if (covers && st.data_ready) {
                out.kind  = Forward::FORWARD;
                out.data  = extract(st, ld);
                ambiguous = false;                 // these bytes are settled
            } else {
                ambiguous = true;
                out.kind  = Forward::NO_MATCH;
            }
        }

        if (ambiguous) {
            out.kind = Forward::REPLAY;
            out.data = 0;
        }
        return out;
    }

    void squash_after(SeqNum seq) {
        lq_.squash_after(seq);
        sq_.squash_after(seq);
    }

    void clear() {
        lq_.clear();
        sq_.clear();
    }

private:
    // The load's bytes out of the store's word, zero-extended. Sign extension
    // belongs to the load itself, which is the only thing that knows its op.
    static uint32_t extract(const LsqEntry& st, const LsqEntry& ld) {
        const uint32_t off = ld.addr - st.addr;
        uint32_t v = st.data >> (8u * off);
        if (ld.size < 4) v &= (1u << (8u * ld.size)) - 1u;
        return v;
    }

    LsqQueue lq_;
    LsqQueue sq_;
};
