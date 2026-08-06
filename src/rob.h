#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include "config.h"
#include "types.h"

// What an in-flight instruction needs at commit time. The value it produced
// lives in the physical register file, not here.
struct RobEntry {
    SeqNum   seq          = INVALID_SEQNUM;
    uint32_t pc           = 0;
    uint32_t next_pc      = 0;                 // where the front end should go after it
    ArchReg  dest_arch    = INVALID_ARCHREG;   // invalid if it writes no register
    PhysReg  dest_phys    = INVALID_PHYSREG;
    PhysReg  stale_phys   = INVALID_PHYSREG;   // returned to the free list at commit
    bool     is_branch    = false;
    bool     is_store     = false;             // its store queue entry drains at commit
    bool     complete     = false;             // set by writeback; commit waits on it
    bool     mispredicted = false;
};

// Circular buffer of in-flight instructions, oldest at the head. Entries are
// allocated in program order and popped from the head, so the buffer is what
// program order means once execution stops following it.
//
// Sequence numbers are stamped here and never reused, including after a
// squash, so comparing them is always a valid "older than" test.
class Rob {
public:
    explicit Rob(uint32_t capacity) : slots_(capacity) {}
    explicit Rob(const Config& cfg) : Rob(cfg.rob_size) {}

    uint32_t capacity() const { return static_cast<uint32_t>(slots_.size()); }
    uint32_t size()     const { return count_; }
    bool     empty()    const { return count_ == 0; }
    bool     full()     const { return count_ == capacity(); }
    SeqNum   next_seq() const { return next_seq_; }

    // Oldest entry, and the slot the next allocation takes. tail() == head()
    // both when empty and when full; size() tells the two apart.
    RobIndex head() const { return empty() ? INVALID_ROBINDEX : head_; }
    RobIndex tail() const { return wrap(head_ + count_); }

    // Slot of the k-th oldest entry, k == 0 being the head.
    RobIndex nth(uint32_t k) const {
        return k < count_ ? wrap(head_ + k) : INVALID_ROBINDEX;
    }

    // The k-th oldest entry itself, for callers walking the live range. Takes
    // the age rather than a slot, so no sentinel can reach the storage.
    RobEntry&       nth_entry(uint32_t k)       { return slots_[wrap(head_ + k)]; }
    const RobEntry& nth_entry(uint32_t k) const { return slots_[wrap(head_ + k)]; }

    // Distance from the head, or an out-of-range value if `idx` is not live.
    uint32_t age_of(RobIndex idx) const {
        if (idx >= capacity()) return capacity();
        return wrap(idx + capacity() - head_);
    }
    bool in_flight(RobIndex idx) const { return age_of(idx) < count_; }

    // Append in program order, stamping the sequence number. Caller checks
    // full() first.
    RobIndex allocate(RobEntry e) {
        e.seq = next_seq_++;
        const RobIndex idx = tail();
        slots_[idx] = e;
        ++count_;
        return idx;
    }

    RobEntry&       at(RobIndex idx)       { return slots_[idx]; }
    const RobEntry& at(RobIndex idx) const { return slots_[idx]; }

    // Retire the oldest entry, once writeback has marked it complete.
    std::optional<RobEntry> pop_head_if_complete() {
        if (empty() || !slots_[head_].complete) return std::nullopt;
        const RobEntry e = slots_[head_];
        head_ = wrap(head_ + 1);
        --count_;
        return e;
    }

    // Squash everything younger than `idx`, which survives — the shape of
    // recovery from a mispredicted branch. Returns the discarded entries
    // youngest first, the order their physical registers must be freed in.
    // An index that is not live squashes nothing.
    std::vector<RobEntry> truncate_to(RobIndex idx) {
        if (!in_flight(idx)) return {};
        return drop_to(age_of(idx) + 1);
    }

    // Squash every entry, youngest first. Used when nothing survives.
    std::vector<RobEntry> squash_all() { return drop_to(0); }

private:
    uint32_t wrap(uint32_t v) const { return v % capacity(); }

    // Shrink to `keep` oldest entries, returning the rest youngest first.
    std::vector<RobEntry> drop_to(uint32_t keep) {
        std::vector<RobEntry> squashed;
        squashed.reserve(count_ - keep);
        while (count_ > keep) {
            --count_;
            squashed.push_back(slots_[wrap(head_ + count_)]);
        }
        return squashed;
    }

    std::vector<RobEntry> slots_;
    RobIndex head_     = 0;
    uint32_t count_    = 0;
    SeqNum   next_seq_ = 0;
};
