#pragma once

#include <cstdint>
#include <vector>

#include "config.h"
#include "types.h"

// Non-data-capture issue queue: entries hold physical register tags and ready
// bits, never values. Operands are read from the PRF at select, so a waiting
// entry costs two tags instead of two 32-bit words and a wakeup is a tag
// compare rather than a value copy.
//
// Entries are kept in dispatch order, which is program order, so "oldest
// ready" is the first match in a forward scan.

class IssueQueue {
public:
    struct Entry {
        SeqNum   seq        = INVALID_SEQNUM;
        RobIndex rob        = INVALID_ROBINDEX;
        OpKind   kind       = OpKind::ALU;
        PhysReg  src1       = 0;
        PhysReg  src2       = 0;
        PhysReg  dest       = INVALID_PHYSREG;
        bool     src1_ready = true;
        bool     src2_ready = true;
        uint32_t latency    = 1;

        bool ready() const { return src1_ready && src2_ready; }
    };

    explicit IssueQueue(uint32_t capacity) : capacity_(capacity) {}
    explicit IssueQueue(const Config& cfg) : IssueQueue(cfg.iq_size) {}

    uint32_t capacity() const { return capacity_; }
    uint32_t size()     const { return static_cast<uint32_t>(entries_.size()); }
    bool     empty()    const { return entries_.empty(); }
    bool     full()     const { return size() >= capacity_; }

    const std::vector<Entry>& entries() const { return entries_; }

    // Append a dispatched op. The caller checks full() first.
    void insert(const Entry& e) { entries_.push_back(e); }

    // A result was broadcast: every entry waiting on that tag is now ready.
    void wakeup(PhysReg p) {
        if (p == INVALID_PHYSREG) return;
        for (Entry& e : entries_) {
            if (!e.src1_ready && e.src1 == p) e.src1_ready = true;
            if (!e.src2_ready && e.src2 == p) e.src2_ready = true;
        }
    }

    // Ready entries, oldest first, at most `limit`. Whether one of them can
    // actually go depends on function units and writeback ports, which the
    // caller owns; passing size() gives it the whole ready set to choose from.
    std::vector<Entry> select(uint32_t limit) const {
        std::vector<Entry> out;
        for (const Entry& e : entries_) {
            if (out.size() >= limit) break;
            if (e.ready()) out.push_back(e);
        }
        return out;
    }

    // Drop an entry that has issued.
    void erase(SeqNum seq) {
        for (std::size_t i = 0; i < entries_.size(); ++i) {
            if (entries_[i].seq == seq) {
                entries_.erase(entries_.begin() + static_cast<std::ptrdiff_t>(i));
                return;
            }
        }
    }

    // Recovery: everything younger than the surviving branch goes.
    void squash_after(SeqNum seq) {
        std::vector<Entry> kept;
        kept.reserve(entries_.size());
        for (const Entry& e : entries_) {
            if (e.seq <= seq) kept.push_back(e);
        }
        entries_.swap(kept);
    }

    void clear() { entries_.clear(); }

private:
    std::vector<Entry> entries_;
    uint32_t           capacity_;
};
