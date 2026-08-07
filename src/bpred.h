#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include "config.h"
#include "types.h"

// What the target predictor has to tell apart. Direction comes from history
// for a conditional branch, from the return stack for a return, and is simply
// "taken" for everything else.
enum class BranchKind : uint8_t {
    NONE,
    CONDITIONAL,
    JUMP,          // unconditional, target in the BTB
    CALL,          // pushes a return address
    RETURN,        // pops one
};

// gshare: a table of two-bit saturating counters indexed by the global history
// XORed with the PC. The XOR is the whole idea — the same branch under
// different histories lands in different counters, so a branch that is only
// predictable in context becomes predictable.
class Gshare {
public:
    Gshare(uint32_t ghr_bits, uint32_t pht_size)
        : pht_(pht_size ? pht_size : 1, 1),      // weakly not taken
          ghr_mask_(ghr_bits >= 32 ? ~0u : (1u << ghr_bits) - 1u) {}
    explicit Gshare(const Config& cfg) : Gshare(cfg.ghr_bits, cfg.pht_size) {}

    uint32_t ghr()  const { return ghr_; }
    uint32_t size() const { return static_cast<uint32_t>(pht_.size()); }

    void set_ghr(uint32_t g) { ghr_ = g & ghr_mask_; }
    void shift(bool taken) { ghr_ = ((ghr_ << 1) | (taken ? 1u : 0u)) & ghr_mask_; }

    uint32_t index(uint32_t pc) const {
        return (ghr_ ^ (pc >> 2)) & (static_cast<uint32_t>(pht_.size()) - 1u);
    }

    // Top bit of the counter, which is the prediction.
    bool predict_at(uint32_t idx) const { return pht_[idx] >= 2; }

    // Trained at commit, from the index the prediction actually used, so a
    // wrong-path outcome can never reach the table.
    void update(uint32_t idx, bool taken) {
        uint8_t& c = pht_[idx];
        if (taken) { if (c < 3) ++c; }
        else       { if (c > 0) --c; }
    }

    uint8_t counter(uint32_t idx) const { return pht_[idx]; }

private:
    std::vector<uint8_t> pht_;
    uint32_t             ghr_      = 0;
    uint32_t             ghr_mask_ = 0;
};

// PC-tagged, set-associative, least-recently-used. A miss means the front end
// has never seen this PC take a branch, so it falls through to the next
// instruction — which is what hardware does, and the source of a whole class
// of bug if a model pretends otherwise.
class Btb {
public:
    struct Hit {
        bool       valid  = false;
        uint32_t   target = 0;
        BranchKind kind   = BranchKind::NONE;
    };

    Btb(uint32_t sets, uint32_t ways)
        : sets_(sets ? sets : 1), ways_(ways ? ways : 1),
          entries_(static_cast<std::size_t>(sets_) * ways_) {}
    explicit Btb(const Config& cfg) : Btb(cfg.btb_sets, cfg.btb_ways) {}

    uint32_t sets() const { return sets_; }
    uint32_t ways() const { return ways_; }

    Hit lookup(uint32_t pc) const {
        const uint32_t base = set_of(pc) * ways_;
        for (uint32_t w = 0; w < ways_; ++w) {
            const Entry& e = entries_[base + w];
            if (e.valid && e.tag == pc) return {true, e.target, e.kind};
        }
        return {};
    }

    // Installed at commit only: a wrong-path target must never be cached.
    void update(uint32_t pc, uint32_t target, BranchKind kind) {
        const uint32_t base = set_of(pc) * ways_;
        uint32_t victim = 0;
        uint64_t oldest = UINT64_MAX;
        for (uint32_t w = 0; w < ways_; ++w) {
            Entry& e = entries_[base + w];
            if (e.valid && e.tag == pc) { fill(e, pc, target, kind); return; }
            if (!e.valid) { fill(entries_[base + w], pc, target, kind); return; }
            if (e.used < oldest) { oldest = e.used; victim = w; }
        }
        fill(entries_[base + victim], pc, target, kind);
    }

private:
    struct Entry {
        bool       valid  = false;
        uint32_t   tag    = 0;
        uint32_t   target = 0;
        BranchKind kind   = BranchKind::NONE;
        uint64_t   used   = 0;
    };

    uint32_t set_of(uint32_t pc) const { return (pc >> 2) % sets_; }
    void fill(Entry& e, uint32_t pc, uint32_t target, BranchKind kind) {
        e = Entry{true, pc, target, kind, ++clock_};
    }

    uint32_t           sets_;
    uint32_t           ways_;
    std::vector<Entry> entries_;
    uint64_t           clock_ = 0;      // LRU ordering, not cycles
};

// Return address stack. Fixed storage and trivially copyable, because every
// fetch has to be able to snapshot it cheaply for a possible recovery.
class Ras {
public:
    static constexpr uint32_t MAX_ENTRIES = 32;

    explicit Ras(uint32_t size)
        : size_(size == 0 ? 1 : (size > MAX_ENTRIES ? MAX_ENTRIES : size)) {}
    explicit Ras(const Config& cfg) : Ras(cfg.ras_size) {}

    uint32_t capacity() const { return size_; }
    uint32_t depth()    const { return count_; }
    bool     empty()    const { return count_ == 0; }

    // Overwrites the deepest entry once full, which is what costs a very deep
    // recursion its outermost return predictions and nothing else.
    void push(uint32_t addr) {
        top_ = (top_ + 1) % size_;
        stack_[top_] = addr;
        if (count_ < size_) ++count_;
    }

    // Empty is a real state, not an error: the prediction just falls back.
    uint32_t pop() {
        if (count_ == 0) return 0;
        const uint32_t addr = stack_[top_];
        top_ = (top_ + size_ - 1) % size_;
        --count_;
        return addr;
    }

    uint32_t peek() const { return count_ == 0 ? 0 : stack_[top_]; }

private:
    std::array<uint32_t, MAX_ENTRIES> stack_{};
    uint32_t size_;
    uint32_t top_   = 0;
    uint32_t count_ = 0;
};

// The three structures behind one fetch-time question: where do I go next?
//
// History and the return stack move speculatively, at fetch, because the very
// next prediction depends on them. The tables that learn — the counters and
// the target cache — are only written at commit, so nothing a wrong path did
// can train them.
class BranchPredictor {
public:
    struct Prediction {
        bool       taken     = false;
        uint32_t   target    = 0;
        bool       btb_hit   = false;
        BranchKind kind      = BranchKind::NONE;
        uint32_t   pht_index = 0;
        bool       from_ras  = false;
    };

    explicit BranchPredictor(const Config& cfg)
        : gshare_(cfg), btb_(cfg), ras_(cfg) {}

    Gshare&       gshare()       { return gshare_; }
    const Gshare& gshare() const { return gshare_; }
    Btb&          btb()          { return btb_; }
    const Btb&    btb()    const { return btb_; }
    const Ras&    ras()    const { return ras_; }

    uint32_t ghr() const { return gshare_.ghr(); }

    // Speculative state, as a pair a checkpoint can hold and restore.
    struct Snapshot {
        uint32_t ghr = 0;
        Ras      ras{1u};
    };
    Snapshot snapshot() const { return {gshare_.ghr(), ras_}; }
    void restore(const Snapshot& s) {
        gshare_.set_ghr(s.ghr);
        ras_ = s.ras;
    }
    void shift_history(bool taken) { gshare_.shift(taken); }

    // One PC in, the next PC out. Only a BTB hit can redirect the front end,
    // so a branch the machine has never committed is always predicted to fall
    // through and is corrected when it executes.
    Prediction predict(uint32_t pc) {
        Prediction p;
        p.pht_index = gshare_.index(pc);

        const Btb::Hit hit = btb_.lookup(pc);
        p.btb_hit = hit.valid;
        if (!hit.valid) return p;

        p.kind = hit.kind;
        if (hit.kind == BranchKind::CONDITIONAL) {
            p.taken = gshare_.predict_at(p.pht_index);
            gshare_.shift(p.taken);
            if (!p.taken) return p;
            p.target = hit.target;
            return p;
        }

        p.taken  = true;
        p.target = hit.target;
        if (hit.kind == BranchKind::CALL) {
            ras_.push(pc + 4);
        } else if (hit.kind == BranchKind::RETURN && !ras_.empty()) {
            p.target   = ras_.pop();
            p.from_ras = true;
        }
        return p;
    }

    // Everything the tables learn, applied in commit order.
    void commit(uint32_t pc, uint32_t pht_index, BranchKind kind,
                bool taken, uint32_t target) {
        if (kind == BranchKind::CONDITIONAL) gshare_.update(pht_index, taken);
        if (taken) btb_.update(pc, target, kind);
    }

private:
    Gshare gshare_;
    Btb    btb_;
    Ras    ras_;
};
