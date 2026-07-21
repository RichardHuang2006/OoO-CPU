// gshare direction predictor + set-associative BTB + return address stack.
// The GHR is updated speculatively in Fetch and restored from a branch
// checkpoint on a misprediction.
#pragma once
#include <cstdint>
#include <vector>

class Gshare {
public:
    Gshare(int ghr_bits, int pht_entries)
        : ghr_mask_((ghr_bits >= 32) ? 0xffffffffu : ((1u << ghr_bits) - 1)),
          pht_((size_t)pht_entries, 1),          // weakly not-taken
          idx_mask_((uint32_t)pht_entries - 1) {}

    uint32_t index(uint32_t pc) const {
        // PC bits [1] and below are always zero for 32-bit instructions.
        return ((pc >> 2) ^ ghr_) & idx_mask_;
    }

    bool predict(uint32_t pc) const { return pht_[index(pc)] >= 2; }

    void update(uint32_t pc, uint32_t ghr_at_predict, bool taken) {
        uint32_t i = (((pc >> 2) ^ ghr_at_predict) & idx_mask_);
        uint8_t& c = pht_[i];
        if (taken) { if (c < 3) c++; }
        else       { if (c > 0) c--; }
    }

    uint32_t ghr() const { return ghr_; }
    void set_ghr(uint32_t g) { ghr_ = g & ghr_mask_; }
    void shift(bool taken) { ghr_ = ((ghr_ << 1) | (taken ? 1u : 0u)) & ghr_mask_; }

private:
    uint32_t ghr_ = 0;
    uint32_t ghr_mask_;
    std::vector<uint8_t> pht_;
    uint32_t idx_mask_;
};

class BTB {
public:
    struct Entry { bool valid = false; uint32_t tag = 0; uint32_t target = 0;
                   bool is_ret = false; uint32_t lru = 0; };

    BTB(int sets, int ways) : sets_(sets), ways_(ways), e_((size_t)(sets * ways)) {}

    bool lookup(uint32_t pc, uint32_t& target, bool& is_ret) const {
        int s = set_of(pc);
        for (int w = 0; w < ways_; w++) {
            const Entry& x = e_[(size_t)(s * ways_ + w)];
            if (x.valid && x.tag == pc) { target = x.target; is_ret = x.is_ret; return true; }
        }
        return false;
    }

    void update(uint32_t pc, uint32_t target, bool is_ret) {
        int s = set_of(pc);
        Entry* victim = nullptr;
        for (int w = 0; w < ways_; w++) {
            Entry& x = e_[(size_t)(s * ways_ + w)];
            if (x.valid && x.tag == pc) { victim = &x; break; }
            if (!x.valid) { victim = &x; break; }
        }
        if (!victim) {  // evict least recently used way
            victim = &e_[(size_t)(s * ways_)];
            for (int w = 1; w < ways_; w++) {
                Entry& x = e_[(size_t)(s * ways_ + w)];
                if (x.lru < victim->lru) victim = &x;
            }
        }
        victim->valid = true; victim->tag = pc;
        victim->target = target; victim->is_ret = is_ret;
        victim->lru = ++clock_;
    }

    void touch(uint32_t pc) {
        int s = set_of(pc);
        for (int w = 0; w < ways_; w++) {
            Entry& x = e_[(size_t)(s * ways_ + w)];
            if (x.valid && x.tag == pc) { x.lru = ++clock_; return; }
        }
    }

private:
    int set_of(uint32_t pc) const { return (int)((pc >> 2) % (uint32_t)sets_); }
    int sets_, ways_;
    std::vector<Entry> e_;
    uint32_t clock_ = 0;
};

class RAS {
public:
    explicit RAS(int n) : s_((size_t)n, 0), n_(n) {}

    void push(uint32_t addr) {
        top_ = (top_ + 1) % n_;
        s_[(size_t)top_] = addr;
        if (depth_ < n_) depth_++;
    }

    uint32_t peek() const { return s_[(size_t)top_]; }

    uint32_t pop() {
        uint32_t a = s_[(size_t)top_];
        top_ = (top_ - 1 + n_) % n_;
        if (depth_ > 0) depth_--;
        return a;
    }

    bool valid() const { return depth_ > 0; }

    // Checkpoint / restore of the whole stack (small enough to copy).
    std::vector<uint32_t> snapshot() const { return s_; }
    int top() const { return top_; }
    int depth() const { return depth_; }
    void restore(const std::vector<uint32_t>& s, int top, int depth) {
        s_ = s; top_ = top; depth_ = depth;
    }

private:
    std::vector<uint32_t> s_;
    int n_;
    int top_ = 0;
    int depth_ = 0;
};
