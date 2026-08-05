#pragma once

#include <cstdint>
#include <type_traits>

// All structural parameters live here as one field per knob. Downstream code
// takes a `const Config&` and reads sizes off it — no `constexpr` hardcoded
// dimensions anywhere else. This is what makes --width, --rob, etc. sweepable
// in Step 1.5 and what lets [§8.2](../DESIGN.md#82-configuration-sweep) run
// the same workload across six configurations without recompilation.

struct Config {
    // Pipeline width: fetch, decode, rename, dispatch, issue, commit each
    // process at most `width` uops per cycle.
    uint32_t width = 2;

    // Speculative structures.
    uint32_t rob_size = 32;
    uint32_t prf_size = 64;   // rob_size + 32 by the starvation-free rule below
    uint32_t iq_size  = 16;
    uint32_t lq_size  = 8;
    uint32_t sq_size  = 8;
    uint32_t num_cdb  = 2;    // writeback ports

    // Per-class function-unit counts.
    uint32_t num_alu    = 2;
    uint32_t num_branch = 1;
    uint32_t num_mul    = 1;
    uint32_t num_div    = 1;  // blocking; one op holds the unit for div_latency
    uint32_t num_mem    = 1;

    // Per-class execution latencies (cycles).
    uint32_t alu_latency    = 1;
    uint32_t branch_latency = 1;
    uint32_t mul_latency    = 3;    // pipelined
    uint32_t div_latency    = 20;   // blocking
    uint32_t mem_latency    = 2;    // load-use, flat memory model

    // Branch predictor: gshare direction, PC-tagged set-associative BTB, RAS.
    uint32_t ghr_bits        = 12;
    uint32_t pht_size        = 4096;
    uint32_t btb_sets        = 128;   // 128 sets × 4 ways = 512 BTB entries
    uint32_t btb_ways        = 4;
    uint32_t ras_size        = 16;
    uint32_t num_checkpoints = 16;    // max in-flight branches with a snapshot

    // DESIGN.md §3.2: PRF ≥ ROB + 32 makes register starvation impossible —
    // every in-flight instruction can hold one new mapping on top of the 32
    // committed ones. Smaller values are legal and simply stall Rename, which
    // is the point of sweeping --prf during characterization.
    bool prf_can_starve() const {
        return prf_size < rob_size + 32;
    }
};

// A snapshot of Config must be bit-copyable — Step 7.4's branch checkpoints
// don't touch this, but tooling that swaps configs across a run relies on it.
static_assert(std::is_trivially_copyable_v<Config>);
