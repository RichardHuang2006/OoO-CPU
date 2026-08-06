#pragma once

#include <cstdint>
#include <type_traits>

// One field per structural knob. Downstream code reads sizes off a
// `const Config&` so a configuration sweep needs no recompilation.

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

    // PRF >= ROB + 32 makes register starvation impossible: every in-flight
    // instruction can hold one mapping on top of the 32 committed ones.
    // Smaller values are legal and just stall rename.
    bool prf_can_starve() const {
        return prf_size < rob_size + 32;
    }
};

// Configs get copied around wholesale; keep them memcpy-able.
static_assert(std::is_trivially_copyable_v<Config>);
