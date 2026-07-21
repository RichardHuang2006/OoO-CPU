// Machine configuration. Every structural parameter of the simulated core
// lives here; nothing downstream hardcodes a size.
#pragma once
#include <cstdint>
#include <string>

struct Config {
    // ---- pipeline widths -------------------------------------------------
    int fetch_width    = 2;
    int decode_width   = 2;
    int rename_width   = 2;
    int dispatch_width = 2;
    int issue_width    = 2;
    int commit_width   = 2;

    // ---- inter-stage latch depths (in uops) ------------------------------
    int fetch_queue    = 8;   // fetch  -> decode
    int decode_queue   = 8;   // decode -> rename
    int dispatch_queue = 8;   // rename -> dispatch

    // ---- out-of-order structures ----------------------------------------
    int rob_entries = 32;
    int phys_regs   = 64;     // must be >= rob_entries + 32
    int iq_entries  = 16;
    int lq_entries  = 8;
    int sq_entries  = 8;
    int num_cdbs    = 2;      // writeback ports

    // ---- function units --------------------------------------------------
    int num_alu    = 2;
    int num_mul    = 1;
    int num_mem    = 1;
    int num_branch = 1;

    int alu_latency = 1;      // back-to-back dependent ALU ops
    int mul_latency = 3;      // pipelined
    int div_latency = 20;     // not pipelined, blocks its unit
    int mem_latency = 2;      // load use-latency (AGU + array access)

    // ---- branch prediction ----------------------------------------------
    int ghr_bits        = 12;
    int pht_entries     = 4096;
    int btb_sets        = 128; // 128 sets x 4 ways = 512 entries
    int btb_ways        = 4;
    int ras_entries     = 16;
    int max_checkpoints = 16;  // in-flight branch RAT snapshots

    // ---- simulation ------------------------------------------------------
    uint64_t max_cycles = 100000000;
    bool trace          = false;

    // PRF = ROB + 32 is the sizing rule that makes physical register
    // starvation impossible (every in-flight instruction can hold one new
    // mapping on top of the 32 committed ones). Smaller values are legal and
    // simply stall rename -- that is what makes --prf worth sweeping -- but
    // fewer than 33 registers cannot make forward progress at all.
    void normalize() {
        if (phys_regs < 33) phys_regs = 33;
    }

    bool prf_can_starve() const { return phys_regs < rob_entries + 32; }
};
