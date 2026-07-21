#pragma once
#include <cstdint>

struct Stats {
    uint64_t cycles = 0;

    uint64_t fetched = 0, decoded = 0, renamed = 0, dispatched = 0;
    uint64_t issued = 0, committed = 0, squashed = 0;

    // Control flow
    uint64_t branches = 0;        // conditional branches committed
    uint64_t jumps = 0;           // JAL/JALR committed
    uint64_t mispredicts = 0;     // all control mispredicts
    uint64_t cond_mispredicts = 0;
    uint64_t btb_misses = 0;
    uint64_t ras_predictions = 0, ras_correct = 0;

    // Memory
    uint64_t loads = 0, stores = 0;
    uint64_t store_forwards = 0, load_replays = 0;

    // Stall causes, counted in instruction-slots lost per cycle
    uint64_t stall_fetch_queue = 0;  // fetch blocked, decode queue backed up
    uint64_t stall_decode_queue = 0;
    uint64_t stall_rob_full = 0;
    uint64_t stall_no_phys = 0;      // physical register starvation
    uint64_t stall_no_chkpt = 0;     // branch checkpoint starvation
    uint64_t stall_dispatch_queue = 0;
    uint64_t stall_iq_full = 0;
    uint64_t stall_lq_full = 0;
    uint64_t stall_sq_full = 0;
    uint64_t stall_fu_busy = 0;      // select lost to a structural hazard
    uint64_t stall_cdb = 0;          // select or writeback lost a CDB

    // Commit-side blockage: head of ROB not done
    uint64_t commit_blocked_cycles = 0;
    uint64_t idle_fetch_cycles = 0;
};
