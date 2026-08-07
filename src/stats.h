#pragma once

#include <array>
#include <cstdint>

// Why a uop that was otherwise ready did not move this cycle. Every stall the
// machine can suffer is one of these, so the breakdown accounts for the whole
// gap between issued and width.
enum class Stall : uint8_t {
    ROB_FULL,
    PHYSREG,        // free list empty
    CHECKPOINT,     // no snapshot slot for a branch
    IQ_FULL,
    LQ_FULL,
    SQ_FULL,
    ALU_PORT,
    BRANCH_PORT,
    MUL_PORT,
    DIV_PORT,
    MEM_PORT,
    CDB,            // no writeback port free at the cycle the result lands
    OPERANDS,       // waiting on a producer
    STORE_ORDER,    // a load held behind an older store
    COUNT,
};

inline const char* stall_name(Stall s) {
    switch (s) {
    case Stall::ROB_FULL:    return "rob_full";
    case Stall::PHYSREG:     return "physreg_starved";
    case Stall::CHECKPOINT:  return "checkpoint_starved";
    case Stall::IQ_FULL:     return "iq_full";
    case Stall::LQ_FULL:     return "lq_full";
    case Stall::SQ_FULL:     return "sq_full";
    case Stall::ALU_PORT:    return "alu_port_full";
    case Stall::BRANCH_PORT: return "branch_port_full";
    case Stall::MUL_PORT:    return "mul_port_full";
    case Stall::DIV_PORT:    return "div_port_full";
    case Stall::MEM_PORT:    return "mem_port_full";
    case Stall::CDB:         return "cdb_reserved_full";
    case Stall::OPERANDS:    return "operands_not_ready";
    case Stall::STORE_ORDER: return "store_order";
    case Stall::COUNT:       break;
    }
    return "none";
}

// Everything the run reports. Counters are plain fields because every one of
// them is written from exactly one place in the pipeline.
struct Stats {
    uint64_t cycles   = 0;
    uint64_t fetched  = 0;
    uint64_t decoded  = 0;
    uint64_t renamed  = 0;
    uint64_t dispatched = 0;
    uint64_t issued   = 0;
    uint64_t wrote_back = 0;
    uint64_t retired  = 0;
    uint64_t squashed = 0;      // uops killed by recovery, never retired

    uint64_t branches    = 0;
    uint64_t mispredicts = 0;
    uint64_t btb_lookups = 0;
    uint64_t btb_hits    = 0;
    uint64_t ras_pops    = 0;
    uint64_t ras_hits    = 0;

    uint64_t loads          = 0;
    uint64_t stores         = 0;
    uint64_t load_forwards  = 0;   // served from the store queue
    uint64_t load_replays   = 0;
    uint64_t load_memory    = 0;   // went to memory

    std::array<uint64_t, static_cast<std::size_t>(Stall::COUNT)> stalls{};

    void stall(Stall s) { ++stalls[static_cast<std::size_t>(s)]; }
    uint64_t stall_count(Stall s) const {
        return stalls[static_cast<std::size_t>(s)];
    }

    double ipc() const { return cycles ? static_cast<double>(retired) / cycles : 0.0; }
    double cpi() const { return retired ? static_cast<double>(cycles) / retired : 0.0; }

    double mispredict_rate() const {
        return branches ? static_cast<double>(mispredicts) / branches : 0.0;
    }
    // Mispredicts per thousand retired instructions.
    double mpki() const {
        return retired ? 1000.0 * static_cast<double>(mispredicts) / retired : 0.0;
    }
    double btb_hit_rate() const {
        return btb_lookups ? static_cast<double>(btb_hits) / btb_lookups : 0.0;
    }
    double ras_accuracy() const {
        return ras_pops ? static_cast<double>(ras_hits) / ras_pops : 0.0;
    }

    // The resource the machine ran out of most often. Waiting on a producer is
    // not a resource — it is the program — so it does not compete here, or it
    // would win on every workload and diagnose nothing.
    Stall dominant_stall() const {
        Stall best = Stall::COUNT;
        uint64_t most = 0;
        for (std::size_t i = 0; i < stalls.size(); ++i) {
            if (static_cast<Stall>(i) == Stall::OPERANDS) continue;
            if (stalls[i] > most) {
                most = stalls[i];
                best = static_cast<Stall>(i);
            }
        }
        return best;
    }
};
