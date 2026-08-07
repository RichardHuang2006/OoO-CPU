// Top-level driver for Mini-CPU.
//
// `int main()` is guarded so tests can #include this file and drive
// parse_args / print_help directly. Everything else is inline or file-static,
// so including it twice does not violate ODR.
//
// Programs run on the out-of-order model by default; --ref runs the in-order
// interpreter the model is validated against.

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "config.h"
#include "cpu.h"
#include "loader.h"
#include "memory.h"
#include "ref.h"
#include "stats.h"

// ============================================================================
// CLI surface
// ============================================================================

struct CliOpts {
    std::string hex_path;
    std::string raw_path;
    std::string elf_path;
    uint32_t    base = 0;
    bool        has_base = false;
    bool        print_regs = false;
    bool        trace = false;
    bool        show_help = false;
    bool        use_ref = false;      // interpreter instead of the pipeline
    bool        show_stats = false;
    bool        ipc_table = false;    // same program, four machines
    uint64_t    max_insts = 100'000'000;
    uint64_t    max_cycles = 1'000'000'000;
    Config      cfg;
};

// One entry per Config knob. The pointer-to-member lets the parser assign to
// any field uniformly.
struct ConfigKnob {
    const char*         flag;
    uint32_t Config::*  member;
    const char*         desc;
};

inline const ConfigKnob KNOBS[] = {
    {"--width",       &Config::width,           "pipeline width (fetch..commit)"},
    {"--rob",         &Config::rob_size,        "reorder buffer entries"},
    {"--prf",         &Config::prf_size,        "physical register file size"},
    {"--iq",          &Config::iq_size,         "issue queue entries"},
    {"--lq",          &Config::lq_size,         "load queue entries"},
    {"--sq",          &Config::sq_size,         "store queue entries"},
    {"--cdb",         &Config::num_cdb,         "writeback ports (common data buses)"},
    {"--alu",         &Config::num_alu,         "ALU function units"},
    {"--branch",      &Config::num_branch,      "branch function units"},
    {"--mul",         &Config::num_mul,         "multiply units"},
    {"--div",         &Config::num_div,         "divide units"},
    {"--mem",         &Config::num_mem,         "memory (load/store) units"},
    {"--alu-lat",     &Config::alu_latency,     "ALU latency (cycles)"},
    {"--branch-lat",  &Config::branch_latency,  "branch latency (cycles)"},
    {"--mul-lat",     &Config::mul_latency,     "multiply latency (pipelined, cycles)"},
    {"--div-lat",     &Config::div_latency,     "divide latency (blocking, cycles)"},
    {"--mem-lat",     &Config::mem_latency,     "load-use latency (cycles)"},
    {"--ghr",         &Config::ghr_bits,        "gshare GHR width (bits)"},
    {"--pht",         &Config::pht_size,        "gshare PHT entries"},
    {"--btb-sets",    &Config::btb_sets,        "BTB sets"},
    {"--btb-ways",    &Config::btb_ways,        "BTB associativity"},
    {"--ras",         &Config::ras_size,        "return address stack entries"},
    {"--chkpt",       &Config::num_checkpoints, "in-flight branch checkpoints"},
};

inline constexpr std::size_t NUM_KNOBS = sizeof(KNOBS) / sizeof(KNOBS[0]);

namespace mini_cpu_cli_detail {

inline bool parse_uint(const std::string& s, uint32_t& out) {
    try {
        std::size_t pos = 0;
        const unsigned long v = std::stoul(s, &pos, 0);   // base 0 → auto 0x
        if (pos != s.size() || v > 0xFFFFFFFFul) return false;
        out = static_cast<uint32_t>(v);
        return true;
    } catch (...) { return false; }
}
inline bool parse_u64(const std::string& s, uint64_t& out) {
    try {
        std::size_t pos = 0;
        out = std::stoull(s, &pos, 0);
        return pos == s.size();
    } catch (...) { return false; }
}
inline std::string flag_of(const std::string& tok) {
    const auto eq = tok.find('=');
    return eq == std::string::npos ? tok : tok.substr(0, eq);
}

}  // namespace mini_cpu_cli_detail

// Returns 0 on success, non-zero on error with the message already on stderr.
inline int parse_args(int argc, char** argv, CliOpts& opts) {
    using namespace mini_cpu_cli_detail;
    for (int i = 1; i < argc; ++i) {
        const std::string tok = argv[i];
        const std::string flag = flag_of(tok);

        auto take_value = [&](std::string& val) -> bool {
            const auto eq = tok.find('=');
            if (eq != std::string::npos) { val = tok.substr(eq + 1); return true; }
            if (i + 1 >= argc) {
                std::fprintf(stderr, "oooc: %s requires a value\n", flag.c_str());
                return false;
            }
            val = argv[++i];
            return true;
        };

        if (flag == "--help" || flag == "-h") { opts.show_help = true; return 0; }
        if (flag == "--regs")      { opts.print_regs = true; continue; }
        if (flag == "--trace")     { opts.trace = true; continue; }
        if (flag == "--ref")       { opts.use_ref = true; continue; }
        if (flag == "--stats")     { opts.show_stats = true; continue; }
        if (flag == "--ipc-table") { opts.ipc_table = true; continue; }

        if (flag == "--hex") { if (!take_value(opts.hex_path)) return 1; continue; }
        if (flag == "--raw") { if (!take_value(opts.raw_path)) return 1; continue; }
        if (flag == "--base") {
            std::string v;
            if (!take_value(v))       return 1;
            if (!parse_uint(v, opts.base)) {
                std::fprintf(stderr, "oooc: bad --base '%s'\n", v.c_str()); return 1;
            }
            opts.has_base = true;
            continue;
        }
        if (flag == "--max-insts") {
            std::string v;
            if (!take_value(v))                     return 1;
            if (!parse_u64(v, opts.max_insts)) {
                std::fprintf(stderr, "oooc: bad --max-insts '%s'\n", v.c_str()); return 1;
            }
            continue;
        }
        if (flag == "--max-cycles") {
            std::string v;
            if (!take_value(v))                      return 1;
            if (!parse_u64(v, opts.max_cycles)) {
                std::fprintf(stderr, "oooc: bad --max-cycles '%s'\n", v.c_str()); return 1;
            }
            continue;
        }

        // Config knob table
        bool matched = false;
        for (const auto& k : KNOBS) {
            if (flag == k.flag) {
                std::string v;
                if (!take_value(v)) return 1;
                uint32_t parsed;
                if (!parse_uint(v, parsed)) {
                    std::fprintf(stderr, "oooc: bad %s '%s'\n", k.flag, v.c_str());
                    return 1;
                }
                opts.cfg.*(k.member) = parsed;
                matched = true;
                break;
            }
        }
        if (matched) continue;

        // Positional: exactly one ELF path allowed
        if (!tok.empty() && tok[0] != '-') {
            if (!opts.elf_path.empty()) {
                std::fprintf(stderr, "oooc: multiple positional arguments\n"); return 1;
            }
            opts.elf_path = tok;
            continue;
        }

        std::fprintf(stderr, "oooc: unknown flag '%s'\n", flag.c_str());
        return 1;
    }
    return 0;
}

inline void print_help() {
    std::printf("Usage: oooc [OPTIONS] [ELF]\n");
    std::printf("\nInput selection (exactly one):\n");
    std::printf("  --hex PATH        plain hex-word program (one 32-bit word per line)\n");
    std::printf("  --raw PATH        raw binary blob (requires --base)\n");
    std::printf("  ELF               positional path to an RV32 ELF32 executable\n");
    std::printf("\nGeneral:\n");
    std::printf("  --base ADDR       load address for --hex / --raw (accepts 0x prefix)\n");
    std::printf("  --regs            print architectural registers on exit\n");
    std::printf("  --stats           print cycles, IPC, prediction and the stall breakdown\n");
    std::printf("  --ipc-table       run the program 1-, 2-, 4-wide and starved, and compare\n");
    std::printf("  --ref             run the in-order reference interpreter instead\n");
    std::printf("  --trace           print each retired instruction to stderr (--ref only)\n");
    std::printf("  --max-insts N     abort after N retired instructions (default 1e8)\n");
    std::printf("  --max-cycles N    abort after N cycles (default 1e9)\n");
    std::printf("  --help, -h        this message\n");
    std::printf("\nMicroarchitectural knobs:\n");
    for (const auto& k : KNOBS) {
        std::printf("  %-14s  %s\n", k.flag, k.desc);
    }
}

// ============================================================================
// Reporting
// ============================================================================

// Everything the run measured, in the order it happens: the funnel from fetch
// to commit, then what the front end guessed, then memory, then the reasons
// issue slots went unused.
inline void print_stats(const Stats& s) {
    std::printf("\ncycles %llu  retired %llu  IPC %.3f  CPI %.3f\n",
                static_cast<unsigned long long>(s.cycles),
                static_cast<unsigned long long>(s.retired), s.ipc(), s.cpi());

    std::printf("  stages   fetch %llu  decode %llu  rename %llu  dispatch %llu"
                "  issue %llu  writeback %llu  squashed %llu\n",
                static_cast<unsigned long long>(s.fetched),
                static_cast<unsigned long long>(s.decoded),
                static_cast<unsigned long long>(s.renamed),
                static_cast<unsigned long long>(s.dispatched),
                static_cast<unsigned long long>(s.issued),
                static_cast<unsigned long long>(s.wrote_back),
                static_cast<unsigned long long>(s.squashed));

    std::printf("  branch   %llu  mispredicted %llu (%.1f%%, %.1f MPKI)"
                "  BTB %.1f%%  RAS %.1f%%\n",
                static_cast<unsigned long long>(s.branches),
                static_cast<unsigned long long>(s.mispredicts),
                100.0 * s.mispredict_rate(), s.mpki(),
                100.0 * s.btb_hit_rate(), 100.0 * s.ras_accuracy());

    std::printf("  memory   loads %llu (forwarded %llu, from memory %llu, replayed %llu)"
                "  stores %llu\n",
                static_cast<unsigned long long>(s.loads),
                static_cast<unsigned long long>(s.load_forwards),
                static_cast<unsigned long long>(s.load_memory),
                static_cast<unsigned long long>(s.load_replays),
                static_cast<unsigned long long>(s.stores));

    uint64_t total = 0;
    for (const uint64_t n : s.stalls) total += n;
    std::printf("  stalls   %llu lost slots\n", static_cast<unsigned long long>(total));
    if (total == 0) return;
    for (std::size_t i = 0; i < s.stalls.size(); ++i) {
        if (s.stalls[i] == 0) continue;
        std::printf("    %-20s %10llu  %5.1f%%\n", stall_name(static_cast<Stall>(i)),
                    static_cast<unsigned long long>(s.stalls[i]),
                    100.0 * static_cast<double>(s.stalls[i]) / static_cast<double>(total));
    }
    const Stall worst = s.dominant_stall();
    std::printf("  limited by: %s\n", worst == Stall::COUNT ? "no structure" : stall_name(worst));
}

// Same program, four machines. The point is the shape of the column: what a
// program gains from width says more about the program than about the machine.
inline void print_ipc_table(const Memory& image, uint32_t entry, uint64_t max_cycles) {
    struct Row { const char* name; Config cfg; };

    Config w1;  w1.width = 1;
    Config w2;                                    // the default is 2-wide
    Config w4;  w4.width = 4; w4.rob_size = 128; w4.prf_size = 160; w4.iq_size = 32;
    w4.num_alu = 4; w4.num_cdb = 4;
    Config tiny; tiny.rob_size = 4; tiny.prf_size = 40; tiny.iq_size = 2;

    const Row rows[] = {
        {"1-wide",      w1},
        {"2-wide",      w2},
        {"4-wide",      w4},
        {"ROB=4, IQ=2", tiny},
    };

    std::printf("\n%-14s %10s %10s %8s %8s\n", "machine", "cycles", "retired", "IPC", "vs 1-wide");
    double base = 0.0;
    for (const Row& r : rows) {
        Memory mem = image;                       // every run starts from the image
        Cpu cpu(mem, r.cfg, entry);
        cpu.run(max_cycles);
        const double ipc = cpu.stats().ipc();
        if (base == 0.0) base = ipc;
        std::printf("%-14s %10llu %10llu %8.2f %8.2fx\n", r.name,
                    static_cast<unsigned long long>(cpu.stats().cycles),
                    static_cast<unsigned long long>(cpu.stats().retired),
                    ipc, base > 0.0 ? ipc / base : 0.0);
    }
}

// ============================================================================
// Entry point (compiled out when included from a test TU).
// ============================================================================

#ifndef MINI_CPU_NO_ENTRY
int main(int argc, char** argv) {
    CliOpts opts;
    if (const int rc = parse_args(argc, argv, opts); rc != 0) return rc;
    if (opts.show_help) { print_help(); return 0; }

    const int inputs_given = int(!opts.hex_path.empty())
                           + int(!opts.raw_path.empty())
                           + int(!opts.elf_path.empty());
    if (inputs_given != 1) {
        std::fprintf(stderr, "oooc: give exactly one of --hex, --raw, or a positional ELF\n");
        return 2;
    }
    if (!opts.raw_path.empty() && !opts.has_base) {
        std::fprintf(stderr, "oooc: --raw requires --base\n");
        return 2;
    }

    Memory     mem;
    LoadResult loaded;
    try {
        if (!opts.hex_path.empty()) {
            std::ifstream in(opts.hex_path);
            if (!in) { std::fprintf(stderr, "oooc: cannot open %s\n", opts.hex_path.c_str()); return 2; }
            loaded = load_hex(mem, in, opts.base);
        } else if (!opts.raw_path.empty()) {
            std::ifstream in(opts.raw_path, std::ios::binary);
            if (!in) { std::fprintf(stderr, "oooc: cannot open %s\n", opts.raw_path.c_str()); return 2; }
            loaded = load_raw(mem, in, opts.base);
        } else {
            std::ifstream in(opts.elf_path, std::ios::binary);
            if (!in) { std::fprintf(stderr, "oooc: cannot open %s\n", opts.elf_path.c_str()); return 2; }
            loaded = load_elf(mem, in);
        }
    } catch (const std::exception& e) {
        std::fprintf(stderr, "oooc: load failed: %s\n", e.what());
        return 2;
    }

    if (opts.ipc_table) {
        print_ipc_table(mem, loaded.entry, opts.max_cycles);
        return 0;
    }

    std::array<uint32_t, 32> regs{};
    bool     halted = false, trapped = false, budget = false;
    uint64_t retired = 0;
    uint32_t exit_code = 0;

    if (opts.use_ref) {
        ref::Options ropts;
        ropts.max_insts = opts.max_insts;
        ropts.trace     = opts.trace;
        const ref::Result r = ref::run(mem, loaded.entry, ropts);
        for (int i = 0; i < 32; ++i) regs[i] = r.regs[i];
        halted = r.halted; trapped = r.trapped; budget = r.budget;
        retired = r.retired; exit_code = r.exit_code;
    } else {
        if (opts.trace) {
            std::fprintf(stderr, "oooc: --trace needs --ref\n");
            return 2;
        }
        Cpu cpu(mem, opts.cfg, loaded.entry);
        cpu.run(opts.max_cycles);
        regs      = cpu.regs();
        halted    = cpu.halted();
        trapped   = cpu.trapped();
        budget    = !cpu.done();
        retired   = cpu.retired();
        exit_code = cpu.exit_code();
        if (opts.show_stats) print_stats(cpu.stats());
    }

    if (opts.print_regs) {
        for (int i = 0; i < 32; ++i) {
            std::printf("x%-2d = 0x%08X%s", i, regs[i], (i % 4 == 3) ? "\n" : "  ");
        }
    }
    std::printf("halted=%d trapped=%d budget=%d retired=%llu exit=%u\n",
                halted ? 1 : 0, trapped ? 1 : 0, budget ? 1 : 0,
                static_cast<unsigned long long>(retired), exit_code);
    return trapped ? 1 : static_cast<int>(exit_code);
}
#endif
