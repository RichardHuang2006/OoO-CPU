#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>

#include "cpu.h"
#include "loader.h"
#include "memory.h"

namespace {

const char* kUsage =
    "usage: oooc [options] <program>\n"
    "\n"
    "  <program>  RISC-V ELF32, raw binary (--raw), or hex words (--hex)\n"
    "\n"
    "loading\n"
    "  --raw               load the file as a flat binary at --base\n"
    "  --hex               load the file as text hex words at --base\n"
    "  --base ADDR         load address for --raw/--hex        (0x1000)\n"
    "  --entry ADDR        override the entry PC\n"
    "  --sp ADDR           initial stack pointer                (0x80000)\n"
    "\n"
    "machine\n"
    "  --width N           fetch/decode/rename/dispatch/issue/commit width (2)\n"
    "  --rob N             reorder buffer entries               (32)\n"
    "  --prf N             physical registers                   (64)\n"
    "  --iq N              issue queue entries                  (16)\n"
    "  --lq N / --sq N     load / store queue entries           (8 / 8)\n"
    "  --cdbs N            writeback ports                      (2)\n"
    "  --alus N            ALU units                            (2)\n"
    "  --mem-units N       address units                        (1)\n"
    "  --mem-lat N         load use latency                     (2)\n"
    "  --mul-lat N         pipelined multiplier latency         (3)\n"
    "  --chkpts N          branch checkpoints                   (16)\n"
    "\n"
    "branch prediction\n"
    "  --ghr N             gshare history bits                  (12)\n"
    "  --pht N             PHT entries                          (4096)\n"
    "  --btb-sets N        BTB sets                             (128)\n"
    "  --btb-ways N        BTB ways                             (4)\n"
    "  --ras N             return address stack entries         (16)\n"
    "\n"
    "run control\n"
    "  --max-cycles N      cycle limit                          (1e8)\n"
    "  --trace             per-uop pipeline trace\n"
    "  --regs              dump architectural registers at halt\n"
    "  -h, --help          this message\n";

uint32_t parse_num(const char* s) { return (uint32_t)strtoul(s, nullptr, 0); }

const char* kRegNames[32] = {
    "zero", "ra", "sp",  "gp",  "tp", "t0", "t1", "t2",
    "s0",   "s1", "a0",  "a1",  "a2", "a3", "a4", "a5",
    "a6",   "a7", "s2",  "s3",  "s4", "s5", "s6", "s7",
    "s8",   "s9", "s10", "s11", "t3", "t4", "t5", "t6"};

} // namespace

int main(int argc, char** argv) {
    Config cfg;
    std::string path;
    bool raw = false, hex = false, dump_regs = false;
    uint32_t base = 0x1000, sp = 0x80000;
    bool have_entry = false;
    uint32_t entry = 0;

    for (int i = 1; i < argc; i++) {
        const std::string a = argv[i];
        auto next = [&]() -> const char* {
            if (i + 1 >= argc) { std::cerr << "missing value for " << a << "\n"; exit(2); }
            return argv[++i];
        };

        if (a == "-h" || a == "--help") { std::cout << kUsage; return 0; }
        else if (a == "--raw")   raw = true;
        else if (a == "--hex")   hex = true;
        else if (a == "--trace") cfg.trace = true;
        else if (a == "--regs")  dump_regs = true;
        else if (a == "--base")  base = parse_num(next());
        else if (a == "--sp")    sp = parse_num(next());
        else if (a == "--entry") { entry = parse_num(next()); have_entry = true; }
        else if (a == "--width") {
            int w = (int)parse_num(next());
            cfg.fetch_width = cfg.decode_width = cfg.rename_width =
                cfg.dispatch_width = cfg.issue_width = cfg.commit_width = w;
        }
        else if (a == "--rob")        cfg.rob_entries = (int)parse_num(next());
        else if (a == "--prf")        cfg.phys_regs = (int)parse_num(next());
        else if (a == "--iq")         cfg.iq_entries = (int)parse_num(next());
        else if (a == "--lq")         cfg.lq_entries = (int)parse_num(next());
        else if (a == "--sq")         cfg.sq_entries = (int)parse_num(next());
        else if (a == "--cdbs")       cfg.num_cdbs = (int)parse_num(next());
        else if (a == "--alus")       cfg.num_alu = (int)parse_num(next());
        else if (a == "--mem-units")  cfg.num_mem = (int)parse_num(next());
        else if (a == "--mem-lat")    cfg.mem_latency = (int)parse_num(next());
        else if (a == "--mul-lat")    cfg.mul_latency = (int)parse_num(next());
        else if (a == "--chkpts")     cfg.max_checkpoints = (int)parse_num(next());
        else if (a == "--ghr")        cfg.ghr_bits = (int)parse_num(next());
        else if (a == "--pht")        cfg.pht_entries = (int)parse_num(next());
        else if (a == "--btb-sets")   cfg.btb_sets = (int)parse_num(next());
        else if (a == "--btb-ways")   cfg.btb_ways = (int)parse_num(next());
        else if (a == "--ras")        cfg.ras_entries = (int)parse_num(next());
        else if (a == "--max-cycles") cfg.max_cycles = strtoull(next(), nullptr, 0);
        else if (!a.empty() && a[0] == '-') {
            std::cerr << "unknown option " << a << "\n\n" << kUsage;
            return 2;
        }
        else path = a;
    }

    if (path.empty()) { std::cerr << kUsage; return 2; }
    cfg.normalize();

    Memory mem;
    LoadResult lr;
    if (hex) {
        lr = load_hex(path, mem, base);
    } else {
        bool ok = false;
        std::vector<uint8_t> bytes = read_file(path, ok);
        if (!ok) { std::cerr << "cannot open " << path << "\n"; return 1; }
        lr = raw ? load_raw(bytes, mem, base) : load_elf32(bytes, mem);
    }
    if (!lr.ok) { std::cerr << "load failed: " << lr.error << "\n"; return 1; }

    Cpu cpu(cfg, mem, have_entry ? entry : lr.entry);

    // A freshly reset machine has x2 (sp) pointing at the top of the stack.
    // p2 is x2's initial mapping and is still live before any rename.
    // (Written through the PRF-backed architectural view.)
    cpu.set_arch_reg(2, sp);

    std::cout << "loaded " << path << "  entry=0x" << std::hex
              << (have_entry ? entry : lr.entry) << std::dec << "\n";
    std::cout << "config: " << cfg.fetch_width << "-wide  rob=" << cfg.rob_entries
              << " prf=" << cfg.phys_regs << " iq=" << cfg.iq_entries
              << " lq/sq=" << cfg.lq_entries << "/" << cfg.sq_entries
              << " cdbs=" << cfg.num_cdbs << "  gshare(" << cfg.ghr_bits
              << "b, " << cfg.pht_entries << ") btb=" << (cfg.btb_sets * cfg.btb_ways)
              << "/" << cfg.btb_ways << "-way ras=" << cfg.ras_entries << "\n";

    cpu.run();
    cpu.print_stats(std::cout);

    if (dump_regs) {
        std::cout << "\n=== Architectural registers ===\n";
        for (int r = 0; r < 32; r++) {
            std::printf("  x%-2d %-5s 0x%08x %12d%s", r, kRegNames[r],
                        cpu.arch_reg(r), (int32_t)cpu.arch_reg(r),
                        (r % 2) ? "\n" : "   ");
        }
        std::cout << "\n";
    }
    return cpu.exit_code();
}
