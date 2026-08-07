// Writes the bundled workloads to examples/*.hex so the CLI has something to
// run without the test binary.
//
// The programs come from the same assembler and the same corpus the test suite
// validates, so an example file cannot drift away from what was verified.

#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include "workloads.h"

namespace {

// The subset worth shipping: one of each interesting shape.
const char* const WANTED[] = {"sieve", "matmul", "bubble_sort", "fib", "crc32"};

bool write_hex(const wl::Workload& w, const std::string& dir) {
    const std::string path = dir + "/" + w.name + ".hex";
    std::ofstream out(path);
    if (!out) {
        std::fprintf(stderr, "gen_examples: cannot write %s\n", path.c_str());
        return false;
    }

    out << "# " << w.name << " — " << w.words.size() << " words, load at 0x"
        << std::hex << wl::TEXT << std::dec << "\n";
    out << "# run: oooc --hex examples/" << w.name << ".hex --base 0x"
        << std::hex << wl::TEXT << std::dec;
    if (w.expect_exit) out << "   (exits " << *w.expect_exit << ")";
    out << "\n";

    for (const uint32_t word : w.words) {
        char buf[16];
        std::snprintf(buf, sizeof(buf), "%08X\n", word);
        out << buf;
    }
    if (!out) {
        std::fprintf(stderr, "gen_examples: write failed for %s\n", path.c_str());
        return false;
    }
    std::printf("examples/%s.hex  %zu words\n", w.name.c_str(), w.words.size());
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    const std::string dir = argc > 1 ? argv[1] : "examples";

    int written = 0;
    for (const char* name : WANTED) {
        bool found = false;
        for (const wl::Workload& w : wl::corpus()) {
            if (w.name != name) continue;
            found = true;
            if (!write_hex(w, dir)) return 1;
            ++written;
        }
        if (!found) {
            std::fprintf(stderr, "gen_examples: no workload named %s\n", name);
            return 1;
        }
    }
    std::printf("wrote %d example(s) to %s/\n", written, dir.c_str());
    return 0;
}
