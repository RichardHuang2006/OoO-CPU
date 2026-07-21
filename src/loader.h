// Program loaders: 32-bit little-endian RISC-V ELF, raw binary, and a plain
// text hex-word format (one 32-bit instruction per line) so the simulator is
// usable without a cross toolchain.
#pragma once
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "memory.h"

struct LoadResult {
    bool ok = false;
    uint32_t entry = 0;
    std::string error;
};

inline std::vector<uint8_t> read_file(const std::string& path, bool& ok) {
    std::ifstream f(path, std::ios::binary);
    if (!f) { ok = false; return {}; }
    ok = true;
    return std::vector<uint8_t>((std::istreambuf_iterator<char>(f)),
                                 std::istreambuf_iterator<char>());
}

inline LoadResult load_elf32(const std::vector<uint8_t>& b, Memory& mem) {
    LoadResult r;
    if (b.size() < 52 || std::memcmp(b.data(), "\x7f" "ELF", 4) != 0) {
        r.error = "not an ELF file";
        return r;
    }
    if (b[4] != 1 || b[5] != 1) { r.error = "expected 32-bit little-endian ELF"; return r; }

    auto rd16 = [&](size_t o) { return (uint16_t)(b[o] | (b[o + 1] << 8)); };
    auto rd32 = [&](size_t o) {
        return (uint32_t)b[o] | ((uint32_t)b[o + 1] << 8) |
               ((uint32_t)b[o + 2] << 16) | ((uint32_t)b[o + 3] << 24);
    };

    const uint16_t machine = rd16(18);
    if (machine != 243) { r.error = "not a RISC-V ELF"; return r; }

    r.entry = rd32(24);
    const uint32_t phoff = rd32(28);
    const uint16_t phentsize = rd16(42);
    const uint16_t phnum = rd16(44);

    for (uint16_t i = 0; i < phnum; i++) {
        const size_t p = phoff + (size_t)i * phentsize;
        if (p + 32 > b.size()) { r.error = "truncated program header"; return r; }
        if (rd32(p) != 1) continue;              // PT_LOAD only
        const uint32_t off = rd32(p + 4);
        const uint32_t vaddr = rd32(p + 8);
        const uint32_t filesz = rd32(p + 16);
        const uint32_t memsz = rd32(p + 20);
        if (off + filesz > b.size()) { r.error = "segment past end of file"; return r; }
        mem.load_blob(vaddr, b.data() + off, filesz);
        for (uint32_t k = filesz; k < memsz; k++) mem.write8(vaddr + k, 0);  // .bss
    }
    r.ok = true;
    return r;
}

inline LoadResult load_raw(const std::vector<uint8_t>& b, Memory& mem, uint32_t base) {
    LoadResult r;
    mem.load_blob(base, b.data(), b.size());
    r.ok = true;
    r.entry = base;
    return r;
}

// One 32-bit hex word per line; '#' starts a comment.
inline LoadResult load_hex(const std::string& path, Memory& mem, uint32_t base) {
    LoadResult r;
    std::ifstream f(path);
    if (!f) { r.error = "cannot open " + path; return r; }

    uint32_t addr = base;
    std::string line;
    while (std::getline(f, line)) {
        const size_t hash = line.find('#');
        if (hash != std::string::npos) line.resize(hash);
        std::istringstream ss(line);
        std::string tok;
        while (ss >> tok) {
            uint32_t word = 0;
            if (std::sscanf(tok.c_str(), "%x", &word) != 1) {
                r.error = "bad hex word: " + tok;
                return r;
            }
            mem.write32(addr, word);
            addr += 4;
        }
    }
    r.ok = true;
    r.entry = base;
    return r;
}
