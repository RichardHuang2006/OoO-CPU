#pragma once

#include <cstdint>
#include <istream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "memory.h"

// Three program-loading paths for `src/main.cpp` (Step 1.5). Each takes an
// istream so tests drive them from std::istringstream and the CLI drives
// them from std::ifstream — no filesystem coupling here.

struct LoadResult {
    uint32_t entry = 0;    // program entry PC (base for hex/raw, e_entry for ELF)

    // Read-only ranges (PF_R without PF_W) extracted from ELF PT_LOADs.
    // Empty for hex/raw. Handed back to the caller rather than enforced in
    // Memory itself, so the OoO simulator can decide how strictly to police
    // wrong-path stores.
    std::vector<std::pair<uint32_t, uint32_t>> ro_ranges;   // [start, end)
};

// ---- plain hex-word format ------------------------------------------------
// One 32-bit hex word per line. `#` and `//` start a line comment; blank
// lines are skipped; an optional `0x` / `0X` prefix is accepted; the first
// word lands at `base`, subsequent words at `base+4`, `base+8`, ...
inline LoadResult load_hex(Memory& mem, std::istream& in, uint32_t base = 0) {
    LoadResult r;
    r.entry = base;
    uint32_t addr = base;
    std::string line;
    while (std::getline(in, line)) {
        // Strip trailing '\r' (Windows line endings) before comment stripping
        // so a '#' or '//' at end-of-line isn't shadowed by a stray '\r'.
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (auto p = line.find("//"); p != std::string::npos) line.erase(p);
        if (auto p = line.find('#');  p != std::string::npos) line.erase(p);
        auto is_ws = [](char c) { return c == ' ' || c == '\t'; };
        while (!line.empty() && is_ws(line.front())) line.erase(0, 1);
        while (!line.empty() && is_ws(line.back()))  line.pop_back();
        if (line.empty()) continue;

        if (line.size() >= 2 && line[0] == '0' && (line[1] == 'x' || line[1] == 'X')) {
            line.erase(0, 2);
        }
        if (line.empty() || line.find_first_not_of("0123456789abcdefABCDEF")
                                != std::string::npos) {
            throw std::runtime_error("load_hex: invalid hex word '" + line + "'");
        }
        const uint32_t word = static_cast<uint32_t>(std::stoul(line, nullptr, 16));
        mem.store_u32(addr, word);
        addr += 4;
    }
    return r;
}

// ---- raw binary blob at `base` -------------------------------------------
inline LoadResult load_raw(Memory& mem, std::istream& in, uint32_t base) {
    LoadResult r;
    r.entry = base;
    std::vector<char> buf((std::istreambuf_iterator<char>(in)),
                          std::istreambuf_iterator<char>());
    if (!buf.empty()) {
        mem.write_bytes(base, reinterpret_cast<const uint8_t*>(buf.data()),
                        buf.size());
    }
    return r;
}

// ---- ELF32 little-endian executable --------------------------------------
// Walks program headers, loads every PT_LOAD segment at its p_vaddr, records
// PF_R & ~PF_W segments in ro_ranges. Fields are read at fixed byte offsets
// so no packed struct is required — no ABI risk from padding.
inline LoadResult load_elf(Memory& mem, std::istream& in) {
    LoadResult r;
    std::vector<uint8_t> buf((std::istreambuf_iterator<char>(in)),
                             std::istreambuf_iterator<char>());
    if (buf.size() < 52) {
        throw std::runtime_error("load_elf: file too small for Elf32_Ehdr");
    }
    if (buf[0] != 0x7F || buf[1] != 'E' || buf[2] != 'L' || buf[3] != 'F') {
        throw std::runtime_error("load_elf: bad ELF magic");
    }
    if (buf[4] != 1) throw std::runtime_error("load_elf: not ELFCLASS32");
    if (buf[5] != 1) throw std::runtime_error("load_elf: not ELFDATA2LSB");

    auto rd_u16 = [&](std::size_t off) -> uint16_t {
        return static_cast<uint16_t>(buf[off]) |
               static_cast<uint16_t>(buf[off + 1] << 8);
    };
    auto rd_u32 = [&](std::size_t off) -> uint32_t {
        return  static_cast<uint32_t>(buf[off])            |
               (static_cast<uint32_t>(buf[off + 1]) <<  8) |
               (static_cast<uint32_t>(buf[off + 2]) << 16) |
               (static_cast<uint32_t>(buf[off + 3]) << 24);
    };

    r.entry             = rd_u32(24);
    const uint32_t phoff = rd_u32(28);
    const uint16_t phentsize = rd_u16(42);
    const uint16_t phnum = rd_u16(44);

    constexpr uint32_t PT_LOAD = 1;
    constexpr uint32_t PF_W = 2;
    constexpr uint32_t PF_R = 4;

    for (uint16_t i = 0; i < phnum; ++i) {
        const std::size_t base_off =
            static_cast<std::size_t>(phoff) + static_cast<std::size_t>(i) * phentsize;
        if (base_off + 32 > buf.size()) {
            throw std::runtime_error("load_elf: Elf32_Phdr past EOF");
        }
        const uint32_t p_type   = rd_u32(base_off +  0);
        const uint32_t p_offset = rd_u32(base_off +  4);
        const uint32_t p_vaddr  = rd_u32(base_off +  8);
        const uint32_t p_filesz = rd_u32(base_off + 16);
        const uint32_t p_memsz  = rd_u32(base_off + 20);
        const uint32_t p_flags  = rd_u32(base_off + 24);

        if (p_type != PT_LOAD) continue;
        if (static_cast<std::size_t>(p_offset) + p_filesz > buf.size()) {
            throw std::runtime_error("load_elf: PT_LOAD segment past EOF");
        }
        mem.write_bytes(p_vaddr, buf.data() + p_offset, p_filesz);
        // p_memsz > p_filesz (BSS) leaves the tail unmapped; Memory returns 0
        // for reads of unmapped bytes, which matches BSS semantics.

        if ((p_flags & PF_R) && !(p_flags & PF_W)) {
            r.ro_ranges.emplace_back(p_vaddr, p_vaddr + p_memsz);
        }
    }
    return r;
}
