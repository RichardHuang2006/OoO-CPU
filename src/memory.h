// Sparse little-endian byte-addressable memory, 4 KiB pages allocated lazily.
#pragma once
#include <cstdint>
#include <cstring>
#include <unordered_map>
#include <vector>

class Memory {
public:
    static constexpr uint32_t kPageBits = 12;
    static constexpr uint32_t kPageSize = 1u << kPageBits;

    uint8_t read8(uint32_t addr) const {
        auto it = pages_.find(addr >> kPageBits);
        if (it == pages_.end()) return 0;
        return it->second[addr & (kPageSize - 1)];
    }

    void write8(uint32_t addr, uint8_t v) {
        page(addr >> kPageBits)[addr & (kPageSize - 1)] = v;
    }

    uint32_t read(uint32_t addr, int bytes) const {
        uint32_t v = 0;
        for (int i = 0; i < bytes; i++) v |= (uint32_t)read8(addr + i) << (8 * i);
        return v;
    }

    void write(uint32_t addr, uint32_t v, int bytes) {
        for (int i = 0; i < bytes; i++) write8(addr + i, (uint8_t)(v >> (8 * i)));
    }

    uint32_t read32(uint32_t addr) const { return read(addr, 4); }
    void write32(uint32_t addr, uint32_t v) { write(addr, v, 4); }

    void load_blob(uint32_t addr, const uint8_t* data, size_t len) {
        for (size_t i = 0; i < len; i++) write8(addr + (uint32_t)i, data[i]);
    }

    bool mapped(uint32_t addr) const { return pages_.count(addr >> kPageBits) != 0; }

private:
    std::vector<uint8_t>& page(uint32_t pn) {
        auto it = pages_.find(pn);
        if (it == pages_.end())
            it = pages_.emplace(pn, std::vector<uint8_t>(kPageSize, 0)).first;
        return it->second;
    }

    mutable std::unordered_map<uint32_t, std::vector<uint8_t>> pages_;
};
