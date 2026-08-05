#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <unordered_map>

// Flat, paged, always-hit physical memory.
//
// * Pages are 4 KiB, allocated lazily on first write. Reads from an
//   unmapped page return zero and do NOT allocate — the OoO simulator
//   will speculatively touch memory (Step 6.2's load pipeline), and
//   speculation must not allocate.
// * RV32IM permits misaligned loads and stores; wider accesses assemble
//   themselves from byte accesses, so a store that straddles a page
//   boundary just allocates whichever pages it touches.
// * Little-endian throughout — RISC-V's endianness.

class Memory {
public:
    static constexpr uint32_t PAGE_BITS = 12;
    static constexpr uint32_t PAGE_SIZE = 1u << PAGE_BITS;   // 4 KiB
    static constexpr uint32_t PAGE_MASK = PAGE_SIZE - 1;

    uint8_t  load_u8 (uint32_t addr) const;
    uint16_t load_u16(uint32_t addr) const;
    uint32_t load_u32(uint32_t addr) const;

    void store_u8 (uint32_t addr, uint8_t  v);
    void store_u16(uint32_t addr, uint16_t v);
    void store_u32(uint32_t addr, uint32_t v);

    // Bulk write used by the program loaders in Step 1.4.
    void write_bytes(uint32_t addr, const uint8_t* data, std::size_t len);

    // Introspection.
    std::size_t num_pages() const noexcept { return pages_.size(); }
    bool has_page(uint32_t addr) const noexcept {
        return pages_.find(addr & ~PAGE_MASK) != pages_.end();
    }

private:
    using Page = std::array<uint8_t, PAGE_SIZE>;
    std::unordered_map<uint32_t, Page> pages_;
};

// ---- inline implementation --------------------------------------------------

inline uint8_t Memory::load_u8(uint32_t addr) const {
    const auto it = pages_.find(addr & ~PAGE_MASK);
    if (it == pages_.end()) return 0;
    return it->second[addr & PAGE_MASK];
}

inline uint16_t Memory::load_u16(uint32_t addr) const {
    const uint32_t b0 = load_u8(addr);
    const uint32_t b1 = load_u8(addr + 1);
    return static_cast<uint16_t>(b0 | (b1 << 8));
}

inline uint32_t Memory::load_u32(uint32_t addr) const {
    const uint32_t b0 = load_u8(addr);
    const uint32_t b1 = load_u8(addr + 1);
    const uint32_t b2 = load_u8(addr + 2);
    const uint32_t b3 = load_u8(addr + 3);
    return b0 | (b1 << 8) | (b2 << 16) | (b3 << 24);
}

inline void Memory::store_u8(uint32_t addr, uint8_t v) {
    // operator[] value-initializes the mapped std::array to zeros on insert.
    pages_[addr & ~PAGE_MASK][addr & PAGE_MASK] = v;
}

inline void Memory::store_u16(uint32_t addr, uint16_t v) {
    store_u8(addr,     static_cast<uint8_t>(v));
    store_u8(addr + 1, static_cast<uint8_t>(v >> 8));
}

inline void Memory::store_u32(uint32_t addr, uint32_t v) {
    store_u8(addr,     static_cast<uint8_t>(v));
    store_u8(addr + 1, static_cast<uint8_t>(v >>  8));
    store_u8(addr + 2, static_cast<uint8_t>(v >> 16));
    store_u8(addr + 3, static_cast<uint8_t>(v >> 24));
}

inline void Memory::write_bytes(uint32_t addr, const uint8_t* data, std::size_t len) {
    for (std::size_t i = 0; i < len; ++i) {
        store_u8(addr + static_cast<uint32_t>(i), data[i]);
    }
}
