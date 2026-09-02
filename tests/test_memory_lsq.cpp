// Memory-system tests: the paged byte memory, the three program loaders, the
// load/store queues (forwarding, partial overlap, replay), store-at-commit
// discipline through the pipeline, and load-use timing.

#include <fstream>
#include <functional>

#include "loader.h"
#include "lsq.h"
#include "test_support.h"

// ------------------------------------------------------ @section("memory") ---
SECTION("memory") {
    // ---- Fresh memory: no pages, loads return zero and don't allocate ----
    Memory m;
    REQUIRE(m.num_pages() == 0);
    REQUIRE(m.load_u8 (0x1000) == 0);
    REQUIRE(m.load_u16(0x1000) == 0);
    REQUIRE(m.load_u32(0x1000) == 0);
    REQUIRE(m.num_pages() == 0);

    // A single store lazily allocates one page.
    m.store_u8(0x1000, 0xAA);
    REQUIRE(m.num_pages() == 1);
    REQUIRE( m.has_page(0x1000));
    REQUIRE( m.has_page(0x1FFF));      // last byte of same page
    REQUIRE(!m.has_page(0x2000));      // next page not yet touched
    REQUIRE(m.load_u8(0x1000) == 0xAA);

    m.store_u8(0x2000, 0xBB);
    REQUIRE(m.num_pages() == 2);

    // ---- Misaligned store, read back at three widths ----------------------
    // 0xDEADBEEF at 0x1002, read back as one lw, two lh's, four lb's.
    Memory misa;
    misa.store_u32(0x1002, 0xDEADBEEFu);
    REQUIRE(misa.load_u32(0x1002) == 0xDEADBEEFu);
    REQUIRE(misa.load_u16(0x1002) == 0xBEEFu);
    REQUIRE(misa.load_u16(0x1004) == 0xDEADu);
    REQUIRE(misa.load_u8 (0x1002) == 0xEFu);
    REQUIRE(misa.load_u8 (0x1003) == 0xBEu);
    REQUIRE(misa.load_u8 (0x1004) == 0xADu);
    REQUIRE(misa.load_u8 (0x1005) == 0xDEu);
    // Bytes 0x1002..0x1005 all fall in the 0x1000 page.
    REQUIRE(misa.num_pages() == 1);

    // ---- Cross-page misaligned store --------------------------------------
    // 0x0FFE..0x1001 straddles the 0x0000 and 0x1000 pages.
    Memory cross;
    cross.store_u32(0x0FFE, 0xCAFEBABEu);
    REQUIRE(cross.num_pages() == 2);
    REQUIRE(cross.load_u32(0x0FFE) == 0xCAFEBABEu);
    REQUIRE(cross.load_u16(0x0FFE) == 0xBABEu);   // low half in page 0x0000
    REQUIRE(cross.load_u16(0x1000) == 0xCAFEu);   // high half in page 0x1000
    REQUIRE(cross.load_u8(0x0FFE) == 0xBEu);
    REQUIRE(cross.load_u8(0x0FFF) == 0xBAu);
    REQUIRE(cross.load_u8(0x1000) == 0xFEu);
    REQUIRE(cross.load_u8(0x1001) == 0xCAu);

    // ---- Speculative reads must not allocate ------------------------------
    Memory readonly;
    (void)readonly.load_u8 (0xDEAD);
    (void)readonly.load_u16(0xDEAD);
    (void)readonly.load_u32(0xDEAD);
    (void)readonly.load_u32(0x0FFE);   // even the straddling case
    REQUIRE(readonly.num_pages() == 0);

    // ---- write_bytes: loader.h's bulk-copy path ---------------------------
    Memory bulk;
    const uint8_t data[] = {0x11, 0x22, 0x33, 0x44, 0x55};
    bulk.write_bytes(0x2000, data, sizeof(data));
    REQUIRE(bulk.load_u32(0x2000) == 0x44332211u);
    REQUIRE(bulk.load_u8 (0x2004) == 0x55u);

    // A zero-length write is a no-op and does not allocate.
    Memory zero;
    zero.write_bytes(0x3000, data, 0);
    REQUIRE(zero.num_pages() == 0);
}

// ------------------------------------------------------ @section("loader") ---
namespace loadertest {
    // Build a minimal ELF32 little-endian executable with one PT_LOAD segment.
    inline std::vector<uint8_t> make_elf32(uint32_t entry, uint32_t vaddr,
                                           const uint8_t* payload,
                                           uint32_t size, uint32_t p_flags) {
        constexpr uint16_t ehdr_size = 52;
        constexpr uint16_t phdr_size = 32;
        const std::size_t payload_off = ehdr_size + phdr_size;
        std::vector<uint8_t> elf(payload_off + size, 0);

        auto wr8  = [&](std::size_t o, uint8_t v)  { elf[o] = v; };
        auto wr16 = [&](std::size_t o, uint16_t v) {
            elf[o]     = static_cast<uint8_t>(v);
            elf[o + 1] = static_cast<uint8_t>(v >> 8);
        };
        auto wr32 = [&](std::size_t o, uint32_t v) {
            elf[o]     = static_cast<uint8_t>(v);
            elf[o + 1] = static_cast<uint8_t>(v >>  8);
            elf[o + 2] = static_cast<uint8_t>(v >> 16);
            elf[o + 3] = static_cast<uint8_t>(v >> 24);
        };

        // Ehdr
        wr8(0, 0x7F); wr8(1, 'E'); wr8(2, 'L'); wr8(3, 'F');
        wr8(4, 1);            // ELFCLASS32
        wr8(5, 1);            // ELFDATA2LSB
        wr8(6, 1);            // EV_CURRENT
        wr16(16, 2);          // e_type = ET_EXEC
        wr16(18, 0xF3);       // e_machine = EM_RISCV
        wr32(20, 1);          // e_version
        wr32(24, entry);
        wr32(28, ehdr_size);  // e_phoff
        wr32(32, 0);          // e_shoff
        wr32(36, 0);          // e_flags
        wr16(40, ehdr_size);
        wr16(42, phdr_size);
        wr16(44, 1);          // e_phnum
        wr16(46, 40);         // e_shentsize
        wr16(48, 0);          // e_shnum
        wr16(50, 0);          // e_shstrndx

        // Phdr
        wr32(ehdr_size +  0, 1);                                             // PT_LOAD
        wr32(ehdr_size +  4, static_cast<uint32_t>(payload_off));            // p_offset
        wr32(ehdr_size +  8, vaddr);                                         // p_vaddr
        wr32(ehdr_size + 12, vaddr);                                         // p_paddr
        wr32(ehdr_size + 16, size);                                          // p_filesz
        wr32(ehdr_size + 20, size);                                          // p_memsz
        wr32(ehdr_size + 24, p_flags);                                       // p_flags
        wr32(ehdr_size + 28, 4);                                             // p_align

        for (uint32_t i = 0; i < size; ++i) elf[payload_off + i] = payload[i];
        return elf;
    }

    inline bool threw(std::function<void()> fn) {
        try { fn(); return false; } catch (const std::exception&) { return true; }
    }
}

SECTION("loader") {
    // Common payload: three 32-bit words at base 0x1000.
    const uint32_t words[] = {0xDEADBEEFu, 0xCAFEBABEu, 0x12345678u};
    const uint32_t base = 0x1000;

    uint8_t payload[sizeof(words)];
    for (std::size_t i = 0; i < 3; ++i) {
        payload[i*4 + 0] = static_cast<uint8_t>(words[i]);
        payload[i*4 + 1] = static_cast<uint8_t>(words[i] >>  8);
        payload[i*4 + 2] = static_cast<uint8_t>(words[i] >> 16);
        payload[i*4 + 3] = static_cast<uint8_t>(words[i] >> 24);
    }

    // ---- Hex ----------------------------------------------------------------
    Memory m_hex;
    {
        std::istringstream ss(
            "DEADBEEF\n"
            "# a comment on its own line\n"
            "\n"
            "  0xCAFEBABE  // trailing comment, leading spaces\n"
            "12345678\n");
        const auto r = load_hex(m_hex, ss, base);
        REQUIRE(r.entry == base);
        REQUIRE(r.ro_ranges.empty());
    }

    // ---- Raw ----------------------------------------------------------------
    Memory m_raw;
    {
        std::string bytes(reinterpret_cast<const char*>(payload), sizeof(payload));
        std::istringstream ss(bytes);
        const auto r = load_raw(m_raw, ss, base);
        REQUIRE(r.entry == base);
        REQUIRE(r.ro_ranges.empty());
    }

    // ---- ELF32 --------------------------------------------------------------
    Memory m_elf;
    {
        const auto blob = loadertest::make_elf32(base, base, payload,
                                                 sizeof(payload),
                                                 /*PF_R | PF_X =*/ 5);
        std::string s(reinterpret_cast<const char*>(blob.data()), blob.size());
        std::istringstream ss(s);
        const auto r = load_elf(m_elf, ss);
        REQUIRE(r.entry == base);
        REQUIRE(r.ro_ranges.size() == 1);
        REQUIRE(r.ro_ranges[0].first  == base);
        REQUIRE(r.ro_ranges[0].second == base + sizeof(payload));
    }

    // ---- All three formats land identical bytes ---------------------------
    for (std::size_t i = 0; i < 3; ++i) {
        REQUIRE(m_hex.load_u32(base + i * 4) == words[i]);
        REQUIRE(m_raw.load_u32(base + i * 4) == words[i]);
        REQUIRE(m_elf.load_u32(base + i * 4) == words[i]);
    }

    // ---- Writable ELF segment does NOT get flagged read-only --------------
    {
        Memory m;
        const auto blob = loadertest::make_elf32(base, base, payload,
                                                 sizeof(payload),
                                                 /*PF_R | PF_W =*/ 6);
        std::string s(reinterpret_cast<const char*>(blob.data()), blob.size());
        std::istringstream ss(s);
        const auto r = load_elf(m, ss);
        REQUIRE(r.ro_ranges.empty());
    }

    // ---- ELF BSS (p_memsz > p_filesz) reads back as zeros -----------------
    {
        Memory m;
        // Build ELF with p_filesz=size, p_memsz=size+8 (fake BSS tail).
        auto blob = loadertest::make_elf32(base, base, payload,
                                           sizeof(payload), 6);
        // Patch p_memsz at Phdr offset 20 → size + 8.
        const std::size_t phdr = 52;
        const uint32_t new_memsz = sizeof(payload) + 8;
        blob[phdr + 20] = static_cast<uint8_t>(new_memsz);
        blob[phdr + 21] = static_cast<uint8_t>(new_memsz >>  8);
        blob[phdr + 22] = static_cast<uint8_t>(new_memsz >> 16);
        blob[phdr + 23] = static_cast<uint8_t>(new_memsz >> 24);
        std::string s(reinterpret_cast<const char*>(blob.data()), blob.size());
        std::istringstream ss(s);
        (void)load_elf(m, ss);
        REQUIRE(m.load_u32(base) == words[0]);
        // Bytes beyond p_filesz read as zero (Memory returns 0 for unmapped).
        REQUIRE(m.load_u32(base + sizeof(payload))     == 0u);
        REQUIRE(m.load_u32(base + sizeof(payload) + 4) == 0u);
    }

    // ---- Error paths ------------------------------------------------------
    REQUIRE(loadertest::threw([]{
        Memory m; std::istringstream ss("NOTHEX\n"); load_hex(m, ss, 0);
    }));
    REQUIRE(loadertest::threw([]{
        Memory m; std::istringstream ss("not an elf at all"); load_elf(m, ss);
    }));
    REQUIRE(loadertest::threw([]{
        // Correct magic but ELFCLASS64
        Memory m;
        std::string s(52, '\0');
        s[0] = 0x7F; s[1] = 'E'; s[2] = 'L'; s[3] = 'F';
        s[4] = 2;    // ELFCLASS64
        s[5] = 1;
        std::istringstream ss(s);
        load_elf(m, ss);
    }));
}

// --------------------------------------------------------- @section("lsq") ---
SECTION("lsq") {
    // ---- Seats are handed out in order, and given back in order -----------
    {
        Lsq lsq(2, 2);
        REQUIRE(lsq.loads().empty());
        REQUIRE(lsq.stores().capacity() == 2);

        const std::optional<uint32_t> a = lsq.alloc_store(0, 4);
        const std::optional<uint32_t> b = lsq.alloc_store(1, 4);
        REQUIRE(a.has_value());
        REQUIRE(b.has_value());
        REQUIRE(lsq.stores().full());
        REQUIRE(!lsq.alloc_store(2, 4).has_value());   // dispatch stalls here

        lsq.stores().pop_head();
        REQUIRE(lsq.stores().size() == 1);
        REQUIRE(lsq.stores().nth(0).seq == 1);
        REQUIRE(lsq.alloc_store(2, 4).has_value());
    }

    // ---- A resolved store that covers the load forwards it ----------------
    {
        Lsq lsq(4, 4);
        const uint32_t s = *lsq.alloc_store(0, 4);
        lsq.resolve_addr(s, 0x100);
        lsq.resolve_data(s, 0xDEADBEEF);

        const uint32_t l = *lsq.alloc_load(1, 4);
        lsq.resolve_load_addr(l, 0x100);

        const ForwardResult r = lsq.search_older_stores(l);
        REQUIRE(r.kind == Forward::FORWARD);
        REQUIRE(r.data == 0xDEADBEEFu);
    }

    // ---- An unresolved older store forces a replay ------------------------
    // The address is unknown, so it might be this one. Guessing is the bug
    // this rule exists to prevent.
    {
        Lsq lsq(4, 4);
        const uint32_t s = *lsq.alloc_store(0, 4);     // address never resolved
        const uint32_t l = *lsq.alloc_load(1, 4);
        lsq.resolve_load_addr(l, 0x200);
        REQUIRE(lsq.search_older_stores(l).kind == Forward::REPLAY);

        // Resolving it somewhere else clears the ambiguity.
        lsq.resolve_addr(s, 0x300);
        lsq.resolve_data(s, 1);
        REQUIRE(lsq.search_older_stores(l).kind == Forward::NO_MATCH);
    }

    // ---- A store with a known address but no data yet also replays --------
    {
        Lsq lsq(4, 4);
        const uint32_t s = *lsq.alloc_store(0, 4);
        lsq.resolve_addr(s, 0x100);
        const uint32_t l = *lsq.alloc_load(1, 4);
        lsq.resolve_load_addr(l, 0x100);
        REQUIRE(lsq.search_older_stores(l).kind == Forward::REPLAY);

        lsq.resolve_data(s, 77);
        const ForwardResult r = lsq.search_older_stores(l);
        REQUIRE(r.kind == Forward::FORWARD);
        REQUIRE(r.data == 77u);
    }

    // ---- No older store aliases the address -------------------------------
    {
        Lsq lsq(4, 4);
        const uint32_t s = *lsq.alloc_store(0, 4);
        lsq.resolve_addr(s, 0x100);
        lsq.resolve_data(s, 5);
        const uint32_t l = *lsq.alloc_load(1, 4);
        lsq.resolve_load_addr(l, 0x104);               // adjacent, not overlapping
        REQUIRE(lsq.search_older_stores(l).kind == Forward::NO_MATCH);
    }

    // ---- A younger store is not this load's problem -----------------------
    {
        Lsq lsq(4, 4);
        const uint32_t l = *lsq.alloc_load(0, 4);
        lsq.resolve_load_addr(l, 0x100);
        const uint32_t s = *lsq.alloc_store(1, 4);     // younger
        lsq.resolve_addr(s, 0x100);
        lsq.resolve_data(s, 9);
        REQUIRE(lsq.search_older_stores(l).kind == Forward::NO_MATCH);
    }

    // ---- Sub-word: a covered byte forwards, a straddling half replays -----
    {
        Lsq lsq(4, 4);
        const uint32_t s = *lsq.alloc_store(0, 4);     // sw 0x100 = 0x44332211
        lsq.resolve_addr(s, 0x100);
        lsq.resolve_data(s, 0x44332211);

        const uint32_t b0 = *lsq.alloc_load(1, 1);     // lb 0x100
        lsq.resolve_load_addr(b0, 0x100);
        REQUIRE(lsq.search_older_stores(b0).data == 0x11u);

        const uint32_t b2 = *lsq.alloc_load(2, 1);     // lb 0x102
        lsq.resolve_load_addr(b2, 0x102);
        const ForwardResult r2 = lsq.search_older_stores(b2);
        REQUIRE(r2.kind == Forward::FORWARD);
        REQUIRE(r2.data == 0x33u);

        const uint32_t h = *lsq.alloc_load(3, 2);      // lh 0x102, still covered
        lsq.resolve_load_addr(h, 0x102);
        REQUIRE(lsq.search_older_stores(h).data == 0x4433u);
    }

    // ---- Partial overlap is never stitched together -----------------------
    // A one-byte store under a four-byte load covers some of it, so the load
    // waits for the store to commit rather than guessing at the rest.
    {
        Lsq lsq(4, 4);
        const uint32_t s = *lsq.alloc_store(0, 1);
        lsq.resolve_addr(s, 0x102);
        lsq.resolve_data(s, 0xFF);

        const uint32_t l = *lsq.alloc_load(1, 4);
        lsq.resolve_load_addr(l, 0x100);
        REQUIRE(lsq.search_older_stores(l).kind == Forward::REPLAY);
    }

    // ---- The youngest covering store wins, and shadows older ambiguity ----
    {
        Lsq lsq(4, 4);
        const uint32_t s0 = *lsq.alloc_store(0, 4);    // address unknown
        const uint32_t s1 = *lsq.alloc_store(1, 4);
        lsq.resolve_addr(s1, 0x100);
        lsq.resolve_data(s1, 0xAAAA);

        const uint32_t l = *lsq.alloc_load(2, 4);
        lsq.resolve_load_addr(l, 0x100);

        // s1 defines every byte the load wants, whatever s0 turns out to be.
        const ForwardResult r = lsq.search_older_stores(l);
        REQUIRE(r.kind == Forward::FORWARD);
        REQUIRE(r.data == 0xAAAAu);

        // Reversed: the unknown one is younger, so it could still land on top.
        Lsq other(4, 4);
        const uint32_t t0 = *other.alloc_store(0, 4);
        other.resolve_addr(t0, 0x100);
        other.resolve_data(t0, 0xBBBB);
        other.alloc_store(1, 4);                       // unresolved, younger
        const uint32_t l2 = *other.alloc_load(2, 4);
        other.resolve_load_addr(l2, 0x100);
        REQUIRE(other.search_older_stores(l2).kind == Forward::REPLAY);
        (void)s0;
    }

    // ---- Recovery drops the entries that never happened -------------------
    {
        Lsq lsq(8, 8);
        for (SeqNum s = 0; s < 5; ++s) lsq.alloc_load(s, 4);
        for (SeqNum s = 0; s < 5; ++s) lsq.alloc_store(s + 10, 4);
        lsq.squash_after(2);
        REQUIRE(lsq.loads().size() == 3);
        REQUIRE(lsq.stores().empty());                 // every store was younger

        lsq.clear();
        REQUIRE(lsq.loads().empty());
    }
}

// ------------------------------------------------ @section("load_forward") ---
SECTION("load_forward") {
    using namespace asmc;

    // ---- A load reads a store that has not committed yet ------------------
    // Memory still holds zero when the load produces its value, so the only
    // place the answer could have come from is the store queue.
    {
        Config cfg;
        cfg.width = 1;
        Assembler p;
        p.li(t0, 0x400);
        p.li(t1, 0x123);
        p.sw(t1, t0, 0);
        p.lw(a0, t0, 0);
        p.li(a7, 93);
        p.ecall();
        Memory m = cputest::image(p.assemble());
        Cpu cpu(m, cfg, wl::TEXT);
        REQUIRE(cpu.run(1000));

        REQUIRE(cpu.exit_code() == 0x123u);
        REQUIRE(cpu.stats().load_forwards == 1);
        REQUIRE(cpu.stats().load_memory == 0);         // memory never consulted
    }

    // ---- Forwarding does not pay memory latency ---------------------------
    // The same program at four different memory latencies takes exactly as
    // long, because the load never goes there.
    {
        auto cycles_at = [](uint32_t lat) {
            Config cfg;
            cfg.width       = 1;
            cfg.mem_latency = lat;
            Assembler p;
            p.li(t0, 0x400);
            p.li(t1, 7);
            p.sw(t1, t0, 0);
            p.lw(a0, t0, 0);
            p.add(a1, a0, a0);
            p.li(a7, 93);
            p.ecall();
            Memory m = cputest::image(p.assemble());
            Cpu cpu(m, cfg, wl::TEXT);
            cpu.run(1000);
            return cpu.cycle();
        };
        REQUIRE(cycles_at(2) == cycles_at(8));
    }

    // ---- An unresolved older store makes the load wait, then finish -------
    // The store's address depends on a 20-cycle divide, so the load replays
    // until the ambiguity clears, then reads the correct bytes.
    {
        Config cfg;
        cfg.width = 1;
        Assembler p;
        p.li(t0, 0x400);
        p.li(t1, 8);
        p.li(t2, 2);
        p.div_(t3, t1, t2);          // 4, after 20 cycles
        p.add(t4, t0, t3);           // address 0x404, unknown until then
        p.li(t5, 0x99);
        p.sw(t5, t4, 0);             // store to it
        p.lw(a0, t0, 4);             // same address, younger
        p.li(a7, 93);
        p.ecall();
        Memory m = cputest::image(p.assemble());
        Cpu cpu(m, cfg, wl::TEXT);
        REQUIRE(cpu.run(1000));

        REQUIRE(cpu.exit_code() == 0x99u);
        REQUIRE(cpu.stats().load_replays > 0);
        REQUIRE(cpu.stats().load_forwards == 1);
    }

    // ---- A load that no store covers goes to memory -----------------------
    {
        Config cfg;
        cfg.width = 1;
        Assembler p;
        p.li(t0, 0x400);
        p.li(t1, 0x55);
        p.sw(t1, t0, 0);
        p.lw(a0, t0, 16);            // a different word entirely
        p.li(a7, 93);
        p.ecall();
        Memory m = cputest::image(p.assemble());
        Cpu cpu(m, cfg, wl::TEXT);
        REQUIRE(cpu.run(1000));

        REQUIRE(cpu.exit_code() == 0);
        REQUIRE(cpu.stats().load_memory == 1);
        REQUIRE(cpu.stats().load_forwards == 0);
    }

    // ---- Sub-word forwarding through the pipeline -------------------------
    {
        Config cfg;
        cfg.width = 1;
        Assembler p;
        p.li(t0, 0x400);
        p.li(t1, 0x44332211);
        p.sw(t1, t0, 0);
        p.lbu(a0, t0, 2);            // the third byte of the pending store
        p.li(a7, 93);
        p.ecall();
        Memory m = cputest::image(p.assemble());
        Cpu cpu(m, cfg, wl::TEXT);
        REQUIRE(cpu.run(1000));

        REQUIRE(cpu.exit_code() == 0x33u);
        REQUIRE(cpu.stats().load_forwards == 1);
    }

    // ---- A full load queue stalls dispatch --------------------------------
    {
        Config cfg;
        cfg.width       = 1;
        cfg.lq_size     = 1;
        cfg.mem_latency = 6;
        Assembler p;
        p.li(t0, 0x400);
        for (int i = 0; i < 8; ++i) p.lw(t1, t0, 4 * i);
        p.li(a7, 93);
        p.ecall();
        Memory m = cputest::image(p.assemble());
        Cpu cpu(m, cfg, wl::TEXT);
        REQUIRE(cpu.run(2000));

        REQUIRE(cpu.halted());
        REQUIRE(cpu.stats().stall_count(Stall::LQ_FULL) > 0);
    }
}

// ------------------------------------------------- @section("store_commit") ---
SECTION("store_commit") {
    using namespace asmc;

    // ---- Nothing reaches memory before its store commits ------------------
    {
        Config cfg;
        cfg.width = 1;
        Assembler p;
        p.li(t0, 0x400);
        p.li(t1, 1);  p.sw(t1, t0, 0);
        p.li(t2, 2);  p.sw(t2, t0, 4);
        p.li(t3, 3);  p.sw(t3, t0, 8);
        p.li(a7, 93);
        p.ecall();
        Memory m = cputest::image(p.assemble());
        Cpu cpu(m, cfg, wl::TEXT);

        // The set of written words is a prefix of the program's stores at
        // every point, never just at the end.
        while (!cpu.done() && cpu.cycle() < 200) {
            cpu.tick();
            uint32_t written = 0;
            while (written < 3 && m.load_u32(0x400 + 4 * written) != 0) ++written;
            for (uint32_t k = written; k < 3; ++k) {
                REQUIRE(m.load_u32(0x400 + 4 * k) == 0);
            }
            // A store is either still queued or already in memory, never both.
            REQUIRE(written + cpu.lsq().stores().size() <= 3);
        }
        REQUIRE(m.load_u32(0x400) == 1);
        REQUIRE(m.load_u32(0x404) == 2);
        REQUIRE(m.load_u32(0x408) == 3);
        REQUIRE(cpu.lsq().stores().empty());
    }

    // ---- A store under a branch that is never taken leaves memory alone ---
    {
        Config cfg;
        cfg.width = 1;
        Assembler p;
        p.li(t0, 0x400);
        p.li(t1, 0xFF);
        p.li(t2, 1);
        p.beq(t2, zero, "skip");     // not taken
        p.j("done");
        p.label("skip");
        p.sw(t1, t0, 0);             // only on the untaken path
        p.label("done");
        p.li(a7, 93);
        p.ecall();
        Memory m = cputest::image(p.assemble());
        Cpu cpu(m, cfg, wl::TEXT);
        REQUIRE(cpu.run(1000));

        REQUIRE(cpu.halted());
        REQUIRE(m.load_u32(0x400) == 0);
        REQUIRE(cpu.stats().stores == 0);
    }

    // ---- Store-heavy workloads still match the interpreter ----------------
    {
        Config cfg;
        diff::ScopedModel swap(&cputest::run_cpu);
        for (const char* name : {"store_forward", "subword", "bubble_sort",
                                 "pointer_chase", "matmul"}) {
            for (const wl::Workload& w : wl::corpus()) {
                if (w.name != name) continue;
                const diff::Report r = diff::diff_run(w, cfg);
                REQUIRE_MSG(r.ok, r.detail);
            }
        }
    }

    // ---- Forwarding is doing real work on the corpus ----------------------
    // If the count were zero the whole path would be untested by the sweep.
    {
        Config cfg;
        for (const wl::Workload& w : wl::corpus()) {
            if (w.name != "store_forward") continue;
            Memory m = cputest::image(w.words);
            Cpu cpu(m, cfg, wl::TEXT);
            REQUIRE(cpu.run(w.budget * 8 + 1000));
            REQUIRE(cpu.stats().load_forwards > 0);
        }
    }
}

// ---------------------------------------------------- @section("load_use") ---
SECTION("load_use") {
    using namespace asmc;

    // ---- Load-use latency tracks mem_latency exactly ----------------------
    // Nothing stored to this address, so the load has to go to memory and
    // pays the full latency.
    {
        auto cycles_at = [](uint32_t lat) {
            Config cfg;
            cfg.width       = 1;
            cfg.mem_latency = lat;
            Assembler p;
            p.li(t0, 0x2000);
            p.lw(a0, t0, 0);
            p.add(a1, a0, a0);                       // consumes the loaded value
            p.li(a7, 93);
            p.ecall();
            Memory m = cputest::image(p.assemble());
            Cpu cpu(m, cfg, wl::TEXT);
            cpu.run(1000);
            return cpu.cycle();
        };
        REQUIRE(cycles_at(3) == cycles_at(2) + 1);
        REQUIRE(cycles_at(4) == cycles_at(2) + 2);
        REQUIRE(cycles_at(8) == cycles_at(2) + 6);
    }

    // ---- Signed and unsigned sub-word loads through the pipeline ----------
    // The same bytes read back four ways; sign extension is decided by the
    // opcode, not by the memory system.
    {
        Config cfg;
        Assembler p;
        p.li(t0, 0x400);
        p.li(t1, -3);                                // 0xFFFFFFFD
        p.sb(t1, t0, 0);
        p.lb (a1, t0, 0);                            // -3, sign-extended
        p.lbu(a2, t0, 0);                            // 253
        p.li(t2, -300);                              // 0xFFFFFED4
        p.sh(t2, t0, 4);
        p.lh (a3, t0, 4);                            // -300
        p.lhu(a4, t0, 4);                            // 65236
        p.li(a7, 93);
        p.li(a0, 0);
        p.ecall();
        Memory m = cputest::image(p.assemble());
        Cpu cpu(m, cfg, wl::TEXT);
        REQUIRE(cpu.run(1000));

        REQUIRE(cpu.halted());
        REQUIRE(cpu.reg(a1) == static_cast<uint32_t>(-3));
        REQUIRE(cpu.reg(a2) == 253u);
        REQUIRE(cpu.reg(a3) == static_cast<uint32_t>(-300));
        REQUIRE(cpu.reg(a4) == 65236u);
    }
}
