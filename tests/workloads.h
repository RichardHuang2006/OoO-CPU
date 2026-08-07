#pragma once

// The validation corpus, shared by the test suite and the example generator.

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "asm.h"

// ================================================================ workloads ===
// The validation corpus: fourteen programs run through both the pipeline
// model and ref.h, covering ALU semantics, loops, arrays, store forwarding,
// sub-word and partial-overlap accesses, nested calls, recursive Fibonacci,
// an unpredictable branch, mul/div with divide-by-zero, a pointer chase, a
// WAW/WAR renaming stress, and a bitwise CRC-32.
//
// Every expected exit code is derived independently of the simulator: by hand
// where the arithmetic is simple, by an equivalent C++ loop where it is not.
// A value read off a previous run would assert nothing.

namespace wl {

inline constexpr uint32_t TEXT  = 0x1000;    // program image
inline constexpr uint32_t DATA  = 0x4000;    // scratch the programs initialize
inline constexpr uint32_t STACK = 0x8000;    // grows down, away from DATA

struct Workload {
    std::string             name;
    std::vector<uint32_t>   words;
    std::optional<uint32_t> expect_exit;   // hand-computed, not observed
    uint64_t                budget = 200000;
};

using asmc::Assembler;
using namespace asmc;   // register aliases: a0, t0, s0, sp, ra, zero, ...

// exit(a0) — every workload leaves its result in a0 and ends here.
inline void exit_now(Assembler& p) {
    p.li(a7, 93);
    p.ecall();
}

// ---- 1. ALU coverage ------------------------------------------------------
inline Workload alu() {
    Assembler p;
    p.li(t0, 12);
    p.li(t1, 5);
    p.li(t2, -1);
    p.add  (a1, t0, t1);          // 17
    p.sub  (a2, t0, t1);          // 7
    p.sll  (a3, t0, t1);          // 384
    p.srl  (a4, t0, t1);          // 0
    p.sra  (a5, t2, t1);          // -1, sign-filled
    p.and_ (s2, t0, t1);          // 4
    p.or_  (s3, t0, t1);          // 13
    p.xor_ (s4, t0, t1);          // 9
    p.slt  (s5, t2, t1);          // 1
    p.sltu (s6, t2, t1);          // 0
    p.lui  (s7, 0x12345);
    p.auipc(s8, 0);
    p.addi (s9,  t0, -20);        // -8
    p.slti (s10, t2, 0);          // 1
    p.xori (s11, t0, 0xFF);       // 243
    p.srai (s1,  t2, 3);          // -1
    p.slli (s0,  t1, 2);          // 20
    p.add  (a0, a1, a2);          // 17 + 7
    exit_now(p);
    return {"alu", p.assemble(), 24, 200000};
}

// ---- 2. Loop --------------------------------------------------------------
inline Workload loop() {
    Assembler p;
    p.li(a0, 0);
    p.li(a1, 1);
    p.li(a2, 101);
    p.label("loop");
    p.beq(a1, a2, "done");
    p.add(a0, a0, a1);
    p.addi(a1, a1, 1);
    p.j("loop");
    p.label("done");
    exit_now(p);
    return {"loop", p.assemble(), 5050, 200000};   // sum 1..100
}

// ---- 3. Arrays: bubble sort -----------------------------------------------
// Sorts a permutation of 1..12, verifies the result is ordered, and exits
// with a[0] * 100 + a[11] — 112 when sorted, 999 if any pair is out of order.
inline Workload bubble_sort() {
    const int32_t input[12] = {9, 4, 7, 1, 12, 3, 8, 2, 11, 5, 10, 6};

    Assembler p;
    p.li(s0, static_cast<int32_t>(DATA));
    for (int i = 0; i < 12; ++i) {
        p.li(t0, input[i]);
        p.sw(t0, s0, i * 4);
    }
    p.li(s1, 12);
    p.li(t0, 0);                              // i
    p.label("outer");
    p.addi(t1, s1, -1);
    p.bge(t0, t1, "outer_done");
    p.li(t2, 0);                              // j
    p.sub(t3, t1, t0);                        // limit = n - 1 - i
    p.label("inner");
    p.bge(t2, t3, "inner_done");
    p.slli(t4, t2, 2);
    p.add(t4, s0, t4);                        // &a[j]
    p.lw(t5, t4, 0);
    p.lw(t6, t4, 4);
    p.bge(t6, t5, "no_swap");
    p.sw(t6, t4, 0);
    p.sw(t5, t4, 4);
    p.label("no_swap");
    p.addi(t2, t2, 1);
    p.j("inner");
    p.label("inner_done");
    p.addi(t0, t0, 1);
    p.j("outer");
    p.label("outer_done");

    p.li(a0, 0);
    p.li(t0, 0);
    p.label("check");
    p.addi(t1, s1, -1);
    p.bge(t0, t1, "check_done");
    p.slli(t2, t0, 2);
    p.add(t2, s0, t2);
    p.lw(t3, t2, 0);
    p.lw(t4, t2, 4);
    p.bge(t4, t3, "in_order");
    p.li(a0, 999);
    p.j("check_done");
    p.label("in_order");
    p.addi(t0, t0, 1);
    p.j("check");
    p.label("check_done");
    p.bne(a0, zero, "finish");
    p.lw(t3, s0, 0);                          // a[0]
    p.lw(t5, s0, 44);                         // a[11]
    p.li(t6, 100);
    p.mul(a0, t3, t6);
    p.add(a0, a0, t5);
    p.label("finish");
    exit_now(p);
    return {"bubble_sort", p.assemble(), 112, 200000};
}

// ---- 4. Arrays: 4x4 integer matmul ----------------------------------------
// A[i][j] = i + j, B[i][j] = 1 + (i == j), C = A·B, exit = sum of C.
// C[i][k] = Σ_j (i+j)(1 + [j==k]) = (4i + 6) + (i + k) = 5i + k + 6, so the
// total is Σ_i Σ_k (5i + k + 6) = 240.
inline Workload matmul() {
    Assembler p;
    p.li(s0, static_cast<int32_t>(DATA));         // A
    p.li(s1, static_cast<int32_t>(DATA + 64));    // B
    p.li(s2, static_cast<int32_t>(DATA + 128));   // C
    p.li(s3, 4);                                  // n

    p.li(t0, 0);                                  // i
    p.label("fill_i");
    p.bge(t0, s3, "fill_done");
    p.li(t1, 0);                                  // j
    p.label("fill_j");
    p.bge(t1, s3, "fill_i_next");
    p.mul(t2, t0, s3);
    p.add(t2, t2, t1);
    p.slli(t2, t2, 2);                            // (i*n + j) * 4
    p.add(t3, s0, t2);
    p.add(t4, t0, t1);
    p.sw(t4, t3, 0);                              // A[i][j] = i + j
    p.add(t3, s1, t2);
    p.li(t5, 1);
    p.bne(t0, t1, "store_b");
    p.li(t5, 2);
    p.label("store_b");
    p.sw(t5, t3, 0);                              // B[i][j] = 1 + (i == j)
    p.addi(t1, t1, 1);
    p.j("fill_j");
    p.label("fill_i_next");
    p.addi(t0, t0, 1);
    p.j("fill_i");
    p.label("fill_done");

    p.li(a0, 0);
    p.li(t0, 0);                                  // i
    p.label("mm_i");
    p.bge(t0, s3, "mm_done");
    p.li(t1, 0);                                  // k
    p.label("mm_k");
    p.bge(t1, s3, "mm_i_next");
    p.li(t2, 0);                                  // j
    p.li(s4, 0);                                  // accumulator
    p.label("mm_j");
    p.bge(t2, s3, "mm_store");
    p.mul(t3, t0, s3);
    p.add(t3, t3, t2);
    p.slli(t3, t3, 2);
    p.add(t3, s0, t3);
    p.lw(t4, t3, 0);                              // A[i][j]
    p.mul(t5, t2, s3);
    p.add(t5, t5, t1);
    p.slli(t5, t5, 2);
    p.add(t5, s1, t5);
    p.lw(t6, t5, 0);                              // B[j][k]
    p.mul(t4, t4, t6);
    p.add(s4, s4, t4);
    p.addi(t2, t2, 1);
    p.j("mm_j");
    p.label("mm_store");
    p.mul(t3, t0, s3);
    p.add(t3, t3, t1);
    p.slli(t3, t3, 2);
    p.add(t3, s2, t3);
    p.sw(s4, t3, 0);                              // C[i][k]
    p.add(a0, a0, s4);
    p.addi(t1, t1, 1);
    p.j("mm_k");
    p.label("mm_i_next");
    p.addi(t0, t0, 1);
    p.j("mm_i");
    p.label("mm_done");
    exit_now(p);
    return {"matmul", p.assemble(), 240, 200000};
}

// ---- 5. Arrays: sieve of Eratosthenes -------------------------------------
inline Workload sieve() {
    Assembler p;
    p.li(s0, static_cast<int32_t>(DATA));
    p.li(s1, 100);                            // n

    p.li(t0, 0);
    p.label("zero");
    p.bge(t0, s1, "zero_done");
    p.add(t1, s0, t0);
    p.sb(zero, t1, 0);
    p.addi(t0, t0, 1);
    p.j("zero");
    p.label("zero_done");

    p.li(t0, 2);
    p.label("sv_i");
    p.mul(t1, t0, t0);
    p.bge(t1, s1, "sv_done");                 // i*i >= n → every composite marked
    p.add(t2, s0, t0);
    p.lbu(t3, t2, 0);
    p.bne(t3, zero, "sv_next");               // i is composite; skip
    p.mv(t4, t1);
    p.label("mark");
    p.bge(t4, s1, "sv_next");
    p.add(t5, s0, t4);
    p.li(t6, 1);
    p.sb(t6, t5, 0);
    p.add(t4, t4, t0);
    p.j("mark");
    p.label("sv_next");
    p.addi(t0, t0, 1);
    p.j("sv_i");
    p.label("sv_done");

    p.li(a0, 0);
    p.li(t0, 2);
    p.label("count");
    p.bge(t0, s1, "count_done");
    p.add(t1, s0, t0);
    p.lbu(t2, t1, 0);
    p.bne(t2, zero, "count_next");
    p.addi(a0, a0, 1);
    p.label("count_next");
    p.addi(t0, t0, 1);
    p.j("count");
    p.label("count_done");
    exit_now(p);
    return {"sieve", p.assemble(), 25, 200000};   // 25 primes below 100
}

// ---- 6. Store-to-load forwarding ------------------------------------------
// Twenty dependent store→load pairs at one address, each feeding the next.
// The pipeline forwards them out of the store queue; the reference just sees
// memory, which is the disagreement worth testing for.
inline Workload store_forward() {
    Assembler p;
    p.li(s0, static_cast<int32_t>(DATA));
    p.li(s1, 20);
    p.li(t0, 0);                              // value
    p.li(t1, 0);                              // i
    p.label("sf");
    p.bge(t1, s1, "sf_done");
    p.sw(t0, s0, 0);
    p.lw(t2, s0, 0);                          // same address → forwards
    p.addi(t0, t2, 3);
    p.sw(t0, s0, 4);                          // different address
    p.lw(t3, s0, 0);                          // must not see the +4 store
    p.addi(t1, t1, 1);
    p.j("sf");
    p.label("sf_done");
    p.mv(a0, t0);
    exit_now(p);
    return {"store_forward", p.assemble(), 60, 200000};   // 3 per iteration
}

// ---- 7. Sub-word and partial-overlap accesses -----------------------------
// Each access partially overlaps an earlier store, the case a store queue
// has to answer with a replay rather than a forward.
inline Workload subword() {
    Assembler p;
    p.li(s0, static_cast<int32_t>(DATA));
    p.li(t0, 0x12345678);
    p.sw(t0, s0, 0);                          // bytes: 78 56 34 12
    p.lbu(a1, s0, 1);                         // 0x56
    p.lbu(a2, s0, 3);                         // 0x12
    p.li(t1, 0xABCD);
    p.sh(t1, s0, 2);                          // word becomes 0xABCD5678
    p.lw(a3, s0, 0);
    p.li(t2, -1);
    p.sb(t2, s0, 0);                          // word becomes 0xABCD56FF
    p.lw(a4, s0, 0);
    p.lb(a5, s0, 0);                          // -1, sign-extended
    p.lhu(a6, s0, 0);                         // 0x56FF
    p.li(t3, 0x0F0F0F0F);
    p.sw(t3, s0, 6);                          // misaligned, straddles the above
    p.lw(t4, s0, 4);                          // reads across two stores
    p.lhu(t5, s0, 6);                         // 0x0F0F
    p.andi(t6, a4, 0xFF);                     // 0xFF
    p.add(a0, a1, a2);
    p.add(a0, a0, t6);
    p.add(a0, a0, t5);
    exit_now(p);
    // 0x56 + 0x12 + 0xFF + 0x0F0F = 86 + 18 + 255 + 3855
    return {"subword", p.assemble(), 4214, 200000};
}

// ---- 8. Nested calls ------------------------------------------------------
// A four-deep call chain, ten times over: the return-address stack at
// several depths, plus ra save/restore.
inline Workload nested_calls() {
    Assembler p;
    p.li(sp, static_cast<int32_t>(STACK));
    p.li(a0, 0);
    p.li(s0, 10);
    p.li(s1, 0);
    p.label("nc_loop");
    p.bge(s1, s0, "nc_done");
    p.call("f1");
    p.addi(s1, s1, 1);
    p.j("nc_loop");
    p.label("nc_done");
    exit_now(p);

    // f1 → f2 → f3 → f4, adding 4, 3, 2, 1 on the way back out.
    const struct { const char* self; const char* callee; int32_t add; } frames[] = {
        {"f1", "f2", 4},
        {"f2", "f3", 3},
        {"f3", "f4", 2},
    };
    for (const auto& f : frames) {
        p.label(f.self);
        p.addi(sp, sp, -8);
        p.sw(ra, sp, 4);
        p.call(f.callee);
        p.addi(a0, a0, f.add);
        p.lw(ra, sp, 4);
        p.addi(sp, sp, 8);
        p.ret_();
    }
    p.label("f4");
    p.addi(a0, a0, 1);
    p.ret_();

    return {"nested_calls", p.assemble(), 100, 200000};   // 10 × (1+2+3+4)
}

// ---- 9. Recursive Fibonacci -----------------------------------------------
inline Workload fib() {
    Assembler p;
    p.li(sp, static_cast<int32_t>(STACK));
    p.li(a0, 12);
    p.call("fib");
    exit_now(p);

    p.label("fib");
    p.addi(sp, sp, -16);
    p.sw(ra, sp, 12);
    p.sw(s0, sp, 8);
    p.sw(s1, sp, 4);
    p.li(t0, 2);
    p.blt(a0, t0, "fib_done");                // fib(0) = 0, fib(1) = 1
    p.mv(s0, a0);
    p.addi(a0, s0, -1);
    p.call("fib");
    p.mv(s1, a0);
    p.addi(a0, s0, -2);
    p.call("fib");
    p.add(a0, a0, s1);
    p.label("fib_done");
    p.lw(ra, sp, 12);
    p.lw(s0, sp, 8);
    p.lw(s1, sp, 4);
    p.addi(sp, sp, 16);
    p.ret_();

    return {"fib", p.assemble(), 144, 2000000};   // fib(12)
}

// ---- 10. Unpredictable branch ---------------------------------------------
// A branch driven by a bit of an LCG stream, so no history pattern predicts
// it. The expected count comes from the same recurrence in C++.
inline Workload lcg_branch() {
    constexpr uint32_t SEED = 12345, MUL = 1103515245, INC = 12345, ITERS = 200;

    uint32_t x = SEED, taken = 0;
    for (uint32_t i = 0; i < ITERS; ++i) {
        x = x * MUL + INC;
        taken += (x >> 16) & 1u;
    }

    Assembler p;
    p.li(s0, static_cast<int32_t>(ITERS));
    p.li(s1, 0);                              // i
    p.li(s2, static_cast<int32_t>(SEED));     // x
    p.li(s3, static_cast<int32_t>(MUL));
    p.li(s4, static_cast<int32_t>(INC));
    p.li(a0, 0);                              // count
    p.label("lcg");
    p.bge(s1, s0, "lcg_done");
    p.mul(s2, s2, s3);
    p.add(s2, s2, s4);
    p.srli(t0, s2, 16);
    p.andi(t0, t0, 1);
    p.beq(t0, zero, "not_taken");
    p.addi(a0, a0, 1);
    p.label("not_taken");
    p.addi(s1, s1, 1);
    p.j("lcg");
    p.label("lcg_done");
    exit_now(p);
    return {"lcg_branch", p.assemble(), taken, 200000};
}

// ---- 11. mul / div, including divide-by-zero ------------------------------
inline Workload muldiv() {
    uint32_t acc = 0;
    for (uint32_t i = 1; i <= 20; ++i) acc += 1000u / i + 1000u % i;
    acc += 0xFFFFFFFFu;                       // the divide-by-zero result, -1

    Assembler p;
    p.li(a1, -1);
    p.li(a2, 10);
    p.li(a3, 0);
    p.li(a4, static_cast<int32_t>(0x80000000));

    p.div_(t0, a4, a1);                       // INT_MIN / -1 → INT_MIN, no trap
    p.rem (t1, a4, a1);                       // → 0
    p.div_(t2, a2, a3);                       // x / 0 → -1
    p.divu(t3, a2, a3);
    p.rem (t4, a2, a3);                       // x % 0 → x
    p.remu(t5, a2, a3);
    p.mulh  (a5, a1, a1);
    p.mulhu (a6, a1, a1);
    p.mulhsu(a7, a1, a2);

    p.li(s0, 1);                              // i
    p.li(s1, 21);
    p.li(s2, 0);                              // acc
    p.li(s3, 1000);
    p.label("md");
    p.bge(s0, s1, "md_done");
    p.div_(t0, s3, s0);
    p.rem (t1, s3, s0);
    p.add(s2, s2, t0);
    p.add(s2, s2, t1);
    p.addi(s0, s0, 1);
    p.j("md");
    p.label("md_done");
    p.div_(t2, s3, a3);                       // divide by zero once more
    p.add(s2, s2, t2);
    p.mv(a0, s2);
    exit_now(p);
    return {"muldiv", p.assemble(), acc, 200000};
}

// ---- 12. Pointer chase ----------------------------------------------------
// 32 two-word nodes in a stride-7 cycle (7 is coprime with 32, so the cycle
// covers every node). The traversal is a chain of dependent loads, which no
// amount of issue width can accelerate.
inline Workload pointer_chase() {
    Assembler p;
    p.li(s0, static_cast<int32_t>(DATA));
    p.li(s1, 32);

    p.li(t0, 0);
    p.label("build");
    p.bge(t0, s1, "build_done");
    p.slli(t1, t0, 3);
    p.add(t1, s0, t1);                        // &node[i]
    p.addi(t2, t0, 1);
    p.sw(t2, t1, 0);                          // node[i].value = i + 1
    p.addi(t3, t0, 7);
    p.andi(t3, t3, 31);
    p.slli(t3, t3, 3);
    p.add(t3, s0, t3);
    p.sw(t3, t1, 4);                          // node[i].next = &node[(i+7) % 32]
    p.addi(t0, t0, 1);
    p.j("build");
    p.label("build_done");

    p.li(a0, 0);
    p.mv(t0, s0);                             // p = &node[0]
    p.li(t1, 0);
    p.label("chase");
    p.bge(t1, s1, "chase_done");
    p.lw(t2, t0, 0);
    p.add(a0, a0, t2);
    p.lw(t0, t0, 4);                          // p = p->next
    p.addi(t1, t1, 1);
    p.j("chase");
    p.label("chase_done");
    exit_now(p);
    return {"pointer_chase", p.assemble(), 528, 200000};   // sum 1..32
}

// ---- 13. WAW / WAR renaming stress ----------------------------------------
// t0 is written five times per iteration and read in between. Each write
// must land in a distinct physical register or a later reader sees the
// wrong value.
inline Workload waw_war() {
    Assembler p;
    p.li(a0, 0);
    p.li(s0, 10);
    p.li(s1, 0);
    p.label("w_loop");
    p.bge(s1, s0, "w_done");
    p.li(t0, 1);
    p.add(a0, a0, t0);                        // +1
    p.li(t0, 2);                              // WAW
    p.add(a0, a0, t0);                        // +2
    p.li(t0, 3);                              // WAW
    p.add(a0, a0, t0);                        // +3
    p.mv(t1, t0);                             // read t0 ...
    p.li(t0, 4);                              // ... then WAR over it
    p.add(a0, a0, t1);                        // +3 (the pre-WAR value)
    p.add(a0, a0, t0);                        // +4
    p.addi(s1, s1, 1);
    p.j("w_loop");
    p.label("w_done");
    exit_now(p);
    return {"waw_war", p.assemble(), 130, 200000};   // 10 × 13
}

// ---- 14. CRC-32 -----------------------------------------------------------
// Bitwise CRC-32 (reflected, polynomial 0xEDB88320) over the 256 bytes 0..255.
// The inner loop branches on one data-dependent bit per iteration, which no
// history length can predict, so this is the corpus' mispredict case. The
// expected value is zlib's, which makes it an ISA check against an outside
// authority rather than against ref.h.
inline Workload crc32() {
    Assembler p;
    p.li(s0, static_cast<int32_t>(DATA));

    p.li(t0, 0);
    p.li(s2, 256);
    p.label("fill");
    p.bge(t0, s2, "fill_done");
    p.add(t1, s0, t0);
    p.sb(t0, t1, 0);                          // buf[i] = i
    p.addi(t0, t0, 1);
    p.j("fill");
    p.label("fill_done");

    p.li(a0, -1);                             // crc = 0xFFFFFFFF
    p.li(s3, static_cast<int32_t>(0xEDB88320));
    p.li(s1, 0);
    p.label("byte");
    p.bge(s1, s2, "byte_done");
    p.add(t0, s0, s1);
    p.lbu(t1, t0, 0);
    p.xor_(a0, a0, t1);
    p.li(t2, 0);
    p.label("bit");
    p.li(t3, 8);
    p.bge(t2, t3, "bit_done");
    p.andi(t4, a0, 1);                        // the unpredictable bit
    p.srli(a0, a0, 1);
    p.beq(t4, zero, "no_xor");
    p.xor_(a0, a0, s3);
    p.label("no_xor");
    p.addi(t2, t2, 1);
    p.j("bit");
    p.label("bit_done");
    p.addi(s1, s1, 1);
    p.j("byte");
    p.label("byte_done");

    p.li(t0, -1);
    p.xor_(a0, a0, t0);                       // final inversion
    exit_now(p);
    return {"crc32", p.assemble(), 0x29058C73u, 200000};
}

inline const std::vector<Workload>& corpus() {
    static const std::vector<Workload> c = [] {
        std::vector<Workload> v;
        v.push_back(alu());
        v.push_back(loop());
        v.push_back(bubble_sort());
        v.push_back(matmul());
        v.push_back(sieve());
        v.push_back(store_forward());
        v.push_back(subword());
        v.push_back(nested_calls());
        v.push_back(fib());
        v.push_back(lcg_branch());
        v.push_back(muldiv());
        v.push_back(pointer_chase());
        v.push_back(waw_war());
        v.push_back(crc32());
        return v;
    }();
    return c;
}

}  // namespace wl
