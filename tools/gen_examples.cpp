// Emits the example workloads in examples/ as hex-word files, so the
// simulator can be exercised without a RISC-V cross toolchain.
//   build/gen-examples examples/
#include <cstdio>
#include <string>
#include <vector>

#include "../tests/asm.h"

using namespace rvasm;

namespace {

constexpr uint32_t kBase = 0x1000;
constexpr uint32_t kHeap = 0x40000;

// Sieve of Eratosthenes over [2, N); returns the prime count in a0.
std::vector<uint32_t> sieve(int n) {
    Asm a(kBase);
    a.li(s0, kHeap);
    a.li(s1, n);

    a.li(t0, 0);                       // zero the flag array
    a.label("clear");
    a.slli(t1, t0, 2);
    a.add(t1, s0, t1);
    a.sw(x0, t1, 0);
    a.addi(t0, t0, 1);
    a.blt(t0, s1, "clear");

    a.li(t0, 2);                       // for i in 2..n
    a.label("outer");
    a.slli(t1, t0, 2);
    a.add(t1, s0, t1);
    a.lw(t2, t1, 0);
    a.bne(t2, x0, "next_i");
    a.add(t3, t0, t0);                 // j = 2i
    a.label("inner");
    a.bge(t3, s1, "next_i");
    a.slli(t4, t3, 2);
    a.add(t4, s0, t4);
    a.li(t5, 1);
    a.sw(t5, t4, 0);
    a.add(t3, t3, t0);
    a.j("inner");
    a.label("next_i");
    a.addi(t0, t0, 1);
    a.blt(t0, s1, "outer");

    a.li(a0, 0);                       // count the survivors
    a.li(t0, 2);
    a.label("count");
    a.slli(t1, t0, 2);
    a.add(t1, s0, t1);
    a.lw(t2, t1, 0);
    a.bne(t2, x0, "skip");
    a.addi(a0, a0, 1);
    a.label("skip");
    a.addi(t0, t0, 1);
    a.blt(t0, s1, "count");

    a.li(a7, 93);
    a.ecall();
    return a.code();
}

// N x N integer matrix multiply; returns the trace of the product in a0.
std::vector<uint32_t> matmul(int n) {
    const uint32_t A = kHeap, B = kHeap + 0x4000, C = kHeap + 0x8000;
    Asm a(kBase);
    a.li(s0, n);

    // A[i][j] = i + j, B[i][j] = i - j
    a.li(t0, 0);
    a.label("fill_i");
    a.li(t1, 0);
    a.label("fill_j");
    a.mul(t2, t0, s0);
    a.add(t2, t2, t1);
    a.slli(t2, t2, 2);
    a.li(t3, A);
    a.add(t3, t3, t2);
    a.add(t4, t0, t1);
    a.sw(t4, t3, 0);
    a.li(t3, B);
    a.add(t3, t3, t2);
    a.sub(t4, t0, t1);
    a.sw(t4, t3, 0);
    a.addi(t1, t1, 1);
    a.blt(t1, s0, "fill_j");
    a.addi(t0, t0, 1);
    a.blt(t0, s0, "fill_i");

    // C = A * B
    a.li(s1, 0);                       // i
    a.label("i_loop");
    a.li(s2, 0);                       // j
    a.label("j_loop");
    a.li(s3, 0);                       // k
    a.li(s4, 0);                       // acc
    a.label("k_loop");
    a.mul(t0, s1, s0);                 // &A[i][k]
    a.add(t0, t0, s3);
    a.slli(t0, t0, 2);
    a.li(t1, A);
    a.add(t0, t0, t1);
    a.lw(t2, t0, 0);
    a.mul(t0, s3, s0);                 // &B[k][j]
    a.add(t0, t0, s2);
    a.slli(t0, t0, 2);
    a.li(t1, B);
    a.add(t0, t0, t1);
    a.lw(t3, t0, 0);
    a.mul(t2, t2, t3);
    a.add(s4, s4, t2);
    a.addi(s3, s3, 1);
    a.blt(s3, s0, "k_loop");
    a.mul(t0, s1, s0);                 // C[i][j] = acc
    a.add(t0, t0, s2);
    a.slli(t0, t0, 2);
    a.li(t1, C);
    a.add(t0, t0, t1);
    a.sw(s4, t0, 0);
    a.addi(s2, s2, 1);
    a.blt(s2, s0, "j_loop");
    a.addi(s1, s1, 1);
    a.blt(s1, s0, "i_loop");

    a.li(a0, 0);                       // trace(C)
    a.li(s1, 0);
    a.label("tr");
    a.mul(t0, s1, s0);
    a.add(t0, t0, s1);
    a.slli(t0, t0, 2);
    a.li(t1, C);
    a.add(t0, t0, t1);
    a.lw(t2, t0, 0);
    a.add(a0, a0, t2);
    a.addi(s1, s1, 1);
    a.blt(s1, s0, "tr");

    a.li(a7, 93);
    a.ecall();
    return a.code();
}

// Bubble sort of a pseudo-random array; returns the number of swaps in a0.
std::vector<uint32_t> bubble_sort(int n) {
    Asm a(kBase);
    a.li(s0, kHeap);
    a.li(s1, n);

    a.li(t0, 0);                       // fill with an LCG sequence
    a.li(t5, 22695477);
    a.li(t6, 1);
    a.li(t4, 12345);
    a.label("fill");
    a.mul(t6, t6, t5);
    a.add(t6, t6, t4);
    a.srli(t1, t6, 20);
    a.slli(t2, t0, 2);
    a.add(t2, s0, t2);
    a.sw(t1, t2, 0);
    a.addi(t0, t0, 1);
    a.blt(t0, s1, "fill");

    a.li(a0, 0);                       // swap count
    a.li(t0, 0);                       // i
    a.label("outer");
    a.li(t1, 0);                       // j
    a.addi(t3, s1, -1);
    a.sub(t3, t3, t0);                 // bound = n-1-i
    a.label("inner");
    a.bge(t1, t3, "next_i");
    a.slli(t2, t1, 2);
    a.add(t2, s0, t2);
    a.lw(t4, t2, 0);
    a.lw(t5, t2, 4);
    a.bge(t5, t4, "no_swap");          // data-dependent, hard to predict
    a.sw(t5, t2, 0);
    a.sw(t4, t2, 4);
    a.addi(a0, a0, 1);
    a.label("no_swap");
    a.addi(t1, t1, 1);
    a.j("inner");
    a.label("next_i");
    a.addi(t0, t0, 1);
    a.addi(t2, s1, -1);
    a.blt(t0, t2, "outer");

    a.li(a7, 93);
    a.ecall();
    return a.code();
}

// Recursive Fibonacci: call/return heavy, exercises the RAS.
std::vector<uint32_t> fib(int n) {
    Asm a(kBase);
    a.li(sp, 0x80000);
    a.li(a0, n);
    a.call("fib");
    a.li(a7, 93);
    a.ecall();

    a.label("fib");
    a.li(t0, 2);
    a.bge(a0, t0, "recurse");
    a.ret();
    a.label("recurse");
    a.addi(sp, sp, -12);
    a.sw(ra, sp, 0);
    a.sw(a0, sp, 4);
    a.addi(a0, a0, -1);
    a.call("fib");
    a.sw(a0, sp, 8);
    a.lw(a0, sp, 4);
    a.addi(a0, a0, -2);
    a.call("fib");
    a.lw(t1, sp, 8);
    a.add(a0, a0, t1);
    a.lw(ra, sp, 0);
    a.addi(sp, sp, 12);
    a.ret();
    return a.code();
}

// CRC-32 over a generated buffer: a tight dependent loop with a table lookup.
std::vector<uint32_t> crc32(int bytes) {
    Asm a(kBase);
    a.li(s0, kHeap);
    a.li(s1, bytes);

    a.li(t0, 0);
    a.label("fill");
    a.andi(t1, t0, 0xff);
    a.add(t2, s0, t0);
    a.sb(t1, t2, 0);
    a.addi(t0, t0, 1);
    a.blt(t0, s1, "fill");

    a.li(a0, -1);                      // crc = 0xffffffff
    a.li(t0, 0);
    a.li(s2, 0xedb88320u);
    a.label("byte_loop");
    a.add(t1, s0, t0);
    a.lbu(t2, t1, 0);
    a.xor_(a0, a0, t2);
    a.li(t3, 8);
    a.label("bit_loop");
    a.andi(t4, a0, 1);
    a.srli(a0, a0, 1);
    a.beq(t4, x0, "no_xor");
    a.xor_(a0, a0, s2);
    a.label("no_xor");
    a.addi(t3, t3, -1);
    a.bne(t3, x0, "bit_loop");
    a.addi(t0, t0, 1);
    a.blt(t0, s1, "byte_loop");
    a.xori(a0, a0, -1);

    a.li(a7, 93);
    a.ecall();
    return a.code();
}

bool write_hex(const std::string& path, const std::vector<uint32_t>& code,
               const char* desc) {
    FILE* f = std::fopen(path.c_str(), "w");
    if (!f) { std::fprintf(stderr, "cannot write %s\n", path.c_str()); return false; }
    std::fprintf(f, "# %s\n# load at 0x1000: build/oooc --hex %s\n", desc, path.c_str());
    for (uint32_t w : code) std::fprintf(f, "%08x\n", w);
    std::fclose(f);
    std::printf("wrote %-28s %4zu instructions\n", path.c_str(), code.size());
    return true;
}

} // namespace

int main(int argc, char** argv) {
    const std::string dir = (argc > 1) ? argv[1] : "examples";
    auto p = [&](const char* n) { return dir + "/" + n; };

    bool ok = true;
    ok &= write_hex(p("sieve.hex"), sieve(512),
                    "sieve of eratosthenes up to 512; prime count in a0");
    ok &= write_hex(p("matmul.hex"), matmul(16),
                    "16x16 integer matrix multiply; trace of the product in a0");
    ok &= write_hex(p("bubble_sort.hex"), bubble_sort(64),
                    "bubble sort of 64 pseudo-random words; swap count in a0");
    ok &= write_hex(p("fib.hex"), fib(20),
                    "recursive fib(20); result in a0");
    ok &= write_hex(p("crc32.hex"), crc32(256),
                    "crc-32 of a 256 byte buffer; checksum in a0");
    return ok ? 0 : 1;
}
