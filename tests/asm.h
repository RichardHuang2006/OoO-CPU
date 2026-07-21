// A tiny RV32IM assembler used to build test programs in-process, so the test
// suite does not need a RISC-V cross toolchain.
#pragma once
#include <cassert>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace rvasm {

// Register aliases.
enum : int {
    x0 = 0, ra = 1, sp = 2, gp = 3, tp = 4, t0 = 5, t1 = 6, t2 = 7,
    s0 = 8, s1 = 9, a0 = 10, a1 = 11, a2 = 12, a3 = 13, a4 = 14, a5 = 15,
    a6 = 16, a7 = 17, s2 = 18, s3 = 19, s4 = 20, s5 = 21, s6 = 22, s7 = 23,
    s8 = 24, s9 = 25, s10 = 26, s11 = 27, t3 = 28, t4 = 29, t5 = 30, t6 = 31,
};

inline uint32_t r_type(uint32_t f7, int rs2, int rs1, uint32_t f3, int rd, uint32_t op) {
    return (f7 << 25) | ((uint32_t)rs2 << 20) | ((uint32_t)rs1 << 15) |
           (f3 << 12) | ((uint32_t)rd << 7) | op;
}
inline uint32_t i_type(int32_t imm, int rs1, uint32_t f3, int rd, uint32_t op) {
    return (((uint32_t)imm & 0xfff) << 20) | ((uint32_t)rs1 << 15) |
           (f3 << 12) | ((uint32_t)rd << 7) | op;
}
inline uint32_t s_type(int32_t imm, int rs2, int rs1, uint32_t f3, uint32_t op) {
    const uint32_t u = (uint32_t)imm;
    return (((u >> 5) & 0x7f) << 25) | ((uint32_t)rs2 << 20) | ((uint32_t)rs1 << 15) |
           (f3 << 12) | ((u & 0x1f) << 7) | op;
}
inline uint32_t b_type(int32_t imm, int rs2, int rs1, uint32_t f3, uint32_t op) {
    const uint32_t u = (uint32_t)imm;
    return (((u >> 12) & 1) << 31) | (((u >> 5) & 0x3f) << 25) |
           ((uint32_t)rs2 << 20) | ((uint32_t)rs1 << 15) | (f3 << 12) |
           (((u >> 1) & 0xf) << 8) | (((u >> 11) & 1) << 7) | op;
}
inline uint32_t u_type(int32_t imm, int rd, uint32_t op) {
    return ((uint32_t)imm & 0xfffff000u) | ((uint32_t)rd << 7) | op;
}
inline uint32_t j_type(int32_t imm, int rd, uint32_t op) {
    const uint32_t u = (uint32_t)imm;
    return (((u >> 20) & 1) << 31) | (((u >> 1) & 0x3ff) << 21) |
           (((u >> 11) & 1) << 20) | (((u >> 12) & 0xff) << 12) |
           ((uint32_t)rd << 7) | op;
}

// Assembles into a word vector, patching forward label references at the end.
class Asm {
public:
    explicit Asm(uint32_t base = 0x1000) : base_(base) {}

    uint32_t base() const { return base_; }
    uint32_t pc_of(size_t idx) const { return base_ + (uint32_t)idx * 4; }
    uint32_t here() const { return pc_of(code_.size()); }

    void label(const std::string& n) { labels_[n] = here(); }

    const std::vector<uint32_t>& code() {
        patch();
        return code_;
    }

    // ---- U / J ----
    void lui(int rd, int32_t imm)   { emit(u_type(imm, rd, 0x37)); }
    void auipc(int rd, int32_t imm) { emit(u_type(imm, rd, 0x17)); }
    void jal(int rd, const std::string& l)  { fix(l, 'j', rd); }
    void jalr(int rd, int rs1, int32_t imm) { emit(i_type(imm, rs1, 0, rd, 0x67)); }
    void j(const std::string& l) { jal(x0, l); }
    void call(const std::string& l) { jal(ra, l); }
    void ret() { jalr(x0, ra, 0); }

    // ---- branches ----
    void beq(int a, int b, const std::string& l)  { fixb(l, a, b, 0); }
    void bne(int a, int b, const std::string& l)  { fixb(l, a, b, 1); }
    void blt(int a, int b, const std::string& l)  { fixb(l, a, b, 4); }
    void bge(int a, int b, const std::string& l)  { fixb(l, a, b, 5); }
    void bltu(int a, int b, const std::string& l) { fixb(l, a, b, 6); }
    void bgeu(int a, int b, const std::string& l) { fixb(l, a, b, 7); }

    // ---- loads / stores ----
    void lb(int rd, int rs1, int32_t o)  { emit(i_type(o, rs1, 0, rd, 0x03)); }
    void lh(int rd, int rs1, int32_t o)  { emit(i_type(o, rs1, 1, rd, 0x03)); }
    void lw(int rd, int rs1, int32_t o)  { emit(i_type(o, rs1, 2, rd, 0x03)); }
    void lbu(int rd, int rs1, int32_t o) { emit(i_type(o, rs1, 4, rd, 0x03)); }
    void lhu(int rd, int rs1, int32_t o) { emit(i_type(o, rs1, 5, rd, 0x03)); }
    void sb(int rs2, int rs1, int32_t o) { emit(s_type(o, rs2, rs1, 0, 0x23)); }
    void sh(int rs2, int rs1, int32_t o) { emit(s_type(o, rs2, rs1, 1, 0x23)); }
    void sw(int rs2, int rs1, int32_t o) { emit(s_type(o, rs2, rs1, 2, 0x23)); }

    // ---- OP-IMM ----
    void addi(int rd, int rs1, int32_t i)  { emit(i_type(i, rs1, 0, rd, 0x13)); }
    void slti(int rd, int rs1, int32_t i)  { emit(i_type(i, rs1, 2, rd, 0x13)); }
    void sltiu(int rd, int rs1, int32_t i) { emit(i_type(i, rs1, 3, rd, 0x13)); }
    void xori(int rd, int rs1, int32_t i)  { emit(i_type(i, rs1, 4, rd, 0x13)); }
    void ori(int rd, int rs1, int32_t i)   { emit(i_type(i, rs1, 6, rd, 0x13)); }
    void andi(int rd, int rs1, int32_t i)  { emit(i_type(i, rs1, 7, rd, 0x13)); }
    void slli(int rd, int rs1, int sh)     { emit(r_type(0x00, sh, rs1, 1, rd, 0x13)); }
    void srli(int rd, int rs1, int sh)     { emit(r_type(0x00, sh, rs1, 5, rd, 0x13)); }
    void srai(int rd, int rs1, int sh)     { emit(r_type(0x20, sh, rs1, 5, rd, 0x13)); }
    void nop() { addi(x0, x0, 0); }
    void mv(int rd, int rs) { addi(rd, rs, 0); }
    void li(int rd, int32_t v) {   // 2-instruction materialization
        if (v >= -2048 && v < 2048) { addi(rd, x0, v); return; }
        const int32_t hi = (v + 0x800) & ~0xfff;
        lui(rd, hi);
        if (v - hi) addi(rd, rd, v - hi);
    }

    // ---- OP ----
    void add(int rd, int a, int b)  { emit(r_type(0x00, b, a, 0, rd, 0x33)); }
    void sub(int rd, int a, int b)  { emit(r_type(0x20, b, a, 0, rd, 0x33)); }
    void sll(int rd, int a, int b)  { emit(r_type(0x00, b, a, 1, rd, 0x33)); }
    void slt(int rd, int a, int b)  { emit(r_type(0x00, b, a, 2, rd, 0x33)); }
    void sltu(int rd, int a, int b) { emit(r_type(0x00, b, a, 3, rd, 0x33)); }
    void xor_(int rd, int a, int b) { emit(r_type(0x00, b, a, 4, rd, 0x33)); }
    void srl(int rd, int a, int b)  { emit(r_type(0x00, b, a, 5, rd, 0x33)); }
    void sra(int rd, int a, int b)  { emit(r_type(0x20, b, a, 5, rd, 0x33)); }
    void or_(int rd, int a, int b)  { emit(r_type(0x00, b, a, 6, rd, 0x33)); }
    void and_(int rd, int a, int b) { emit(r_type(0x00, b, a, 7, rd, 0x33)); }

    // ---- M ----
    void mul(int rd, int a, int b)   { emit(r_type(0x01, b, a, 0, rd, 0x33)); }
    void mulh(int rd, int a, int b)  { emit(r_type(0x01, b, a, 1, rd, 0x33)); }
    void mulhu(int rd, int a, int b) { emit(r_type(0x01, b, a, 3, rd, 0x33)); }
    void div(int rd, int a, int b)   { emit(r_type(0x01, b, a, 4, rd, 0x33)); }
    void divu(int rd, int a, int b)  { emit(r_type(0x01, b, a, 5, rd, 0x33)); }
    void rem(int rd, int a, int b)   { emit(r_type(0x01, b, a, 6, rd, 0x33)); }

    // ---- system ----
    void fence() { emit(0x0ff0000f); }
    void ecall() { emit(0x00000073); }
    void ebreak() { emit(0x00100073); }
    void exit_(int32_t code) { li(a0, code); li(a7, 93); ecall(); }

private:
    void emit(uint32_t w) { code_.push_back(w); }

    void fix(const std::string& l, char type, int rd) {
        fixups_.push_back({code_.size(), l, type, rd, 0, 0, 0});
        emit(0);
    }
    void fixb(const std::string& l, int rs1, int rs2, uint32_t f3) {
        fixups_.push_back({code_.size(), l, 'b', 0, rs1, rs2, f3});
        emit(0);
    }

    void patch() {
        for (const auto& f : fixups_) {
            auto it = labels_.find(f.label);
            assert(it != labels_.end() && "undefined label");
            const int32_t off = (int32_t)(it->second - pc_of(f.idx));
            if (f.type == 'j') code_[f.idx] = j_type(off, f.rd, 0x6f);
            else               code_[f.idx] = b_type(off, f.rs2, f.rs1, f.f3, 0x63);
        }
        fixups_.clear();
    }

    struct Fixup {
        size_t idx; std::string label; char type;
        int rd; int rs1; int rs2; uint32_t f3;
    };

    uint32_t base_;
    std::vector<uint32_t> code_;
    std::map<std::string, uint32_t> labels_;
    std::vector<Fixup> fixups_;
};

} // namespace rvasm
