#pragma once

// Header-only RV32IM assembler for the tests, so no external riscv-toolchain
// is needed. Covers the whole ISA plus the pseudos li / mv / nop / j / jr /
// ret / call, resolves labels in a second pass, and emits a word vector.

#include <cstdint>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

namespace asmc {

// ---- RISC-V ABI register aliases ------------------------------------------
constexpr uint32_t zero = 0,  ra = 1,  sp = 2,  gp = 3,  tp = 4;
constexpr uint32_t t0   = 5,  t1 = 6,  t2 = 7;
constexpr uint32_t s0   = 8,  fp = 8,  s1 = 9;
constexpr uint32_t a0   = 10, a1 = 11, a2 = 12, a3 = 13;
constexpr uint32_t a4   = 14, a5 = 15, a6 = 16, a7 = 17;
constexpr uint32_t s2   = 18, s3 = 19, s4 = 20, s5 = 21;
constexpr uint32_t s6   = 22, s7 = 23, s8 = 24, s9 = 25, s10 = 26, s11 = 27;
constexpr uint32_t t3   = 28, t4 = 29, t5 = 30, t6 = 31;

class Assembler {
public:
    // ---- Placement -------------------------------------------------------
    void label(std::string_view name) {
        labels_[std::string(name)] = words_.size();
    }
    uint32_t pc() const { return static_cast<uint32_t>(words_.size() * 4); }

    // Resolve every fixup and return the finished word stream. Idempotent.
    std::vector<uint32_t> assemble() {
        for (const Fixup& f : fixups_) resolve(f);
        return words_;
    }

    // ---- U-type ----------------------------------------------------------
    void lui  (uint32_t rd, uint32_t imm20) { emit(enc_U(0x37, rd, imm20)); }
    void auipc(uint32_t rd, uint32_t imm20) { emit(enc_U(0x17, rd, imm20)); }

    // ---- J-type ----------------------------------------------------------
    // Absolute-offset form; use the label overload for everything else.
    void jal(uint32_t rd, int32_t imm) { emit(enc_J(rd, imm)); }
    void jal(uint32_t rd, std::string_view lbl) {
        add_fixup(FixKind::J, lbl);
        emit(enc_J(rd, 0));                             // placeholder
    }

    // ---- I-type (jump) --------------------------------------------------
    void jalr(uint32_t rd, uint32_t rs1, int32_t imm) {
        emit(enc_I(0x67, rd, 0x0, rs1, imm));
    }

    // ---- B-type ----------------------------------------------------------
    void beq (uint32_t r1, uint32_t r2, std::string_view lbl) { branch(0x0, r1, r2, lbl); }
    void bne (uint32_t r1, uint32_t r2, std::string_view lbl) { branch(0x1, r1, r2, lbl); }
    void blt (uint32_t r1, uint32_t r2, std::string_view lbl) { branch(0x4, r1, r2, lbl); }
    void bge (uint32_t r1, uint32_t r2, std::string_view lbl) { branch(0x5, r1, r2, lbl); }
    void bltu(uint32_t r1, uint32_t r2, std::string_view lbl) { branch(0x6, r1, r2, lbl); }
    void bgeu(uint32_t r1, uint32_t r2, std::string_view lbl) { branch(0x7, r1, r2, lbl); }

    // ---- Loads (dst, base, offset) --------------------------------------
    void lb (uint32_t rd, uint32_t rs1, int32_t off) { emit(enc_I(0x03, rd, 0x0, rs1, off)); }
    void lh (uint32_t rd, uint32_t rs1, int32_t off) { emit(enc_I(0x03, rd, 0x1, rs1, off)); }
    void lw (uint32_t rd, uint32_t rs1, int32_t off) { emit(enc_I(0x03, rd, 0x2, rs1, off)); }
    void lbu(uint32_t rd, uint32_t rs1, int32_t off) { emit(enc_I(0x03, rd, 0x4, rs1, off)); }
    void lhu(uint32_t rd, uint32_t rs1, int32_t off) { emit(enc_I(0x03, rd, 0x5, rs1, off)); }

    // ---- Stores (src, base, offset) — matches the encoding order --------
    void sb(uint32_t rs2, uint32_t rs1, int32_t off) { emit(enc_S(0x0, rs1, rs2, off)); }
    void sh(uint32_t rs2, uint32_t rs1, int32_t off) { emit(enc_S(0x1, rs1, rs2, off)); }
    void sw(uint32_t rs2, uint32_t rs1, int32_t off) { emit(enc_S(0x2, rs1, rs2, off)); }

    // ---- I-type ALU (immediate) -----------------------------------------
    void addi (uint32_t rd, uint32_t rs1, int32_t imm) { emit(enc_I(0x13, rd, 0x0, rs1, imm)); }
    void slti (uint32_t rd, uint32_t rs1, int32_t imm) { emit(enc_I(0x13, rd, 0x2, rs1, imm)); }
    void sltiu(uint32_t rd, uint32_t rs1, int32_t imm) { emit(enc_I(0x13, rd, 0x3, rs1, imm)); }
    void xori (uint32_t rd, uint32_t rs1, int32_t imm) { emit(enc_I(0x13, rd, 0x4, rs1, imm)); }
    void ori  (uint32_t rd, uint32_t rs1, int32_t imm) { emit(enc_I(0x13, rd, 0x6, rs1, imm)); }
    void andi (uint32_t rd, uint32_t rs1, int32_t imm) { emit(enc_I(0x13, rd, 0x7, rs1, imm)); }
    void slli (uint32_t rd, uint32_t rs1, uint32_t sh) { emit(enc_R(0x13, rd, 0x1, rs1, sh & 0x1F, 0x00)); }
    void srli (uint32_t rd, uint32_t rs1, uint32_t sh) { emit(enc_R(0x13, rd, 0x5, rs1, sh & 0x1F, 0x00)); }
    void srai (uint32_t rd, uint32_t rs1, uint32_t sh) { emit(enc_R(0x13, rd, 0x5, rs1, sh & 0x1F, 0x20)); }

    // ---- R-type ALU ------------------------------------------------------
    void add (uint32_t rd, uint32_t rs1, uint32_t rs2) { emit(enc_R(0x33, rd, 0x0, rs1, rs2, 0x00)); }
    void sub (uint32_t rd, uint32_t rs1, uint32_t rs2) { emit(enc_R(0x33, rd, 0x0, rs1, rs2, 0x20)); }
    void sll (uint32_t rd, uint32_t rs1, uint32_t rs2) { emit(enc_R(0x33, rd, 0x1, rs1, rs2, 0x00)); }
    void slt (uint32_t rd, uint32_t rs1, uint32_t rs2) { emit(enc_R(0x33, rd, 0x2, rs1, rs2, 0x00)); }
    void sltu(uint32_t rd, uint32_t rs1, uint32_t rs2) { emit(enc_R(0x33, rd, 0x3, rs1, rs2, 0x00)); }
    void xor_(uint32_t rd, uint32_t rs1, uint32_t rs2) { emit(enc_R(0x33, rd, 0x4, rs1, rs2, 0x00)); }
    void srl (uint32_t rd, uint32_t rs1, uint32_t rs2) { emit(enc_R(0x33, rd, 0x5, rs1, rs2, 0x00)); }
    void sra (uint32_t rd, uint32_t rs1, uint32_t rs2) { emit(enc_R(0x33, rd, 0x5, rs1, rs2, 0x20)); }
    void or_ (uint32_t rd, uint32_t rs1, uint32_t rs2) { emit(enc_R(0x33, rd, 0x6, rs1, rs2, 0x00)); }
    void and_(uint32_t rd, uint32_t rs1, uint32_t rs2) { emit(enc_R(0x33, rd, 0x7, rs1, rs2, 0x00)); }

    // ---- MISC-MEM / SYSTEM ----------------------------------------------
    void fence()   { emit(0x0000000Fu); }               // pred=succ=0
    void fence_i() { emit(0x0000100Fu); }
    void ecall()   { emit(0x00000073u); }
    void ebreak()  { emit(0x00100073u); }

    // ---- M extension -----------------------------------------------------
    void mul   (uint32_t rd, uint32_t rs1, uint32_t rs2) { emit(enc_R(0x33, rd, 0x0, rs1, rs2, 0x01)); }
    void mulh  (uint32_t rd, uint32_t rs1, uint32_t rs2) { emit(enc_R(0x33, rd, 0x1, rs1, rs2, 0x01)); }
    void mulhsu(uint32_t rd, uint32_t rs1, uint32_t rs2) { emit(enc_R(0x33, rd, 0x2, rs1, rs2, 0x01)); }
    void mulhu (uint32_t rd, uint32_t rs1, uint32_t rs2) { emit(enc_R(0x33, rd, 0x3, rs1, rs2, 0x01)); }
    void div_  (uint32_t rd, uint32_t rs1, uint32_t rs2) { emit(enc_R(0x33, rd, 0x4, rs1, rs2, 0x01)); }
    void divu  (uint32_t rd, uint32_t rs1, uint32_t rs2) { emit(enc_R(0x33, rd, 0x5, rs1, rs2, 0x01)); }
    void rem   (uint32_t rd, uint32_t rs1, uint32_t rs2) { emit(enc_R(0x33, rd, 0x6, rs1, rs2, 0x01)); }
    void remu  (uint32_t rd, uint32_t rs1, uint32_t rs2) { emit(enc_R(0x33, rd, 0x7, rs1, rs2, 0x01)); }

    // ---- Pseudoinstructions ---------------------------------------------
    void nop()                                { addi(0, 0, 0); }
    void mv (uint32_t rd, uint32_t rs)        { addi(rd, rs, 0); }
    void j  (std::string_view lbl)            { jal(0, lbl); }
    void jr (uint32_t rs)                     { jalr(0, rs, 0); }
    void ret_()                               { jalr(0, 1, 0); }
    void li (uint32_t rd, int32_t imm) {
        if (imm >= -2048 && imm <= 2047) { addi(rd, 0, imm); return; }
        // Split with lower-12 sign compensation so lo12 lands in [-2048, 2047].
        const int32_t  hi = static_cast<int32_t>(
                                (static_cast<uint32_t>(imm) + 0x800u) >> 12);
        const uint32_t hi20 = static_cast<uint32_t>(hi) & 0xFFFFFu;
        const int32_t  lo12 = imm - static_cast<int32_t>(hi20 << 12);
        lui (rd, hi20);
        addi(rd, rd, lo12);
    }
    void call(std::string_view lbl) {
        // auipc ra + jalr ra; the fixup patches both once the label resolves.
        add_fixup(FixKind::CALL, lbl);
        emit(enc_U(0x17, 1, 0));                      // placeholder auipc ra
        emit(enc_I(0x67, 1, 0x0, 1, 0));              // placeholder jalr ra
    }

private:
    enum class FixKind { B, J, CALL };
    struct Fixup {
        std::size_t word_idx;
        std::string name;
        FixKind     kind;
    };

    std::vector<uint32_t>                          words_;
    std::unordered_map<std::string, std::size_t>   labels_;
    std::vector<Fixup>                             fixups_;

    // ---- primitives -----------------------------------------------------
    void emit(uint32_t w) { words_.push_back(w); }

    void add_fixup(FixKind k, std::string_view name) {
        fixups_.push_back({words_.size(), std::string(name), k});
    }

    void branch(uint32_t f3, uint32_t r1, uint32_t r2, std::string_view lbl) {
        add_fixup(FixKind::B, lbl);
        emit(enc_B(f3, r1, r2, 0));                    // placeholder
    }

    void resolve(const Fixup& f) {
        auto it = labels_.find(f.name);
        if (it == labels_.end()) {
            throw std::runtime_error("asmc: undefined label '" + f.name + "'");
        }
        const int64_t site   = static_cast<int64_t>(f.word_idx) * 4;
        const int64_t target = static_cast<int64_t>(it->second) * 4;
        const int64_t off    = target - site;

        switch (f.kind) {
        case FixKind::B: {
            if (off < -4096 || off > 4094 || (off & 1)) {
                throw std::runtime_error("asmc: branch offset unencodable for '" + f.name + "'");
            }
            const uint32_t orig = words_[f.word_idx];
            const uint32_t stripped = orig & ~kBmask;
            words_[f.word_idx] = stripped | b_imm_bits(static_cast<int32_t>(off));
            break;
        }
        case FixKind::J: {
            if (off < -1048576 || off > 1048574 || (off & 1)) {
                throw std::runtime_error("asmc: jal offset unencodable for '" + f.name + "'");
            }
            const uint32_t orig = words_[f.word_idx];
            const uint32_t stripped = orig & ~kJmask;
            words_[f.word_idx] = stripped | j_imm_bits(static_cast<int32_t>(off));
            break;
        }
        case FixKind::CALL: {
            // hi and the sign-extended lo must sum to `off`, same trick as li.
            const int32_t off32 = static_cast<int32_t>(off);
            const uint32_t hi20 = (static_cast<uint32_t>(off32) + 0x800u) >> 12;
            const int32_t  lo12 = off32 - static_cast<int32_t>(hi20 << 12);

            // Rewrite the auipc word with hi20 (rd already = ra in placeholder).
            uint32_t auipc_word = words_[f.word_idx] & ~0xFFFFF000u;
            auipc_word |= ((hi20 & 0xFFFFF) << 12);
            words_[f.word_idx] = auipc_word;

            // Rewrite the jalr word with lo12.
            uint32_t jalr_word = words_[f.word_idx + 1] & ~0xFFF00000u;
            jalr_word |= ((static_cast<uint32_t>(lo12) & 0xFFF) << 20);
            words_[f.word_idx + 1] = jalr_word;
            break;
        }
        }
    }

    // ---- format encoders ------------------------------------------------
    static constexpr uint32_t enc_R(uint32_t op, uint32_t rd, uint32_t f3,
                                    uint32_t rs1, uint32_t rs2, uint32_t f7) {
        return (f7 << 25) | ((rs2 & 0x1F) << 20) | ((rs1 & 0x1F) << 15) |
               ((f3 & 0x7) << 12) | ((rd & 0x1F) << 7) | (op & 0x7F);
    }
    static constexpr uint32_t enc_I(uint32_t op, uint32_t rd, uint32_t f3,
                                    uint32_t rs1, int32_t imm) {
        const uint32_t u = static_cast<uint32_t>(imm) & 0xFFF;
        return (u << 20) | ((rs1 & 0x1F) << 15) | ((f3 & 0x7) << 12) |
               ((rd & 0x1F) << 7) | (op & 0x7F);
    }
    static constexpr uint32_t enc_S(uint32_t f3, uint32_t rs1, uint32_t rs2, int32_t imm) {
        const uint32_t u  = static_cast<uint32_t>(imm) & 0xFFF;
        const uint32_t hi = (u >> 5) & 0x7F;
        const uint32_t lo = u & 0x1F;
        return (hi << 25) | ((rs2 & 0x1F) << 20) | ((rs1 & 0x1F) << 15) |
               ((f3 & 0x7) << 12) | (lo << 7) | 0x23;
    }
    static constexpr uint32_t enc_B(uint32_t f3, uint32_t rs1, uint32_t rs2, int32_t imm) {
        return 0x63 | ((f3 & 0x7) << 12) | ((rs1 & 0x1F) << 15) |
               ((rs2 & 0x1F) << 20) | b_imm_bits(imm);
    }
    static constexpr uint32_t enc_U(uint32_t op, uint32_t rd, uint32_t imm_hi20) {
        return ((imm_hi20 & 0xFFFFF) << 12) | ((rd & 0x1F) << 7) | (op & 0x7F);
    }
    static constexpr uint32_t enc_J(uint32_t rd, int32_t imm) {
        return 0x6F | ((rd & 0x1F) << 7) | j_imm_bits(imm);
    }

    // Pack B/J immediates, leaving other fields alone so a fixup can strip
    // and re-OR them.
    static constexpr uint32_t kBmask = (1u << 31) | (0x3Fu << 25) | (0xFu << 8) | (1u << 7);
    static constexpr uint32_t b_imm_bits(int32_t imm) {
        const uint32_t u = static_cast<uint32_t>(imm) & 0x1FFF;
        return (((u >> 12) & 1) << 31) | (((u >> 5) & 0x3F) << 25) |
               (((u >> 1) & 0xF) << 8) | (((u >> 11) & 1) << 7);
    }
    static constexpr uint32_t kJmask = 0xFFFFF000u;
    static constexpr uint32_t j_imm_bits(int32_t imm) {
        const uint32_t u = static_cast<uint32_t>(imm) & 0x1FFFFF;
        return (((u >> 20) & 1) << 31) | (((u >> 1) & 0x3FF) << 21) |
               (((u >> 11) & 1) << 20) | (((u >> 12) & 0xFF) << 12);
    }
};

}  // namespace asmc
