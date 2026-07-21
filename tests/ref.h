// Straightforward in-order reference interpreter. The out-of-order core is
// validated by checking that it reaches the same architectural state.
#pragma once
#include <cstdint>
#include <string>

#include "../src/alu.h"
#include "../src/decoder.h"
#include "../src/memory.h"

struct RefResult {
    uint32_t regs[32] = {};
    uint64_t instructions = 0;
    int exit_code = 0;
    std::string halt_reason;
};

inline RefResult run_reference(Memory& mem, uint32_t entry, uint32_t sp,
                               uint64_t max_insts = 100000000) {
    RefResult r;
    r.regs[2] = sp;
    uint32_t pc = entry;

    while (r.instructions < max_insts) {
        const DecodedInst d = decode(mem.read32(pc), pc);
        if (d.illegal) { r.halt_reason = "illegal instruction"; r.exit_code = -1; break; }
        r.instructions++;

        const uint32_t a = r.regs[d.rs1];
        const uint32_t b = r.regs[d.rs2];
        uint32_t next = pc + 4;
        uint32_t result = 0;
        bool writes = d.has_rd;

        if (d.is_system) {
            if (d.op == Op::ECALL && r.regs[17] == 93) {
                r.exit_code = (int)r.regs[10];
                r.halt_reason = "exit syscall";
            } else {
                r.halt_reason = (d.op == Op::EBREAK) ? "ebreak" : "environment call";
            }
            break;
        } else if (d.is_load) {
            result = load_extend(d.op, mem.read(a + (uint32_t)d.imm, d.mem_bytes()));
        } else if (d.is_store) {
            mem.write(a + (uint32_t)d.imm, b, d.mem_bytes());
            writes = false;
        } else if (d.is_branch) {
            if (branch_taken(d.op, a, b)) next = pc + (uint32_t)d.imm;
            writes = false;
        } else if (d.op == Op::JAL) {
            result = pc + 4;
            next = pc + (uint32_t)d.imm;
        } else if (d.op == Op::JALR) {
            result = pc + 4;
            next = (a + (uint32_t)d.imm) & ~1u;
        } else {
            result = alu_execute(d, a, b);
        }

        if (writes && d.rd != 0) r.regs[d.rd] = result;
        r.regs[0] = 0;
        pc = next;
    }
    return r;
}
