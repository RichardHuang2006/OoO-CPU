#pragma once

#include <cstdint>
#include <cstdio>
#include <string>
#include <unordered_map>
#include <vector>

#include "config.h"
#include "cpu.h"
#include "disasm.h"
#include "stats.h"
#include "types.h"

// Newline-delimited JSON, one object per cycle, written straight out as the
// run proceeds. The trace is a photograph of the machine, never an opinion
// about it: every number here is read from a structure the pipeline maintains,
// so a viewer can render a cycle without knowing what a pipeline is and
// without ever having to recompute one.
//
// The first line is a header record — it has no "cycle" key, which is how a
// reader tells it apart — carrying the configuration, so a viewer can show
// occupancy against capacity instead of a bare count.
//
// Two things keep a long trace to a size a browser can hold, neither of which
// costs the reader an inference:
//
//   * Instruction text is a property of the program, not of the cycle. Each PC
//     is disassembled once, into the "disasm" map on the first record that
//     mentions it; entries afterwards carry the PC alone. (A program that
//     rewrites its own code re-registers the PC, and the last spelling wins —
//     the one case where the map is a simplification.)
//   * A false boolean is written by leaving the key out. Absent reads as
//     false, absent numbers read as "not known", and both are what a viewer
//     does with a missing field anyway.
//
// Nothing in here mutates the Cpu: emitting a trace cannot change the run it
// describes, which is the only property that makes a trace worth trusting.

namespace trace_detail {

// The stall keys the trace uses are the enumerator names rather than the CLI's
// prose, because a viewer wants a stable key to colour and group by. The
// static_assert makes a new stall cause a compile error here rather than a
// silently missing bar in the histogram.
inline const char* stall_key(Stall s) {
    switch (s) {
    case Stall::ROB_FULL:    return "ROB_FULL";
    case Stall::PHYSREG:     return "PHYSREG";
    case Stall::CHECKPOINT:  return "CHECKPOINT";
    case Stall::IQ_FULL:     return "IQ_FULL";
    case Stall::LQ_FULL:     return "LQ_FULL";
    case Stall::SQ_FULL:     return "SQ_FULL";
    case Stall::ALU_PORT:    return "ALU_PORT";
    case Stall::BRANCH_PORT: return "BRANCH_PORT";
    case Stall::MUL_PORT:    return "MUL_PORT";
    case Stall::DIV_PORT:    return "DIV_PORT";
    case Stall::MEM_PORT:    return "MEM_PORT";
    case Stall::CDB:         return "CDB";
    case Stall::OPERANDS:    return "OPERANDS";
    case Stall::STORE_ORDER: return "STORE_ORDER";
    case Stall::COUNT:       break;
    }
    return "UNKNOWN";
}
static_assert(static_cast<int>(Stall::COUNT) == 14, "stall_key is missing a cause");

inline const char* kind_name(OpKind k) {
    switch (k) {
    case OpKind::ALU:    return "ALU";
    case OpKind::BRANCH: return "BRANCH";
    case OpKind::MUL:    return "MUL";
    case OpKind::DIV:    return "DIV";
    case OpKind::LOAD:   return "LOAD";
    case OpKind::STORE:  return "STORE";
    case OpKind::NOP:    return "NOP";
    case OpKind::TRAP:   return "TRAP";
    }
    return "?";
}

inline const char* trap_name(TrapCause c) {
    switch (c) {
    case TrapCause::NONE:          return "none";
    case TrapCause::ILLEGAL:       return "illegal";
    case TrapCause::EBREAK:        return "ebreak";
    case TrapCause::ECALL_UNKNOWN: return "ecall_unknown";
    }
    return "?";
}

// A minimal streaming JSON writer: containers track whether they still owe a
// comma, so no caller has to.
class Json {
public:
    explicit Json(std::FILE* f) : f_(f) { first_.push_back(true); }

    void obj_begin() { sep(); std::fputc('{', f_); first_.push_back(true); }
    void obj_end()   { std::fputc('}', f_); first_.pop_back(); }
    void arr_begin() { sep(); std::fputc('[', f_); first_.push_back(true); }
    void arr_end()   { std::fputc(']', f_); first_.pop_back(); }

    // A key owes no comma from its value, so the value writes none.
    void key(const char* k) {
        sep();
        quote(k);
        std::fputc(':', f_);
        first_.back() = true;
    }

    // A numeric key, for maps indexed by an address.
    void key(uint32_t k) {
        sep();
        std::fprintf(f_, "\"%u\":", k);
        first_.back() = true;
    }

    void num(uint64_t v) { sep(); std::fprintf(f_, "%llu", static_cast<unsigned long long>(v)); }
    void num(int64_t v)  { sep(); std::fprintf(f_, "%lld", static_cast<long long>(v)); }
    void num(uint32_t v) { num(static_cast<uint64_t>(v)); }
    void num(int v)      { num(static_cast<int64_t>(v)); }
    void boolean(bool b) { sep(); std::fputs(b ? "true" : "false", f_); }
    void null()          { sep(); std::fputs("null", f_); }
    void str(const std::string& s) { sep(); quote(s.c_str()); }

    // A sentinel is absence, and absence in JSON is null — never a magic
    // 4294967295 the viewer would have to know to recognise.
    void num_or_null(uint32_t v, uint32_t invalid) {
        if (v == invalid) null(); else num(v);
    }

    void kv(const char* k, uint64_t v) { key(k); num(v); }
    void kv(const char* k, uint32_t v) { key(k); num(v); }
    void kv(const char* k, int v)      { key(k); num(v); }
    void kv(const char* k, bool v)     { key(k); boolean(v); }
    void kv(const char* k, const std::string& v) { key(k); str(v); }
    void kv(const char* k, const char* v) { key(k); str(std::string(v)); }

    // A flag that says something only when it is set: false is the absence of
    // the key, which is what a reader falls back to anyway.
    void flag(const char* k, bool v) { if (v) kv(k, true); }

    void line_end() { std::fputc('\n', f_); first_.back() = true; }

private:
    void sep() {
        if (!first_.back()) std::fputc(',', f_);
        first_.back() = false;
    }

    void quote(const char* s) {
        std::fputc('"', f_);
        for (const char* p = s; *p; ++p) {
            const unsigned char c = static_cast<unsigned char>(*p);
            switch (c) {
            case '"':  std::fputs("\\\"", f_); break;
            case '\\': std::fputs("\\\\", f_); break;
            case '\n': std::fputs("\\n", f_);  break;
            case '\r': std::fputs("\\r", f_);  break;
            case '\t': std::fputs("\\t", f_);  break;
            default:
                if (c < 0x20) std::fprintf(f_, "\\u%04X", c);
                else          std::fputc(static_cast<char>(c), f_);
            }
        }
        std::fputc('"', f_);
    }

    std::FILE*        f_;
    std::vector<bool> first_;
};

}  // namespace trace_detail

class CycleTrace {
public:
    explicit CycleTrace(std::FILE* out) : j_(out) {}

    uint64_t records() const { return records_; }

    // Written once, before the first cycle. `entry_pc` is where fetch started,
    // which nothing in a per-cycle record can tell you after the fact.
    void header(const Config& cfg, uint32_t entry_pc, uint64_t cdb_window) {
        using namespace trace_detail;
        j_.obj_begin();
        j_.kv("kind", "header");
        j_.kv("format", 1u);
        j_.kv("entry_pc", entry_pc);
        j_.kv("cdb_window", cdb_window);

        j_.key("config");
        j_.obj_begin();
        j_.kv("width", cfg.width);
        j_.kv("rob_size", cfg.rob_size);
        j_.kv("prf_size", cfg.prf_size);
        j_.kv("iq_size", cfg.iq_size);
        j_.kv("lq_size", cfg.lq_size);
        j_.kv("sq_size", cfg.sq_size);
        j_.kv("num_cdb", cfg.num_cdb);
        j_.kv("num_alu", cfg.num_alu);
        j_.kv("num_branch", cfg.num_branch);
        j_.kv("num_mul", cfg.num_mul);
        j_.kv("num_div", cfg.num_div);
        j_.kv("num_mem", cfg.num_mem);
        j_.kv("alu_latency", cfg.alu_latency);
        j_.kv("branch_latency", cfg.branch_latency);
        j_.kv("mul_latency", cfg.mul_latency);
        j_.kv("div_latency", cfg.div_latency);
        j_.kv("mem_latency", cfg.mem_latency);
        j_.kv("ghr_bits", cfg.ghr_bits);
        j_.kv("pht_size", cfg.pht_size);
        j_.kv("btb_sets", cfg.btb_sets);
        j_.kv("btb_ways", cfg.btb_ways);
        j_.kv("ras_size", cfg.ras_size);
        j_.kv("num_checkpoints", cfg.num_checkpoints);
        j_.obj_end();

        j_.obj_end();
        j_.line_end();
    }

    // One cycle, as it stands at the end of tick().
    void snapshot(const Cpu& cpu) {
        using namespace trace_detail;
        const Rob& rob = cpu.rob();

        collect_disasm(cpu);

        j_.obj_begin();
        j_.kv("cycle", cpu.cycle());
        j_.kv("pc", cpu.fetch_pc());
        j_.kv("arch_pc", cpu.arch_pc());
        j_.kv("halted", cpu.halted());
        if (cpu.trapped()) j_.kv("trap", trap_name(cpu.trap_cause()));
        j_.flag("fetch_stalled", cpu.fetch_stalled());
        emit_disasm();

        emit_stalls(cpu.stats());
        emit_stats(cpu.stats());

        j_.key("fetch_q");  emit_front_end(cpu.fetch_uops());
        j_.key("decode_q"); emit_front_end(cpu.decode_uops());
        j_.key("rename_q"); emit_front_end(cpu.rename_uops());

        emit_iq(cpu);
        emit_rob(cpu);

        // An empty buffer has no head; a slot number would be a fiction.
        j_.key("rob_head"); j_.num_or_null(rob.head(), INVALID_ROBINDEX);
        j_.kv("rob_count", rob.size());
        j_.kv("rob_size", rob.capacity());

        j_.key("executing"); emit_in_flight(cpu.executing_uops());
        j_.key("wb_fast");   emit_in_flight(cpu.wb_fast_uops());
        j_.key("wb_slow");   emit_in_flight(cpu.wb_slow_uops());

        emit_maps(cpu);
        emit_free_list(cpu);
        emit_prf(cpu);
        emit_lsq(cpu);
        emit_fu(cpu);
        emit_cdb(cpu);
        emit_bpred(cpu);

        j_.key("squashed_seqs"); emit_seqs(cpu.squashed_seqs());
        j_.key("retired_seqs");  emit_seqs(cpu.retired_seqs());

        j_.key("events");
        j_.arr_begin();
        for (const std::string& e : cpu.events()) j_.str(e);
        j_.arr_end();

        j_.obj_end();
        j_.line_end();
        ++records_;
    }

private:
    // Decode has not run on a uop still in the fetch queue, so its text comes
    // from the word rather than from a `Decoded` that is not filled in yet.
    static std::string text_of(const Uop& u) {
        return u.at.decode ? disasm(u.dec, u.pc) : disasm(u.raw, u.pc);
    }

    // Every PC this cycle mentions for the first time, ready to be written as
    // the record's "disasm" map. Collected before the record opens, because a
    // streaming writer cannot go back and add a key.
    void collect_disasm(const Cpu& cpu) {
        pending_.clear();
        auto note = [&](const Uop& u) {
            const auto it = seen_.find(u.pc);
            if (it != seen_.end() && it->second == u.raw) return;
            seen_[u.pc] = u.raw;
            pending_.emplace_back(u.pc, text_of(u));
        };

        for (const Uop& u : cpu.fetch_uops())     note(u);
        for (const Uop& u : cpu.decode_uops())    note(u);
        for (const Uop& u : cpu.rename_uops())    note(u);
        for (const Uop& u : cpu.executing_uops()) note(u);
        for (const Uop& u : cpu.wb_fast_uops())   note(u);
        for (const Uop& u : cpu.wb_slow_uops())   note(u);
        for (uint32_t k = 0; k < cpu.rob().size(); ++k) {
            note(cpu.inflight(cpu.rob().nth(k)));
        }
    }

    void emit_disasm() {
        if (pending_.empty()) return;
        j_.key("disasm");
        j_.obj_begin();
        for (const auto& [pc, text] : pending_) {
            j_.key(pc);
            j_.str(text);
        }
        j_.obj_end();
    }

    void emit_stalls(const Stats& s) {
        using namespace trace_detail;
        j_.key("stalls");
        j_.obj_begin();
        for (std::size_t i = 0; i < s.stalls.size(); ++i) {
            j_.kv(stall_key(static_cast<Stall>(i)), s.stalls[i]);
        }
        j_.obj_end();
    }

    void emit_stats(const Stats& s) {
        j_.key("stats");
        j_.obj_begin();
        j_.kv("fetched", s.fetched);
        j_.kv("decoded", s.decoded);
        j_.kv("renamed", s.renamed);
        j_.kv("dispatched", s.dispatched);
        j_.kv("issued", s.issued);
        j_.kv("wrote_back", s.wrote_back);
        j_.kv("retired", s.retired);
        j_.kv("squashed", s.squashed);
        j_.kv("branches", s.branches);
        j_.kv("mispredicts", s.mispredicts);
        j_.kv("loads", s.loads);
        j_.kv("stores", s.stores);
        j_.kv("load_forwards", s.load_forwards);
        j_.kv("load_replays", s.load_replays);
        j_.kv("load_memory", s.load_memory);
        j_.obj_end();
    }

    // Fetch, decode and rename queues. A uop only has a sequence number once
    // rename has stamped one, so before that the trace says null and carries
    // the fetch-order uid instead — the only handle that spans the front end.
    template <typename Q>
    void emit_front_end(const Q& q) {
        j_.arr_begin();
        for (const Uop& u : q) {
            j_.obj_begin();
            j_.key("seq"); j_.num_or_null(u.seq, INVALID_SEQNUM);
            j_.kv("uid", u.uid);
            j_.kv("pc", u.pc);
            j_.flag("pred_taken", u.pred_taken);
            if (u.pred_taken) j_.kv("pred_target", u.pred_target);
            j_.obj_end();
        }
        j_.arr_end();
    }

    template <typename Q>
    void emit_in_flight(const Q& q) {
        j_.arr_begin();
        for (const Uop& u : q) {
            j_.obj_begin();
            j_.kv("seq", u.seq);
            j_.kv("pc", u.pc);
            j_.kv("kind", trace_detail::kind_name(u.dec.kind));
            j_.kv("wb_cycle", u.wb_cycle);
            j_.obj_end();
        }
        j_.arr_end();
    }

    void emit_iq(const Cpu& cpu) {
        j_.key("iq");
        j_.arr_begin();
        for (const IssueQueue::Entry& e : cpu.iq().entries()) {
            const Uop& u = cpu.inflight(e.rob);
            j_.obj_begin();
            j_.kv("seq", e.seq);
            j_.kv("pc", u.pc);
            j_.kv("kind", trace_detail::kind_name(e.kind));
            j_.kv("src1", e.src1);
            j_.kv("src1_ready", e.src1_ready);
            j_.kv("src2", e.src2);
            j_.kv("src2_ready", e.src2_ready);
            j_.key("dest"); j_.num_or_null(e.dest, INVALID_PHYSREG);
            j_.kv("latency", e.latency);
            j_.obj_end();
        }
        j_.arr_end();
    }

    // The reorder buffer, oldest first, so index 0 is the head in both the
    // trace and the machine.
    void emit_rob(const Cpu& cpu) {
        const Rob& rob = cpu.rob();
        j_.key("rob");
        j_.arr_begin();
        for (uint32_t k = 0; k < rob.size(); ++k) {
            const RobIndex   slot = rob.nth(k);
            const RobEntry&  e    = rob.nth_entry(k);
            const Uop&       u    = cpu.inflight(slot);

            j_.obj_begin();
            j_.kv("seq", e.seq);
            j_.kv("uid", u.uid);
            j_.kv("slot", slot);
            j_.kv("pc", e.pc);
            j_.kv("kind", trace_detail::kind_name(u.dec.kind));
            j_.key("dest_arch");  j_.num_or_null(e.dest_arch, INVALID_ARCHREG);
            j_.key("dest_phys");  j_.num_or_null(e.dest_phys, INVALID_PHYSREG);
            j_.key("stale_phys"); j_.num_or_null(e.stale_phys, INVALID_PHYSREG);
            j_.kv("src1", u.src1);
            j_.kv("src2", u.src2);
            j_.kv("complete", e.complete);
            j_.flag("mispredicted", e.mispredicted);
            j_.flag("is_branch", e.is_branch);
            j_.flag("is_store", e.is_store);
            j_.flag("is_load", u.dec.is_load);
            if (e.ckpt != INVALID_CHECKPOINT) j_.kv("ckpt", e.ckpt);
            j_.flag("pred_taken", u.pred_taken);
            if (u.pred_taken) j_.kv("pred_target", u.pred_target);

            // Where it actually goes is only settled at execute; before that
            // the field still holds the fall-through decode filled in, which
            // is not an answer — so the key is simply absent until then.
            if (u.at.issue) {
                j_.kv("next_pc", e.next_pc);
                j_.kv("wb_cycle", u.wb_cycle);
            }
            if (u.dec.is_load || u.dec.is_store) j_.kv("mem_addr", u.mem_addr);

            // [fetch, decode, rename, dispatch, issue, complete], truncated at
            // the stage it has reached. An array because these six stamps ride
            // on every in-flight instruction in every cycle, and six names per
            // entry per cycle is most of a trace.
            j_.key("at");
            j_.arr_begin();
            const uint64_t stamps[6] = {u.at.fetch, u.at.decode, u.at.rename,
                                        u.at.dispatch, u.at.issue, u.at.complete};
            std::size_t last = 0;
            for (std::size_t i = 0; i < 6; ++i) if (stamps[i]) last = i + 1;
            for (std::size_t i = 0; i < last; ++i) {
                if (stamps[i]) j_.num(stamps[i]); else j_.null();
            }
            j_.arr_end();

            j_.obj_end();
        }
        j_.arr_end();
    }

    void emit_maps(const Cpu& cpu) {
        j_.key("rat");
        j_.arr_begin();
        for (ArchReg a = 0; a < 32; ++a) j_.num(cpu.rat().map(a));
        j_.arr_end();

        j_.key("arch_rat");
        j_.arr_begin();
        for (ArchReg a = 0; a < 32; ++a) j_.num(cpu.committed_map(a));
        j_.arr_end();
    }

    void emit_free_list(const Cpu& cpu) {
        j_.key("free_list");
        j_.obj_begin();
        j_.kv("count", cpu.free_list().num_free());
        j_.key("head");
        j_.arr_begin();
        uint32_t n = 0;
        for (const PhysReg p : cpu.free_list().order()) {
            if (n++ >= FREE_LIST_PREVIEW) break;
            j_.num(p);
        }
        j_.arr_end();
        j_.obj_end();
    }

    // Only the registers something can still name: the two mapping tables and
    // every mapping the ROB is holding. The rest of the file is free storage
    // whose contents mean nothing.
    void emit_prf(const Cpu& cpu) {
        std::vector<uint8_t> live(cpu.prf().capacity(), 0);
        auto mark = [&](PhysReg p) {
            if (p < live.size()) live[p] = 1;
        };
        for (ArchReg a = 0; a < 32; ++a) {
            mark(cpu.rat().map(a));
            mark(cpu.committed_map(a));
        }
        for (uint32_t k = 0; k < cpu.rob().size(); ++k) {
            const RobEntry& e = cpu.rob().nth_entry(k);
            mark(e.dest_phys);
            mark(e.stale_phys);
        }

        j_.key("prf");
        j_.arr_begin();
        for (PhysReg p = 0; p < live.size(); ++p) {
            if (!live[p]) continue;
            // A register is ready unless something marked it pending, so
            // pending is the flag worth writing — and the value beside it is
            // then last generation's, which is why the viewer must not show it.
            j_.obj_begin();
            j_.kv("phys", p);
            j_.kv("value", cpu.prf().read(p));
            j_.flag("pending", !cpu.prf().is_ready(p));
            j_.obj_end();
        }
        j_.arr_end();
    }

    void emit_lsq(const Cpu& cpu) {
        const LsqQueue& lq = cpu.lsq().loads();
        j_.key("lq");
        j_.arr_begin();
        for (uint32_t k = 0; k < lq.size(); ++k) {
            const LsqEntry& e = lq.nth(k);
            j_.obj_begin();
            j_.kv("seq", e.seq);
            if (e.addr_ready) j_.kv("addr", e.addr);
            j_.kv("size", static_cast<uint32_t>(e.size));
            j_.flag("addr_ready", e.addr_ready);
            j_.obj_end();
        }
        j_.arr_end();

        const LsqQueue& sq = cpu.lsq().stores();
        j_.key("sq");
        j_.arr_begin();
        for (uint32_t k = 0; k < sq.size(); ++k) {
            const LsqEntry& e = sq.nth(k);
            j_.obj_begin();
            j_.kv("seq", e.seq);
            if (e.addr_ready) j_.kv("addr", e.addr);
            j_.kv("size", static_cast<uint32_t>(e.size));
            if (e.data_ready) j_.kv("data", e.data);
            j_.flag("addr_ready", e.addr_ready);
            j_.flag("data_ready", e.data_ready);
            j_.obj_end();
        }
        j_.arr_end();
    }

    void emit_fu(const Cpu& cpu) {
        j_.key("fu");
        j_.obj_begin();
        for (int cls = 0; cls < Cpu::FU_CLASSES; ++cls) {
            j_.key(Cpu::fu_class_name(cls));
            j_.arr_begin();
            for (const uint64_t free_at : cpu.fu_free_at()[cls]) j_.num(free_at);
            j_.arr_end();
        }
        j_.obj_end();
    }

    // The booking ring, rotated so index 0 is this cycle and index k is k
    // cycles from now — an offset means the same thing in every record, a ring
    // slot does not.
    void emit_cdb(const Cpu& cpu) {
        const std::vector<uint32_t>& ring = cpu.cdb_bookings();
        j_.key("cdb_booked");
        j_.arr_begin();
        for (std::size_t k = 0; k < ring.size(); ++k) {
            j_.num(ring[(cpu.cycle() + k) % ring.size()]);
        }
        j_.arr_end();
    }

    void emit_bpred(const Cpu& cpu) {
        const BranchPredictor& bp = cpu.bpred();
        j_.key("bpred");
        j_.obj_begin();
        j_.kv("ghr", bp.ghr());
        j_.kv("ghr_bits", cpu.config().ghr_bits);

        // Top of stack first: entry 0 is what the next return would pop.
        j_.key("ras");
        j_.arr_begin();
        for (uint32_t k = 0; k < bp.ras().depth(); ++k) j_.num(bp.ras().peek_at(k));
        j_.arr_end();
        j_.kv("ras_depth", bp.ras().depth());
        j_.kv("free_checkpoints", cpu.rat().num_free_checkpoints());

        // The counters actually in play: the one the next fetch would read,
        // and the one every in-flight branch was predicted from.
        std::vector<uint32_t> idx;
        idx.push_back(bp.gshare().index(cpu.fetch_pc()));
        for (uint32_t k = 0; k < cpu.rob().size() && idx.size() < PHT_SAMPLE; ++k) {
            if (!cpu.rob().nth_entry(k).is_branch) continue;
            const uint32_t i = cpu.inflight(cpu.rob().nth(k)).pht_index;
            bool seen = false;
            for (const uint32_t have : idx) seen = seen || have == i;
            if (!seen) idx.push_back(i);
        }

        j_.key("pht_sample");
        j_.arr_begin();
        for (const uint32_t i : idx) {
            j_.obj_begin();
            j_.kv("index", i);
            j_.kv("counter", static_cast<uint32_t>(bp.gshare().counter(i)));
            j_.obj_end();
        }
        j_.arr_end();
        j_.obj_end();
    }

    void emit_seqs(const std::vector<SeqNum>& seqs) {
        j_.arr_begin();
        for (const SeqNum s : seqs) j_.num(s);
        j_.arr_end();
    }

    static constexpr uint32_t FREE_LIST_PREVIEW = 8;   // what rename takes next
    static constexpr std::size_t PHT_SAMPLE     = 12;

    trace_detail::Json j_;
    uint64_t           records_ = 0;

    // Which PCs have been disassembled into the stream already, and the word
    // that was disassembled, so rewritten code is described again.
    std::unordered_map<uint32_t, uint32_t>           seen_;
    std::vector<std::pair<uint32_t, std::string>>    pending_;
};
