#pragma once

#include <cstdint>
#include <ostream>
#include <string>
#include <vector>

#include "cpu.h"

// Cycle-level tracing for debugging and visualization — not a performance
// counter (aggregate measurements live in stats.h). The output is
// newline-delimited JSON (NDJSON) that tools/oooviz.html renders:
//
//   line 1    {"type":"header", ...}   the full Config, entry PC, trace window
//   line 2+   {"type":"cycle",  ...}   one record per traced cycle
//
// Each cycle record carries two things:
//
//   "ev"      the events of the cycle, each stamped with the instruction's
//             sequence number (or fetch id before rename assigns one):
//             F fetch, D decode, R rename, DI dispatch, I issue, RP replay,
//             C complete, CM commit, SQ squash, SF front-end squash,
//             MP mispredict. Stage timestamps fall out of these: an
//             instruction's lifetime is the set of cycles its events land in.
//
//   snapshots the machine state at the end of the cycle: the fetch/decode/
//             rename queues, ROB, issue queue, load and store queues, the
//             speculative and committed RAT, physical-register values and
//             ready bits, free-list and checkpoint headroom, function-unit
//             occupancy, future CDB reservations, predictor state (GHR and
//             RAS), and a few cumulative counters.
//
// The tracer is attached with Cpu::attach_trace() and only observes through
// const references; a traced run is cycle-for-cycle identical to an untraced
// one. The window is controlled by Options: recording starts at `start` and
// stops after `max_cycles` records, so a long run can trace just the region
// of interest.

class Trace {
public:
    struct Options {
        uint64_t start      = 0;      // first cycle to record
        uint64_t max_cycles = 5000;   // maximum number of cycle records
    };

    Trace(std::ostream& out, const Config& cfg, uint32_t entry_pc)
        : Trace(out, cfg, entry_pc, Options{}) {}

    Trace(std::ostream& out, const Config& cfg, uint32_t entry_pc, Options opt)
        : out_(out), opt_(opt) {
        out_ << "{\"type\":\"header\",\"tool\":\"mini-cpu\",\"version\":1"
             << ",\"entry_pc\":" << entry_pc
             << ",\"trace_start\":" << opt_.start
             << ",\"trace_max\":" << opt_.max_cycles
             << ",\"config\":{"
             << "\"width\":" << cfg.width
             << ",\"rob_size\":" << cfg.rob_size
             << ",\"prf_size\":" << cfg.prf_size
             << ",\"iq_size\":" << cfg.iq_size
             << ",\"lq_size\":" << cfg.lq_size
             << ",\"sq_size\":" << cfg.sq_size
             << ",\"num_cdb\":" << cfg.num_cdb
             << ",\"num_alu\":" << cfg.num_alu
             << ",\"num_branch\":" << cfg.num_branch
             << ",\"num_mul\":" << cfg.num_mul
             << ",\"num_div\":" << cfg.num_div
             << ",\"num_mem\":" << cfg.num_mem
             << ",\"alu_latency\":" << cfg.alu_latency
             << ",\"branch_latency\":" << cfg.branch_latency
             << ",\"mul_latency\":" << cfg.mul_latency
             << ",\"div_latency\":" << cfg.div_latency
             << ",\"mem_latency\":" << cfg.mem_latency
             << ",\"ghr_bits\":" << cfg.ghr_bits
             << ",\"pht_size\":" << cfg.pht_size
             << ",\"btb_sets\":" << cfg.btb_sets
             << ",\"btb_ways\":" << cfg.btb_ways
             << ",\"ras_size\":" << cfg.ras_size
             << ",\"num_checkpoints\":" << cfg.num_checkpoints
             << "}}\n";
    }

    uint64_t cycles_recorded() const { return recorded_; }

    // ---- event hooks, called by the pipeline mid-cycle ----------------------
    // Buffered and attached to the record end_cycle() emits. `fid` identifies
    // an instruction before rename; `seq` after.

    void on_fetch(const Uop& u) {
        event("{\"e\":\"F\",\"fid\":" + std::to_string(u.fid) +
              ",\"pc\":" + std::to_string(u.pc) + "}");
    }
    void on_decode(const Uop& u) {
        event("{\"e\":\"D\",\"fid\":" + std::to_string(u.fid) +
              ",\"pc\":" + std::to_string(u.pc) +
              ",\"asm\":\"" + escape(disasm(u.dec, u.pc)) + "\"}");
    }
    void on_rename(const Uop& u) {
        event("{\"e\":\"R\",\"fid\":" + std::to_string(u.fid) +
              ",\"seq\":" + std::to_string(u.seq) +
              ",\"pc\":" + std::to_string(u.pc) +
              ",\"asm\":\"" + escape(disasm(u.dec, u.pc)) +
              "\",\"dest\":" + reg_or_null(u.dest) +
              ",\"stale\":" + reg_or_null(u.stale) +
              ",\"s1\":" + std::to_string(u.src1) +
              ",\"s2\":" + std::to_string(u.src2) + "}");
    }
    void on_dispatch(const Uop& u) {
        event("{\"e\":\"DI\",\"seq\":" + std::to_string(u.seq) + "}");
    }
    void on_issue(const Uop& u) {
        event("{\"e\":\"I\",\"seq\":" + std::to_string(u.seq) +
              ",\"wb\":" + std::to_string(u.wb_cycle) + "}");
    }
    void on_replay(const Uop& u) {
        event("{\"e\":\"RP\",\"seq\":" + std::to_string(u.seq) + "}");
    }
    void on_complete(const Uop& u) {
        event("{\"e\":\"C\",\"seq\":" + std::to_string(u.seq) +
              ",\"dest\":" + reg_or_null(u.dest) +
              ",\"val\":" + std::to_string(u.result) + "}");
    }
    void on_commit(const Uop& u) {
        event("{\"e\":\"CM\",\"seq\":" + std::to_string(u.seq) +
              ",\"pc\":" + std::to_string(u.pc) + "}");
    }
    void on_squash(SeqNum seq) {
        event("{\"e\":\"SQ\",\"seq\":" + std::to_string(seq) + "}");
    }
    void on_squash_fe(uint64_t fid) {
        event("{\"e\":\"SF\",\"fid\":" + std::to_string(fid) + "}");
    }
    void on_mispredict(const Uop& u) {
        event("{\"e\":\"MP\",\"seq\":" + std::to_string(u.seq) +
              ",\"pc\":" + std::to_string(u.pc) +
              ",\"next\":" + std::to_string(u.next_pc) + "}");
    }

    // ---- one record per traced cycle, emitted after the stages ran ----------
    void end_cycle(const Cpu& cpu) {
        const uint64_t cycle = cpu.cycle();
        if (saturated_ || cycle < opt_.start) {
            events_.clear();
            saturated_ = saturated_ || recorded_ >= opt_.max_cycles;
            return;
        }
        if (recorded_ >= opt_.max_cycles) {
            saturated_ = true;
            events_.clear();
            return;
        }

        out_ << "{\"type\":\"cycle\",\"cycle\":" << cycle
             << ",\"pc\":" << cpu.fetch_pc()
             << ",\"fetch_stalled\":" << (cpu.fetch_stalled() ? "true" : "false");

        // events of this cycle
        out_ << ",\"ev\":[";
        for (std::size_t i = 0; i < events_.size(); ++i) {
            if (i) out_ << ',';
            out_ << events_[i];
        }
        out_ << ']';
        events_.clear();

        // front-end queues (latches between the in-order stages)
        emit_fe_queue("fq", cpu.fetch_q_);
        emit_fe_queue("dq", cpu.decode_q_);
        out_ << ",\"rq\":[";
        for (std::size_t i = 0; i < cpu.rename_q_.size(); ++i) {
            const Uop& u = cpu.rename_q_[i];
            out_ << (i ? "," : "") << "{\"seq\":" << u.seq << ",\"pc\":" << u.pc << '}';
        }
        out_ << ']';

        // reorder buffer, oldest first
        out_ << ",\"rob\":[";
        for (uint32_t k = 0; k < cpu.rob().size(); ++k) {
            const RobEntry& e = cpu.rob().nth_entry(k);
            const Uop&      u = cpu.uop_at(cpu.rob().nth(k));
            out_ << (k ? "," : "")
                 << "{\"seq\":" << e.seq
                 << ",\"pc\":" << e.pc
                 << ",\"asm\":\"" << escape(disasm(u.dec, u.pc))
                 << "\",\"done\":" << (e.complete ? "true" : "false")
                 << ",\"br\":" << (e.is_branch ? "true" : "false")
                 << ",\"st\":" << (e.is_store ? "true" : "false")
                 << ",\"mis\":" << (e.mispredicted ? "true" : "false")
                 << ",\"dest\":" << reg_or_null(e.dest_phys)
                 << ",\"stale\":" << reg_or_null(e.stale_phys) << '}';
        }
        out_ << ']';

        // issue queue, dispatch order
        out_ << ",\"iq\":[";
        {
            const std::vector<IssueQueue::Entry>& es = cpu.iq().entries();
            for (std::size_t i = 0; i < es.size(); ++i) {
                const IssueQueue::Entry& e = es[i];
                out_ << (i ? "," : "")
                     << "{\"seq\":" << e.seq
                     << ",\"kind\":\"" << kind_name(e.kind)
                     << "\",\"s1\":" << e.src1
                     << ",\"r1\":" << (e.src1_ready ? "true" : "false")
                     << ",\"s2\":" << e.src2
                     << ",\"r2\":" << (e.src2_ready ? "true" : "false")
                     << ",\"dest\":" << reg_or_null(e.dest) << '}';
            }
        }
        out_ << ']';

        // load and store queues, oldest first
        out_ << ",\"lq\":[";
        for (uint32_t k = 0; k < cpu.lsq().loads().size(); ++k) {
            const LsqEntry& e = cpu.lsq().loads().nth(k);
            out_ << (k ? "," : "")
                 << "{\"seq\":" << e.seq << ",\"addr\":" << e.addr
                 << ",\"aok\":" << (e.addr_ready ? "true" : "false")
                 << ",\"sz\":" << static_cast<uint32_t>(e.size) << '}';
        }
        out_ << "],\"sq\":[";
        for (uint32_t k = 0; k < cpu.lsq().stores().size(); ++k) {
            const LsqEntry& e = cpu.lsq().stores().nth(k);
            out_ << (k ? "," : "")
                 << "{\"seq\":" << e.seq << ",\"addr\":" << e.addr
                 << ",\"aok\":" << (e.addr_ready ? "true" : "false")
                 << ",\"data\":" << e.data
                 << ",\"dok\":" << (e.data_ready ? "true" : "false")
                 << ",\"sz\":" << static_cast<uint32_t>(e.size) << '}';
        }
        out_ << ']';

        // speculative and committed register mappings
        out_ << ",\"rat\":[";
        for (uint32_t a = 0; a < 32; ++a) {
            out_ << (a ? "," : "") << cpu.rat().map(a);
        }
        out_ << "],\"arat\":[";
        for (uint32_t a = 0; a < 32; ++a) {
            out_ << (a ? "," : "") << cpu.committed_map(a);
        }
        out_ << ']';

        // physical register file: values and ready bits
        out_ << ",\"prf\":{\"val\":[";
        for (uint32_t r = 0; r < cpu.prf().capacity(); ++r) {
            out_ << (r ? "," : "") << cpu.prf().read(r);
        }
        out_ << "],\"ready\":[";
        for (uint32_t r = 0; r < cpu.prf().capacity(); ++r) {
            out_ << (r ? "," : "") << (cpu.prf().is_ready(r) ? 1 : 0);
        }
        out_ << "]}";

        out_ << ",\"free_regs\":" << cpu.free_list().num_free()
             << ",\"free_ckpts\":" << cpu.checkpoints().num_free();

        // function-unit occupancy: the cycle each unit becomes free again
        out_ << ",\"fu\":{";
        emit_fu(cpu, "alu",    Cpu::Fu::ALU,    true);
        emit_fu(cpu, "branch", Cpu::Fu::BRANCH, false);
        emit_fu(cpu, "mul",    Cpu::Fu::MUL,    false);
        emit_fu(cpu, "div",    Cpu::Fu::DIV,    false);
        emit_fu(cpu, "mem",    Cpu::Fu::MEM,    false);
        out_ << '}';

        // CDB reservations for upcoming cycles
        out_ << ",\"cdb\":[";
        {
            bool first = true;
            for (uint64_t k = 1; k < cpu.cdb_window_; ++k) {
                const uint64_t c = cycle + k;
                const uint32_t n = cpu.cdb_reserved_at(c);
                if (n == 0) continue;
                out_ << (first ? "" : ",") << "{\"c\":" << c << ",\"n\":" << n << '}';
                first = false;
            }
        }
        out_ << ']';

        // branch predictor speculative state
        out_ << ",\"bp\":{\"ghr\":" << cpu.bpred().ghr() << ",\"ras\":[";
        {
            const std::vector<uint32_t> ras = cpu.bpred().ras().entries();
            for (std::size_t i = 0; i < ras.size(); ++i) {
                out_ << (i ? "," : "") << ras[i];
            }
        }
        out_ << "]}";

        // a few cumulative counters, so the viewer can show deltas
        const Stats& s = cpu.stats();
        out_ << ",\"ctr\":{\"retired\":" << s.retired
             << ",\"issued\":" << s.issued
             << ",\"mispredicts\":" << s.mispredicts
             << ",\"forwards\":" << s.load_forwards
             << ",\"replays\":" << s.load_replays
             << ",\"squashed\":" << s.squashed << "}}\n";

        ++recorded_;
        if (recorded_ >= opt_.max_cycles) saturated_ = true;
    }

private:
    void event(std::string s) {
        if (!saturated_) events_.push_back(std::move(s));
    }

    // INVALID_PHYSREG prints as null so the viewer needs no sentinel logic.
    static std::string reg_or_null(PhysReg r) {
        return r == INVALID_PHYSREG ? "null" : std::to_string(r);
    }

    static const char* kind_name(OpKind k) {
        switch (k) {
        case OpKind::ALU:    return "alu";
        case OpKind::BRANCH: return "branch";
        case OpKind::MUL:    return "mul";
        case OpKind::DIV:    return "div";
        case OpKind::LOAD:   return "load";
        case OpKind::STORE:  return "store";
        case OpKind::NOP:    return "nop";
        case OpKind::TRAP:   return "trap";
        }
        return "?";
    }

    static std::string escape(const std::string& s) {
        std::string out;
        out.reserve(s.size());
        for (const char c : s) {
            if (c == '"' || c == '\\') { out.push_back('\\'); out.push_back(c); }
            else if (static_cast<unsigned char>(c) < 0x20) { out.push_back(' '); }
            else out.push_back(c);
        }
        return out;
    }

    void emit_fe_queue(const char* name, const std::deque<Uop>& q) {
        out_ << ",\"" << name << "\":[";
        for (std::size_t i = 0; i < q.size(); ++i) {
            out_ << (i ? "," : "") << "{\"fid\":" << q[i].fid
                 << ",\"pc\":" << q[i].pc << '}';
        }
        out_ << ']';
    }

    void emit_fu(const Cpu& cpu, const char* name, Cpu::Fu f, bool first) {
        out_ << (first ? "" : ",") << '"' << name << "\":[";
        const std::vector<uint64_t>& pool = cpu.fu_free_at_[static_cast<int>(f)];
        for (std::size_t i = 0; i < pool.size(); ++i) {
            out_ << (i ? "," : "") << pool[i];
        }
        out_ << ']';
    }

    std::ostream&            out_;
    Options                  opt_;
    std::vector<std::string> events_;
    uint64_t                 recorded_  = 0;
    bool                     saturated_ = false;
};
