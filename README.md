# Mini-CPU

A cycle-level, out-of-order RV32IM processor simulator in C++17, written to be
read. It models a seven-stage out-of-order pipeline with R10000-style register
renaming, a non-data-capture issue queue, load/store queues with store-to-load
forwarding, a gshare + BTB + RAS front end, and per-branch checkpoint recovery
— validated against an in-tree in-order reference interpreter. No dependencies
beyond a C++17 compiler and `make`.

This README is ordered as a learning guide: read it top to bottom, then read
the source in the order §5 recommends.

## 1. What Mini-CPU teaches

How a modern out-of-order core extracts parallelism from a sequential program
without ever *appearing* to leave program order:

- how register renaming removes WAW/WAR hazards so only true dependences remain,
- how a reorder buffer turns chaotic completion back into precise, in-order commit,
- how an issue queue picks work by data readiness instead of program position,
- how loads safely read stores that haven't reached memory yet,
- how a speculating front end guesses branch outcomes, and how the machine
  unwinds cleanly — per branch, not wholesale — when a guess is wrong.

Every structure is small enough to read in one sitting, and every behavior is
pinned by a test you can point at.

## 2. Resume-level feature summary

- **Seven-stage out-of-order pipeline** with in-order commit from the ROB head,
  configurable width, evaluated in reverse stage order each `tick()`.
- **R10000-style register renaming**: register alias table (RAT), unified
  physical register file (PRF) with per-register ready bits, free list, reorder
  buffer, stale mappings freed only at commit.
- **Non-data-capture issue queue**: tag wakeup, oldest-ready selection,
  per-class functional-unit eligibility, and CDB (writeback-port) reservation —
  an op cannot issue unless a port is booked for the cycle its result lands.
- **Load/store queues** allocated in program order: store-to-load forwarding,
  partial-overlap detection, replay while an older store address is unresolved,
  stores written to memory only at commit.
- **Branch prediction**: gshare direction predictor (global history × PC into
  2-bit saturating counters), PC-tagged set-associative BTB with LRU
  replacement, and a return address stack.
- **Per-branch checkpoint recovery**: every branch snapshots the RAT, global
  history, and RAS; misprediction restores exactly that snapshot, reclaims
  younger physical registers youngest-first, cancels younger CDB reservations,
  squashes younger IQ/ROB/LSQ entries, and redirects fetch in the same cycle.
- **Cycle tracing** (NDJSON + an HTML visualizer) and **aggregate statistics**
  with an exact stall-cause accounting of unused issue slots.

## 3. Pipeline overview

Seven modeled stages, `width` instructions per stage per cycle:

```
      in order ─────────────────────────►│◄────── out of order ──────►│◄─ in order
                                         │                            │
  ┌───────┐  ┌────────┐  ┌─────────────┐ │ ┌─────────────┐ ┌────────┐ │ ┌────────┐
  │ 1     │  │ 2      │  │ 3           │ │ │ 4           │ │ 5      │ │ │ 7      │
  │ FETCH ├─►│ DECODE ├─►│ RENAME /    ├─┼►│ ISSUE /     ├►│ EXECUTE├─┼►│ COMMIT │
  │ +bpred│  │        │  │ DISPATCH    │ │ │ REG READ    │ │        │ │ │ (ROB   │
  └───────┘  └────────┘  │ RAT·ROB·IQ· │ │ │ oldest-ready│ │ ALU BR │ │ │  head) │
      ▲                  │ LSQ seats   │ │ │ + CDB book  │ │ MUL DIV│ │ └────────┘
      │                  └─────────────┘ │ └─────────────┘ │ MEM ───┼─┼──► 6 MEMORY/
      │                                  │                 └────────┘ │    WRITEBACK
      └────────────── misprediction recovery: restore checkpoint, ◄───┘    (PRF write,
                      squash younger, redirect fetch                        tag wakeup)
```

In the code (`cpu.cpp`), stage 3 is two functions (`rename()` then
`dispatch()`, with a queue between them so each does one job), and stage 6 is
the pair `execute_load()` (the memory access itself, resolved against the
store queue) plus `writeback()` (PRF write and tag broadcast). `tick()` runs
commit first and fetch last — reverse order — so the queues between stages
behave like hardware latches: each stage consumes what its producer left last
cycle. The one deliberate exception: `writeback()` runs before `issue()`, so a
result broadcast can wake a dependent op in the same cycle and single-cycle
dependence chains run back to back.

Three kinds of state to keep separate while reading:

- **Architectural state** — what a program is allowed to observe: the committed
  register mapping (`arch_rat_`), memory, and the committed PC. Only `commit()`
  changes it.
- **Speculative state** — everything in flight: the RAT, PRF values not yet
  committed, ROB/IQ/LSQ contents, the fetch PC, speculative GHR and RAS.
  Recovery may erase any of it at any moment.
- **Committed state** is the architectural state plus the predictor's learning
  tables (counters, BTB), which are written only at commit so wrong paths can
  never train them.

## 4. Repository structure

```
src/
  config.h            every sizing and latency knob, one POD struct
  instruction.h/.cpp  types, RV32IM decode, ALU/branch/mul-div semantics, disasm
  memory.h            lazily paged flat byte memory
  loader.h            .hex, raw-binary, and ELF32 program loaders
  rename.h            PRF + free list + RAT + checkpoint pool (the renaming story)
  rob.h               reorder buffer: completion vs. commit
  issue_queue.h       non-data-capture IQ: tag wakeup, oldest-ready select
  lsq.h               load/store queues: forward / no-match / replay
  branch_predictor.h  gshare + BTB + RAS + branch classification + snapshots
  cpu.h/.cpp          the pipeline: all seven stages and tick()
  trace.h             NDJSON cycle tracer (observation only)
  stats.h             aggregate counters, rates, stall-cause breakdown
  main.cpp            CLI driver

tests/
  test_support.h            harness + differential scaffolding shared by all suites
  asm.h                     in-tree RV32IM assembler (no toolchain needed)
  ref.h                     in-order reference interpreter, the oracle
  workloads.h               the 14-program validation corpus
  test_instruction.cpp      decode, semantics, assembler, disassembly
  test_rename_issue.cpp     ROB/RAT/free-list/PRF units, renaming, IQ, CDB
  test_memory_lsq.cpp       memory, loaders, LSQ, forwarding, store-at-commit
  test_branch_recovery.cpp  gshare/BTB/RAS, checkpoints, recovery, CDB cancel
  test_pipeline.cpp         stage movement, width, latency, stats, tracing
  test_differential.cpp     corpus × configuration matrix vs. the interpreter

tools/
  gen_examples.cpp    assembles the bundled workloads into examples/*.hex
  oooviz.html         trace visualizer (open in a browser, drop a trace on it)
```

## 5. Recommended reading order

1. `src/instruction.h` — what an instruction *is*, before any pipeline exists.
2. `src/memory.h`, `src/loader.h` — where programs live.
3. `tests/ref.h` — the whole ISA as a 100-line interpreter; everything after
   this is "the same semantics, but fast and speculative."
4. `src/rename.h` — the renaming transaction, steps 1–7, in one header.
5. `src/rob.h` — completion vs. commit.
6. `src/issue_queue.h` — readiness, wakeup, oldest-ready selection.
7. `src/lsq.h` — the three load outcomes.
8. `src/branch_predictor.h` — direction, target, returns, and the snapshot.
9. `src/cpu.cpp` — start at `tick()` and follow one instruction through
   `fetch → decode_stage → rename → dispatch → issue → execute → writeback →
   commit`; then read `recover()`.
10. `src/trace.h`, `src/stats.h` — how to watch it and how to measure it.

## 6. RV32IM instruction flow

`instruction.h/.cpp` owns everything about one instruction, pipeline-free:

- `decode(raw)` → `Decoded {op, kind, rd, rs1, rs2, imm, flags}`. Immediates
  are extracted per format (I/S/B/U/J) and sign-extended exactly once, at
  decode. Unknown encodings become `Op::INVALID` with `OpKind::TRAP`, so an
  illegal instruction traps *precisely at commit*, not at fetch.
- `OpKind` routes to a functional-unit class (ALU/BRANCH/MUL/DIV/LOAD/STORE);
  `Op` picks the semantics within the class.
- `alu::` holds the pure semantics: RV32I integer ops, branch comparisons and
  target arithmetic (`branch_taken`, `branch_target`, `jalr_target`,
  `link_address`), and the M extension with its ISA-defined corner cases —
  divide-by-zero returns −1 (rem returns the dividend), `INT_MIN / −1` returns
  `INT_MIN` without trapping. All `constexpr`, so tests pin them at compile time.
- `disasm()` renders any word as text (`addi t0, t0, 1`, `lw a0, 4(sp)`,
  pseudo-forms like `li/mv/j/ret`); the tracer uses it, execution never does.
- Loads/stores carry width and signedness in the `Op` (LB/LBU/LH/LHU/LW);
  sign extension happens once, in the pipeline's `extend_load`, identically
  for forwarded and memory data.

## 7. Register renaming

`rename.h` — read the header comment first; it walks the whole transaction:

```
                 speculative RAT                        unified PRF
  rs1=t0 ──► [ t0 → p38 ]──────────► src1 tag p38    p37: 12   ready
  rs2=t1 ──► [ t1 → p41 ]──────────► src2 tag p41    p38:  7   ready
                                                     p41:  ?   PENDING
  rd=t0  ──► old: t0 → p38  ── stale, into the ROB   p52:  ?   PENDING ◄─ new
             new: t0 → p52  ── from the free list                        dest
```

1. read source mappings; 2. allocate a destination physical register (stall if
the free list is empty); 3. remember the displaced mapping; 4. install the new
mapping; 5. mark the new register not-ready; 6. at commit, the mapping becomes
architectural and the *stale* register returns to the free list; 7. after a
squash, each killed instruction returns the register it *allocated* — youngest
first — and the RAT is restored from a checkpoint.

Two writes to `t0` now live in different physical registers, so WAW and WAR
hazards cannot exist; the only ordering left is true data flow through tags.
`x0` is permanently `p0`: always ready, never allocated, never freed.

## 8. ROB and precise commit

`rob.h`. Entries are allocated in program order at rename and stamped with a
monotonically increasing sequence number (never reused, so `seq_a < seq_b`
always means "older"). The key distinction:

- **completion** — writeback sets `complete`; the value exists in the PRF but
  the instruction is still speculative and invisible.
- **commit** — `pop_head_if_complete()` at the head only. Now the register
  write becomes architectural, the store writes memory, the stale physical
  register is freed, traps fire, the predictor trains.

A divide can sit complete for fifteen cycles behind a slow older op; nothing
outside can tell. `truncate_to(branch)` squashes younger entries and hands
them back youngest-first — precisely the order recovery must free their
registers in.

## 9. Issue, wakeup, selection, and CDB reservation

`issue_queue.h` + `Cpu::issue()`:

```
   writeback broadcasts tag p52 ──────────────┐  (same cycle, wakeup→select)
                                              ▼
   IQ (dispatch order = program order): [seq 7: p38✓ p41✓]  ← oldest ready ─┐
     entries hold TAGS + ready bits,    [seq 8: p52✗→✓ p9✓]                 │
     never operand values               [seq 9: p52✗→✓ p13✓]                │
                                              │                             │
   select: scan forward, take ready ops ──────┘         issue ◄─────────────┘
   for each: need a free unit of its class  AND  a CDB slot at cycle+latency
             (reserve_cdb) — no slot, no issue; it waits, others pass it
   then: read operand VALUES from the PRF, execute
```

Non-data-capture means the queue stores two tags per entry instead of two
32-bit values; wakeup is a tag comparison. Selection is oldest-ready (a
forward scan of dispatch order). A fixed-latency op books a writeback port for
the exact cycle its result lands and cannot issue without one, so CDB
bandwidth is modeled, not assumed; loads can't know their latency at issue
(forward vs. memory), so they take leftover ports at writeback and retry if
none remain. Squashed ops give their reservations back (`release_cdb`).

## 10. Load/store ordering and forwarding

`lsq.h`. Both queues are program-ordered at dispatch; stores write memory only
at commit, making the store queue a speculative write buffer. A load that has
its address searches all older stores:

```
   older stores ──────────────► youngest    load lw 0x100 asks:
   [sw 0x200 ✓] [sw 0x100 ✓=7] [sb ????]
        │             │            └── address unknown ──► REPLAY (wait a cycle)
        │             └── youngest full cover, data known ─► FORWARD (7, 1 cycle)
        └── disjoint — irrelevant
   none overlap and all resolved ─────────────────────────► NO_MATCH → read memory
                                                             (pays mem_latency)
```

Partial overlap (a byte store under a word load) is never stitched together
from fragments — the load replays until the store commits. Replayed loads keep
their queue seat and issue-queue entry and cost a `STORE_ORDER` stall, visible
in the statistics.

## 11. Branch prediction

`branch_predictor.h`, consulted on every fetch:

- **BTB**: PC-tagged, set-associative, LRU. A miss predicts fall-through — a
  branch that never committed taken costs one recovery, then it's cached.
- **gshare**: the global history register XORed with the PC indexes a table of
  2-bit saturating counters; the top bit is the direction. History gives the
  same branch different counters in different contexts.
- **RAS**: calls (JAL/JALR with a link register `ra`/`t0`… precisely `x1`/`x5`)
  push the return address; returns pop it — the BTB alone would always predict
  the *previous* caller.
- Classification (`classify()`) tells the four kinds apart straight from the
  decoded instruction.

Speculative vs. learned state: GHR and RAS advance at fetch (the next
prediction depends on them) and are therefore checkpointed and restored;
counters and BTB are updated only at commit, so wrong paths never train them.

## 12. Checkpoints and recovery

Every branch takes a `Checkpoint` at rename — the speculative RAT plus the
front end's `{GHR, RAS}` snapshot from its fetch — from a bounded pool
(`rename.h`). Pool empty ⇒ the branch stalls at rename, because an
unrecoverable branch must not enter the window.

```
 mispredict detected at issue (resolved target ≠ predicted target)
   │
   ├─ ROB.truncate_to(branch): younger entries returned youngest-first
   │     each: free_list.free(its dest), checkpoint pool.free(its ckpt)
   ├─ IQ.squash_after(seq) · LSQ.squash_after(seq) · executing/writeback queues
   │     each squashed op also releases its future CDB reservation
   ├─ RAT.adopt(ckpt.rat)          ← exact speculative mapping at the branch
   ├─ bpred.restore(ckpt.front_end)← exact GHR + RAS at its fetch
   │     then shift in the branch's *resolved* direction
   └─ fetch_q/decode_q cleared, pc = correct target      (all in one cycle)
```

The branch itself survives and keeps its checkpoint until it commits. Nothing
is reset wholesale: older in-flight instructions keep executing across the
recovery, which is what "per-branch" means. Tests assert the machine afterward
is indistinguishable from one that never guessed — no leaked registers,
checkpoints, queue seats, or reservations.

## 13. Cycle tracing

`trace.h` is a debugging/visualization subsystem, deliberately separate from
the statistics. `--trace out.ndjson` writes newline-delimited JSON: one header
record carrying the full configuration, then one record per traced cycle with
the cycle's events (fetch/decode/rename/dispatch/issue/replay/complete/commit/
squash/mispredict, each stamped with the instruction's sequence number) and
end-of-cycle snapshots of the ROB, issue queue, load/store queues, speculative
and committed RAT, PRF values and ready bits, free-list and checkpoint
headroom, functional-unit occupancy, future CDB reservations, and predictor
state. `--trace-start N` and `--trace-cycles N` bound the window.

The tracer only observes: a traced run is cycle-for-cycle and
statistic-for-statistic identical to an untraced one (a test enforces this).
Open `tools/oooviz.html` in a browser and drop a trace on it to scrub through
a pipeline diagram and per-cycle machine state.

A *cycle trace* answers "what happened on cycle 812?"; the *aggregate
statistics* answer "how did the whole run go?" — keep the two ideas apart.

## 14. Performance statistics

`stats.h` accumulates whole-run counters (no per-cycle data): cycles, retired
instructions, IPC/CPI, issue-slot utilization, branches, mispredictions (rate
and MPKI), BTB lookups/hits, RAS accuracy, loads by source (forwarded vs.
memory) and replays, stores, and a stall-cause breakdown with one counter per
resource (ROB/PRF/checkpoint/IQ/LQ/SQ full, each FU port, CDB, operands,
store-order). The issue-side breakdown is *exact*: each cycle offers `width`
issue slots, and every unused slot is charged to exactly one cause —
`dominant_stall()` then names the resource that actually limits the run.
`--stats` prints all of it; `--ipc-table` compares four machine shapes.

## 15. Correctness strategy

Differential testing against `tests/ref.h`, an in-order interpreter whose
execute logic is written independently of the pipeline's: fourteen workloads
(`tests/workloads.h` — loops, sorts, matmul, sieve, forwarding patterns,
sub-word/partial-overlap accesses, nested calls, recursive fib, an
LCG-driven unpredictable branch, mul/div corner cases, a pointer chase, a
WAW/WAR renaming stress, bitwise CRC-32) are run on both and compared on all
32 registers, the exit code, and the retired-instruction count — across a
six-machine configuration sweep plus a five-config matrix, since renaming,
wakeup, and recovery bugs are usually configuration-dependent. Expected exit
codes are derived independently of the simulator; `crc32` is additionally
pinned to zlib's published value via a table-driven implementation, an
*external* reference. On top of that sit microarchitectural property tests:
physical-register conservation every cycle, no leaked checkpoints or queue
seats, in-order commit, back-to-back dependent issue, exact load-use latency,
zero-cost forwarding, stall accounting that fits the offered slots, and
trace invariance. `make debug` runs everything under ASan + UBSan.

## 16. Build and run commands

```bash
make                 # build build/oooc (release, -O2)
make test            # build and run all six test binaries (regenerates examples/)
make debug           # the same suite + CLI binary under ASan + UBSan
make examples        # assemble the bundled workloads into examples/*.hex
make clean           # remove build/ and examples/

# run a program on the out-of-order pipeline
build/oooc --hex examples/matmul.hex --base 0x1000 --stats
build/oooc --hex examples/crc32.hex  --base 0x1000 --ipc-table
build/oooc program.elf --regs

# run the in-order reference interpreter instead
build/oooc --hex examples/fib.hex --base 0x1000 --ref

# record a cycle trace and view it
build/oooc --hex examples/fib.hex --base 0x1000 --trace fib.ndjson --trace-cycles 2000
# then open tools/oooviz.html in a browser and drop fib.ndjson on it
# (--ref --trace PATH writes a plain retired-instruction log instead)

# every microarchitectural knob is a flag; see them all:
build/oooc --help    # --width --rob --prf --iq --lq --sq --cdb --alu --mul ...
```

Programs load from a hex-word file (`--hex` + `--base`), a raw binary
(`--raw` + `--base`), or a positional ELF32 path. A program exits via the
proxy-kernel convention: `ecall` with `a7 = 93` halts with exit code `a0`.
