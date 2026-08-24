# Mini-CPU

A cycle-level out-of-order RV32IM processor simulator in C++17. It models a 7-stage
out-of-order pipeline with R10000-style register renaming, speculative execution, and
per-branch checkpoint recovery, and validates itself against an in-tree in-order reference
interpreter. There are no dependencies beyond a C++17 compiler and `make`.

## Features

- **7-stage out-of-order pipeline**: fetch, decode, rename, dispatch, issue, execute,
  writeback, feeding **in-order commit** from the ROB head. `--width` uops per stage per
  cycle. Stages are evaluated in reverse pipeline order each `tick()`, so the inter-stage
  queues behave as latches and a result written back in a cycle can wake a consumer in the
  same cycle.
- **R10000-style register renaming**: a register alias table (RAT), a unified physical
  register file with per-register ready bits, a free list, and a reorder buffer (ROB). WAW
  and WAR hazards are removed by construction; the displaced mapping is held in the ROB
  entry and returned to the free list at commit.
- **Non-data-capture issue queue**: entries hold physical register tags and ready bits,
  never operand values. **Tag wakeup** on the result broadcast, **oldest-ready select** by
  a forward scan of dispatch order, and **CDB reservation** at issue: a fixed-latency op
  books a writeback port for the cycle its result lands and cannot issue without one, so
  writeback port contention is modelled rather than assumed away.
- **Load/store queues with store-to-load forwarding**: queue seats are allocated in
  program order at dispatch. A load searches older stores and either forwards, goes to
  memory, or replays while an older store address is still unresolved. Stores reach memory
  only at commit.
- **Branch prediction**: a **gshare** direction predictor (global history XORed with the
  PC into a table of two-bit saturating counters), a PC-tagged **set-associative BTB** with
  LRU replacement, and a **return address stack**. Global history and the return stack
  advance speculatively at fetch; the counters and the BTB are written only at commit.
- **Per-branch checkpoint recovery**: every branch takes a checkpoint of the RAT, the
  global history, and the return stack at rename, and stalls if the checkpoint pool is
  full. On misprediction, recovery reclaims younger physical registers youngest first,
  flushes younger entries from every structure, restores the checkpoint, and redirects
  fetch in the same cycle.
- **Configurable function units**: separate ALU, branch, multiply, divide, and memory
  pools with per-class counts and latencies. Multiply is pipelined; divide is blocking.
- **Statistics**: IPC/CPI, mispredict rate and MPKI, BTB hit rate, RAS accuracy, and a
  slot-accurate stall-cause breakdown that charges every unused issue slot to exactly one
  of fourteen causes.

Every structure is sized from a single `Config` struct, so configuration sweeps need no
recompilation; `--help` lists a flag for each of the 23 knobs.

## Source map

```
src/
  types.h        register/index aliases, OpKind, sentinels
  config.h       every sizing and latency knob in one POD struct
  decoder.h/.cpp RV32IM instruction decode
  alu.h          integer/branch/mul/div functional semantics
  memory.h       lazily paged flat byte memory
  loader.h       .hex, raw-binary and ELF32 program loaders
  disasm.h       one decoded instruction as text, for traces and humans
  trace.h        the cycle-by-cycle JSON trace writer
  rob.h          reorder buffer
  prf.h          unified physical register file with ready bits
  freelist.h     physical register free list
  rat.h          register alias table + checkpoint pool
  issue_queue.h  non-data-capture issue queue, tag wakeup, oldest-ready select
  lsq.h          load/store queues with store-to-load forwarding search
  bpred.h        gshare + set-associative BTB + return address stack
  stats.h        IPC, mispredict, and slot-accurate stall-cause statistics
  cpu.h/.cpp     the pipeline: all stages and the tick() loop
  main.cpp       CLI driver

tests/
  asm.h          in-tree RV32IM assembler (no RISC-V toolchain needed)
  ref.h          in-order reference interpreter, the correctness oracle
  workloads.h    the 14-program validation corpus
  test_main.cpp  the whole suite: differential, sweep, and property tests

tools/
  gen_examples.cpp  assembles the bundled workloads into examples/*.hex
  oooviz.html       single-file cycle-by-cycle trace viewer (no build, no server)
```

## Build

```bash
make                # build build/oooc (release, -O2)
make test           # build and run the full test suite (~8s from clean)
make debug          # build and run the suite under ASan + UBSan
make examples       # assemble the bundled programs into examples/
make clean          # remove build/ and examples/
```

## Usage

```bash
build/oooc --hex examples/matmul.hex --base 0x1000 --stats
build/oooc --hex examples/crc32.hex  --base 0x1000 --ipc-table
build/oooc program.elf --regs
build/oooc --help
```

Programs are loaded from a plain hex-word file (`--hex` with `--base`), a raw binary blob
(`--raw` with `--base`), or a positional ELF32 path. `--stats` prints cycles, IPC, branch
and memory behaviour, and the stall-cause breakdown. `--ipc-table` runs the same program
on four machine configurations and reports the scaling. `--ref` runs the in-order
reference interpreter instead of the pipeline, and `--trace` records the run for the
viewer described below (with `--ref`, the retired-instruction trace instead).

## Watching a run, cycle by cycle

`--trace` writes one JSON object per cycle, and `tools/oooviz.html` renders it. The viewer is
a single file with no build step, no dependencies and no server: open it from the filesystem
and pick a trace, or drop one on the page.

```bash
make trace                                   # build/fib.ndjson, 2000 cycles
open tools/oooviz.html                       # then load that file

build/oooc --hex examples/crc32.hex --base 0x1000 \
  --trace=crc.ndjson --trace-from 5000 --trace-max 2000     # a window of a long run
```

Records average about 8 KB, so trace a window rather than a whole long program: `--trace-from`
skips to the interesting part at full speed and `--trace-max` bounds the file. The run
continues to completion after recording stops, so the exit code and `--stats` match an
untraced run; the test suite asserts exactly that.

The viewer shows every structure the machine has: the three front-end queues, the issue queue
with ready bits, the reorder buffer with its head marker, both mapping tables, the physical
registers in use, the free list, the load and store queues, function-unit occupancy, writeback
port bookings, the branch predictor's history, return stack and counters, the events of the
cycle, and the cumulative stall breakdown. Changed entries flash; clicking anything with a
sequence number selects that instruction and highlights it in every pane at once, with a detail
panel giving its tags, its producers, the cycle each stage happened, and its predicted versus
actual next PC. `v` switches to a pipeline diagram — one row per instruction, one column per
cycle, cells showing `F D R S X W C` — the view that makes a stall obvious at a glance. Arrow
keys step, space plays, `Home`/`End` jump, and the find box takes `seq 61`, `0x1010`,
`mispredict` or `replay`.

### Trace format

Newline-delimited JSON. The first line is a header (no `cycle` key) carrying the `Config`, so
the viewer can show occupancy against capacity; every line after it is one cycle. Two
conventions keep the files small enough for a browser:

- **Instruction text is sent once.** A PC is disassembled into the `disasm` map on the first
  record that mentions it; later entries carry the PC alone.
- **A false boolean is an absent key**, and a sentinel (`INVALID_PHYSREG` and friends) is
  written as `null` or left out — never as `4294967295`.

Each in-flight instruction carries `at`, the cycles its stages happened, as
`[fetch, decode, rename, dispatch, issue, complete]` truncated at the stage it has reached. The
viewer never infers a field the trace does not carry: a missing one is drawn as `n/a`.

## Validation

Correctness is decided by differential testing against the in-order interpreter in
`tests/ref.h`, which duplicates the pipeline's execute logic independently rather than
sharing it. Every workload is run through both and compared on all 32 architectural
registers, the exit code, and the retired-instruction count.

The suite (37 sections, `make test`) layers four kinds of check on top of that:

- **Differential runs** of the 14-program corpus in `tests/workloads.h`, covering ALU
  semantics, loops, arrays, store-to-load forwarding, sub-word and partial-overlap
  accesses, nested calls, recursive Fibonacci, an unpredictable branch, mul/div including
  divide-by-zero, a pointer chase, a WAW/WAR renaming stress, and a bitwise CRC-32. Each
  expected exit code is derived independently of the simulator.
- **A six-configuration sweep**: default, 1-wide/1-CDB, 4-wide/ROB=128, resource-starved,
  long-latency, and 1-entry predictors. Renaming, wakeup, and recovery bugs are usually
  configuration-dependent, and the sweep also asserts that the six machines produce six
  distinct cycle counts.
- **Microarchitectural property assertions** that do not appeal to the interpreter: every
  renamed uop either retires or is squashed, every load is served either by forwarding or
  by memory, no physical register or checkpoint is leaked across a run, commit is in
  program order, dependent single-cycle ops issue back to back, load-use latency tracks the
  configured value exactly while a forwarded load pays none of it, the stall breakdown fits
  inside the issue slots the machine offered, and the predictor's mispredict count stays
  constant as loop trip count grows tenfold.
- **An external reference point**: the `crc32` workload reproduces zlib's CRC-32 of the
  bytes 0..255, checked against an independently written table-driven implementation.
