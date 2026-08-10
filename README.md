# Mini-CPU

A cycle-level out-of-order RV32IM processor simulator in C++17, with no dependencies beyond a
compiler and `make`.

## Architecture

The simulator models a seven-stage out-of-order pipeline — fetch, decode, rename, dispatch,
issue, execute, writeback — feeding in-order commit off a reorder buffer, `--width`
instructions per stage per cycle. Stages are evaluated in reverse pipeline order each `tick()`
so a result written back in a cycle can wake a consumer in that same cycle.

- **Register renaming** — R10000-style explicit renaming with a physical register file, a free
  list, and a register alias table. WAW and WAR hazards are removed by construction; stale
  mappings are reclaimed at commit.
- **Out-of-order issue** — an age-ordered, non-data-capture issue queue; operands are read from
  the PRF at select, and wakeup reaches select within the same cycle.
- **Function units** — separate ALU / branch / mul / div / memory pools with configurable
  counts and latencies. Fixed-latency ops reserve a common data bus slot at issue, so writeback
  port contention is modelled rather than assumed away.
- **Memory** — load and store queues with store-to-load forwarding and replay on unresolved
  older stores. Stores mutate memory only at commit, which is what makes squashing free.
- **Branch prediction** — a gshare direction predictor, a PC-tagged set-associative BTB, and a
  return address stack.
- **Recovery** — per-branch checkpoints of the RAT, global history, and return stack allow a
  one-cycle unwind on misprediction; younger physical registers are returned to the free list.

Every structure is sized from a single `Config`, so configuration sweeps need no
recompilation; `--help` lists a flag for each knob.

### Source layout

```
src/
  types.h        register/index aliases, OpKind, sentinels
  config.h       every sizing and latency knob in one POD struct
  decoder.h/.cpp RV32IM instruction decode
  alu.h          integer/branch/mul/div functional semantics
  memory.h       lazily paged flat byte memory
  loader.h       .hex, raw-binary and ELF32 program loaders
  rob.h          reorder buffer
  prf.h          physical register file with ready bits
  freelist.h     physical register free list
  rat.h          register alias table + checkpoint pool
  issue_queue.h  age-ordered non-data-capture issue queue
  lsq.h          load/store queues with forwarding search
  bpred.h        gshare + BTB + return address stack
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
```

### Validation

Correctness is decided by differential testing against the in-order interpreter in
`tests/ref.h`: every workload is run through both, comparing all 32 architectural registers,
the exit code, and the retired-instruction count. The suite adds a six-configuration sweep
(renaming, wakeup and recovery bugs are usually configuration-dependent), microarchitectural
property assertions that do not appeal to the interpreter, and one outside authority — the
`crc32` workload reproduces zlib's CRC-32 byte for byte.

## Build and run

```bash
make                # build build/oooc (release, -O2)
make test           # build and run the full test suite (~8s from clean)
make debug          # build and run the suite under ASan + UBSan
make examples       # assemble the bundled programs into examples/
make clean          # remove build/ and examples/
```

Run a program and inspect the machine:

```bash
build/oooc --hex examples/matmul.hex --base 0x1000 --stats
build/oooc --hex examples/crc32.hex  --base 0x1000 --ipc-table
build/oooc --help
```

`--stats` prints cycles, IPC, branch and memory behaviour, and a slot-accurate stall-cause
breakdown that charges every unused issue slot to exactly one cause. `--ipc-table` runs the
same program on four machine widths and reports the scaling.
