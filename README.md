<div align="center">

# Mini-CPU

**A cycle-level out-of-order RV32IM processor simulator in C++17**

Register renaming · speculative execution · precise commit · no dependencies

</div>

---

## Quick start

```bash
make test                                   # build and run the suite (~8s from clean)
make all                                    # build/oooc
make examples                               # assemble the bundled programs to examples/

build/oooc --hex examples/matmul.hex --base 0x1000 --stats
build/oooc --hex examples/crc32.hex  --base 0x1000 --ipc-table
build/oooc --help
```

Nothing to install: a C++17 compiler and `make` are the whole toolchain.

## What it models

A seven-stage out-of-order pipeline — fetch, decode, rename, dispatch, issue, execute,
writeback — feeding in-order commit off the reorder buffer, `--width` instructions per stage
per cycle.

| Mechanism | Implementation |
|---|---|
| Register renaming | Physical register file, free list, RAT; WAW and WAR stop existing |
| Out-of-order issue | Age-ordered non-data-capture issue queue, wakeup reaches select in the same cycle |
| Function units | Separate ALU / branch / mul / div / memory pools with configurable counts and latencies |
| Writeback | Booked common data buses, so port contention is real rather than assumed |
| Memory | Load and store queues, store-to-load forwarding, replay on an unresolved older store |
| Stores | Written to memory at commit, which is what makes a squash free |
| Prediction | gshare direction, PC-tagged set-associative BTB, return address stack |
| Recovery | Per-branch checkpoints of the map, the history and the return stack; one-cycle unwind |

Every structure is sized from a `Config`, so a sweep needs no recompilation. `--help` lists a
flag for each knob.

## What it reports

```
$ build/oooc --hex examples/matmul.hex --base 0x1000 --stats

cycles 946  retired 1418  IPC 1.499  CPI 0.667
  stages   fetch 1779  decode 1701  rename 1622  dispatch 1552  issue 1430  writeback 1418  squashed 204
  branch   250  mispredicted 36 (14.4%, 25.4 MPKI)  BTB 86.4%  RAS 0.0%
  memory   loads 128 (forwarded 0, from memory 128, replayed 0)  stores 48
  stalls   364 lost slots
    rob_full                     47   12.9%
    mul_port_full                37   10.2%
    mem_port_full                10    2.7%
    cdb_reserved_full            20    5.5%
    operands_not_ready          250   68.7%
  limited by: rob_full
```

The stall breakdown is slot-accurate: a cycle offers `width` issue slots, and every slot that
went unused is charged to exactly one cause. Starve one resource and the breakdown names that
resource, which is what makes it a diagnosis rather than decoration.

`--ipc-table` runs the same program on four machines:

```
machine            cycles    retired      IPC vs 1-wide
1-wide               1644       1418     0.86     1.00x
2-wide                946       1418     1.50     1.74x
4-wide                547       1418     2.59     3.01x
ROB=4, IQ=2          2024       1418     0.70     0.81x
```

## How it is validated

Correctness is decided by an in-order interpreter (`tests/ref.h`), never by a previous run of
the simulator:

- **Differential testing.** 14 hand-written programs — loops, arrays, nested and recursive
  calls, sub-word and partially overlapping memory traffic, an unpredictable branch, divide by
  zero, a pointer chase, a WAW/WAR stress and a bitwise CRC-32 — compared on all 32 registers,
  the exit code, and the retired-instruction count. A single wrong-path instruction reaching
  commit shows up in that last number even when it changes nothing else.
- **A six-configuration sweep.** Every program on default, 1-wide, 4-wide, starved,
  long-latency and no-predictor machines. Renaming, wakeup and recovery bugs are usually
  configuration-dependent, so running the corpus once is not enough.
- **Property assertions** that do not appeal to the interpreter at all: 200 dependent adds
  issue back to back, load-use latency is exactly `--mem-lat`, a forwarded load pays none of
  it, physical registers and checkpoints are conserved cycle by cycle, and the predictor's
  cost on a loop does not grow with the trip count.
- **One outside authority.** The `crc32` workload reproduces zlib's CRC-32 of the 256 bytes
  `00..FF`, byte for byte.

`make debug` builds the same sources under AddressSanitizer and UndefinedBehaviorSanitizer.

## Layout

```
src/       simulator: cpu, rename, issue queue, LSQ, branch predictor, stats, CLI
tests/     assembler, reference interpreter, workload corpus, the whole suite
tools/     example generator
examples/  generated .hex programs (make examples)
```

Read [`DESIGN.md`](./DESIGN.md) for the architecture and the measured characterization, and
[`PLAN.md`](./PLAN.md) for the order it was built in.
