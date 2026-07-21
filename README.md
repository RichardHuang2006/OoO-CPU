# 7-Stage OoO RV32I CPU

A cycle-accurate, 7-stage out-of-order RV32I(M) CPU simulator in C++17 with
explicit register renaming, a non-data-capture wakeup/select issue queue, and
gshare branch prediction.

In-order front end, out-of-order back end, in-order commit.

```
Fetch -> Decode -> Rename -> Dispatch -> Issue -> Execute -> Writeback
                                                                 |
                                      Commit (in order, off the ROB head)
```

## Build and run

```sh
make            # builds build/oooc and regenerates examples/
make test       # 2870 validation checks
make debug      # same suite under ASan + UBSan

./build/oooc --hex examples/sieve.hex --regs
./build/oooc program.elf                      # RISC-V ELF32
./build/oooc --raw --base 0x1000 program.bin
```

No RISC-V toolchain is required: `tools/gen_examples.cpp` assembles the example
workloads in-process via `tests/asm.h`, and the simulator reads a plain hex-word
format. ELF32 is supported for real toolchain output.

`--help` lists every knob. Every structural parameter is a `Config` field, so
nothing downstream hardcodes a size.

## Default configuration

| | |
|---|---|
| Width | 2 (fetch/decode/rename/dispatch/issue/commit) |
| ROB | 32 entries |
| PRF | 64 physical registers (= ROB + 32) |
| Issue queue | 16 entries |
| LQ / SQ | 8 / 8 |
| CDBs | 2 writeback ports |
| Function units | 2 ALU, 1 branch, 1 mul, 1 mem |
| Latencies | ALU 1c, load use 2c, mul 3c (pipelined), div 20c (blocking) |
| gshare | 12-bit GHR, 4096-entry PHT of 2-bit counters |
| BTB | 512 entries, 4-way, PC-tagged, LRU |
| RAS | 16 entries |
| Checkpoints | 16 in-flight branches |

## Architecture

### Renaming (explicit / R10000-style)

Values live in a **unified physical register file** holding both committed and
speculative state — not in the ROB. At rename an instruction reads the RAT for
its sources, allocates a fresh physical register for its destination from the
**free list**, and records the **stale** mapping in its ROB entry. At commit the
stale register is reclaimed; that is the only place registers are freed on the
correct path.

This removes WAW and WAR hazards outright. The only dependence left is RAW,
carried entirely by physical tags.

`x0` is never renamed: it keeps its reset mapping to `p0`, and an instruction
writing `x0` allocates no destination at all.

The sizing rule `PRF = ROB + 32` makes register starvation impossible — every
in-flight instruction can hold one new mapping on top of the 32 committed ones.
Smaller values are legal and simply stall rename, which is what makes `--prf`
worth sweeping; `Config::prf_can_starve()` reports whether a config is below the
rule.

### Issue queue: wakeup and select in one atomic cycle

Entries hold source **tags and ready bits**, never values (`issue_queue.h`).
Operands are read from the PRF after select.

The timing that matters is in `Cpu::tick()`. Stages are evaluated in reverse
pipeline order (Commit → Writeback → Execute → Issue → Dispatch → Rename →
Decode → Fetch) so each stage sees the previous cycle's output of its producer —
the inter-stage queues behave as latches. The single deliberate intra-cycle path
is **Writeback → Issue**: a tag broadcast in cycle T clears ready bits *before*
select runs in the same cycle T. That is what lets a dependent 1-cycle ALU op
issue back-to-back with its producer, and the test suite asserts it (200
dependent `addi`s on a 1-wide machine complete in under 260 cycles).

Two wakeup paths:

- **Fast / speculative** — ALU, branch, mul, div. Latency is known at select, so
  the op books its writeback port up front (`reserve_cdb`). That reservation is
  what makes speculative wakeup safe: the broadcast can never be delayed by
  contention.
- **Slow** — loads. A load cannot reserve a port (its completion may replay
  against the store queue), so it broadcasts only after winning a CDB in the
  writeback arbiter.

Select is age-ordered (oldest ready first) and bounded by issue width, per-class
FU ports, and CDB availability. Losing any of those is counted separately.

### Memory

Load queue and store queue are allocated in program order at dispatch, so "older
than me" is a sequence-number comparison. Stores write memory only at commit, in
order. Loads search the store queue for older overlapping stores:

- fully-covering store with known data → **forward**
- older store with an unresolved address → **replay** (retried in place each
  cycle until the ambiguity clears)
- partial overlap → conservatively treated as unresolved, never forwarded

### Branch prediction and recovery

gshare (`GHR XOR PC >> 2` → PHT of 2-bit counters) for direction, a PC-tagged
4-way BTB for targets, and a RAS pushed on call / popped on return. The GHR is
updated **speculatively** in Fetch.

The front end redirects only on a BTB hit, as real hardware must — a cold BTB
falls through and is corrected at Execute.

On a misprediction detected at Execute, recovery runs once for the oldest
offending branch (`handle_recovery`):

1. return the destination registers of every younger ROB entry to the free list,
   youngest first, and truncate the ROB
2. squash younger issue-queue entries, in-flight FU ops (releasing their booked
   CDB slots), pending writebacks, and LQ/SQ entries
3. restore the RAT, GHR, and RAS from the branch's **checkpoint**, then re-shift
   the GHR with the true outcome
4. flush the front-end latches and redirect Fetch

Checkpoints are a finite pool; exhausting it stalls rename on the next branch.
The RAS is snapshotted in *Fetch* rather than Rename, because younger fetches
mutate it before a branch reaches the rename stage. The RAT is snapshotted in
Rename, after the branch's own destination is mapped, since the branch itself
survives recovery.

The predictors themselves are updated non-speculatively at commit.

## Statistics

IPC/CPI, per-stage uop counts, wrong-path squashes, mispredict rate and MPKI,
BTB/RAS behaviour, load/store and forwarding counts, and a stall-cause breakdown
attributing lost issue slots to ROB-full, physreg starvation, checkpoint
starvation, IQ-full, LQ/SQ-full, FU structural hazards, and CDB contention.

Measured on the bundled workloads (IPC):

| workload | 1-wide | 2-wide | 4-wide | ROB=4, IQ=2 |
|---|---|---|---|---|
| sieve | 0.88 | 1.43 | 2.03 | 0.86 |
| matmul | 0.90 | 1.77 | 3.49 | 0.70 |
| bubble_sort | 0.87 | 1.44 | 1.86 | 0.83 |
| fib (recursive) | 0.97 | 1.65 | 2.54 | 0.90 |
| crc32 | 0.66 | 0.88 | 0.98 | 0.69 |

`matmul` has the ILP to use a 4-wide machine; `crc32` is a dependent bit-serial
loop with a ~38% mispredict rate and barely moves. A 4-entry ROB erases the
benefit of width entirely.

History length on `fib`, which has strongly correlated call/return branches:
52.7 → 20.2 → 8.9 → 5.1 MPKI at 0/4/8/12 GHR bits. The short benchmarks
(`sieve`, `crc32`) go the other way, since a 12-bit history spreads a few
thousand dynamic branches across 4096 PHT entries that never warm up.

## Validation

`make test` runs 13 hand-written programs (ALU coverage, loops, arrays, store
forwarding, sub-word and partial-overlap accesses, nested calls, recursive fib,
an LCG-driven unpredictable branch, mul/div including divide-by-zero, a pointer
chase, and a WAW/WAR renaming stress) against an in-order reference interpreter,
comparing **all 32 architectural registers, the exit code, and the committed
instruction count** — so no wrong-path instruction may ever retire.

Each program runs on six machine configurations: the default, 1-wide/1-CDB,
4-wide with a 128-entry ROB, a deliberately starved `ROB=4 IQ=2 LQ=SQ=1
chkpt=1`, long FU latencies, and degenerate 1-entry predictors. Any renaming,
wakeup, or recovery bug shows up as a configuration-dependent result.

On top of that, targeted checks assert the microarchitectural properties
themselves: back-to-back dependent ALU issue, load-use latency tracking
`mem_latency`, store-queue forwarding and replay, that starved machines actually
report the resource they ran out of, and that gshare beats a random branch on a
regular loop. As an independent ISA check, the `crc32` workload reproduces
zlib's CRC-32 of `bytes(range(256))` exactly.

## Layout

| file | contents |
|---|---|
| `src/cpu.{h,cpp}` | the core: all seven stages, recovery, statistics |
| `src/rat.h` | RAT and the branch checkpoint pool |
| `src/prf.h` `src/freelist.h` | unified PRF and free list |
| `src/rob.h` | reorder buffer |
| `src/issue_queue.h` | non-data-capture IQ, wakeup and age-ordered select |
| `src/lsq.h` | load/store queues and store-to-load forwarding |
| `src/bpred.h` | gshare, BTB, RAS |
| `src/decoder.h/.cpp` `src/alu.h` | RV32IM decode and the functional model |
| `src/memory.h` `src/loader.h` | paged memory; ELF32 / raw / hex loaders |
| `tests/` | assembler, reference interpreter, validation suite |
| `tools/gen_examples.cpp` | generates `examples/*.hex` |

## Scope

FENCE and FENCE.I retire as NOPs (no caches or coherence to order). ECALL and
EBREAK trap at commit and stop the machine; `a7 == 93` is honoured as
`exit(a0)`. There are no caches or TLBs — memory is a flat, always-hit array, so
`mem_latency` is a fixed load-use latency rather than a hit/miss distribution.
No CSRs, no privileged modes, no interrupts, RV32 only.
