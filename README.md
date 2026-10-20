# CJA* — Corridor-Jump A*

[![verify](https://github.com/skaffa/corridor-jump-astar/actions/workflows/verify.yml/badge.svg)](https://github.com/skaffa/corridor-jump-astar/actions/workflows/verify.yml)

**A corridor-aware A\* replacement for 2-D grid maps, ported to Python, C, Rust, and JavaScript.**

CJA* keeps A*'s optimality guarantee but removes a specific structural
redundancy: in a one-cell-wide straight corridor, it skips every cell that
has no branch or turn, replacing the whole run with one macro edge. On
non-corridor maps it falls back to a flat-array A* with no look-ahead
overhead.

The algorithm is pure code — no ML, no training, no external weights, no
runtime dependencies. The only data structures are grids, arrays, and a
binary min-heap.

## How this started: an AI-assisted engineering experiment

This project began with a simple question: **can AI help design a pure
algorithmic replacement for A\* that is faster in wall-clock time, not just
better on paper?** The constraint was deliberately strict: the shipped
algorithm could contain **no machine learning, no neural network, and no
learned weights** — only portable grid, array, and heap code.

The AI was used as a research and engineering collaborator, not as a runtime
component:

- propose structural candidates (tie-breaking, corridor contraction, lean
  state, JPS, integer costs, bucket queues);
- implement each candidate behind the same benchmark harness;
- profile actual wall-clock time and expanded-node counts;
- reject ideas that broke optimality or only won on paper;
- port the surviving algorithm to C, Rust, and JavaScript;
- generate deterministic cross-language fixtures and verify exact costs and
  node counts.

Several plausible ideas did not survive. Integer-scaled 8-dir costs (`2/3`)
produced a real suboptimal path; `70/99` and `1000/1414` were not exact and
gave no meaningful speedup. A 4-dir bucket queue was faster but expanded
more nodes. Inlining the jump loops was 20–35% slower on Python 3.11. A
direct port of standard JPS forced-neighbour rules failed under this
project's no-corner-cutting movement rule. The details are in the
[optimization audit](python/astar_nextgen.py#L1001).

What survived was **CJA\***: a lean A\* with an exact corridor-macro
extension, enabled only on maps where straight-through cells are common.
The AI did not add intelligence to the search; it helped build, measure, and
discard. The runtime algorithm is ordinary deterministic code.

## Results

All numbers are from the same Windows host. 100 deterministic fixtures:
5 seeds × 5 map families × 2 sizes (100, 200) × 2 connectivities.
Each query is best-of-5; times include search, heap work, and path
reconstruction, not map parsing.

Positive wall-clock percentages mean CJA* is faster than A*; negative
percentages mean it is slower.

| Language | Build / runtime | A\* ms/query | CJA\* ms/query | Wall-clock change | A\* nodes | CJA\* nodes | Node cut |
|---|---|--:|--:|--:|--:|--:|--:|
| C | GCC 16.2 `-O3` | 1.195 | 1.068 | **+11.9%** (1.119x) | 5196.2 | 4728.2 | 9.0% |
| Rust | rustc 1.99 `-O` | 1.155 | 1.055 | **+9.4%** (1.094x) | 5196.2 | 4728.2 | 9.0% |
| JavaScript | Node 18.12 | 3.300 | 2.705 | **+22.0%** (1.220x) | 5196.2 | 4728.2 | 9.0% |
| Python | CPython 3.11 | 26.460 | 21.268 | **+24.4%** (1.244x) | 5196.2 | 4728.2 | 9.0% |

Every port returns the **same cost and the same node count** on every
fixture. The node count is a useful cross-language invariant: it confirms
that the heap ordering and corridor activation are identical.

### Per-connectivity speedups

| Language | 4-dir | 8-dir |
|---|--:|--:|
| C | **+42.2%** (1.422x) | **+5.3%** (1.053x) |
| Rust | **+82.3%** (1.823x) | **-3.8%** (0.962x) |
| JavaScript | **+43.6%** (1.436x) | **-6.0%** (0.940x) |
| Python | **+29.0%** (1.290x) | **+23.5%** (1.235x) |

CJA* is corridor-specialized. On the 8-dir random/bottleneck subset, the
topology guard and jump dispatch can cost more than the corridor wins in
Rust/JS; the aggregate over the full 100-fixture suite is still positive in
every language. C and Python win both connectivities.

> These are intra-language comparisons. Python A* uses dict/set state
> because that is idiomatic Python; the compiled ports use flat arrays.
> The absolute Python/C ratio is not an algorithm comparison.

## Verification

100/100 fixtures pass in every port, with exact cost and exact expanded-node
counts:

| Port | Build | Verification |
|---|---|--:|
| Python | CPython 3.11 | 100/100 |
| C | GCC 16.2 | 100/100 |
| Rust | rustc 1.99.1 | 100/100 |
| JavaScript | Node 18.12 | 100/100 |

Fixtures live in [`tests/fixtures/`](tests/fixtures/). They are generated
from the Python reference by
[`tests/generate_fixtures.py`](tests/generate_fixtures.py) and include
random 20/30/40% obstacle maps, a long-corridor maze, and a room/bottleneck
maze. See [`VERIFICATION.md`](VERIFICATION.md) for the exact recorded
summaries.

## Quick start

### Python

```bash
python python/cja_star.py verify tests/fixtures/manifest.txt
python python/cja_star.py bench  tests/fixtures/manifest.txt 5
```

The Python reference implementation is
[`python/astar_nextgen.py`](python/astar_nextgen.py); `cja_star.py` is the
fixture/benchmark runner.

### C

```bash
cc -O3 -std=c11 -o cja_star c/cja_star.c -lm
./cja_star verify tests/fixtures/manifest.txt
./cja_star bench  tests/fixtures/manifest.txt 5
```

### Rust

```bash
rustc -O rust/cja_star.rs -o cja_star
./cja_star verify tests/fixtures/manifest.txt
./cja_star bench  tests/fixtures/manifest.txt 5
```

### JavaScript

```bash
node javascript/cja_star.js verify tests/fixtures/manifest.txt
node javascript/cja_star.js bench  tests/fixtures/manifest.txt 5
```

Expected:

```text
SUMMARY language=... cases=100 pass=100 fail=0
```

## How it works

CJA* is A* over a graph whose edges are either normal grid moves or exact
straight-corridor macro moves.

1. **Cheap topology check.** A 64-probe LCG sample estimates the fraction of
   free cells that are straight-through cells, i.e. cells whose free
   cardinal neighbours are exactly `{back, forward}`.
2. **Guard off.** If the map is not corridor-dominated, run lean A*:
   flat `g`/`parent`/`closed` arrays, no look-ahead.
3. **Guard on.** Walk each degree-2 straight run inline. Push only the first
   junction, turn, dead end, or goal. The macro edge cost is the number of
   traversed cells (or `steps * sqrt(2)` for an 8-connected diagonal).
4. **No corner cutting.** 8-connected diagonals are legal only when both
   adjacent cardinal cells are free. Diagonal macro edges are never
   contracted; only cardinal corridors are.

A skipped cell has no side entrance and no branch, so removing it cannot
change the shortest-path cost. The search still pops the minimum `f`, uses
the same admissible Manhattan/octile heuristic, and stops when the goal is
popped. The returned cost is exact.

### Implementation notes

- **4-connected mode** packs `(f, h, node)` into one integer heap entry:
  `key = ((f * (w+h+1) + h) << node_bits) | node`. The heap compares
  scalars, not tuples.
- **8-connected mode** uses a binary heap of `(f, h, node)` with `f64`
  costs and inlined octile arithmetic.
- **Guard** is a static map property of 64 samples; it costs microseconds.
- **Path** returned is a jump-node path. Macro edges skip degree-2 cells;
  the cost is exact. Expand macro edges if you need every cell.

## Repository layout

```text
c/
  cja_star.c              C port + fixture runner
rust/
  cja_star.rs             single-file Rust port + fixture runner
javascript/
  cja_star.js             Node.js port + fixture runner
python/
  astar_nextgen.py        Python reference algorithm + academic benchmark
  cja_star.py             Python fixture verifier / benchmark runner
tests/
  fixtures/               100 committed cross-language fixtures
  generate_fixtures.py    regenerates them from the Python reference
.github/workflows/
  verify.yml              CI for all four ports
```

## Fixture format

Each fixture is plain text:

```text
W 100
H 100
MODE 4
START 1 1
GOAL 98 98
BASE_COST 194.000000000000
CJA_COST 194.000000000000
BASE_NODES 194
CJA_NODES 20
GRID
#...##...
...
```

Ports parse the same files and compare both algorithms against the recorded
cost and node counts.

## Name

**CJA\*** is the technical short name: **Corridor-Jump A\***. It is
descriptive and paper-friendly, but `CJA` is not very searchable and the
star is redundant with "A\*". For a repository, `corridor-jump-astar` is the
better URL. If you want a brand rather than an acronym, **Threadline A\***
(`threadline-astar`) is the cleanest alternative: it evokes long straight
corridor runs and is easy to search for.

## License

MIT. See [LICENSE](LICENSE).

