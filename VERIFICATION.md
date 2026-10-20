# Verification

All four ports were run against the same 100 committed fixtures. A pass
requires both algorithms to match the recorded **cost** and **expanded-node
count** exactly.

| Port | Toolchain | Result |
|---|---|--:|
| Python | CPython 3.11.0 | 100/100 |
| C | GCC 16.2.0, `-O3 -std=c11` | 100/100 |
| Rust | rustc 1.99.1, `-O` | 100/100 |
| JavaScript | Node.js 18.12.0 | 100/100 |

## Reproduce

From the `cja-star/` directory:

```bash
python tests/verify_all.py
```

Or individually:

```bash
python python/cja_star.py verify tests/fixtures/manifest.txt

cc -O3 -std=c11 -o cja_star c/cja_star.c -lm
./cja_star verify tests/fixtures/manifest.txt

rustc -O rust/cja_star.rs -o cja_star_rs
./cja_star_rs verify tests/fixtures/manifest.txt

node javascript/cja_star.js verify tests/fixtures/manifest.txt
```

## Recorded summaries

```text
SUMMARY language=python cases=100 pass=100 fail=0
SUMMARY language=c cases=100 pass=100 fail=0
SUMMARY language=rust cases=100 pass=100 fail=0
SUMMARY language=javascript cases=100 pass=100 fail=0
```

## Node-count invariant

Every port reports:

```text
baseline_nodes=5196.2
cja_nodes=4728.2
```

This is stronger than a cost-only check: it confirms that heap tie-breaking,
corridor activation, and expansion order agree across all four
implementations.

## Fixture set

- 5 seeds
- 5 map families: random 20%, random 30%, random 40%, corridor, bottleneck
- 2 grid sizes: 100x100, 200x200
- 2 connectivities: 4-dir, 8-dir
- 100 cases total

The fixtures are generated from the Python reference by
[`tests/generate_fixtures.py`](tests/generate_fixtures.py).
