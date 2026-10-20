#!/usr/bin/env python3
"""Generate deterministic cross-language fixtures from the Python reference."""

import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "python"))

from astar_nextgen import baseline_astar, corridor_jump_astar, map_set  # noqa: E402


def main():
    out_dir = os.path.join(ROOT, "tests", "fixtures")
    os.makedirs(out_dir, exist_ok=True)
    manifest = []
    for size in (100, 200):
        for seed in range(5):
            for map_name, grid, start, goal in map_set(size, 1234 + seed):
                for diagonal in (False, True):
                    mode = 8 if diagonal else 4
                    base_cost, base_nodes, _ = baseline_astar(grid, start, goal, diagonal)
                    cja_cost, cja_nodes, _ = corridor_jump_astar(grid, start, goal, diagonal)
                    if abs(base_cost - cja_cost) > 1e-9:
                        raise SystemExit(
                            f"optimality failure: {map_name} {size} {mode} "
                            f"{base_cost} vs {cja_cost}"
                        )
                    name = f"{size}_{map_name}_{mode}d_{seed}.txt"
                    path = os.path.join(out_dir, name)
                    with open(path, "w", encoding="utf-8", newline="\n") as f:
                        f.write("# cja-star fixture v1\n")
                        f.write(f"W {size}\n")
                        f.write(f"H {size}\n")
                        f.write(f"MODE {mode}\n")
                        f.write(f"START {start[0]} {start[1]}\n")
                        f.write(f"GOAL {goal[0]} {goal[1]}\n")
                        f.write(f"BASE_COST {base_cost:.12f}\n")
                        f.write(f"CJA_COST {cja_cost:.12f}\n")
                        f.write(f"BASE_NODES {base_nodes}\n")
                        f.write(f"CJA_NODES {cja_nodes}\n")
                        f.write("GRID\n")
                        for row in grid:
                            f.write("".join("." if v == 0 else "#" for v in row))
                            f.write("\n")
                    manifest.append(name)
                    print(f"wrote {name} cost={cja_cost:.3f} "
                          f"base_nodes={base_nodes} cja_nodes={cja_nodes}")
    with open(os.path.join(out_dir, "manifest.txt"), "w", encoding="utf-8",
              newline="\n") as f:
        for name in manifest:
            f.write(name + "\n")
    print(f"wrote {len(manifest)} fixtures")


if __name__ == "__main__":
    main()
