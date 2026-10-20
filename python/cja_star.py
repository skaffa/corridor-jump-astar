#!/usr/bin/env python3
"""CJA* reference runner: verify/benchmark the Python port against fixtures."""

import os
import sys
import time

from astar_nextgen import baseline_astar, corridor_jump_astar


def load_fixture(path):
    meta = {}
    grid_rows = []
    with open(path, "r", encoding="utf-8") as f:
        lines = f.read().splitlines()
    i = 0
    while i < len(lines):
        line = lines[i].strip()
        i += 1
        if not line or line.startswith("#"):
            continue
        if line == "GRID":
            break
        key, value = line.split(None, 1)
        meta[key] = value
    w = int(meta["W"])
    h = int(meta["H"])
    while i < len(lines) and len(grid_rows) < h:
        line = lines[i].strip()
        i += 1
        if line:
            grid_rows.append(line)
    blocked = bytearray(w * h)
    for y, row in enumerate(grid_rows):
        for x, ch in enumerate(row):
            if ch != ".":
                blocked[y * w + x] = 1
    sx, sy = map(int, meta["START"].split())
    gx, gy = map(int, meta["GOAL"].split())
    return {
        "name": os.path.basename(path),
        "w": w,
        "h": h,
        "diagonal": meta["MODE"] == "8",
        "start": (sx, sy),
        "goal": (gx, gy),
        "blocked": blocked,
        "base_cost": float(meta["BASE_COST"]),
        "cja_cost": float(meta["CJA_COST"]),
        "base_nodes": int(meta["BASE_NODES"]),
        "cja_nodes": int(meta["CJA_NODES"]),
    }


def manifest_paths(manifest):
    base = os.path.dirname(os.path.abspath(manifest))
    with open(manifest, "r", encoding="utf-8") as f:
        return [os.path.join(base, line.strip()) for line in f
                if line.strip() and not line.startswith("#")]


def verify(manifest):
    paths = manifest_paths(manifest)
    passed = 0
    for path in paths:
        fx = load_fixture(path)
        g = [fx["blocked"][y * fx["w"]:(y + 1) * fx["w"]] for y in range(fx["h"])]
        bc, bn, _ = baseline_astar(g, fx["start"], fx["goal"], fx["diagonal"])
        cc, cn, _ = corridor_jump_astar(g, fx["start"], fx["goal"], fx["diagonal"])
        ok = (abs(bc - fx["base_cost"]) < 1e-9 and bn == fx["base_nodes"]
              and abs(cc - fx["cja_cost"]) < 1e-9 and cn == fx["cja_nodes"])
        status = "PASS" if ok else "FAIL"
        print(f"VERIFY {fx['name']} {status} base={bc:.9f}/{bn} "
              f"cja={cc:.9f}/{cn}")
        if ok:
            passed += 1
    print(f"SUMMARY language=python cases={len(paths)} pass={passed} "
          f"fail={len(paths) - passed}")
    return 0 if passed == len(paths) else 1


def bench(manifest, reps):
    paths = manifest_paths(manifest)
    base_total = 0.0
    cja_total = 0.0
    base_nodes = 0
    cja_nodes = 0
    for path in paths:
        fx = load_fixture(path)
        g = [fx["blocked"][y * fx["w"]:(y + 1) * fx["w"]] for y in range(fx["h"])]
        # Warm both paths once.
        baseline_astar(g, fx["start"], fx["goal"], fx["diagonal"])
        corridor_jump_astar(g, fx["start"], fx["goal"], fx["diagonal"])
        btimes = []
        ctimes = []
        for _ in range(reps):
            t0 = time.perf_counter_ns()
            bc, bn, _ = baseline_astar(g, fx["start"], fx["goal"], fx["diagonal"])
            btimes.append((time.perf_counter_ns() - t0) / 1e6)
            t0 = time.perf_counter_ns()
            cc, cn, _ = corridor_jump_astar(g, fx["start"], fx["goal"], fx["diagonal"])
            ctimes.append((time.perf_counter_ns() - t0) / 1e6)
        base_total += min(btimes)
        cja_total += min(ctimes)
        base_nodes += bn
        cja_nodes += cn
    cases = len(paths)
    print(f"BENCH language=python cases={cases} reps={reps} "
          f"baseline_ms={base_total / cases:.6f} cja_ms={cja_total / cases:.6f} "
          f"speedup={base_total / cja_total:.4f} "
          f"baseline_nodes={base_nodes / cases:.1f} cja_nodes={cja_nodes / cases:.1f}")
    return 0


def main():
    if len(sys.argv) < 3:
        print("usage: cja_star.py verify|bench <manifest> [reps]")
        return 2
    action = sys.argv[1]
    manifest = sys.argv[2]
    if action == "verify":
        return verify(manifest)
    if action == "bench":
        reps = int(sys.argv[3]) if len(sys.argv) > 3 else 5
        return bench(manifest, reps)
    print("unknown action")
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
