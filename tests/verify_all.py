#!/usr/bin/env python3
"""Build and verify every port against the committed cross-language fixtures."""

import os
import shutil
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BUILD = os.path.join(ROOT, "tests", ".build")
MANIFEST = os.path.join(ROOT, "tests", "fixtures", "manifest.txt")


def run(cmd, cwd=ROOT):
    print("+", " ".join(cmd))
    p = subprocess.run(cmd, cwd=cwd)
    if p.returncode:
        raise SystemExit(f"command failed ({p.returncode}): {' '.join(cmd)}")


def compiler(*names):
    for name in names:
        path = shutil.which(name)
        if path:
            return path
    raise SystemExit(f"none of these compilers found: {', '.join(names)}")


def main():
    os.makedirs(BUILD, exist_ok=True)
    cc = compiler("cc", "gcc", "clang")
    rustc = shutil.which("rustc")
    node = shutil.which("node")

    run([sys.executable, "python/cja_star.py", "verify", MANIFEST])

    c_bin = os.path.join(BUILD, "cja_star_c.exe" if os.name == "nt" else "cja_star_c")
    run([cc, "-O3", "-std=c11", "-o", c_bin, "c/cja_star.c", "-lm"])
    run([c_bin, "verify", MANIFEST])

    if rustc:
        r_bin = os.path.join(BUILD, "cja_star_rs.exe" if os.name == "nt" else "cja_star_rs")
        run([rustc, "-O", "rust/cja_star.rs", "-o", r_bin])
        run([r_bin, "verify", MANIFEST])
    else:
        print("SKIP Rust: rustc not on PATH")

    if node:
        run([node, "javascript/cja_star.js", "verify", MANIFEST])
    else:
        print("SKIP JavaScript: node not on PATH")

    print("all available ports verified")


if __name__ == "__main__":
    main()
