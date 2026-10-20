#!/usr/bin/env node
/*
 * CJA* — Corridor-Jump A* (JavaScript port)
 *
 * Use:
 *   node cja_star.js verify tests/fixtures/manifest.txt
 *   node cja_star.js bench  tests/fixtures/manifest.txt 5
 */

"use strict";

const fs = require("fs");
const path = require("path");

const SQRT2 = Math.SQRT2;
const INF = Infinity;

class Heap4 {
  constructor() {
    this.a = [];
  }
  get size() {
    return this.a.length;
  }
  push(key) {
    const a = this.a;
    a.push(key);
    let i = a.length - 1;
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (a[p] <= a[i]) break;
      const t = a[p];
      a[p] = a[i];
      a[i] = t;
      i = p;
    }
  }
  pop() {
    const a = this.a;
    const out = a[0];
    const last = a.pop();
    if (a.length === 0) return out;
    a[0] = last;
    let i = 0;
    for (;;) {
      const l = (i << 1) + 1;
      const r = l + 1;
      let m = i;
      if (l < a.length && a[l] < a[m]) m = l;
      if (r < a.length && a[r] < a[m]) m = r;
      if (m === i) break;
      const t = a[m];
      a[m] = a[i];
      a[i] = t;
      i = m;
    }
    return out;
  }
}

class Heap8 {
  constructor() {
    this.f = [];
    this.h = [];
    this.node = [];
  }
  get size() {
    return this.f.length;
  }
  less(i, j) {
    const fi = this.f[i], fj = this.f[j];
    if (fi < fj) return true;
    if (fi > fj) return false;
    const hi = this.h[i], hj = this.h[j];
    if (hi < hj) return true;
    if (hi > hj) return false;
    return this.node[i] < this.node[j];
  }
  swap(i, j) {
    let t = this.f[i]; this.f[i] = this.f[j]; this.f[j] = t;
    t = this.h[i]; this.h[i] = this.h[j]; this.h[j] = t;
    t = this.node[i]; this.node[i] = this.node[j]; this.node[j] = t;
  }
  push(f, h, node) {
    this.f.push(f);
    this.h.push(h);
    this.node.push(node);
    let i = this.f.length - 1;
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (!this.less(i, p)) break;
      this.swap(i, p);
      i = p;
    }
  }
  pop() {
    const out = [this.f[0], this.h[0], this.node[0]];
    const n = this.f.length - 1;
    if (n === 0) {
      this.f.pop(); this.h.pop(); this.node.pop();
      return out;
    }
    this.f[0] = this.f[n];
    this.h[0] = this.h[n];
    this.node[0] = this.node[n];
    this.f.pop(); this.h.pop(); this.node.pop();
    let i = 0;
    for (;;) {
      const l = (i << 1) + 1;
      const r = l + 1;
      let m = i;
      if (l < this.f.length && this.less(l, m)) m = l;
      if (r < this.f.length && this.less(r, m)) m = r;
      if (m === i) break;
      this.swap(i, m);
      i = m;
    }
    return out;
  }
}

function manhattan(x, y, gx, gy) {
  return Math.abs(x - gx) + Math.abs(y - gy);
}

function octile(x, y, gx, gy) {
  const dx = Math.abs(x - gx);
  const dy = Math.abs(y - gy);
  return dx < dy ? dx * SQRT2 + (dy - dx) : dy * SQRT2 + (dx - dy);
}

function reconstruct(parent, goal, cost, expanded) {
  const out = [];
  let n = goal;
  while (n !== -1) {
    out.push(n);
    n = parent[n];
  }
  out.reverse();
  return { cost, expanded, path: out };
}

function baselineAStar(grid, sx, sy, gx, gy, diagonal) {
  const { w, h, n: ncell, cell } = grid;
  const start = sy * w + sx;
  const goal = gy * w + gx;
  const gs = new Float64Array(ncell).fill(INF);
  const parent = new Int32Array(ncell).fill(-1);
  const closed = new Uint8Array(ncell);
  gs[start] = 0;
  const h0 = diagonal ? octile(sx, sy, gx, gy) : manhattan(sx, sy, gx, gy);
  const heap = new Heap8();
  heap.push(h0, h0, start);
  let expanded = 0;
  while (heap.size) {
    const [f, hh, n] = heap.pop();
    if (closed[n]) continue;
    if (n === goal) return reconstruct(parent, goal, gs[n], expanded);
    closed[n] = 1;
    expanded++;
    const x = n % w;
    const y = (n / w) | 0;
    const g0 = gs[n];
    for (let dy = -1; dy <= 1; dy++) {
      const ny = y + dy;
      if (ny < 0 || ny >= h) continue;
      for (let dx = -1; dx <= 1; dx++) {
        if (dx === 0 && dy === 0) continue;
        if (!diagonal && dx !== 0 && dy !== 0) continue;
        const nx = x + dx;
        if (nx < 0 || nx >= w) continue;
        if (cell[ny * w + nx]) continue;
        if (dx !== 0 && dy !== 0 && (cell[y * w + nx] || cell[ny * w + x])) continue;
        const nn = ny * w + nx;
        const ng = g0 + (dx !== 0 && dy !== 0 ? SQRT2 : 1);
        if (ng < gs[nn]) {
          gs[nn] = ng;
          parent[nn] = n;
          const hn = diagonal ? octile(nx, ny, gx, gy) : manhattan(nx, ny, gx, gy);
          heap.push(ng + hn, hn, nn);
        }
      }
    }
  }
  return { cost: INF, expanded, path: [] };
}

function corridorHeavy(grid) {
  const { w, h, cell } = grid;
  let state = 0x9E3779B9;
  let freeCells = 0;
  let straight = 0;
  for (let i = 0; i < 64; i++) {
    state = (Math.imul(state, 1664525) + 1013904223) >>> 0;
    const x = (state >>> 16) % w;
    state = (Math.imul(state, 1664525) + 1013904223) >>> 0;
    const y = (state >>> 16) % h;
    if (cell[y * w + x]) continue;
    freeCells++;
    const up = y > 0 && !cell[(y - 1) * w + x];
    const down = y + 1 < h && !cell[(y + 1) * w + x];
    const left = x > 0 && !cell[y * w + x - 1];
    const right = x + 1 < w && !cell[y * w + x + 1];
    if ((up && down && !left && !right) || (left && right && !up && !down)) straight++;
  }
  return freeCells >= 16 && straight * 10 >= freeCells * 3;
}

function jump4(grid, x, y, dx, dy, goal) {
  const { w, h, cell } = grid;
  let jx = x + dx;
  let jy = y + dy;
  let jn = jy * w + jx;
  let steps = 1;
  if (dx !== 0) {
    while (jn !== goal) {
      if (jy > 0 && !cell[(jy - 1) * w + jx]) break;
      if (jy + 1 < h && !cell[(jy + 1) * w + jx]) break;
      const nx = jx + dx;
      if (nx < 0 || nx >= w || cell[jy * w + nx]) break;
      jx = nx;
      jn = jy * w + jx;
      steps++;
    }
  } else {
    while (jn !== goal) {
      if (jx > 0 && !cell[jy * w + jx - 1]) break;
      if (jx + 1 < w && !cell[jy * w + jx + 1]) break;
      const ny = jy + dy;
      if (ny < 0 || ny >= h || cell[ny * w + jx]) break;
      jy = ny;
      jn = jy * w + jx;
      steps++;
    }
  }
  return [jn, steps];
}

function nodeBitsFor(ncell) {
  let bits = 1;
  while (2 ** bits < ncell) bits++;
  return bits;
}

function cja4Search(grid, sx, sy, gx, gy, useJump) {
  const { w, h, n: ncell, cell } = grid;
  const start = sy * w + sx;
  const goal = gy * w + gx;
  const gs = new Int32Array(ncell).fill(0x7FFFFFFF);
  const parent = new Int32Array(ncell).fill(-1);
  const closed = new Uint8Array(ncell);
  gs[start] = 0;
  const keybase = w + h + 1;
  const nbits = nodeBitsFor(ncell);
  const nodeMul = 2 ** nbits;
  const nodeMask = nodeMul - 1;
  const h0 = manhattan(sx, sy, gx, gy);
  const heap = new Heap4();
  heap.push((h0 * keybase + h0) * nodeMul + start);
  let expanded = 0;
  const DX = [0, 1, 0, -1];
  const DY = [-1, 0, 1, 0];
  while (heap.size) {
    const item = heap.pop();
    const n = item % nodeMul;
    if (closed[n]) continue;
    if (n === goal) return reconstruct(parent, goal, gs[n], expanded);
    closed[n] = 1;
    expanded++;
    const x = n % w;
    const y = (n / w) | 0;
    const g0 = gs[n];
    for (let d = 0; d < 4; d++) {
      const dx = DX[d], dy = DY[d];
      const nx = x + dx, ny = y + dy;
      if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
      if (cell[ny * w + nx]) continue;
      let jn, steps;
      if (useJump) [jn, steps] = jump4(grid, x, y, dx, dy, goal);
      else { jn = ny * w + nx; steps = 1; }
      const ng = g0 + steps;
      if (ng < gs[jn]) {
        gs[jn] = ng;
        parent[jn] = n;
        const jx = jn % w;
        const jy = (jn / w) | 0;
        const hn = manhattan(jx, jy, gx, gy);
        heap.push((ng + hn) * keybase * nodeMul + hn * nodeMul + jn);
      }
    }
  }
  return { cost: INF, expanded, path: [] };
}

function cja8Search(grid, sx, sy, gx, gy, useJump) {
  const { w, h, n: ncell, cell } = grid;
  const start = sy * w + sx;
  const goal = gy * w + gx;
  const gs = new Float64Array(ncell).fill(INF);
  const parent = new Int32Array(ncell).fill(-1);
  const closed = new Uint8Array(ncell);
  gs[start] = 0;
  const heap = new Heap8();
  const h0 = octile(sx, sy, gx, gy);
  heap.push(h0, h0, start);
  let expanded = 0;
  const DX = [0, 1, 0, -1, 1, 1, -1, -1];
  const DY = [-1, 0, 1, 0, -1, 1, 1, -1];
  while (heap.size) {
    const e = heap.pop();
    const n = e[2];
    if (closed[n]) continue;
    if (n === goal) return reconstruct(parent, goal, gs[n], expanded);
    closed[n] = 1;
    expanded++;
    const x = n % w;
    const y = (n / w) | 0;
    const g0 = gs[n];
    for (let d = 0; d < 8; d++) {
      const dx = DX[d], dy = DY[d];
      const nx = x + dx, ny = y + dy;
      if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
      if (cell[ny * w + nx]) continue;
      let jn, steps;
      if (dx !== 0 && dy !== 0) {
        if (cell[y * w + nx] || cell[ny * w + x]) continue;
        jn = ny * w + nx;
        steps = 1;
      } else if (useJump) {
        [jn, steps] = jump4(grid, x, y, dx, dy, goal);
      } else {
        jn = ny * w + nx;
        steps = 1;
      }
      const ng = g0 + (dx !== 0 && dy !== 0 ? SQRT2 : 1) * steps;
      if (ng < gs[jn]) {
        gs[jn] = ng;
        parent[jn] = n;
        const jx = jn % w;
        const jy = (jn / w) | 0;
        const hn = octile(jx, jy, gx, gy);
        heap.push(ng + hn, hn, jn);
      }
    }
  }
  return { cost: INF, expanded, path: [] };
}

function cjaStar(grid, sx, sy, gx, gy, diagonal) {
  const heavy = corridorHeavy(grid);
  return diagonal
    ? cja8Search(grid, sx, sy, gx, gy, heavy)
    : cja4Search(grid, sx, sy, gx, gy, heavy);
}

function loadFixture(file) {
  const text = fs.readFileSync(file, "utf8");
  const lines = text.split(/\r?\n/);
  const meta = {};
  let i = 0;
  for (; i < lines.length; i++) {
    const line = lines[i].trim();
    if (!line || line.startsWith("#")) continue;
    if (line === "GRID") { i++; break; }
    const sp = line.indexOf(" ");
    meta[line.slice(0, sp)] = line.slice(sp + 1).trim();
  }
  const w = parseInt(meta.W, 10);
  const h = parseInt(meta.H, 10);
  const rows = [];
  while (i < lines.length && rows.length < h) {
    const line = lines[i++].trim();
    if (line) rows.push(line);
  }
  const cell = new Uint8Array(w * h);
  for (let y = 0; y < h; y++) {
    for (let x = 0; x < w; x++) {
      if (rows[y][x] !== ".") cell[y * w + x] = 1;
    }
  }
  const [sx, sy] = meta.START.split(/\s+/).map(Number);
  const [gx, gy] = meta.GOAL.split(/\s+/).map(Number);
  return {
    name: path.basename(file),
    grid: { w, h, n: w * h, cell },
    diagonal: meta.MODE === "8",
    sx, sy, gx, gy,
    baseCost: parseFloat(meta.BASE_COST),
    cjaCost: parseFloat(meta.CJA_COST),
    baseNodes: parseInt(meta.BASE_NODES, 10),
    cjaNodes: parseInt(meta.CJA_NODES, 10),
  };
}

function manifestPaths(manifest) {
  const dir = path.dirname(manifest);
  return fs.readFileSync(manifest, "utf8").split(/\r?\n/)
    .map((s) => s.trim())
    .filter((s) => s && !s.startsWith("#"))
    .map((s) => path.join(dir, s));
}

function verify(manifest) {
  const paths = manifestPaths(manifest);
  let pass = 0;
  for (const file of paths) {
    const fx = loadFixture(file);
    const b = baselineAStar(fx.grid, fx.sx, fx.sy, fx.gx, fx.gy, fx.diagonal);
    const c = cjaStar(fx.grid, fx.sx, fx.sy, fx.gx, fx.gy, fx.diagonal);
    const ok = Math.abs(b.cost - fx.baseCost) < 1e-9 && b.expanded === fx.baseNodes &&
      Math.abs(c.cost - fx.cjaCost) < 1e-9 && c.expanded === fx.cjaNodes;
    console.log(`VERIFY ${fx.name} ${ok ? "PASS" : "FAIL"} ` +
      `base=${b.cost.toFixed(9)}/${b.expanded} cja=${c.cost.toFixed(9)}/${c.expanded}`);
    if (ok) pass++;
  }
  console.log(`SUMMARY language=javascript cases=${paths.length} pass=${pass} fail=${paths.length - pass}`);
  return pass === paths.length ? 0 : 1;
}

function bench(manifest, reps) {
  const paths = manifestPaths(manifest);
  let baseMs = 0, cjaMs = 0;
  let baseNodes = 0, cjaNodes = 0;
  for (const file of paths) {
    const fx = loadFixture(file);
    baselineAStar(fx.grid, fx.sx, fx.sy, fx.gx, fx.gy, fx.diagonal);
    cjaStar(fx.grid, fx.sx, fx.sy, fx.gx, fx.gy, fx.diagonal);
    let bestB = Infinity, bestC = Infinity;
    let bn = 0, cn = 0;
    for (let r = 0; r < reps; r++) {
      let t0 = process.hrtime.bigint();
      const b = baselineAStar(fx.grid, fx.sx, fx.sy, fx.gx, fx.gy, fx.diagonal);
      let t1 = process.hrtime.bigint();
      const c = cjaStar(fx.grid, fx.sx, fx.sy, fx.gx, fx.gy, fx.diagonal);
      let t2 = process.hrtime.bigint();
      const bms = Number(t1 - t0) / 1e6;
      const cms = Number(t2 - t1) / 1e6;
      if (bms < bestB) bestB = bms;
      if (cms < bestC) bestC = cms;
      bn = b.expanded;
      cn = c.expanded;
    }
    baseMs += bestB;
    cjaMs += bestC;
    baseNodes += bn;
    cjaNodes += cn;
  }
  const n = paths.length;
  console.log(`BENCH language=javascript cases=${n} reps=${reps} ` +
    `baseline_ms=${(baseMs / n).toFixed(6)} cja_ms=${(cjaMs / n).toFixed(6)} ` +
    `speedup=${(baseMs / cjaMs).toFixed(4)} ` +
    `baseline_nodes=${(baseNodes / n).toFixed(1)} cja_nodes=${(cjaNodes / n).toFixed(1)}`);
  return 0;
}

function main() {
  const args = process.argv.slice(2);
  if (args.length < 2) {
    console.error("usage: node cja_star.js verify|bench <manifest> [reps]");
    process.exit(2);
  }
  if (args[0] === "verify") process.exit(verify(args[1]));
  if (args[0] === "bench") {
    const reps = args[2] ? parseInt(args[2], 10) : 5;
    process.exit(bench(args[1], reps));
  }
  console.error("unknown action");
  process.exit(2);
}

if (require.main === module) main();

module.exports = { loadFixture, manifestPaths, baselineAStar, cjaStar, corridorHeavy };
