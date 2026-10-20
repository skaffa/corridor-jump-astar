//! CJA* — Corridor-Jump A* (Rust port)
//!
//! Build:
//!   rustc -O cja_star.rs -o cja_star
//!
//! Use:
//!   cja_star verify tests/fixtures/manifest.txt
//!   cja_star bench  tests/fixtures/manifest.txt 5

use std::cmp::Ordering;
use std::collections::BinaryHeap;
use std::env;
use std::fs;
use std::path::{Path, PathBuf};
use std::time::Instant;

const SQRT2: f64 = std::f64::consts::SQRT_2;
const INF: f64 = f64::INFINITY;

#[derive(Clone, Copy)]
struct Grid {
    w: usize,
    h: usize,
    n: usize,
    cell: *const u8,
}

impl Grid {
    #[inline]
    fn free(&self, x: usize, y: usize) -> bool {
        unsafe { *self.cell.add(y * self.w + x) == 0 }
    }
}

#[derive(Clone, Copy)]
struct Entry8 {
    f: f64,
    h: f64,
    node: usize,
}

impl PartialEq for Entry8 {
    fn eq(&self, other: &Self) -> bool {
        self.f == other.f && self.h == other.h && self.node == other.node
    }
}
impl Eq for Entry8 {}
impl PartialOrd for Entry8 {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}
impl Ord for Entry8 {
    fn cmp(&self, other: &Self) -> Ordering {
        self.f
            .partial_cmp(&other.f)
            .unwrap()
            .then_with(|| self.h.partial_cmp(&other.h).unwrap())
            .then_with(|| self.node.cmp(&other.node))
    }
}

struct SearchResult {
    cost: f64,
    expanded: usize,
    path: Vec<usize>,
}

fn manhattan(x: usize, y: usize, gx: usize, gy: usize) -> f64 {
    let dx = if x > gx { x - gx } else { gx - x };
    let dy = if y > gy { y - gy } else { gy - y };
    (dx + dy) as f64
}

fn octile(x: usize, y: usize, gx: usize, gy: usize) -> f64 {
    let dx = if x > gx { x - gx } else { gx - x };
    let dy = if y > gy { y - gy } else { gy - y };
    if dx < dy {
        dx as f64 * SQRT2 + (dy - dx) as f64
    } else {
        dy as f64 * SQRT2 + (dx - dy) as f64
    }
}

fn reconstruct(parent: &[i32], goal: usize, cost: f64, expanded: usize) -> SearchResult {
    let mut path = Vec::new();
    let mut n = goal as i32;
    while n != -1 {
        path.push(n as usize);
        n = parent[n as usize];
    }
    path.reverse();
    SearchResult { cost, expanded, path }
}

fn baseline_astar(
    grid: &Grid,
    sx: usize,
    sy: usize,
    gx: usize,
    gy: usize,
    diagonal: bool,
) -> SearchResult {
    let w = grid.w;
    let h = grid.h;
    let ncell = grid.n;
    let start = sy * w + sx;
    let goal = gy * w + gx;
    let mut gs = vec![INF; ncell];
    let mut parent = vec![-1i32; ncell];
    let mut closed = vec![false; ncell];
    gs[start] = 0.0;
    let h0 = if diagonal { octile(sx, sy, gx, gy) } else { manhattan(sx, sy, gx, gy) };
    let mut heap: BinaryHeap<std::cmp::Reverse<Entry8>> = BinaryHeap::new();
    heap.push(std::cmp::Reverse(Entry8 { f: h0, h: h0, node: start }));
    let mut expanded = 0usize;
    while let Some(std::cmp::Reverse(e)) = heap.pop() {
        let n = e.node;
        if closed[n] {
            continue;
        }
        if n == goal {
            return reconstruct(&parent, goal, gs[n], expanded);
        }
        closed[n] = true;
        expanded += 1;
        let x = n % w;
        let y = n / w;
        let g0 = gs[n];
        for dy in -1i32..=1 {
            let nyi = y as i32 + dy;
            if nyi < 0 || nyi >= h as i32 {
                continue;
            }
            let ny = nyi as usize;
            for dx in -1i32..=1 {
                if dx == 0 && dy == 0 {
                    continue;
                }
                if !diagonal && dx != 0 && dy != 0 {
                    continue;
                }
                let nxi = x as i32 + dx;
                if nxi < 0 || nxi >= w as i32 {
                    continue;
                }
                let nx = nxi as usize;
                if !grid.free(nx, ny) {
                    continue;
                }
                if dx != 0 && dy != 0 && (!grid.free(nx, y) || !grid.free(x, ny)) {
                    continue;
                }
                let nn = ny * w + nx;
                let ng = g0 + if dx != 0 && dy != 0 { SQRT2 } else { 1.0 };
                if ng < gs[nn] {
                    gs[nn] = ng;
                    parent[nn] = n as i32;
                    let hn = if diagonal { octile(nx, ny, gx, gy) } else { manhattan(nx, ny, gx, gy) };
                    heap.push(std::cmp::Reverse(Entry8 { f: ng + hn, h: hn, node: nn }));
                }
            }
        }
    }
    SearchResult { cost: INF, expanded, path: Vec::new() }
}

fn corridor_heavy(grid: &Grid) -> bool {
    let mut state: u32 = 0x9E37_79B9;
    let mut free_cells = 0i32;
    let mut straight = 0i32;
    for _ in 0..64 {
        state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        let x = ((state >> 16) as usize) % grid.w;
        state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        let y = ((state >> 16) as usize) % grid.h;
        if !grid.free(x, y) {
            continue;
        }
        free_cells += 1;
        let up = y > 0 && grid.free(x, y - 1);
        let down = y + 1 < grid.h && grid.free(x, y + 1);
        let left = x > 0 && grid.free(x - 1, y);
        let right = x + 1 < grid.w && grid.free(x + 1, y);
        if (up && down && !left && !right) || (left && right && !up && !down) {
            straight += 1;
        }
    }
    free_cells >= 16 && straight * 10 >= free_cells * 3
}

fn jump4(
    grid: &Grid,
    x: usize,
    y: usize,
    dx: isize,
    dy: isize,
    goal: usize,
) -> (usize, usize) {
    let mut jx = (x as isize + dx) as usize;
    let mut jy = (y as isize + dy) as usize;
    let mut jn = jy * grid.w + jx;
    let mut steps = 1usize;
    if dx != 0 {
        while jn != goal {
            if jy > 0 && grid.free(jx, jy - 1) {
                break;
            }
            if jy + 1 < grid.h && grid.free(jx, jy + 1) {
                break;
            }
            let nx = jx as isize + dx;
            if nx < 0 || nx >= grid.w as isize || !grid.free(nx as usize, jy) {
                break;
            }
            jx = nx as usize;
            jn = jy * grid.w + jx;
            steps += 1;
        }
    } else {
        while jn != goal {
            if jx > 0 && grid.free(jx - 1, jy) {
                break;
            }
            if jx + 1 < grid.w && grid.free(jx + 1, jy) {
                break;
            }
            let ny = jy as isize + dy;
            if ny < 0 || ny >= grid.h as isize || !grid.free(jx, ny as usize) {
                break;
            }
            jy = ny as usize;
            jn = jy * grid.w + jx;
            steps += 1;
        }
    }
    (jn, steps)
}

fn node_bits_for(ncell: usize) -> u32 {
    let mut bits = 1u32;
    while (1u64 << bits) < ncell as u64 {
        bits += 1;
    }
    bits
}

fn cja4_search<const USE_JUMP: bool>(
    grid: &Grid,
    sx: usize,
    sy: usize,
    gx: usize,
    gy: usize,
) -> SearchResult {
    let w = grid.w;
    let h = grid.h;
    let ncell = grid.n;
    let start = sy * w + sx;
    let goal = gy * w + gx;
    let mut gs = vec![i32::MAX; ncell];
    let mut parent = vec![-1i32; ncell];
    let mut closed = vec![false; ncell];
    gs[start] = 0;
    let keybase = (w + h + 1) as i64;
    let nbits = node_bits_for(ncell);
    let nmask = (1u64 << nbits) - 1;
    let h0 = (if sx > gx { sx - gx } else { gx - sx }) + (if sy > gy { sy - gy } else { gy - sy });
    let mut heap: BinaryHeap<std::cmp::Reverse<u64>> = BinaryHeap::new();
    heap.push(std::cmp::Reverse((((h0 as i64) * keybase + h0 as i64) as u64) << nbits | start as u64));
    let mut expanded = 0usize;
    const DX: [isize; 4] = [0, 1, 0, -1];
    const DY: [isize; 4] = [-1, 0, 1, 0];
    while let Some(std::cmp::Reverse(item)) = heap.pop() {
        let n = (item & nmask) as usize;
        if closed[n] {
            continue;
        }
        if n == goal {
            return reconstruct(&parent, goal, gs[n] as f64, expanded);
        }
        closed[n] = true;
        expanded += 1;
        let x = n % w;
        let y = n / w;
        let g0 = gs[n];
        for d in 0..4 {
            let dx = DX[d];
            let dy = DY[d];
            let nxi = x as isize + dx;
            let nyi = y as isize + dy;
            if nxi < 0 || nxi >= w as isize || nyi < 0 || nyi >= h as isize {
                continue;
            }
            let nx = nxi as usize;
            let ny = nyi as usize;
            if !grid.free(nx, ny) {
                continue;
            }
            let (jn, steps) = if USE_JUMP {
                jump4(grid, x, y, dx, dy, goal)
            } else {
                (ny * w + nx, 1)
            };
            let ng = g0 + steps as i32;
            if ng < gs[jn] {
                gs[jn] = ng;
                parent[jn] = n as i32;
                let jx = jn % w;
                let jy = jn / w;
                let hn = ((if jx > gx { jx - gx } else { gx - jx })
                    + (if jy > gy { jy - gy } else { gy - jy })) as i64;
                let f = ng as i64 + hn;
                let key = ((f * keybase + hn) as u64) << nbits | jn as u64;
                heap.push(std::cmp::Reverse(key));
            }
        }
    }
    SearchResult { cost: INF, expanded, path: Vec::new() }
}

fn cja8_search<const USE_JUMP: bool>(
    grid: &Grid,
    sx: usize,
    sy: usize,
    gx: usize,
    gy: usize,
) -> SearchResult {
    let w = grid.w;
    let h = grid.h;
    let ncell = grid.n;
    let start = sy * w + sx;
    let goal = gy * w + gx;
    let mut gs = vec![INF; ncell];
    let mut parent = vec![-1i32; ncell];
    let mut closed = vec![false; ncell];
    gs[start] = 0.0;
    let h0 = octile(sx, sy, gx, gy);
    let mut heap: BinaryHeap<std::cmp::Reverse<Entry8>> = BinaryHeap::new();
    heap.push(std::cmp::Reverse(Entry8 { f: h0, h: h0, node: start }));
    let mut expanded = 0usize;
    const DX: [isize; 8] = [0, 1, 0, -1, 1, 1, -1, -1];
    const DY: [isize; 8] = [-1, 0, 1, 0, -1, 1, 1, -1];
    while let Some(std::cmp::Reverse(e)) = heap.pop() {
        let n = e.node;
        if closed[n] {
            continue;
        }
        if n == goal {
            return reconstruct(&parent, goal, gs[n], expanded);
        }
        closed[n] = true;
        expanded += 1;
        let x = n % w;
        let y = n / w;
        let g0 = gs[n];
        for d in 0..8 {
            let dx = DX[d];
            let dy = DY[d];
            let nxi = x as isize + dx;
            let nyi = y as isize + dy;
            if nxi < 0 || nxi >= w as isize || nyi < 0 || nyi >= h as isize {
                continue;
            }
            let nx = nxi as usize;
            let ny = nyi as usize;
            if !grid.free(nx, ny) {
                continue;
            }
            let (jn, steps) = if dx != 0 && dy != 0 {
                if !grid.free(nx, y) || !grid.free(x, ny) {
                    continue;
                }
                (ny * w + nx, 1)
            } else if USE_JUMP {
                jump4(grid, x, y, dx, dy, goal)
            } else {
                (ny * w + nx, 1)
            };
            let ng = g0 + if dx != 0 && dy != 0 { SQRT2 } else { 1.0 } * steps as f64;
            if ng < gs[jn] {
                gs[jn] = ng;
                parent[jn] = n as i32;
                let jx = jn % w;
                let jy = jn / w;
                let hn = octile(jx, jy, gx, gy);
                heap.push(std::cmp::Reverse(Entry8 { f: ng + hn, h: hn, node: jn }));
            }
        }
    }
    SearchResult { cost: INF, expanded, path: Vec::new() }
}

fn cja_star(
    grid: &Grid,
    sx: usize,
    sy: usize,
    gx: usize,
    gy: usize,
    diagonal: bool,
) -> SearchResult {
    let heavy = corridor_heavy(grid);
    if diagonal {
        if heavy { cja8_search::<true>(grid, sx, sy, gx, gy) } else { cja8_search::<false>(grid, sx, sy, gx, gy) }
    } else if heavy {
        cja4_search::<true>(grid, sx, sy, gx, gy)
    } else {
        cja4_search::<false>(grid, sx, sy, gx, gy)
    }
}

struct Fixture {
    name: String,
    diagonal: bool,
    sx: usize,
    sy: usize,
    gx: usize,
    gy: usize,
    base_cost: f64,
    cja_cost: f64,
    base_nodes: usize,
    cja_nodes: usize,
    grid: Grid,
    cells: Vec<u8>,
}

fn load_fixture(path: &Path) -> Result<Fixture, String> {
    let text = fs::read_to_string(path).map_err(|e| e.to_string())?;
    let mut w = 0usize;
    let mut h = 0usize;
    let mut diagonal = false;
    let mut sx = 0usize;
    let mut sy = 0usize;
    let mut gx = 0usize;
    let mut gy = 0usize;
    let mut base_cost = 0.0;
    let mut cja_cost = 0.0;
    let mut base_nodes = 0usize;
    let mut cja_nodes = 0usize;
    let mut lines = text.lines();
    let mut grid_started = false;
    let mut rows: Vec<String> = Vec::new();
    for line in lines.by_ref() {
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        if line == "GRID" {
            grid_started = true;
            break;
        }
        let mut parts = line.split_whitespace();
        let key = parts.next().unwrap();
        match key {
            "W" => w = parts.next().unwrap().parse().unwrap(),
            "H" => h = parts.next().unwrap().parse().unwrap(),
            "MODE" => diagonal = parts.next().unwrap() == "8",
            "START" => {
                sx = parts.next().unwrap().parse().unwrap();
                sy = parts.next().unwrap().parse().unwrap();
            }
            "GOAL" => {
                gx = parts.next().unwrap().parse().unwrap();
                gy = parts.next().unwrap().parse().unwrap();
            }
            "BASE_COST" => base_cost = parts.next().unwrap().parse().unwrap(),
            "CJA_COST" => cja_cost = parts.next().unwrap().parse().unwrap(),
            "BASE_NODES" => base_nodes = parts.next().unwrap().parse().unwrap(),
            "CJA_NODES" => cja_nodes = parts.next().unwrap().parse().unwrap(),
            _ => {}
        }
    }
    if !grid_started || w == 0 || h == 0 {
        return Err("bad fixture header".to_string());
    }
    for line in lines {
        let line = line.trim();
        if !line.is_empty() {
            rows.push(line.to_string());
        }
        if rows.len() == h {
            break;
        }
    }
    if rows.len() != h {
        return Err("bad grid".to_string());
    }
    let mut cells = vec![0u8; w * h];
    for y in 0..h {
        for (x, ch) in rows[y].bytes().enumerate().take(w) {
            cells[y * w + x] = if ch == b'.' { 0 } else { 1 };
        }
    }
    let grid = Grid { w, h, n: w * h, cell: cells.as_ptr() };
    Ok(Fixture {
        name: path.file_name().unwrap().to_string_lossy().to_string(),
        diagonal,
        sx,
        sy,
        gx,
        gy,
        base_cost,
        cja_cost,
        base_nodes,
        cja_nodes,
        grid,
        cells,
    })
}

fn manifest_paths(manifest: &Path) -> Result<Vec<PathBuf>, String> {
    let text = fs::read_to_string(manifest).map_err(|e| e.to_string())?;
    let dir = manifest.parent().unwrap_or_else(|| Path::new("."));
    Ok(text
        .lines()
        .map(str::trim)
        .filter(|l| !l.is_empty() && !l.starts_with('#'))
        .map(|l| dir.join(l))
        .collect())
}

fn verify(manifest: &Path) -> i32 {
    let paths = match manifest_paths(manifest) {
        Ok(p) => p,
        Err(e) => {
            eprintln!("{e}");
            return 2;
        }
    };
    let mut passed = 0usize;
    for path in &paths {
        let fx = match load_fixture(path) {
            Ok(f) => f,
            Err(e) => {
                println!("VERIFY {} FAIL {e}", path.display());
                continue;
            }
        };
        let b = baseline_astar(&fx.grid, fx.sx, fx.sy, fx.gx, fx.gy, fx.diagonal);
        let c = cja_star(&fx.grid, fx.sx, fx.sy, fx.gx, fx.gy, fx.diagonal);
        let ok = b.cost.is_finite()
            && c.cost.is_finite()
            && (b.cost - fx.base_cost).abs() < 1e-9
            && b.expanded == fx.base_nodes
            && (c.cost - fx.cja_cost).abs() < 1e-9
            && c.expanded == fx.cja_nodes;
        println!(
            "VERIFY {} {} base={:.9}/{:.0} cja={:.9}/{:.0}",
            fx.name,
            if ok { "PASS" } else { "FAIL" },
            b.cost,
            b.expanded,
            c.cost,
            c.expanded
        );
        if ok {
            passed += 1;
        }
    }
    println!(
        "SUMMARY language=rust cases={} pass={} fail={}",
        paths.len(),
        passed,
        paths.len() - passed
    );
    if passed == paths.len() { 0 } else { 1 }
}

fn bench(manifest: &Path, reps: usize) -> i32 {
    let paths = match manifest_paths(manifest) {
        Ok(p) => p,
        Err(e) => {
            eprintln!("{e}");
            return 2;
        }
    };
    let mut base_ms = 0.0;
    let mut cja_ms = 0.0;
    let mut base_nodes = 0usize;
    let mut cja_nodes = 0usize;
    for path in &paths {
        let fx = match load_fixture(path) {
            Ok(f) => f,
            Err(e) => {
                eprintln!("{e}");
                return 2;
            }
        };
        let _ = baseline_astar(&fx.grid, fx.sx, fx.sy, fx.gx, fx.gy, fx.diagonal);
        let _ = cja_star(&fx.grid, fx.sx, fx.sy, fx.gx, fx.gy, fx.diagonal);
        let mut best_b = f64::MAX;
        let mut best_c = f64::MAX;
        let mut bn = 0usize;
        let mut cn = 0usize;
        for _ in 0..reps {
            let t0 = Instant::now();
            let b = baseline_astar(&fx.grid, fx.sx, fx.sy, fx.gx, fx.gy, fx.diagonal);
            let t1 = Instant::now();
            let c = cja_star(&fx.grid, fx.sx, fx.sy, fx.gx, fx.gy, fx.diagonal);
            let t2 = Instant::now();
            best_b = best_b.min(t1.duration_since(t0).as_secs_f64() * 1000.0);
            best_c = best_c.min(t2.duration_since(t1).as_secs_f64() * 1000.0);
            bn = b.expanded;
            cn = c.expanded;
        }
        base_ms += best_b;
        cja_ms += best_c;
        base_nodes += bn;
        cja_nodes += cn;
    }
    let cases = paths.len() as f64;
    println!(
        "BENCH language=rust cases={} reps={} baseline_ms={:.6} cja_ms={:.6} \
         speedup={:.4} baseline_nodes={:.1} cja_nodes={:.1}",
        paths.len(),
        reps,
        base_ms / cases,
        cja_ms / cases,
        base_ms / cja_ms,
        base_nodes as f64 / cases,
        cja_nodes as f64 / cases
    );
    0
}

fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() < 3 {
        eprintln!("usage: {} verify|bench <manifest> [reps]", args[0]);
        std::process::exit(2);
    }
    let code = match args[1].as_str() {
        "verify" => verify(Path::new(&args[2])),
        "bench" => {
            let reps = args.get(3).and_then(|s| s.parse().ok()).unwrap_or(5);
            bench(Path::new(&args[2]), reps)
        }
        _ => {
            eprintln!("unknown action");
            2
        }
    };
    std::process::exit(code);
}
