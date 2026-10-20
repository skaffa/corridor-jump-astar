/*
 * CJA* — Corridor-Jump A*  (C port)
 *
 * Build:
 *   cc -O3 -std=c11 -o cja_star cja_star.c -lm
 *
 * Use:
 *   ./cja_star verify tests/fixtures/manifest.txt
 *   ./cja_star bench  tests/fixtures/manifest.txt 5
 *
 * The search returns the optimal cost and a jump-node path.  Macro edges may
 * skip degree-2 corridor cells; the returned cost is still exact.
 */

#if !defined(_WIN32) && !defined(_POSIX_C_SOURCE)
#define _POSIX_C_SOURCE 200809L
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <time.h>

#ifdef _WIN32
#include <windows.h>
#endif

#define SQRT2 1.41421356237309504880
#define INF_COST (1.0 / 0.0)

typedef struct {
    int w;
    int h;
    int n;
    unsigned char *cell;
} Grid;

typedef struct {
    double cost;
    int expanded;
    int found;
    int *path;
    int path_len;
} Result;

typedef struct {
    uint64_t *a;
    int n;
    int cap;
} Heap4;

typedef struct {
    double f;
    double h;
    int node;
} Entry8;

typedef struct {
    Entry8 *a;
    int n;
    int cap;
} Heap8;

static void *xmalloc(size_t n) {
    void *p = malloc(n);
    if (!p) {
        fprintf(stderr, "out of memory\n");
        exit(1);
    }
    return p;
}

static void *xrealloc(void *p, size_t n) {
    void *q = realloc(p, n);
    if (!q) {
        fprintf(stderr, "out of memory\n");
        exit(1);
    }
    return q;
}

static void heap4_push(Heap4 *h, uint64_t key) {
    if (h->n == h->cap) {
        h->cap = h->cap ? h->cap * 2 : 1024;
        h->a = (uint64_t *)xrealloc(h->a, (size_t)h->cap * sizeof(uint64_t));
    }
    int i = h->n++;
    h->a[i] = key;
    while (i > 0) {
        int p = (i - 1) >> 1;
        if (h->a[p] <= h->a[i]) break;
        uint64_t t = h->a[p];
        h->a[p] = h->a[i];
        h->a[i] = t;
        i = p;
    }
}

static uint64_t heap4_pop(Heap4 *h) {
    uint64_t out = h->a[0];
    --h->n;
    if (h->n == 0) return out;
    h->a[0] = h->a[h->n];
    int i = 0;
    for (;;) {
        int l = (i << 1) + 1;
        int r = l + 1;
        int m = i;
        if (l < h->n && h->a[l] < h->a[m]) m = l;
        if (r < h->n && h->a[r] < h->a[m]) m = r;
        if (m == i) break;
        uint64_t t = h->a[m];
        h->a[m] = h->a[i];
        h->a[i] = t;
        i = m;
    }
    return out;
}

static int entry8_less(const Entry8 *a, const Entry8 *b) {
    if (a->f < b->f) return 1;
    if (a->f > b->f) return 0;
    if (a->h < b->h) return 1;
    if (a->h > b->h) return 0;
    return a->node < b->node;
}

static void heap8_push(Heap8 *h, double f, double hh, int node) {
    if (h->n == h->cap) {
        h->cap = h->cap ? h->cap * 2 : 1024;
        h->a = (Entry8 *)xrealloc(h->a, (size_t)h->cap * sizeof(Entry8));
    }
    int i = h->n++;
    h->a[i].f = f;
    h->a[i].h = hh;
    h->a[i].node = node;
    while (i > 0) {
        int p = (i - 1) >> 1;
        if (!entry8_less(&h->a[i], &h->a[p])) break;
        Entry8 t = h->a[p];
        h->a[p] = h->a[i];
        h->a[i] = t;
        i = p;
    }
}

static Entry8 heap8_pop(Heap8 *h) {
    Entry8 out = h->a[0];
    --h->n;
    if (h->n == 0) return out;
    h->a[0] = h->a[h->n];
    int i = 0;
    for (;;) {
        int l = (i << 1) + 1;
        int r = l + 1;
        int m = i;
        if (l < h->n && entry8_less(&h->a[l], &h->a[m])) m = l;
        if (r < h->n && entry8_less(&h->a[r], &h->a[m])) m = r;
        if (m == i) break;
        Entry8 t = h->a[m];
        h->a[m] = h->a[i];
        h->a[i] = t;
        i = m;
    }
    return out;
}

static double manhattan(int x, int y, int gx, int gy) {
    int dx = x > gx ? x - gx : gx - x;
    int dy = y > gy ? y - gy : gy - y;
    return (double)(dx + dy);
}

static double octile(int x, int y, int gx, int gy) {
    int dx = x > gx ? x - gx : gx - x;
    int dy = y > gy ? y - gy : gy - y;
    if (dx < dy)
        return (double)dx * SQRT2 + (double)(dy - dx);
    return (double)dy * SQRT2 + (double)(dx - dy);
}

static Result make_result(double cost, int expanded, int *parent, int goal) {
    Result r;
    r.cost = cost;
    r.expanded = expanded;
    r.found = 1;
    int len = 0;
    for (int n = goal; n != -1; n = parent[n]) ++len;
    r.path = (int *)xmalloc((size_t)len * sizeof(int));
    r.path_len = len;
    int i = len - 1;
    for (int n = goal; n != -1; n = parent[n]) r.path[i--] = n;
    return r;
}

static void free_result(Result *r) {
    free(r->path);
    r->path = NULL;
}

/* ------------------------------------------------------------------ */
/* Standard baseline A*                                                */
/* ------------------------------------------------------------------ */

static Result baseline_astar(const Grid *g, int sx, int sy, int gx, int gy,
                             int diagonal) {
    int w = g->w, h = g->h, ncell = g->n;
    int start = sy * w + sx, goal = gy * w + gx;
    double *gs = (double *)xmalloc((size_t)ncell * sizeof(double));
    int *parent = (int *)xmalloc((size_t)ncell * sizeof(int));
    unsigned char *closed = (unsigned char *)calloc((size_t)ncell, 1);
    if (!closed) { fprintf(stderr, "out of memory\n"); exit(1); }
    for (int i = 0; i < ncell; ++i) { gs[i] = INF_COST; parent[i] = -1; }
    gs[start] = 0.0;
    Heap8 heap = {0, 0, 0};
    double h0 = diagonal ? octile(sx, sy, gx, gy) : manhattan(sx, sy, gx, gy);
    heap8_push(&heap, h0, h0, start);
    int expanded = 0;
    while (heap.n) {
        Entry8 e = heap8_pop(&heap);
        int n = e.node;
        if (closed[n]) continue;
        if (n == goal) {
            Result r = make_result(gs[n], expanded, parent, goal);
            free(gs); free(parent); free(closed); free(heap.a);
            return r;
        }
        closed[n] = 1;
        ++expanded;
        int x = n % w, y = n / w;
        double g0 = gs[n];
        for (int dy = -1; dy <= 1; ++dy) {
            int ny = y + dy;
            if (ny < 0 || ny >= h) continue;
            for (int dx = -1; dx <= 1; ++dx) {
                if (!dx && !dy) continue;
                if (!diagonal && dx && dy) continue;
                int nx = x + dx;
                if (nx < 0 || nx >= w) continue;
                if (g->cell[ny * w + nx]) continue;
                if (dx && dy && (g->cell[y * w + nx] || g->cell[ny * w + x]))
                    continue;
                int nn = ny * w + nx;
                double ng = g0 + ((dx && dy) ? SQRT2 : 1.0);
                if (ng < gs[nn]) {
                    gs[nn] = ng;
                    parent[nn] = n;
                    double hn = diagonal ? octile(nx, ny, gx, gy)
                                         : manhattan(nx, ny, gx, gy);
                    heap8_push(&heap, ng + hn, hn, nn);
                }
            }
        }
    }
    Result r = {INF_COST, expanded, 0, NULL, 0};
    free(gs); free(parent); free(closed); free(heap.a);
    return r;
}

/* ------------------------------------------------------------------ */
/* Corridor detection and jump primitives                              */
/* ------------------------------------------------------------------ */

static int corridor_heavy(const Grid *g) {
    uint32_t state = 0x9E3779B9u;
    int free_cells = 0, straight = 0;
    for (int i = 0; i < 64; ++i) {
        state = state * 1664525u + 1013904223u;
        int x = (int)((state >> 16) % (uint32_t)g->w);
        state = state * 1664525u + 1013904223u;
        int y = (int)((state >> 16) % (uint32_t)g->h);
        if (g->cell[y * g->w + x]) continue;
        ++free_cells;
        int up = y > 0 && !g->cell[(y - 1) * g->w + x];
        int down = y + 1 < g->h && !g->cell[(y + 1) * g->w + x];
        int left = x > 0 && !g->cell[y * g->w + x - 1];
        int right = x + 1 < g->w && !g->cell[y * g->w + x + 1];
        if ((up && down && !left && !right) ||
            (left && right && !up && !down)) ++straight;
    }
    return free_cells >= 16 && straight * 10 >= free_cells * 3;
}

static void jump4(const Grid *g, int x, int y, int dx, int dy, int goal,
                  int *out_node, int *out_steps) {
    int jx = x + dx, jy = y + dy;
    int jn = jy * g->w + jx;
    int steps = 1;
    if (dx) {
        while (jn != goal) {
            if (jy > 0 && !g->cell[(jy - 1) * g->w + jx]) break;
            if (jy + 1 < g->h && !g->cell[(jy + 1) * g->w + jx]) break;
            int nx = jx + dx;
            if (nx < 0 || nx >= g->w || g->cell[jy * g->w + nx]) break;
            jx = nx;
            jn = jy * g->w + jx;
            ++steps;
        }
    } else {
        while (jn != goal) {
            if (jx > 0 && !g->cell[jy * g->w + jx - 1]) break;
            if (jx + 1 < g->w && !g->cell[jy * g->w + jx + 1]) break;
            int ny = jy + dy;
            if (ny < 0 || ny >= g->h || g->cell[ny * g->w + jx]) break;
            jy = ny;
            jn = jy * g->w + jx;
            ++steps;
        }
    }
    *out_node = jn;
    *out_steps = steps;
}

/* ------------------------------------------------------------------ */
/* 4-connected CJA*                                                    */
/* ------------------------------------------------------------------ */

static int node_bits_for(int ncell) {
    int bits = 1;
    while ((1LL << bits) < (long long)ncell) ++bits;
    return bits;
}

static Result cja4_search(const Grid *g, int sx, int sy, int gx, int gy,
                          int use_jump) {
    int w = g->w, h = g->h, ncell = g->n;
    int start = sy * w + sx, goal = gy * w + gx;
    double *gs = (double *)xmalloc((size_t)ncell * sizeof(double));
    int *parent = (int *)xmalloc((size_t)ncell * sizeof(int));
    unsigned char *closed = (unsigned char *)calloc((size_t)ncell, 1);
    if (!closed) { fprintf(stderr, "out of memory\n"); exit(1); }
    for (int i = 0; i < ncell; ++i) { gs[i] = INF_COST; parent[i] = -1; }
    gs[start] = 0.0;

    int keybase = w + h + 1;
    int nbits = node_bits_for(ncell);
    uint64_t nmask = ((uint64_t)1 << nbits) - 1;
    int h0 = (sx > gx ? sx - gx : gx - sx) + (sy > gy ? sy - gy : gy - sy);
    uint64_t start_key = (uint64_t)(h0 * keybase + h0);
    Heap4 heap = {0, 0, 0};
    heap4_push(&heap, (start_key << nbits) | (uint64_t)start);
    int expanded = 0;

    static const int DX[4] = {0, 1, 0, -1};
    static const int DY[4] = {-1, 0, 1, 0};

    while (heap.n) {
        uint64_t item = heap4_pop(&heap);
        int n = (int)(item & nmask);
        if (closed[n]) continue;
        if (n == goal) {
            Result r = make_result(gs[n], expanded, parent, goal);
            free(gs); free(parent); free(closed); free(heap.a);
            return r;
        }
        closed[n] = 1;
        ++expanded;
        int x = n % w, y = n / w;
        int g0 = (int)gs[n];
        for (int d = 0; d < 4; ++d) {
            int nx = x + DX[d], ny = y + DY[d];
            if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
            if (g->cell[ny * w + nx]) continue;
            int jn, steps;
            if (use_jump) {
                jump4(g, x, y, DX[d], DY[d], goal, &jn, &steps);
            } else {
                jn = ny * w + nx;
                steps = 1;
            }
            int ng = g0 + steps;
            if ((double)ng < gs[jn]) {
                gs[jn] = (double)ng;
                parent[jn] = n;
                int jx = jn % w, jy = jn / w;
                int hn = (jx > gx ? jx - gx : gx - jx) +
                         (jy > gy ? jy - gy : gy - jy);
                int f = ng + hn;
                uint64_t key = ((uint64_t)(f * keybase + hn) << nbits) |
                               (uint64_t)jn;
                heap4_push(&heap, key);
            }
        }
    }
    Result r = {INF_COST, expanded, 0, NULL, 0};
    free(gs); free(parent); free(closed); free(heap.a);
    return r;
}

/* ------------------------------------------------------------------ */
/* 8-connected CJA*                                                    */
/* ------------------------------------------------------------------ */

static Result cja8_search(const Grid *g, int sx, int sy, int gx, int gy,
                          int use_jump) {
    int w = g->w, h = g->h, ncell = g->n;
    int start = sy * w + sx, goal = gy * w + gx;
    double *gs = (double *)xmalloc((size_t)ncell * sizeof(double));
    int *parent = (int *)xmalloc((size_t)ncell * sizeof(int));
    unsigned char *closed = (unsigned char *)calloc((size_t)ncell, 1);
    if (!closed) { fprintf(stderr, "out of memory\n"); exit(1); }
    for (int i = 0; i < ncell; ++i) { gs[i] = INF_COST; parent[i] = -1; }
    gs[start] = 0.0;

    Heap8 heap = {0, 0, 0};
    double h0 = octile(sx, sy, gx, gy);
    heap8_push(&heap, h0, h0, start);
    int expanded = 0;

    static const int DX[8] = {0, 1, 0, -1, 1, 1, -1, -1};
    static const int DY[8] = {-1, 0, 1, 0, -1, 1, 1, -1};

    while (heap.n) {
        Entry8 e = heap8_pop(&heap);
        int n = e.node;
        if (closed[n]) continue;
        if (n == goal) {
            Result r = make_result(gs[n], expanded, parent, goal);
            free(gs); free(parent); free(closed); free(heap.a);
            return r;
        }
        closed[n] = 1;
        ++expanded;
        int x = n % w, y = n / w;
        double g0 = gs[n];
        for (int d = 0; d < 8; ++d) {
            int dx = DX[d], dy = DY[d];
            int nx = x + dx, ny = y + dy;
            if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
            if (g->cell[ny * w + nx]) continue;
            int jn, steps;
            if (dx && dy) {
                if (g->cell[y * w + nx] || g->cell[ny * w + x]) continue;
                jn = ny * w + nx;
                steps = 1;
            } else if (use_jump) {
                jump4(g, x, y, dx, dy, goal, &jn, &steps);
            } else {
                jn = ny * w + nx;
                steps = 1;
            }
            double ng = g0 + ((dx && dy) ? SQRT2 : 1.0) * (double)steps;
            if (ng < gs[jn]) {
                gs[jn] = ng;
                parent[jn] = n;
                int jx = jn % w, jy = jn / w;
                double hn = octile(jx, jy, gx, gy);
                heap8_push(&heap, ng + hn, hn, jn);
            }
        }
    }
    Result r = {INF_COST, expanded, 0, NULL, 0};
    free(gs); free(parent); free(closed); free(heap.a);
    return r;
}

static Result cja_star(const Grid *g, int sx, int sy, int gx, int gy,
                       int diagonal) {
    int heavy = corridor_heavy(g);
    if (diagonal) return cja8_search(g, sx, sy, gx, gy, heavy);
    return cja4_search(g, sx, sy, gx, gy, heavy);
}

/* ------------------------------------------------------------------ */
/* Fixture I/O                                                         */
/* ------------------------------------------------------------------ */

typedef struct {
    char name[256];
    int w, h, diagonal;
    int sx, sy, gx, gy;
    double base_cost, cja_cost;
    int base_nodes, cja_nodes;
    Grid grid;
} Fixture;

static void strip_eol(char *s) {
    size_t n = strlen(s);
    while (n && (s[n - 1] == '\n' || s[n - 1] == '\r')) s[--n] = '\0';
}

static int load_fixture(const char *path, Fixture *fx) {
    FILE *f = fopen(path, "rb");
    if (!f) { perror(path); return 0; }
    memset(fx, 0, sizeof(*fx));
    const char *base = strrchr(path, '/');
    const char *base2 = strrchr(path, '\\');
    if (base2 && (!base || base2 > base)) base = base2;
    snprintf(fx->name, sizeof(fx->name), "%s", base ? base + 1 : path);
    char line[4096];
    int saw_grid = 0;
    while (fgets(line, sizeof(line), f)) {
        strip_eol(line);
        if (!line[0] || line[0] == '#') continue;
        if (!strcmp(line, "GRID")) { saw_grid = 1; break; }
        char key[64];
        if (sscanf(line, "%63s", key) != 1) continue;
        if (!strcmp(key, "W")) sscanf(line, "W %d", &fx->w);
        else if (!strcmp(key, "H")) sscanf(line, "H %d", &fx->h);
        else if (!strcmp(key, "MODE")) {
            int mode = 4;
            sscanf(line, "MODE %d", &mode);
            fx->diagonal = (mode == 8);
        } else if (!strcmp(key, "START")) sscanf(line, "START %d %d", &fx->sx, &fx->sy);
        else if (!strcmp(key, "GOAL")) sscanf(line, "GOAL %d %d", &fx->gx, &fx->gy);
        else if (!strcmp(key, "BASE_COST")) sscanf(line, "BASE_COST %lf", &fx->base_cost);
        else if (!strcmp(key, "CJA_COST")) sscanf(line, "CJA_COST %lf", &fx->cja_cost);
        else if (!strcmp(key, "BASE_NODES")) sscanf(line, "BASE_NODES %d", &fx->base_nodes);
        else if (!strcmp(key, "CJA_NODES")) sscanf(line, "CJA_NODES %d", &fx->cja_nodes);
    }
    if (!saw_grid || fx->w <= 0 || fx->h <= 0) { fclose(f); return 0; }
    fx->grid.w = fx->w;
    fx->grid.h = fx->h;
    fx->grid.n = fx->w * fx->h;
    fx->grid.cell = (unsigned char *)calloc((size_t)fx->grid.n, 1);
    if (!fx->grid.cell) { fclose(f); return 0; }
    int y = 0;
    while (y < fx->h && fgets(line, sizeof(line), f)) {
        strip_eol(line);
        if (!line[0]) continue;
        for (int x = 0; x < fx->w; ++x)
            fx->grid.cell[y * fx->w + x] = (line[x] == '.') ? 0 : 1;
        ++y;
    }
    fclose(f);
    return y == fx->h;
}

static void free_fixture(Fixture *fx) {
    free(fx->grid.cell);
    fx->grid.cell = NULL;
}

static char **load_manifest(const char *manifest, int *count) {
    FILE *f = fopen(manifest, "rb");
    if (!f) { perror(manifest); return NULL; }
    char dir[1024];
    snprintf(dir, sizeof(dir), "%s", manifest);
    char *slash = strrchr(dir, '/');
    char *bslash = strrchr(dir, '\\');
    if (bslash && (!slash || bslash > slash)) slash = bslash;
    if (slash) *slash = '\0';
    else dir[0] = '\0';

    char **paths = NULL;
    int n = 0, cap = 0;
    char line[2048];
    while (fgets(line, sizeof(line), f)) {
        strip_eol(line);
        if (!line[0] || line[0] == '#') continue;
        if (n == cap) {
            cap = cap ? cap * 2 : 32;
            paths = (char **)xrealloc(paths, (size_t)cap * sizeof(char *));
        }
        size_t len = strlen(dir) + strlen(line) + 2;
        paths[n] = (char *)xmalloc(len);
        if (dir[0]) snprintf(paths[n], len, "%s/%s", dir, line);
        else snprintf(paths[n], len, "%s", line);
        ++n;
    }
    fclose(f);
    *count = n;
    return paths;
}

static double now_ms(void) {
#ifdef _WIN32
    static LARGE_INTEGER freq;
    static int init = 0;
    LARGE_INTEGER t;
    if (!init) { QueryPerformanceFrequency(&freq); init = 1; }
    QueryPerformanceCounter(&t);
    return (double)t.QuadPart * 1000.0 / (double)freq.QuadPart;
#else
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec * 1000.0 + (double)ts.tv_nsec / 1e6;
#endif
}

static int verify(const char *manifest) {
    int count = 0;
    char **paths = load_manifest(manifest, &count);
    if (!paths) return 2;
    int pass = 0;
    for (int i = 0; i < count; ++i) {
        Fixture fx;
        if (!load_fixture(paths[i], &fx)) {
            printf("VERIFY %s FAIL load\n", paths[i]);
            continue;
        }
        Result b = baseline_astar(&fx.grid, fx.sx, fx.sy, fx.gx, fx.gy, fx.diagonal);
        Result c = cja_star(&fx.grid, fx.sx, fx.sy, fx.gx, fx.gy, fx.diagonal);
        int ok = b.found && c.found &&
                 fabs(b.cost - fx.base_cost) < 1e-9 && b.expanded == fx.base_nodes &&
                 fabs(c.cost - fx.cja_cost) < 1e-9 && c.expanded == fx.cja_nodes;
        printf("VERIFY %s %s base=%.9f/%d cja=%.9f/%d\n",
               fx.name, ok ? "PASS" : "FAIL", b.cost, b.expanded, c.cost, c.expanded);
        if (ok) ++pass;
        free_result(&b);
        free_result(&c);
        free_fixture(&fx);
        free(paths[i]);
    }
    free(paths);
    printf("SUMMARY language=c cases=%d pass=%d fail=%d\n", count, pass, count - pass);
    return pass == count ? 0 : 1;
}

static int bench(const char *manifest, int reps) {
    int count = 0;
    char **paths = load_manifest(manifest, &count);
    if (!paths) return 2;
    double base_ms = 0.0, cja_ms = 0.0;
    long long base_nodes = 0, cja_nodes = 0;
    for (int i = 0; i < count; ++i) {
        Fixture fx;
        if (!load_fixture(paths[i], &fx)) { fprintf(stderr, "bad fixture\n"); return 2; }
        Result warm = baseline_astar(&fx.grid, fx.sx, fx.sy, fx.gx, fx.gy, fx.diagonal);
        free_result(&warm);
        warm = cja_star(&fx.grid, fx.sx, fx.sy, fx.gx, fx.gy, fx.diagonal);
        free_result(&warm);
        double best_b = 1e300, best_c = 1e300;
        int bn = 0, cn = 0;
        for (int r = 0; r < reps; ++r) {
            double t0 = now_ms();
            Result b = baseline_astar(&fx.grid, fx.sx, fx.sy, fx.gx, fx.gy, fx.diagonal);
            double t1 = now_ms();
            Result c = cja_star(&fx.grid, fx.sx, fx.sy, fx.gx, fx.gy, fx.diagonal);
            double t2 = now_ms();
            if (t1 - t0 < best_b) best_b = t1 - t0;
            if (t2 - t1 < best_c) best_c = t2 - t1;
            bn = b.expanded;
            cn = c.expanded;
            free_result(&b);
            free_result(&c);
        }
        base_ms += best_b;
        cja_ms += best_c;
        base_nodes += bn;
        cja_nodes += cn;
        free_fixture(&fx);
        free(paths[i]);
    }
    free(paths);
    printf("BENCH language=c cases=%d reps=%d baseline_ms=%.6f cja_ms=%.6f "
           "speedup=%.4f baseline_nodes=%.1f cja_nodes=%.1f\n",
           count, reps, base_ms / count, cja_ms / count, base_ms / cja_ms,
           (double)base_nodes / count, (double)cja_nodes / count);
    return 0;
}

int main(int argc, char **argv) {
    if (argc < 3) {
        fprintf(stderr, "usage: %s verify|bench <manifest> [reps]\n", argv[0]);
        return 2;
    }
    if (!strcmp(argv[1], "verify")) return verify(argv[2]);
    if (!strcmp(argv[1], "bench")) return bench(argv[2], argc > 3 ? atoi(argv[3]) : 5);
    fprintf(stderr, "unknown action\n");
    return 2;
}
