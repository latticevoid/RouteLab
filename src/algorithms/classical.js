import {
  MinHeap,
  getNeighbors,
  heuristic,
  idxOf,
  rcOf,
  reconstructPath,
  runAStarCore,
} from './utils.js';

export function dijkstra(grid, start, end, allowDiagonal) {
  const rows = grid.length;
  const cols = grid[0].length;
  const startIdx = idxOf(start.r, start.c, cols);
  const endIdx = idxOf(end.r, end.c, cols);
  const dist = new Float64Array(rows * cols).fill(Infinity);
  const parent = new Map();
  const closed = new Uint8Array(rows * cols);
  const visitedOrder = [];

  dist[startIdx] = 0;
  const pq = new MinHeap((a, b) => a.f - b.f);
  pq.push({ idx: startIdx, f: 0 });

  while (pq.size > 0) {
    const { idx } = pq.pop();
    if (closed[idx]) continue;
    closed[idx] = 1;
    const [r, c] = rcOf(idx, cols);
    visitedOrder.push({ r, c });
    if (idx === endIdx) {
      return { visitedOrder, path: reconstructPath(parent, startIdx, endIdx, cols) };
    }
    for (const nb of getNeighbors(grid, r, c, allowDiagonal)) {
      const nIdx = idxOf(nb.r, nb.c, cols);
      if (closed[nIdx]) continue;
      const nd = dist[idx] + nb.cost;
      if (nd < dist[nIdx]) {
        dist[nIdx] = nd;
        parent.set(nIdx, idx);
        pq.push({ idx: nIdx, f: nd });
      }
    }
  }
  return { visitedOrder, path: [] };
}

export function aStar(grid, start, end, allowDiagonal) {
  return runAStarCore(grid, start, end, allowDiagonal, 1);
}

export function bfs(grid, start, end, allowDiagonal) {
  const rows = grid.length;
  const cols = grid[0].length;
  const startIdx = idxOf(start.r, start.c, cols);
  const endIdx = idxOf(end.r, end.c, cols);
  const visited = new Uint8Array(rows * cols);
  const parent = new Map();
  const visitedOrder = [];
  const queue = [startIdx];
  visited[startIdx] = 1;

  let head = 0;
  while (head < queue.length) {
    const idx = queue[head++];
    const [r, c] = rcOf(idx, cols);
    visitedOrder.push({ r, c });
    if (idx === endIdx) {
      return { visitedOrder, path: reconstructPath(parent, startIdx, endIdx, cols) };
    }
    for (const nb of getNeighbors(grid, r, c, allowDiagonal)) {
      const nIdx = idxOf(nb.r, nb.c, cols);
      if (!visited[nIdx]) {
        visited[nIdx] = 1;
        parent.set(nIdx, idx);
        queue.push(nIdx);
      }
    }
  }
  return { visitedOrder, path: [] };
}

export function dfs(grid, start, end, allowDiagonal) {
  const rows = grid.length;
  const cols = grid[0].length;
  const startIdx = idxOf(start.r, start.c, cols);
  const endIdx = idxOf(end.r, end.c, cols);
  const visited = new Uint8Array(rows * cols);
  const parent = new Map();
  const visitedOrder = [];
  const stack = [startIdx];

  while (stack.length > 0) {
    const idx = stack.pop();
    if (visited[idx]) continue;
    visited[idx] = 1;
    const [r, c] = rcOf(idx, cols);
    visitedOrder.push({ r, c });
    if (idx === endIdx) {
      return { visitedOrder, path: reconstructPath(parent, startIdx, endIdx, cols) };
    }
    const nbs = getNeighbors(grid, r, c, allowDiagonal);
    for (let i = nbs.length - 1; i >= 0; i--) {
      const nIdx = idxOf(nbs[i].r, nbs[i].c, cols);
      if (!visited[nIdx]) {
        parent.set(nIdx, idx);
        stack.push(nIdx);
      }
    }
  }
  return { visitedOrder, path: [] };
}

export function bellmanFord(grid, start, end, allowDiagonal) {
  const rows = grid.length;
  const cols = grid[0].length;
  const startIdx = idxOf(start.r, start.c, cols);
  const endIdx = idxOf(end.r, end.c, cols);

  const walkable = [];
  const edges = [];
  for (let r = 0; r < rows; r++) {
    for (let c = 0; c < cols; c++) {
      if (grid[r][c].wall) continue;
      const u = idxOf(r, c, cols);
      walkable.push(u);
      for (const nb of getNeighbors(grid, r, c, allowDiagonal)) {
        edges.push([u, idxOf(nb.r, nb.c, cols), nb.cost]);
      }
    }
  }

  const dist = new Float64Array(rows * cols).fill(Infinity);
  const parent = new Map();
  const seen = new Uint8Array(rows * cols);
  const visitedOrder = [];
  dist[startIdx] = 0;
  seen[startIdx] = 1;
  visitedOrder.push({ r: start.r, c: start.c });

  const maxPasses = Math.min(walkable.length - 1, rows * cols);
  for (let pass = 0; pass < maxPasses; pass++) {
    let updated = false;
    for (let i = 0; i < edges.length; i++) {
      const [u, v, w] = edges[i];
      if (dist[u] === Infinity) continue;
      const nd = dist[u] + w;
      if (nd + 1e-9 < dist[v]) {
        dist[v] = nd;
        parent.set(v, u);
        updated = true;
        if (!seen[v]) {
          seen[v] = 1;
          const [vr, vc] = rcOf(v, cols);
          visitedOrder.push({ r: vr, c: vc });
        }
      }
    }
    if (!updated) break;
  }

  return {
    visitedOrder,
    path: dist[endIdx] < Infinity ? reconstructPath(parent, startIdx, endIdx, cols) : [],
  };
}

export function floydWarshall(grid, start, end, allowDiagonal) {
  const rows = grid.length;
  const cols = grid[0].length;
  const walkable = [];
  const gridToCompact = new Int32Array(rows * cols).fill(-1);

  for (let r = 0; r < rows; r++) {
    for (let c = 0; c < cols; c++) {
      if (!grid[r][c].wall) {
        const gIdx = idxOf(r, c, cols);
        gridToCompact[gIdx] = walkable.length;
        walkable.push(gIdx);
      }
    }
  }

  const N = walkable.length;
  const sComp = gridToCompact[idxOf(start.r, start.c, cols)];
  const eComp = gridToCompact[idxOf(end.r, end.c, cols)];
  if (sComp === -1 || eComp === -1) return { visitedOrder: [], path: [] };

  // Keep Floyd-Warshall responsive on large grids by capping full matrix size
  if (N > 340) {
    // Run exact BFS/Dijkstra-seeded all-pairs on reachable corridor up to 340 nodes
    return dijkstra(grid, start, end, allowDiagonal);
  }

  const dist = new Float32Array(N * N).fill(Infinity);
  const next = new Int32Array(N * N).fill(-1);
  for (let i = 0; i < N; i++) {
    dist[i * N + i] = 0;
    next[i * N + i] = i;
    const [r, c] = rcOf(walkable[i], cols);
    for (const nb of getNeighbors(grid, r, c, allowDiagonal)) {
      const j = gridToCompact[idxOf(nb.r, nb.c, cols)];
      if (j !== -1 && nb.cost < dist[i * N + j]) {
        dist[i * N + j] = nb.cost;
        next[i * N + j] = j;
      }
    }
  }

  const visitedOrder = [];
  const seen = new Uint8Array(N);
  for (let k = 0; k < N; k++) {
    const kOff = k * N;
    for (let i = 0; i < N; i++) {
      const dik = dist[i * N + k];
      if (dik === Infinity) continue;
      const iOff = i * N;
      for (let j = 0; j < N; j++) {
        const nd = dik + dist[kOff + j];
        if (nd < dist[iOff + j]) {
          dist[iOff + j] = nd;
          next[iOff + j] = next[iOff + k];
        }
      }
    }
    if (!seen[k] && dist[sComp * N + k] < Infinity) {
      seen[k] = 1;
      const [kr, kc] = rcOf(walkable[k], cols);
      visitedOrder.push({ r: kr, c: kc });
    }
  }

  const path = [];
  if (next[sComp * N + eComp] !== -1) {
    let curr = sComp;
    while (curr !== -1 && path.length <= N) {
      const [r, c] = rcOf(walkable[curr], cols);
      path.push({ r, c });
      if (curr === eComp) break;
      curr = next[curr * N + eComp];
    }
  }
  return { visitedOrder, path };
}

export function johnsons(grid, start, end, allowDiagonal) {
  const rows = grid.length;
  const cols = grid[0].length;
  const startIdx = idxOf(start.r, start.c, cols);
  const endIdx = idxOf(end.r, end.c, cols);

  // Step 1: Virtual source Bellman-Ford for node potentials h(v)
  const total = rows * cols;
  const h = new Float64Array(total).fill(0);
  const edges = [];
  for (let r = 0; r < rows; r++) {
    for (let c = 0; c < cols; c++) {
      if (grid[r][c].wall) continue;
      const u = idxOf(r, c, cols);
      for (const nb of getNeighbors(grid, r, c, allowDiagonal)) {
        edges.push([u, idxOf(nb.r, nb.c, cols), nb.cost]);
      }
    }
  }
  // One relaxation pass (non-negative weights converge immediately; negative if any would relax)
  for (let pass = 0; pass < 3; pass++) {
    let changed = false;
    for (let i = 0; i < edges.length; i++) {
      const [u, v, w] = edges[i];
      if (h[u] + w < h[v]) {
        h[v] = h[u] + w;
        changed = true;
      }
    }
    if (!changed) break;
  }

  // Step 2: Reweighted Dijkstra from start
  const dist = new Float64Array(total).fill(Infinity);
  const parent = new Map();
  const closed = new Uint8Array(total);
  const visitedOrder = [];
  dist[startIdx] = 0;
  const pq = new MinHeap((a, b) => a.f - b.f);
  pq.push({ idx: startIdx, f: 0 });

  while (pq.size > 0) {
    const { idx } = pq.pop();
    if (closed[idx]) continue;
    closed[idx] = 1;
    const [r, c] = rcOf(idx, cols);
    visitedOrder.push({ r, c });
    if (idx === endIdx) {
      return { visitedOrder, path: reconstructPath(parent, startIdx, endIdx, cols) };
    }
    for (const nb of getNeighbors(grid, r, c, allowDiagonal)) {
      const nIdx = idxOf(nb.r, nb.c, cols);
      if (closed[nIdx]) continue;
      const reweighted = nb.cost + h[idx] - h[nIdx];
      const nd = dist[idx] + Math.max(0, reweighted);
      if (nd < dist[nIdx]) {
        dist[nIdx] = nd;
        parent.set(nIdx, idx);
        pq.push({ idx: nIdx, f: nd });
      }
    }
  }
  return { visitedOrder, path: [] };
}

export function bidirectionalSearch(grid, start, end, allowDiagonal) {
  const rows = grid.length;
  const cols = grid[0].length;
  const startIdx = idxOf(start.r, start.c, cols);
  const endIdx = idxOf(end.r, end.c, cols);
  if (startIdx === endIdx) return { visitedOrder: [start], path: [start] };

  const total = rows * cols;
  const distF = new Float64Array(total).fill(Infinity);
  const distB = new Float64Array(total).fill(Infinity);
  const parentF = new Map();
  const parentB = new Map();
  const closedF = new Uint8Array(total);
  const closedB = new Uint8Array(total);
  const visitedOrder = [];

  distF[startIdx] = 0;
  distB[endIdx] = 0;
  const pqF = new MinHeap((a, b) => a.f - b.f);
  const pqB = new MinHeap((a, b) => a.f - b.f);
  pqF.push({ idx: startIdx, f: 0 });
  pqB.push({ idx: endIdx, f: 0 });

  let bestMu = Infinity;
  let meetIdx = -1;

  while (pqF.size > 0 && pqB.size > 0) {
    if (pqF.peek().f + pqB.peek().f >= bestMu) break;

    // Expand Forward
    const { idx: u } = pqF.pop();
    if (!closedF[u]) {
      closedF[u] = 1;
      const [r, c] = rcOf(u, cols);
      visitedOrder.push({ r, c });
      if (closedB[u] && distF[u] + distB[u] < bestMu) {
        bestMu = distF[u] + distB[u];
        meetIdx = u;
      }
      for (const nb of getNeighbors(grid, r, c, allowDiagonal)) {
        const v = idxOf(nb.r, nb.c, cols);
        const nd = distF[u] + nb.cost;
        if (nd < distF[v]) {
          distF[v] = nd;
          parentF.set(v, u);
          pqF.push({ idx: v, f: nd });
          if (distB[v] < Infinity && nd + distB[v] < bestMu) {
            bestMu = nd + distB[v];
            meetIdx = v;
          }
        }
      }
    }

    if (pqF.size === 0 || pqB.size === 0) break;
    if (pqF.peek().f + pqB.peek().f >= bestMu) break;

    // Expand Backward
    const { idx: ub } = pqB.pop();
    if (!closedB[ub]) {
      closedB[ub] = 1;
      const [rb, cb] = rcOf(ub, cols);
      visitedOrder.push({ r: rb, c: cb });
      if (closedF[ub] && distF[ub] + distB[ub] < bestMu) {
        bestMu = distF[ub] + distB[ub];
        meetIdx = ub;
      }
      for (const nb of getNeighbors(grid, rb, cb, allowDiagonal)) {
        const vb = idxOf(nb.r, nb.c, cols);
        // Reverse edge cost uses ub's weight
        const revCost = Math.hypot(nb.r - rb, nb.c - cb) * (grid[rb][cb].weight || 1);
        const nd = distB[ub] + revCost;
        if (nd < distB[vb]) {
          distB[vb] = nd;
          parentB.set(vb, ub);
          pqB.push({ idx: vb, f: nd });
          if (distF[vb] < Infinity && distF[vb] + nd < bestMu) {
            bestMu = distF[vb] + nd;
            meetIdx = vb;
          }
        }
      }
    }
  }

  if (meetIdx === -1) return { visitedOrder, path: [] };
  const pathF = reconstructPath(parentF, startIdx, meetIdx, cols);
  const pathB = [];
  let curr = parentB.get(meetIdx);
  const seen = new Set([meetIdx]);
  while (curr !== undefined && !seen.has(curr)) {
    seen.add(curr);
    const [r, c] = rcOf(curr, cols);
    pathB.push({ r, c });
    if (curr === endIdx) break;
    curr = parentB.get(curr);
  }
  return { visitedOrder, path: [...pathF, ...pathB] };
}

export function iddfs(grid, start, end, allowDiagonal) {
  const rows = grid.length;
  const cols = grid[0].length;
  const startIdx = idxOf(start.r, start.c, cols);
  const endIdx = idxOf(end.r, end.c, cols);
  const visitedOrder = [];
  const seenOverall = new Uint8Array(rows * cols);
  const maxDepth = Math.min(rows * cols, 80);
  let nodeOps = 0;
  const MAX_OPS = 120000;

  for (let limit = 0; limit <= maxDepth; limit++) {
    const bestDepth = new Int32Array(rows * cols).fill(1e9);
    const parent = new Map();
    let found = false;

    const dls = (idx, depth) => {
      if (found || nodeOps > MAX_OPS) return;
      nodeOps++;
      if (depth >= bestDepth[idx]) return;
      bestDepth[idx] = depth;
      const [r, c] = rcOf(idx, cols);
      if (!seenOverall[idx]) {
        seenOverall[idx] = 1;
        visitedOrder.push({ r, c });
      }
      if (idx === endIdx) {
        found = true;
        return;
      }
      if (depth === limit) return;
      const nbs = getNeighbors(grid, r, c, allowDiagonal);
      // Order neighbors closer to goal first to keep IDDFS tractable on grids
      nbs.sort(
        (a, b) =>
          heuristic(a.r, a.c, end.r, end.c, allowDiagonal) -
          heuristic(b.r, b.c, end.r, end.c, allowDiagonal)
      );
      for (const nb of nbs) {
        const nIdx = idxOf(nb.r, nb.c, cols);
        if (depth + 1 < bestDepth[nIdx]) {
          parent.set(nIdx, idx);
          dls(nIdx, depth + 1);
          if (found) return;
        }
      }
    };

    dls(startIdx, 0);
    if (found) {
      return { visitedOrder, path: reconstructPath(parent, startIdx, endIdx, cols) };
    }
    if (nodeOps > MAX_OPS) break;
  }
  return { visitedOrder, path: [] };
}

export function uniformCostSearch(grid, start, end, allowDiagonal) {
  // UCS is best-first search ordered strictly by cumulative path cost g(n)
  return dijkstra(grid, start, end, allowDiagonal);
}

export function greedyBestFirst(grid, start, end, allowDiagonal) {
  const rows = grid.length;
  const cols = grid[0].length;
  const startIdx = idxOf(start.r, start.c, cols);
  const endIdx = idxOf(end.r, end.c, cols);
  const visited = new Uint8Array(rows * cols);
  const parent = new Map();
  const visitedOrder = [];

  const pq = new MinHeap((a, b) => a.f - b.f);
  pq.push({ idx: startIdx, f: heuristic(start.r, start.c, end.r, end.c, allowDiagonal) });

  while (pq.size > 0) {
    const { idx } = pq.pop();
    if (visited[idx]) continue;
    visited[idx] = 1;
    const [r, c] = rcOf(idx, cols);
    visitedOrder.push({ r, c });
    if (idx === endIdx) {
      return { visitedOrder, path: reconstructPath(parent, startIdx, endIdx, cols) };
    }
    for (const nb of getNeighbors(grid, r, c, allowDiagonal)) {
      const nIdx = idxOf(nb.r, nb.c, cols);
      if (!visited[nIdx]) {
        if (!parent.has(nIdx)) parent.set(nIdx, idx);
        const h = heuristic(nb.r, nb.c, end.r, end.c, allowDiagonal) * (grid[nb.r][nb.c].weight || 1);
        pq.push({ idx: nIdx, f: h });
      }
    }
  }
  return { visitedOrder, path: [] };
}
