import {
  MinHeap,
  computeClearanceField,
  getNeighbors,
  heuristic,
  idxOf,
  rcOf,
  reconstructPath,
} from './utils.js';

export function leeAlgorithm(grid, start, end, allowDiagonal) {
  // Classic Lee wave expansion (BFS from start, backtrace by decreasing wave index)
  const rows = grid.length;
  const cols = grid[0].length;
  const startIdx = idxOf(start.r, start.c, cols);
  const endIdx = idxOf(end.r, end.c, cols);
  const wave = new Int32Array(rows * cols).fill(-1);
  const visitedOrder = [];
  const queue = [startIdx];
  wave[startIdx] = 0;

  let head = 0;
  let found = false;
  while (head < queue.length) {
    const idx = queue[head++];
    const [r, c] = rcOf(idx, cols);
    visitedOrder.push({ r, c });
    if (idx === endIdx) {
      found = true;
      break;
    }
    for (const nb of getNeighbors(grid, r, c, allowDiagonal)) {
      const nIdx = idxOf(nb.r, nb.c, cols);
      if (wave[nIdx] === -1) {
        wave[nIdx] = wave[idx] + 1;
        queue.push(nIdx);
      }
    }
  }
  if (!found) return { visitedOrder, path: [] };

  const path = [];
  let curr = endIdx;
  while (curr !== startIdx) {
    const [r, c] = rcOf(curr, cols);
    path.push({ r, c });
    let bestNb = -1;
    for (const nb of getNeighbors(grid, r, c, allowDiagonal)) {
      const nIdx = idxOf(nb.r, nb.c, cols);
      if (wave[nIdx] === wave[curr] - 1) {
        bestNb = nIdx;
        break;
      }
    }
    if (bestNb === -1) break;
    curr = bestNb;
  }
  path.push({ r: start.r, c: start.c });
  path.reverse();
  return { visitedOrder, path };
}

export function wavefrontAlgorithm(grid, start, end, allowDiagonal) {
  // Propagates potential wave backward from goal to start, then follows steepest descent
  const rows = grid.length;
  const cols = grid[0].length;
  const startIdx = idxOf(start.r, start.c, cols);
  const endIdx = idxOf(end.r, end.c, cols);
  const dist = new Float64Array(rows * cols).fill(Infinity);
  const visitedOrder = [];
  const queue = [endIdx];
  dist[endIdx] = 0;

  let head = 0;
  while (head < queue.length) {
    const idx = queue[head++];
    const [r, c] = rcOf(idx, cols);
    visitedOrder.push({ r, c });
    if (idx === startIdx) break;
    for (const nb of getNeighbors(grid, r, c, allowDiagonal)) {
      const nIdx = idxOf(nb.r, nb.c, cols);
      if (dist[nIdx] === Infinity) {
        dist[nIdx] = dist[idx] + nb.cost;
        queue.push(nIdx);
      }
    }
  }

  if (dist[startIdx] === Infinity) return { visitedOrder, path: [] };
  const path = [{ r: start.r, c: start.c }];
  let curr = startIdx;
  const seen = new Set([startIdx]);
  while (curr !== endIdx && path.length < rows * cols) {
    const [r, c] = rcOf(curr, cols);
    let bestIdx = -1;
    let bestVal = dist[curr];
    for (const nb of getNeighbors(grid, r, c, allowDiagonal)) {
      const nIdx = idxOf(nb.r, nb.c, cols);
      if (!seen.has(nIdx) && dist[nIdx] < bestVal) {
        bestVal = dist[nIdx];
        bestIdx = nIdx;
      }
    }
    if (bestIdx === -1) break;
    seen.add(bestIdx);
    const [br, bc] = rcOf(bestIdx, cols);
    path.push({ r: br, c: bc });
    curr = bestIdx;
  }
  return { visitedOrder, path };
}

export function brushfireAlgorithm(grid, start, end, allowDiagonal) {
  // Computes obstacle-clearance distance map (Brushfire), then routes along high-clearance corridor
  const clearance = computeClearanceField(grid);
  const rows = grid.length;
  const cols = grid[0].length;
  const startIdx = idxOf(start.r, start.c, cols);
  const endIdx = idxOf(end.r, end.c, cols);
  const gScore = new Float64Array(rows * cols).fill(Infinity);
  const parent = new Map();
  const closed = new Uint8Array(rows * cols);
  const visitedOrder = [];

  gScore[startIdx] = 0;
  const pq = new MinHeap((a, b) => a.f - b.f);
  pq.push({ idx: startIdx, f: heuristic(start.r, start.c, end.r, end.c, allowDiagonal) });

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
      // Add gentle clearance penalty near walls so path prefers center of corridors
      const clr = Math.max(1, clearance[nb.r][nb.c]);
      const clearancePenalty = 1.2 / clr;
      const tg = gScore[idx] + nb.cost + clearancePenalty;
      if (tg < gScore[nIdx]) {
        gScore[nIdx] = tg;
        parent.set(nIdx, idx);
        pq.push({
          idx: nIdx,
          f: tg + heuristic(nb.r, nb.c, end.r, end.c, allowDiagonal),
        });
      }
    }
  }
  return { visitedOrder, path: [] };
}

export function dStar(grid, start, end, allowDiagonal) {
  // D* (Dynamic A*): Backward focused search from goal to start with k/h key ordering
  const rows = grid.length;
  const cols = grid[0].length;
  const startIdx = idxOf(start.r, start.c, cols);
  const endIdx = idxOf(end.r, end.c, cols);
  const h = new Float64Array(rows * cols).fill(Infinity);
  const backptr = new Map();
  const closed = new Uint8Array(rows * cols);
  const visitedOrder = [];

  h[endIdx] = 0;
  const pq = new MinHeap((a, b) => a.f - b.f);
  pq.push({ idx: endIdx, f: heuristic(end.r, end.c, start.r, start.c, allowDiagonal) });

  while (pq.size > 0) {
    const { idx } = pq.pop();
    if (closed[idx]) continue;
    closed[idx] = 1;
    const [r, c] = rcOf(idx, cols);
    visitedOrder.push({ r, c });
    if (idx === startIdx) break;

    for (const nb of getNeighbors(grid, r, c, allowDiagonal)) {
      const nIdx = idxOf(nb.r, nb.c, cols);
      if (closed[nIdx]) continue;
      const nh = h[idx] + nb.cost;
      if (nh < h[nIdx]) {
        h[nIdx] = nh;
        backptr.set(nIdx, idx);
        pq.push({
          idx: nIdx,
          f: nh + heuristic(nb.r, nb.c, start.r, start.c, allowDiagonal),
        });
      }
    }
  }

  if (h[startIdx] === Infinity) return { visitedOrder, path: [] };
  const path = [];
  let curr = startIdx;
  const seen = new Set();
  while (curr !== undefined && !seen.has(curr)) {
    seen.add(curr);
    const [r, c] = rcOf(curr, cols);
    path.push({ r, c });
    if (curr === endIdx) break;
    curr = backptr.get(curr);
  }
  return { visitedOrder, path };
}

export function dStarLite(grid, start, end, allowDiagonal) {
  // D* Lite: Incremental backward heuristic search maintaining g and rhs values
  const rows = grid.length;
  const cols = grid[0].length;
  const total = rows * cols;
  const startIdx = idxOf(start.r, start.c, cols);
  const endIdx = idxOf(end.r, end.c, cols);
  const g = new Float64Array(total).fill(Infinity);
  const rhs = new Float64Array(total).fill(Infinity);
  const visitedOrder = [];
  const seen = new Uint8Array(total);

  rhs[endIdx] = 0;
  const calcKey = (idx) => {
    const [r, c] = rcOf(idx, cols);
    const m = Math.min(g[idx], rhs[idx]);
    return [m + heuristic(start.r, start.c, r, c, allowDiagonal), m];
  };

  const pq = new MinHeap((a, b) => a.k0 - b.k0 || a.k1 - b.k1);
  const kEnd = calcKey(endIdx);
  pq.push({ idx: endIdx, k0: kEnd[0], k1: kEnd[1] });

  const updateVertex = (u) => {
    if (u !== endIdx) {
      const [r, c] = rcOf(u, cols);
      let minRhs = Infinity;
      for (const nb of getNeighbors(grid, r, c, allowDiagonal)) {
        const v = idxOf(nb.r, nb.c, cols);
        const val = nb.cost + g[v];
        if (val < minRhs) minRhs = val;
      }
      rhs[u] = minRhs;
    }
    if (g[u] !== rhs[u]) {
      const k = calcKey(u);
      pq.push({ idx: u, k0: k[0], k1: k[1] });
    }
  };

  let ops = 0;
  const MAX_OPS = total * 8;
  while (pq.size > 0 && ops++ < MAX_OPS) {
    const top = pq.peek();
    const kStart = calcKey(startIdx);
    const topLtStart = top.k0 < kStart[0] || (top.k0 === kStart[0] && top.k1 < kStart[1]);
    if (!topLtStart && rhs[startIdx] === g[startIdx]) break;

    const uItem = pq.pop();
    const u = uItem.idx;
    if (g[u] === rhs[u]) continue;
    const kNew = calcKey(u);
    if (uItem.k0 > kNew[0] + 1e-9 || (Math.abs(uItem.k0 - kNew[0]) <= 1e-9 && uItem.k1 > kNew[1] + 1e-9)) {
      continue;
    }
    if (uItem.k0 < kNew[0] - 1e-9 || (Math.abs(uItem.k0 - kNew[0]) <= 1e-9 && uItem.k1 < kNew[1] - 1e-9)) {
      pq.push({ idx: u, k0: kNew[0], k1: kNew[1] });
      continue;
    }

    const [r, c] = rcOf(u, cols);
    if (!seen[u]) {
      seen[u] = 1;
      visitedOrder.push({ r, c });
    }

    if (g[u] > rhs[u]) {
      g[u] = rhs[u];
      for (const nb of getNeighbors(grid, r, c, allowDiagonal)) {
        updateVertex(idxOf(nb.r, nb.c, cols));
      }
    } else {
      g[u] = Infinity;
      updateVertex(u);
      for (const nb of getNeighbors(grid, r, c, allowDiagonal)) {
        updateVertex(idxOf(nb.r, nb.c, cols));
      }
    }
  }

  if (g[startIdx] === Infinity) return { visitedOrder, path: [] };
  const path = [{ r: start.r, c: start.c }];
  let curr = startIdx;
  const pathSeen = new Set([startIdx]);
  while (curr !== endIdx && path.length < total) {
    const [r, c] = rcOf(curr, cols);
    let bestNb = -1;
    let bestCost = Infinity;
    for (const nb of getNeighbors(grid, r, c, allowDiagonal)) {
      const nIdx = idxOf(nb.r, nb.c, cols);
      const val = nb.cost + g[nIdx];
      if (!pathSeen.has(nIdx) && val < bestCost) {
        bestCost = val;
        bestNb = nIdx;
      }
    }
    if (bestNb === -1) break;
    pathSeen.add(bestNb);
    const [br, bc] = rcOf(bestNb, cols);
    path.push({ r: br, c: bc });
    curr = bestNb;
  }
  return { visitedOrder, path };
}

export function lpaStar(grid, start, end, allowDiagonal) {
  // Lifelong Planning A*: Forward incremental search with g and rhs values
  const rows = grid.length;
  const cols = grid[0].length;
  const total = rows * cols;
  const startIdx = idxOf(start.r, start.c, cols);
  const endIdx = idxOf(end.r, end.c, cols);
  const g = new Float64Array(total).fill(Infinity);
  const rhs = new Float64Array(total).fill(Infinity);
  const parent = new Map();
  const visitedOrder = [];
  const seen = new Uint8Array(total);

  rhs[startIdx] = 0;
  const calcKey = (idx) => {
    const [r, c] = rcOf(idx, cols);
    const m = Math.min(g[idx], rhs[idx]);
    return [m + heuristic(r, c, end.r, end.c, allowDiagonal), m];
  };

  const pq = new MinHeap((a, b) => a.k0 - b.k0 || a.k1 - b.k1);
  const k0 = calcKey(startIdx);
  pq.push({ idx: startIdx, k0: k0[0], k1: k0[1] });

  let ops = 0;
  while (pq.size > 0 && ops++ < total * 8) {
    const top = pq.peek();
    const kEnd = calcKey(endIdx);
    const topLtEnd = top.k0 < kEnd[0] || (top.k0 === kEnd[0] && top.k1 < kEnd[1]);
    if (!topLtEnd && rhs[endIdx] === g[endIdx]) break;

    const { idx: u } = pq.pop();
    const [r, c] = rcOf(u, cols);
    if (!seen[u]) {
      seen[u] = 1;
      visitedOrder.push({ r, c });
    }

    if (g[u] > rhs[u]) {
      g[u] = rhs[u];
      for (const nb of getNeighbors(grid, r, c, allowDiagonal)) {
        const v = idxOf(nb.r, nb.c, cols);
        if (g[u] + nb.cost < rhs[v]) {
          rhs[v] = g[u] + nb.cost;
          parent.set(v, u);
          if (g[v] !== rhs[v]) {
            const kv = calcKey(v);
            pq.push({ idx: v, k0: kv[0], k1: kv[1] });
          }
        }
      }
    } else {
      g[u] = Infinity;
      const ku = calcKey(u);
      pq.push({ idx: u, k0: ku[0], k1: ku[1] });
    }
  }

  return {
    visitedOrder,
    path: g[endIdx] < Infinity ? reconstructPath(parent, startIdx, endIdx, cols) : [],
  };
}

export function fieldDStar(grid, start, end) {
  // Field D*: Linear-interpolation D* Lite allowing continuous headings
  const base = dStarLite(grid, start, end, true);
  if (base.path.length <= 2) return base;
  // Smooth corners via linear interpolation between adjacent segments
  const smoothPath = base.path.map((p, i) => {
    if (i === 0 || i === base.path.length - 1) return p;
    const prev = base.path[i - 1];
    const next = base.path[i + 1];
    const avgR = 0.25 * prev.r + 0.5 * p.r + 0.25 * next.r;
    const avgC = 0.25 * prev.c + 0.5 * p.c + 0.25 * next.c;
    const rr = Math.round(avgR);
    const cc = Math.round(avgC);
    if (!grid[rr]?.[cc]?.wall) {
      return { r: avgR, c: avgC };
    }
    return p;
  });
  return { visitedOrder: base.visitedOrder, path: base.path, smoothPath };
}

export function truncatedDijkstra(grid, start, end, allowDiagonal) {
  // Truncated Dijkstra: stops immediately once target is settled (bounded expansion)
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
  let bound = Infinity;

  while (pq.size > 0) {
    const { idx, f } = pq.pop();
    if (f > bound) break;
    if (closed[idx]) continue;
    closed[idx] = 1;
    const [r, c] = rcOf(idx, cols);
    visitedOrder.push({ r, c });
    if (idx === endIdx) {
      bound = f;
      return { visitedOrder, path: reconstructPath(parent, startIdx, endIdx, cols) };
    }
    for (const nb of getNeighbors(grid, r, c, allowDiagonal)) {
      const nIdx = idxOf(nb.r, nb.c, cols);
      if (closed[nIdx]) continue;
      const nd = dist[idx] + nb.cost;
      if (nd < dist[nIdx] && nd <= bound) {
        dist[nIdx] = nd;
        parent.set(nIdx, idx);
        pq.push({ idx: nIdx, f: nd });
      }
    }
  }
  return { visitedOrder, path: [] };
}
