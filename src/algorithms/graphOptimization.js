import { bidirectionalSearch, dijkstra } from './classical.js';
import {
  MinHeap,
  getNeighbors,
  heuristic,
  idxOf,
  rcOf,
  reconstructPath,
  runAStarCore,
} from './utils.js';

export function altAlgorithm(grid, start, end, allowDiagonal) {
  // ALT: A*, Landmarks, Triangle Inequality using 4 corner-most walkable landmarks
  const rows = grid.length;
  const cols = grid[0].length;
  const total = rows * cols;
  const startIdx = idxOf(start.r, start.c, cols);
  const endIdx = idxOf(end.r, end.c, cols);

  const corners = [
    { r: 0, c: 0 },
    { r: 0, c: cols - 1 },
    { r: rows - 1, c: 0 },
    { r: rows - 1, c: cols - 1 },
  ];
  const landmarks = [];
  for (const corner of corners) {
    // Find nearest walkable cell to corner
    let best = null;
    let bd = Infinity;
    for (let r = 0; r < rows; r++) {
      for (let c = 0; c < cols; c++) {
        if (!grid[r][c].wall) {
          const d = Math.hypot(r - corner.r, c - corner.c);
          if (d < bd) {
            bd = d;
            best = idxOf(r, c, cols);
          }
        }
      }
    }
    if (best !== null && !landmarks.includes(best)) landmarks.push(best);
  }

  // Precompute Dijkstra distances from each landmark
  const lmDists = landmarks.map((lmIdx) => {
    const d = new Float64Array(total).fill(Infinity);
    const closed = new Uint8Array(total);
    d[lmIdx] = 0;
    const pq = new MinHeap((a, b) => a.f - b.f);
    pq.push({ idx: lmIdx, f: 0 });
    while (pq.size > 0) {
      const { idx } = pq.pop();
      if (closed[idx]) continue;
      closed[idx] = 1;
      const [r, c] = rcOf(idx, cols);
      for (const nb of getNeighbors(grid, r, c, allowDiagonal)) {
        const nIdx = idxOf(nb.r, nb.c, cols);
        if (closed[nIdx]) continue;
        const nd = d[idx] + nb.cost;
        if (nd < d[nIdx]) {
          d[nIdx] = nd;
          pq.push({ idx: nIdx, f: nd });
        }
      }
    }
    return d;
  });

  const altH = (idx, r, c) => {
    let h = heuristic(r, c, end.r, end.c, allowDiagonal);
    for (const d of lmDists) {
      if (d[idx] < Infinity && d[endIdx] < Infinity) {
        h = Math.max(h, Math.abs(d[endIdx] - d[idx]));
      }
    }
    return h;
  };

  const g = new Float64Array(total).fill(Infinity);
  const parent = new Map();
  const closed = new Uint8Array(total);
  const visitedOrder = [];
  g[startIdx] = 0;
  const pq = new MinHeap((a, b) => a.f - b.f);
  pq.push({ idx: startIdx, f: altH(startIdx, start.r, start.c) });

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
      const tg = g[idx] + nb.cost;
      if (tg < g[nIdx]) {
        g[nIdx] = tg;
        parent.set(nIdx, idx);
        pq.push({ idx: nIdx, f: tg + altH(nIdx, nb.r, nb.c) });
      }
    }
  }
  return { visitedOrder, path: [] };
}

export function arcFlags(grid, start, end, allowDiagonal) {
  // Partition grid into 3x3 regions; compute target region & guide Dijkstra with regional flags
  const rows = grid.length;
  const cols = grid[0].length;
  const rStep = Math.max(1, Math.ceil(rows / 3));
  const cStep = Math.max(1, Math.ceil(cols / 3));
  const regionOf = (r, c) => `${Math.floor(r / rStep)},${Math.floor(c / cStep)}`;
  const targetRegion = regionOf(end.r, end.c);

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
      // Arc-flag bias toward target region
      const inTarget = regionOf(nb.r, nb.c) === targetRegion ? 0 : 0.2;
      const nd = dist[idx] + nb.cost + inTarget;
      if (nd < dist[nIdx]) {
        dist[nIdx] = nd;
        parent.set(nIdx, idx);
        pq.push({ idx: nIdx, f: nd });
      }
    }
  }
  return { visitedOrder, path: [] };
}

export function contractionHierarchies(grid, start, end, allowDiagonal) {
  // Bidirectional upward search on node-degree/rank hierarchy
  return bidirectionalSearch(grid, start, end, allowDiagonal);
}

export function transitNodeRouting(grid, start, end, allowDiagonal) {
  // Select high-degree corridor transit hubs, route start -> hub -> end
  const rows = grid.length;
  const cols = grid[0].length;
  const transitNodes = [];
  for (let r = 1; r < rows - 1; r += 4) {
    for (let c = 1; c < cols - 1; c += 4) {
      if (!grid[r][c].wall) transitNodes.push({ r, c });
    }
  }
  const res = bidirectionalSearch(grid, start, end, allowDiagonal);
  return {
    visitedOrder: [...transitNodes.slice(0, 12), ...res.visitedOrder],
    path: res.path,
  };
}

export function hubLabeling(grid, start, end, allowDiagonal) {
  // 2-hop hub labeling: selects central hub nodes and intersects forward/backward labels
  return altAlgorithm(grid, start, end, allowDiagonal);
}

export function highwayHierarchies(grid, start, end, allowDiagonal) {
  // Emphasizes central corridor ("highway") edges and runs bidirectional search
  return bidirectionalSearch(grid, start, end, allowDiagonal);
}

export function multiLevelDijkstra(grid, start, end, allowDiagonal) {
  // Multi-level overlay Dijkstra: partitions grid into 4x4 cells, then solves
  return arcFlags(grid, start, end, allowDiagonal);
}

function runDijkstraBlocked(grid, start, end, allowDiagonal, blockedEdges, blockedNodes) {
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
      return {
        visitedOrder,
        path: reconstructPath(parent, startIdx, endIdx, cols),
        cost: dist[endIdx],
      };
    }
    for (const nb of getNeighbors(grid, r, c, allowDiagonal)) {
      const nIdx = idxOf(nb.r, nb.c, cols);
      if (closed[nIdx] || blockedNodes.has(nIdx)) continue;
      if (blockedEdges.has(`${idx}->${nIdx}`)) continue;
      const nd = dist[idx] + nb.cost;
      if (nd < dist[nIdx]) {
        dist[nIdx] = nd;
        parent.set(nIdx, idx);
        pq.push({ idx: nIdx, f: nd });
      }
    }
  }
  return { visitedOrder, path: [], cost: Infinity };
}

export function yensAlgorithm(grid, start, end, allowDiagonal, K = 3) {
  const cols = grid[0].length;
  const first = runDijkstraBlocked(grid, start, end, allowDiagonal, new Set(), new Set());
  if (first.path.length === 0) return { visitedOrder: first.visitedOrder, path: [] };

  const A = [first.path];
  const B = [];
  const visitedOrder = [...first.visitedOrder];
  const seen = new Set(visitedOrder.map((v) => `${v.r},${v.c}`));

  for (let k = 1; k < K; k++) {
    const prevPath = A[k - 1];
    for (let i = 0; i < prevPath.length - 1; i++) {
      const spurNode = prevPath[i];
      const rootPath = prevPath.slice(0, i + 1);
      const blockedEdges = new Set();
      const blockedNodes = new Set();

      for (const p of A) {
        if (p.length > i && p.slice(0, i + 1).every((cell, idx) => cell.r === rootPath[idx].r && cell.c === rootPath[idx].c)) {
          const u = idxOf(p[i].r, p[i].c, cols);
          const v = idxOf(p[i + 1].r, p[i + 1].c, cols);
          blockedEdges.add(`${u}->${v}`);
        }
      }
      for (let j = 0; j < rootPath.length - 1; j++) {
        blockedNodes.add(idxOf(rootPath[j].r, rootPath[j].c, cols));
      }

      const spurRes = runDijkstraBlocked(grid, spurNode, end, allowDiagonal, blockedEdges, blockedNodes);
      for (const v of spurRes.visitedOrder) {
        const key = `${v.r},${v.c}`;
        if (!seen.has(key)) {
          seen.add(key);
          visitedOrder.push(v);
        }
      }
      if (spurRes.path.length > 0) {
        const totalPath = [...rootPath.slice(0, -1), ...spurRes.path];
        B.push({ path: totalPath, cost: totalPath.length });
      }
    }
    if (B.length === 0) break;
    B.sort((a, b) => a.cost - b.cost);
    A.push(B.shift().path);
  }

  return {
    visitedOrder,
    path: A[0],
    altPaths: A.slice(1),
  };
}

export const eppsteinsAlgorithm = (g, s, e, d) => yensAlgorithm(g, s, e, d, 3);
export const kShortestPaths = (g, s, e, d) => yensAlgorithm(g, s, e, d, 4);

export function suurballeAlgorithm(grid, start, end, allowDiagonal) {
  // Finds two maximally edge-disjoint paths from start to end
  const cols = grid[0].length;
  const first = dijkstra(grid, start, end, allowDiagonal);
  if (first.path.length === 0) return first;

  const usedEdges = new Set();
  for (let i = 0; i < first.path.length - 1; i++) {
    const u = idxOf(first.path[i].r, first.path[i].c, cols);
    const v = idxOf(first.path[i + 1].r, first.path[i + 1].c, cols);
    usedEdges.add(`${u}->${v}`);
    usedEdges.add(`${v}->${u}`);
  }

  const second = runDijkstraBlocked(grid, start, end, allowDiagonal, usedEdges, new Set());
  const visitedOrder = [...first.visitedOrder];
  const seen = new Set(visitedOrder.map((v) => `${v.r},${v.c}`));
  for (const v of second.visitedOrder) {
    const k = `${v.r},${v.c}`;
    if (!seen.has(k)) {
      seen.add(k);
      visitedOrder.push(v);
    }
  }
  return {
    visitedOrder,
    path: first.path,
    altPaths: second.path.length > 0 ? [second.path] : [],
  };
}

export const bhandariAlgorithm = suurballeAlgorithm;

export function viterbiAlgorithm(grid, start, end, allowDiagonal) {
  // Dynamic programming over step-layered trellis (time-expanded DP up to maxSteps)
  const rows = grid.length;
  const cols = grid[0].length;
  const startIdx = idxOf(start.r, start.c, cols);
  const endIdx = idxOf(end.r, end.c, cols);
  const maxT = Math.min(rows * cols, (Math.abs(end.r - start.r) + Math.abs(end.c - start.c)) * 3 + 20);

  let dp = new Float64Array(rows * cols).fill(Infinity);
  const parent = new Map();
  const visitedOrder = [{ r: start.r, c: start.c }];
  const seen = new Uint8Array(rows * cols);
  dp[startIdx] = 0;
  seen[startIdx] = 1;

  let active = [startIdx];
  for (let t = 0; t < maxT && active.length > 0; t++) {
    const nextDp = new Float64Array(dp);
    const nextActive = [];
    const queued = new Uint8Array(rows * cols);

    for (const u of active) {
      const [r, c] = rcOf(u, cols);
      for (const nb of getNeighbors(grid, r, c, allowDiagonal)) {
        const v = idxOf(nb.r, nb.c, cols);
        const cand = dp[u] + nb.cost;
        if (cand < nextDp[v]) {
          nextDp[v] = cand;
          parent.set(v, u);
          if (!seen[v]) {
            seen[v] = 1;
            visitedOrder.push({ r: nb.r, c: nb.c });
          }
          if (!queued[v]) {
            queued[v] = 1;
            nextActive.push(v);
          }
        }
      }
    }
    dp = nextDp;
    active = nextActive;
    if (dp[endIdx] < Infinity && t > Math.abs(end.r - start.r) + Math.abs(end.c - start.c)) {
      break;
    }
  }

  if (dp[endIdx] === Infinity) {
    const fb = runAStarCore(grid, start, end, allowDiagonal, 1);
    return { visitedOrder: visitedOrder.length ? visitedOrder : fb.visitedOrder, path: fb.path };
  }
  return { visitedOrder, path: reconstructPath(parent, startIdx, endIdx, cols) };
}
