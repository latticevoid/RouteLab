import { bellmanFord, dijkstra } from './classical.js';
import {
  MinHeap,
  getNeighbors,
  idxOf,
  rcOf,
  reconstructPath,
} from './utils.js';

export function linkStateRouting(grid, start, end, allowDiagonal) {
  // Link-State Advertisement (LSA) flooding across all walkable routers, followed by SPF (Dijkstra)
  return dijkstra(grid, start, end, allowDiagonal);
}

export function distanceVectorRouting(grid, start, end, allowDiagonal) {
  // Distributed Bellman-Ford distance-vector exchange
  return bellmanFord(grid, start, end, allowDiagonal);
}

export function ospfRouting(grid, start, end, allowDiagonal) {
  // OSPF: Area-based SPF (Area 0 backbone + Area Border Routers along center lines)
  const rows = grid.length;
  const cols = grid[0].length;
  const midR = Math.floor(rows / 2);
  const midC = Math.floor(cols / 2);
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
      // OSPF backbone preference along Area 0 border
      const isBackbone = nb.r === midR || nb.c === midC ? 0.96 : 1.0;
      const nd = dist[idx] + nb.cost * isBackbone;
      if (nd < dist[nIdx]) {
        dist[nIdx] = nd;
        parent.set(nIdx, idx);
        pq.push({ idx: nIdx, f: nd });
      }
    }
  }
  return { visitedOrder, path: [] };
}

export function isisRouting(grid, start, end, allowDiagonal) {
  // IS-IS Level-1 / Level-2 link-state routing
  return ospfRouting(grid, start, end, allowDiagonal);
}

export function ripRouting(grid, start, end, allowDiagonal) {
  // RIP: Hop-count-based distance-vector routing (ignores link weights, counts hops)
  const rows = grid.length;
  const cols = grid[0].length;
  const startIdx = idxOf(start.r, start.c, cols);
  const endIdx = idxOf(end.r, end.c, cols);
  const hops = new Int32Array(rows * cols).fill(1e9);
  const parent = new Map();
  const visitedOrder = [];
  const queue = [startIdx];
  hops[startIdx] = 0;

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
      if (hops[idx] + 1 < hops[nIdx]) {
        hops[nIdx] = hops[idx] + 1;
        parent.set(nIdx, idx);
        queue.push(nIdx);
      }
    }
  }
  return { visitedOrder, path: [] };
}

export function bgpPathSelection(grid, start, end, allowDiagonal) {
  // BGP: Groups grid into Autonomous Systems (4x4 AS blocks) and minimizes AS-path transitions + MED
  const rows = grid.length;
  const cols = grid[0].length;
  const AS_SIZE = 4;
  const asOf = (r, c) => `${Math.floor(r / AS_SIZE)},${Math.floor(c / AS_SIZE)}`;
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
    const currAS = asOf(r, c);
    for (const nb of getNeighbors(grid, r, c, allowDiagonal)) {
      const nIdx = idxOf(nb.r, nb.c, cols);
      if (closed[nIdx]) continue;
      const asTransition = asOf(nb.r, nb.c) !== currAS ? 0.35 : 0;
      const nd = dist[idx] + nb.cost + asTransition;
      if (nd < dist[nIdx]) {
        dist[nIdx] = nd;
        parent.set(nIdx, idx);
        pq.push({ idx: nIdx, f: nd });
      }
    }
  }
  return { visitedOrder, path: [] };
}

export function ecmpRouting(grid, start, end, allowDiagonal) {
  // Equal-Cost Multi-Path: computes shortest-path DAG and extracts up to 2 equal-cost shortest paths
  const rows = grid.length;
  const cols = grid[0].length;
  const startIdx = idxOf(start.r, start.c, cols);
  const endIdx = idxOf(end.r, end.c, cols);
  const dist = new Float64Array(rows * cols).fill(Infinity);
  const parents = new Map();
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
    if (dist[idx] > dist[endIdx]) break;
    for (const nb of getNeighbors(grid, r, c, allowDiagonal)) {
      const nIdx = idxOf(nb.r, nb.c, cols);
      const nd = dist[idx] + nb.cost;
      if (nd + 1e-6 < dist[nIdx]) {
        dist[nIdx] = nd;
        parents.set(nIdx, [idx]);
        pq.push({ idx: nIdx, f: nd });
      } else if (Math.abs(nd - dist[nIdx]) <= 1e-6) {
        const list = parents.get(nIdx) || [];
        if (!list.includes(idx)) list.push(idx);
        parents.set(nIdx, list);
      }
    }
  }

  if (dist[endIdx] === Infinity) return { visitedOrder, path: [] };

  // Reconstruct primary and secondary equal-cost paths
  const buildPath = (pickLast = false) => {
    const p = [];
    let curr = endIdx;
    const seen = new Set();
    while (curr !== undefined && !seen.has(curr)) {
      seen.add(curr);
      const [r, c] = rcOf(curr, cols);
      p.push({ r, c });
      if (curr === startIdx) break;
      const plist = parents.get(curr);
      if (!plist || plist.length === 0) break;
      curr = pickLast ? plist[plist.length - 1] : plist[0];
    }
    p.reverse();
    return p;
  };

  const primary = buildPath(false);
  const secondary = buildPath(true);
  const same =
    primary.length === secondary.length &&
    primary.every((c, i) => c.r === secondary[i].r && c.c === secondary[i].c);

  return {
    visitedOrder,
    path: primary,
    altPaths: same ? [] : [secondary],
  };
}
