import { dStar, dStarLite, lpaStar } from './gridBased.js';
import {
  MinHeap,
  getNeighbors,
  heuristic,
  idxOf,
  rcOf,
  reconstructPath,
  runAStarCore,
} from './utils.js';

export { dStar, dStarLite, lpaStar };

export function adaptiveAStar(grid, start, end, allowDiagonal) {
  // Adaptive A*: runs an initial A* pass, updates h(s) = g(goal) - g(s) for closed states,
  // then executes a refined informed pass.
  const rows = grid.length;
  const cols = grid[0].length;
  const total = rows * cols;
  const startIdx = idxOf(start.r, start.c, cols);
  const endIdx = idxOf(end.r, end.c, cols);

  const hTable = new Float64Array(total);
  for (let r = 0; r < rows; r++) {
    for (let c = 0; c < cols; c++) {
      hTable[idxOf(r, c, cols)] = heuristic(r, c, end.r, end.c, allowDiagonal);
    }
  }

  // Pass 1: discover g-values and goal cost
  const g1 = new Float64Array(total).fill(Infinity);
  const closed1 = [];
  const visited1 = new Uint8Array(total);
  const pq1 = new MinHeap((a, b) => a.f - b.f);
  g1[startIdx] = 0;
  pq1.push({ idx: startIdx, f: hTable[startIdx] });
  const visitedOrder = [];

  while (pq1.size > 0) {
    const { idx } = pq1.pop();
    if (visited1[idx]) continue;
    visited1[idx] = 1;
    closed1.push(idx);
    const [r, c] = rcOf(idx, cols);
    visitedOrder.push({ r, c });
    if (idx === endIdx) break;
    for (const nb of getNeighbors(grid, r, c, allowDiagonal)) {
      const nIdx = idxOf(nb.r, nb.c, cols);
      if (visited1[nIdx]) continue;
      const tg = g1[idx] + nb.cost;
      if (tg < g1[nIdx]) {
        g1[nIdx] = tg;
        pq1.push({ idx: nIdx, f: tg + hTable[nIdx] });
      }
    }
  }

  if (g1[endIdx] === Infinity) return { visitedOrder, path: [] };
  // Update heuristic for all closed states: h_new(s) = g(goal) - g(s)
  for (const s of closed1) {
    hTable[s] = Math.max(hTable[s], g1[endIdx] - g1[s]);
  }

  // Pass 2: A* with updated adaptive heuristic
  const g2 = new Float64Array(total).fill(Infinity);
  const parent = new Map();
  const closed2 = new Uint8Array(total);
  const pq2 = new MinHeap((a, b) => a.f - b.f);
  g2[startIdx] = 0;
  pq2.push({ idx: startIdx, f: hTable[startIdx] });

  while (pq2.size > 0) {
    const { idx } = pq2.pop();
    if (closed2[idx]) continue;
    closed2[idx] = 1;
    if (idx === endIdx) {
      return { visitedOrder, path: reconstructPath(parent, startIdx, endIdx, cols) };
    }
    const [r, c] = rcOf(idx, cols);
    for (const nb of getNeighbors(grid, r, c, allowDiagonal)) {
      const nIdx = idxOf(nb.r, nb.c, cols);
      if (closed2[nIdx]) continue;
      const tg = g2[idx] + nb.cost;
      if (tg < g2[nIdx]) {
        g2[nIdx] = tg;
        parent.set(nIdx, idx);
        pq2.push({ idx: nIdx, f: tg + hTable[nIdx] });
      }
    }
  }
  return { visitedOrder, path: [] };
}

function realTimeLearningSearch(grid, start, end, allowDiagonal, mode = 'lrta') {
  const rows = grid.length;
  const cols = grid[0].length;
  const total = rows * cols;
  const startIdx = idxOf(start.r, start.c, cols);
  const endIdx = idxOf(end.r, end.c, cols);

  const h = new Float64Array(total);
  for (let r = 0; r < rows; r++) {
    for (let c = 0; c < cols; c++) {
      h[idxOf(r, c, cols)] = heuristic(r, c, end.r, end.c, allowDiagonal);
    }
  }

  const visitedOrder = [];
  const seen = new Uint8Array(total);
  const path = [{ r: start.r, c: start.c }];
  let curr = startIdx;
  const maxSteps = total * 4;
  const LOOKAHEAD = mode === 'rtaa' ? 12 : 1;

  for (let step = 0; step < maxSteps && curr !== endIdx; step++) {
    const [r, c] = rcOf(curr, cols);
    if (!seen[curr]) {
      seen[curr] = 1;
      visitedOrder.push({ r, c });
    }

    if (mode === 'rtaa') {
      // Bounded lookahead A* of size LOOKAHEAD
      const g = new Map([[curr, 0]]);
      const parent = new Map();
      const closed = [];
      const closedSet = new Set();
      const pq = new MinHeap((a, b) => a.f - b.f);
      pq.push({ idx: curr, f: h[curr] });
      let bestTarget = -1;
      let expansions = 0;

      while (pq.size > 0 && expansions < LOOKAHEAD) {
        const top = pq.pop();
        if (closedSet.has(top.idx)) continue;
        closedSet.add(top.idx);
        closed.push(top);
        expansions++;
        if (top.idx === endIdx) {
          bestTarget = endIdx;
          break;
        }
        const [tr, tc] = rcOf(top.idx, cols);
        if (!seen[top.idx]) {
          seen[top.idx] = 1;
          visitedOrder.push({ r: tr, c: tc });
        }
        for (const nb of getNeighbors(grid, tr, tc, allowDiagonal)) {
          const nIdx = idxOf(nb.r, nb.c, cols);
          if (closedSet.has(nIdx)) continue;
          const tg = g.get(top.idx) + nb.cost;
          if (tg < (g.get(nIdx) ?? Infinity)) {
            g.set(nIdx, tg);
            parent.set(nIdx, top.idx);
            pq.push({ idx: nIdx, f: tg + h[nIdx] });
          }
        }
      }

      if (bestTarget === -1 && pq.size > 0) {
        bestTarget = pq.peek().idx;
      }
      if (bestTarget === -1) break;

      const fBest = (g.get(bestTarget) ?? 0) + h[bestTarget];
      for (const s of closed) {
        h[s.idx] = Math.max(h[s.idx], fBest - (g.get(s.idx) ?? 0));
      }

      // Advance one step along path to bestTarget
      const sub = reconstructPath(parent, curr, bestTarget, cols);
      if (sub.length < 2) break;
      const nextCell = sub[1];
      curr = idxOf(nextCell.r, nextCell.c, cols);
      path.push(nextCell);
    } else {
      // LRTA* or RTA*
      const nbs = getNeighbors(grid, r, c, allowDiagonal);
      if (nbs.length === 0) break;
      const scored = nbs.map((nb) => {
        const nIdx = idxOf(nb.r, nb.c, cols);
        return { idx: nIdx, r: nb.r, c: nb.c, f: nb.cost + h[nIdx] };
      });
      scored.sort((a, b) => a.f - b.f);
      const best = scored[0];
      const updateVal = mode === 'rta' && scored.length > 1 ? scored[1].f : best.f;
      h[curr] = Math.max(h[curr], updateVal);
      curr = best.idx;
      path.push({ r: best.r, c: best.c });
    }
  }

  if (curr !== endIdx) {
    const fb = runAStarCore(grid, start, end, allowDiagonal, 1);
    return { visitedOrder, path: fb.path };
  }
  return { visitedOrder, path };
}

export function rtaaStar(grid, start, end, allowDiagonal) {
  return realTimeLearningSearch(grid, start, end, allowDiagonal, 'rtaa');
}

export function lrtaStar(grid, start, end, allowDiagonal) {
  return realTimeLearningSearch(grid, start, end, allowDiagonal, 'lrta');
}

export function rtaStar(grid, start, end, allowDiagonal) {
  return realTimeLearningSearch(grid, start, end, allowDiagonal, 'rta');
}

export function movingTargetSearch(grid, start, end, allowDiagonal) {
  // Simulates Moving Target Search: target drifts within a small neighborhood and returns to end,
  // while the agent tracks it in real time with incremental replanning.
  const rows = grid.length;
  const cols = grid[0].length;
  const visitedOrder = [];
  const seen = new Set();
  const path = [{ r: start.r, c: start.c }];

  let agent = { ...start };
  let target = { ...end };
  const maxSteps = rows * cols * 2;

  for (let step = 0; step < maxSteps; step++) {
    if (agent.r === target.r && agent.c === target.c) break;

    // Target moves every 4 steps toward a neighbor (and converges back to end)
    if (step > 0 && step % 4 === 0 && (target.r !== end.r || target.c !== end.c)) {
      const tNbs = getNeighbors(grid, target.r, target.c, allowDiagonal);
      tNbs.sort(
        (a, b) =>
          heuristic(a.r, a.c, end.r, end.c, allowDiagonal) -
          heuristic(b.r, b.c, end.r, end.c, allowDiagonal)
      );
      if (tNbs.length > 0) target = { r: tNbs[0].r, c: tNbs[0].c };
    }

    const res = runAStarCore(grid, agent, target, allowDiagonal, 1);
    for (const v of res.visitedOrder.slice(0, 6)) {
      const k = `${v.r},${v.c}`;
      if (!seen.has(k)) {
        seen.add(k);
        visitedOrder.push(v);
      }
    }
    if (res.path.length < 2) break;
    agent = res.path[1];
    path.push(agent);
  }

  // Ensure path reaches end
  if (agent.r !== end.r || agent.c !== end.c) {
    const tail = runAStarCore(grid, agent, end, allowDiagonal, 1);
    if (tail.path.length > 1) path.push(...tail.path.slice(1));
  }
  return { visitedOrder, path };
}

export function realTimeDijkstra(grid, start, end, allowDiagonal) {
  // Real-Time Dijkstra: bounded-horizon Dijkstra steps from agent's current cell
  const rows = grid.length;
  const cols = grid[0].length;
  const visitedOrder = [];
  const seen = new Set();
  const path = [{ r: start.r, c: start.c }];
  let curr = { ...start };
  const HORIZON = 18;

  for (let step = 0; step < rows * cols; step++) {
    if (curr.r === end.r && curr.c === end.c) break;
    const startIdx = idxOf(curr.r, curr.c, cols);
    const endIdx = idxOf(end.r, end.c, cols);
    const dist = new Map([[startIdx, 0]]);
    const parent = new Map();
    const closed = new Set();
    const pq = new MinHeap((a, b) => a.f - b.f);
    pq.push({ idx: startIdx, f: 0 });
    let bestFrontier = -1;
    let bestH = Infinity;
    let count = 0;

    while (pq.size > 0 && count < HORIZON) {
      const { idx } = pq.pop();
      if (closed.has(idx)) continue;
      closed.add(idx);
      count++;
      const [r, c] = rcOf(idx, cols);
      const k = `${r},${c}`;
      if (!seen.has(k)) {
        seen.add(k);
        visitedOrder.push({ r, c });
      }
      if (idx === endIdx) {
        bestFrontier = endIdx;
        break;
      }
      const h = heuristic(r, c, end.r, end.c, allowDiagonal);
      if (h < bestH && idx !== startIdx) {
        bestH = h;
        bestFrontier = idx;
      }
      for (const nb of getNeighbors(grid, r, c, allowDiagonal)) {
        const nIdx = idxOf(nb.r, nb.c, cols);
        if (closed.has(nIdx)) continue;
        const nd = dist.get(idx) + nb.cost;
        if (nd < (dist.get(nIdx) ?? Infinity)) {
          dist.set(nIdx, nd);
          parent.set(nIdx, idx);
          pq.push({ idx: nIdx, f: nd });
        }
      }
    }

    if (bestFrontier === -1) break;
    const sub = reconstructPath(parent, startIdx, bestFrontier, cols);
    if (sub.length < 2) break;
    // Advance up to 2 steps along the local Dijkstra tree
    const advance = sub.slice(1, Math.min(3, sub.length));
    path.push(...advance);
    curr = advance[advance.length - 1];
  }

  if (curr.r !== end.r || curr.c !== end.c) {
    const fb = runAStarCore(grid, start, end, allowDiagonal, 1);
    return { visitedOrder, path: fb.path };
  }
  return { visitedOrder, path };
}
