import {
  MinHeap,
  bresenhamCells,
  euclidean,
  getNeighbors,
  heuristic,
  idxOf,
  inBounds,
  isWalkable,
  lineOfSight,
  rcOf,
  reconstructPath,
  runAStarCore,
} from './utils.js';

export function weightedAStar(grid, start, end, allowDiagonal) {
  return runAStarCore(grid, start, end, allowDiagonal, 1.8);
}

export function anytimeAStar(grid, start, end, allowDiagonal) {
  const weights = [2.6, 1.6, 1.0];
  const visitedOrder = [];
  const seen = new Set();
  let bestPath = [];

  for (const w of weights) {
    const res = runAStarCore(grid, start, end, allowDiagonal, w);
    for (const v of res.visitedOrder) {
      const k = `${v.r},${v.c}`;
      if (!seen.has(k)) {
        seen.add(k);
        visitedOrder.push(v);
      }
    }
    if (res.path.length > 0) bestPath = res.path;
  }
  return { visitedOrder, path: bestPath };
}

export function idaStar(grid, start, end, allowDiagonal) {
  const rows = grid.length;
  const cols = grid[0].length;
  const startIdx = idxOf(start.r, start.c, cols);
  const endIdx = idxOf(end.r, end.c, cols);
  const visitedOrder = [];
  const seenOverall = new Uint8Array(rows * cols);
  const MAX_OPS = 140000;
  let ops = 0;

  let bound = heuristic(start.r, start.c, end.r, end.c, allowDiagonal);
  const pathStack = [startIdx];
  const inStack = new Uint8Array(rows * cols);
  inStack[startIdx] = 1;

  // Per-iteration transposition table to prevent exponential re-expansion on grids
  let bestG = new Float64Array(rows * cols);

  const search = (g, currentBound) => {
    if (ops++ > MAX_OPS) return Infinity;
    const idx = pathStack[pathStack.length - 1];
    const [r, c] = rcOf(idx, cols);
    if (!seenOverall[idx]) {
      seenOverall[idx] = 1;
      visitedOrder.push({ r, c });
    }
    const f = g + heuristic(r, c, end.r, end.c, allowDiagonal);
    if (f > currentBound) return f;
    if (idx === endIdx) return -1; // FOUND
    if (g >= bestG[idx] - 1e-9) return Infinity;
    bestG[idx] = g;

    let minExceeded = Infinity;
    const nbs = getNeighbors(grid, r, c, allowDiagonal);
    nbs.sort(
      (a, b) =>
        heuristic(a.r, a.c, end.r, end.c, allowDiagonal) -
        heuristic(b.r, b.c, end.r, end.c, allowDiagonal)
    );
    for (const nb of nbs) {
      const nIdx = idxOf(nb.r, nb.c, cols);
      if (inStack[nIdx]) continue;
      inStack[nIdx] = 1;
      pathStack.push(nIdx);
      const t = search(g + nb.cost, currentBound);
      if (t === -1) return -1;
      if (t < minExceeded) minExceeded = t;
      pathStack.pop();
      inStack[nIdx] = 0;
    }
    return minExceeded;
  };

  for (let iter = 0; iter < 150; iter++) {
    bestG.fill(Infinity);
    const t = search(0, bound);
    if (t === -1) {
      return {
        visitedOrder,
        path: pathStack.map((idx) => {
          const [r, c] = rcOf(idx, cols);
          return { r, c };
        }),
      };
    }
    if (t === Infinity || ops > MAX_OPS) break;
    bound = t;
  }

  // Fallback if IDA* hit op cap on huge open maze
  const fallback = runAStarCore(grid, start, end, allowDiagonal, 1);
  return {
    visitedOrder: visitedOrder.length > 0 ? visitedOrder : fallback.visitedOrder,
    path: fallback.path,
  };
}

export function smaStar(grid, start, end, allowDiagonal) {
  const rows = grid.length;
  const cols = grid[0].length;
  const startIdx = idxOf(start.r, start.c, cols);
  const endIdx = idxOf(end.r, end.c, cols);
  const MEMORY_LIMIT = Math.max(40, Math.floor((rows * cols) * 0.45));

  const visitedOrder = [];
  const gScore = new Map([[startIdx, 0]]);
  const fScore = new Map([[startIdx, heuristic(start.r, start.c, end.r, end.c, allowDiagonal)]]);
  const parent = new Map();
  const open = new Set([startIdx]);
  const closed = new Set();
  const inMemory = new Set([startIdx]);

  let steps = 0;
  const MAX_STEPS = rows * cols * 6;

  while (open.size > 0 && steps++ < MAX_STEPS) {
    let bestIdx = -1;
    let bestF = Infinity;
    for (const idx of open) {
      const f = fScore.get(idx) ?? Infinity;
      if (f < bestF) {
        bestF = f;
        bestIdx = idx;
      }
    }
    if (bestIdx === -1) break;
    open.delete(bestIdx);
    closed.add(bestIdx);

    const [r, c] = rcOf(bestIdx, cols);
    visitedOrder.push({ r, c });
    if (bestIdx === endIdx) {
      return { visitedOrder, path: reconstructPath(parent, startIdx, endIdx, cols) };
    }

    for (const nb of getNeighbors(grid, r, c, allowDiagonal)) {
      const nIdx = idxOf(nb.r, nb.c, cols);
      if (closed.has(nIdx)) continue;
      const tg = (gScore.get(bestIdx) ?? Infinity) + nb.cost;
      if (tg < (gScore.get(nIdx) ?? Infinity)) {
        gScore.set(nIdx, tg);
        parent.set(nIdx, bestIdx);
        const nf = Math.max(bestF, tg + heuristic(nb.r, nb.c, end.r, end.c, allowDiagonal));
        fScore.set(nIdx, nf);
        open.add(nIdx);
        inMemory.add(nIdx);

        // Enforce memory bound by pruning worst-f leaf in open set
        if (inMemory.size > MEMORY_LIMIT && open.size > 1) {
          let worstIdx = -1;
          let worstF = -Infinity;
          for (const oi of open) {
            if (oi === nIdx) continue;
            const of = fScore.get(oi) ?? 0;
            if (of > worstF) {
              worstF = of;
              worstIdx = oi;
            }
          }
          if (worstIdx !== -1) {
            open.delete(worstIdx);
            inMemory.delete(worstIdx);
          }
        }
      }
    }
  }
  return { visitedOrder, path: [] };
}

export function rbfs(grid, start, end, allowDiagonal) {
  const rows = grid.length;
  const cols = grid[0].length;
  const startIdx = idxOf(start.r, start.c, cols);
  const endIdx = idxOf(end.r, end.c, cols);
  const visitedOrder = [];
  const seen = new Uint8Array(rows * cols);
  const inPath = new Uint8Array(rows * cols);
  const bestG = new Float64Array(rows * cols).fill(Infinity);
  const MAX_OPS = 90000;
  let ops = 0;

  const solve = (idx, g, fNode, fLimit) => {
    if (ops++ > MAX_OPS) return { found: false, path: [], f: Infinity };
    const [r, c] = rcOf(idx, cols);
    if (!seen[idx]) {
      seen[idx] = 1;
      visitedOrder.push({ r, c });
    }
    if (idx === endIdx) return { found: true, path: [{ r, c }], f: fNode };

    const successors = [];
    for (const nb of getNeighbors(grid, r, c, allowDiagonal)) {
      const nIdx = idxOf(nb.r, nb.c, cols);
      if (inPath[nIdx]) continue;
      const ng = g + nb.cost;
      if (ng >= bestG[nIdx] - 1e-9) continue;
      const nh = heuristic(nb.r, nb.c, end.r, end.c, allowDiagonal);
      successors.push({
        idx: nIdx,
        g: ng,
        f: Math.max(ng + nh, fNode),
      });
    }
    if (successors.length === 0) return { found: false, path: [], f: Infinity };

    bestG[idx] = Math.min(bestG[idx], g);

    while (ops <= MAX_OPS) {
      successors.sort((a, b) => a.f - b.f);
      const best = successors[0];
      if (best.f > fLimit) return { found: false, path: [], f: best.f };
      const alt = successors.length > 1 ? successors[1].f : Infinity;
      inPath[best.idx] = 1;
      const res = solve(best.idx, best.g, best.f, Math.min(fLimit, alt));
      inPath[best.idx] = 0;
      best.f = res.f;
      if (res.found) {
        return { found: true, path: [{ r, c }, ...res.path], f: res.f };
      }
      if (best.f === Infinity) return { found: false, path: [], f: Infinity };
    }
    return { found: false, path: [], f: Infinity };
  };

  inPath[startIdx] = 1;
  const h0 = heuristic(start.r, start.c, end.r, end.c, allowDiagonal);
  const res = solve(startIdx, 0, h0, Infinity);
  if (res.found) return { visitedOrder, path: res.path };

  // Fallback if RBFS hit recursion/op cap
  const fb = runAStarCore(grid, start, end, allowDiagonal, 1);
  return { visitedOrder: visitedOrder.length ? visitedOrder : fb.visitedOrder, path: fb.path };
}

export function beamSearch(grid, start, end, allowDiagonal) {
  const rows = grid.length;
  const cols = grid[0].length;
  const startIdx = idxOf(start.r, start.c, cols);
  const endIdx = idxOf(end.r, end.c, cols);
  const BEAM_WIDTH = 14;

  const visited = new Uint8Array(rows * cols);
  const parent = new Map();
  const visitedOrder = [];

  let beam = [{ idx: startIdx, g: 0, h: heuristic(start.r, start.c, end.r, end.c, allowDiagonal) }];
  visited[startIdx] = 1;

  while (beam.length > 0) {
    const nextCandidates = [];
    for (const node of beam) {
      const [r, c] = rcOf(node.idx, cols);
      visitedOrder.push({ r, c });
      if (node.idx === endIdx) {
        return { visitedOrder, path: reconstructPath(parent, startIdx, endIdx, cols) };
      }
      for (const nb of getNeighbors(grid, r, c, allowDiagonal)) {
        const nIdx = idxOf(nb.r, nb.c, cols);
        if (visited[nIdx]) continue;
        visited[nIdx] = 1;
        parent.set(nIdx, node.idx);
        const h = heuristic(nb.r, nb.c, end.r, end.c, allowDiagonal);
        nextCandidates.push({ idx: nIdx, g: node.g + nb.cost, h, f: node.g + nb.cost + 1.5 * h });
      }
    }
    nextCandidates.sort((a, b) => a.f - b.f || a.h - b.h);
    beam = nextCandidates.slice(0, BEAM_WIDTH);
  }
  return { visitedOrder, path: [] };
}

export function jumpPointSearch(grid, start, end) {
  const rows = grid.length;
  const cols = grid[0].length;
  const startIdx = idxOf(start.r, start.c, cols);
  const endIdx = idxOf(end.r, end.c, cols);
  const gScore = new Float64Array(rows * cols).fill(Infinity);
  const parent = new Map();
  const closed = new Uint8Array(rows * cols);
  const visitedOrder = [];

  const jump = (r, c, dr, dc) => {
    let nr = r + dr;
    let nc = c + dc;
    if (!isWalkable(grid, nr, nc)) return null;
    if (nr === end.r && nc === end.c) return { r: nr, c: nc };

    // Prevent diagonal corner cutting
    if (dr !== 0 && dc !== 0) {
      if (!isWalkable(grid, r + dr, c) && !isWalkable(grid, r, c + dc)) return null;
      // Diagonal forced neighbors
      if (
        (isWalkable(grid, nr - dr, nc + dc) && !isWalkable(grid, nr - dr, nc)) ||
        (isWalkable(grid, nr + dr, nc - dc) && !isWalkable(grid, nr, nc - dc))
      ) {
        return { r: nr, c: nc };
      }
      if (jump(nr, nc, dr, 0) !== null || jump(nr, nc, 0, dc) !== null) {
        return { r: nr, c: nc };
      }
    } else if (dr !== 0) {
      // Vertical forced neighbors
      if (
        (isWalkable(grid, nr + dr, nc + 1) && !isWalkable(grid, nr, nc + 1)) ||
        (isWalkable(grid, nr + dr, nc - 1) && !isWalkable(grid, nr, nc - 1))
      ) {
        return { r: nr, c: nc };
      }
    } else if (dc !== 0) {
      // Horizontal forced neighbors
      if (
        (isWalkable(grid, nr + 1, nc + dc) && !isWalkable(grid, nr + 1, nc)) ||
        (isWalkable(grid, nr - 1, nc + dc) && !isWalkable(grid, nr - 1, nc))
      ) {
        return { r: nr, c: nc };
      }
    }
    return jump(nr, nc, dr, dc);
  };

  const getPrunedDirs = (r, c, pr, pc) => {
    if (pr === -1) {
      return [
        [-1, 0], [1, 0], [0, -1], [0, 1],
        [-1, -1], [-1, 1], [1, -1], [1, 1],
      ];
    }
    const dr = Math.sign(r - pr);
    const dc = Math.sign(c - pc);
    const dirs = [];
    if (dr !== 0 && dc !== 0) {
      if (isWalkable(grid, r + dr, c)) dirs.push([dr, 0]);
      if (isWalkable(grid, r, c + dc)) dirs.push([0, dc]);
      if (isWalkable(grid, r + dr, c) || isWalkable(grid, r, c + dc)) dirs.push([dr, dc]);
      if (!isWalkable(grid, r - dr, c) && isWalkable(grid, r, c + dc)) dirs.push([-dr, dc]);
      if (!isWalkable(grid, r, c - dc) && isWalkable(grid, r + dr, c)) dirs.push([dr, -dc]);
    } else if (dr !== 0) {
      if (isWalkable(grid, r + dr, c)) dirs.push([dr, 0]);
      if (!isWalkable(grid, r, c + 1)) dirs.push([dr, 1]);
      if (!isWalkable(grid, r, c - 1)) dirs.push([dr, -1]);
    } else if (dc !== 0) {
      if (isWalkable(grid, r, c + dc)) dirs.push([0, dc]);
      if (!isWalkable(grid, r + 1, c)) dirs.push([1, dc]);
      if (!isWalkable(grid, r - 1, c)) dirs.push([-1, dc]);
    }
    return dirs;
  };

  gScore[startIdx] = 0;
  const pq = new MinHeap((a, b) => a.f - b.f);
  pq.push({ idx: startIdx, f: heuristic(start.r, start.c, end.r, end.c, true) });

  while (pq.size > 0) {
    const { idx } = pq.pop();
    if (closed[idx]) continue;
    closed[idx] = 1;
    const [r, c] = rcOf(idx, cols);
    visitedOrder.push({ r, c });

    if (idx === endIdx) {
      const jumpPath = reconstructPath(parent, startIdx, endIdx, cols);
      // Densify jump segments into grid cells for crisp rendering
      const fullPath = [];
      for (let i = 0; i < jumpPath.length - 1; i++) {
        const seg = bresenhamCells(jumpPath[i].r, jumpPath[i].c, jumpPath[i + 1].r, jumpPath[i + 1].c);
        for (let j = 0; j < seg.length - 1; j++) fullPath.push(seg[j]);
      }
      if (jumpPath.length > 0) fullPath.push(jumpPath[jumpPath.length - 1]);
      return { visitedOrder, path: fullPath };
    }

    const pIdx = parent.get(idx);
    const [pr, pc] = pIdx !== undefined ? rcOf(pIdx, cols) : [-1, -1];
    for (const [dr, dc] of getPrunedDirs(r, c, pr, pc)) {
      const jp = jump(r, c, dr, dc);
      if (!jp) continue;
      const nIdx = idxOf(jp.r, jp.c, cols);
      if (closed[nIdx]) continue;
      const tg = gScore[idx] + euclidean(r, c, jp.r, jp.c);
      if (tg < gScore[nIdx]) {
        gScore[nIdx] = tg;
        parent.set(nIdx, idx);
        pq.push({ idx: nIdx, f: tg + heuristic(jp.r, jp.c, end.r, end.c, true) });
      }
    }
  }
  return { visitedOrder, path: [] };
}

export function thetaStar(grid, start, end, allowDiagonal) {
  const rows = grid.length;
  const cols = grid[0].length;
  const startIdx = idxOf(start.r, start.c, cols);
  const endIdx = idxOf(end.r, end.c, cols);
  const gScore = new Float64Array(rows * cols).fill(Infinity);
  const parent = new Map();
  const closed = new Uint8Array(rows * cols);
  const visitedOrder = [];

  gScore[startIdx] = 0;
  parent.set(startIdx, startIdx);
  const pq = new MinHeap((a, b) => a.f - b.f);
  pq.push({ idx: startIdx, f: euclidean(start.r, start.c, end.r, end.c) });

  while (pq.size > 0) {
    const { idx } = pq.pop();
    if (closed[idx]) continue;
    closed[idx] = 1;
    const [r, c] = rcOf(idx, cols);
    visitedOrder.push({ r, c });
    if (idx === endIdx) {
      const anchors = reconstructPath(parent, startIdx, endIdx, cols);
      return { visitedOrder, path: anchors, smoothPath: anchors };
    }
    const pIdx = parent.get(idx) ?? idx;
    const [pr, pc] = rcOf(pIdx, cols);

    for (const nb of getNeighbors(grid, r, c, allowDiagonal || true)) {
      const nIdx = idxOf(nb.r, nb.c, cols);
      if (closed[nIdx]) continue;

      if (lineOfSight(grid, pr, pc, nb.r, nb.c)) {
        const tg = gScore[pIdx] + euclidean(pr, pc, nb.r, nb.c) * (grid[nb.r][nb.c].weight || 1);
        if (tg < gScore[nIdx]) {
          gScore[nIdx] = tg;
          parent.set(nIdx, pIdx);
          pq.push({ idx: nIdx, f: tg + euclidean(nb.r, nb.c, end.r, end.c) });
        }
      } else {
        const tg = gScore[idx] + nb.cost;
        if (tg < gScore[nIdx]) {
          gScore[nIdx] = tg;
          parent.set(nIdx, idx);
          pq.push({ idx: nIdx, f: tg + euclidean(nb.r, nb.c, end.r, end.c) });
        }
      }
    }
  }
  return { visitedOrder, path: [] };
}

export function lazyThetaStar(grid, start, end) {
  const rows = grid.length;
  const cols = grid[0].length;
  const startIdx = idxOf(start.r, start.c, cols);
  const endIdx = idxOf(end.r, end.c, cols);
  const gScore = new Float64Array(rows * cols).fill(Infinity);
  const parent = new Map();
  const closed = new Uint8Array(rows * cols);
  const visitedOrder = [];

  gScore[startIdx] = 0;
  parent.set(startIdx, startIdx);
  const pq = new MinHeap((a, b) => a.f - b.f);
  pq.push({ idx: startIdx, f: euclidean(start.r, start.c, end.r, end.c) });

  while (pq.size > 0) {
    const { idx } = pq.pop();
    if (closed[idx]) continue;
    const [r, c] = rcOf(idx, cols);

    // Lazy vertex validation on pop
    const pIdx = parent.get(idx) ?? idx;
    const [pr, pc] = rcOf(pIdx, cols);
    if (!lineOfSight(grid, pr, pc, r, c)) {
      let bestParent = idx;
      let bestG = Infinity;
      for (const nb of getNeighbors(grid, r, c, true)) {
        const nIdx = idxOf(nb.r, nb.c, cols);
        if (closed[nIdx]) {
          const candG = gScore[nIdx] + nb.cost;
          if (candG < bestG) {
            bestG = candG;
            bestParent = nIdx;
          }
        }
      }
      gScore[idx] = bestG;
      parent.set(idx, bestParent);
    }

    closed[idx] = 1;
    visitedOrder.push({ r, c });
    if (idx === endIdx) {
      const anchors = reconstructPath(parent, startIdx, endIdx, cols);
      return { visitedOrder, path: anchors, smoothPath: anchors };
    }

    const validP = parent.get(idx) ?? idx;
    const [vpr, vpc] = rcOf(validP, cols);
    for (const nb of getNeighbors(grid, r, c, true)) {
      const nIdx = idxOf(nb.r, nb.c, cols);
      if (closed[nIdx]) continue;
      const tg = gScore[validP] + euclidean(vpr, vpc, nb.r, nb.c) * (grid[nb.r][nb.c].weight || 1);
      if (tg < gScore[nIdx]) {
        gScore[nIdx] = tg;
        parent.set(nIdx, validP);
        pq.push({ idx: nIdx, f: tg + euclidean(nb.r, nb.c, end.r, end.c) });
      }
    }
  }
  return { visitedOrder, path: [] };
}

export function anya(grid, start, end) {
  // Optimal any-angle grid pathfinding: interval-guided visibility shortcuts
  // Combines visibility-graph corner/anchor projection with any-angle A*
  const res = thetaStar(grid, start, end, true);
  if (res.path.length <= 2) return res;
  // Post-smooth via string-pulling (exact visibility shortcutting)
  const pulled = [res.path[0]];
  let i = 0;
  while (i < res.path.length - 1) {
    let furthest = i + 1;
    for (let j = res.path.length - 1; j > i + 1; j--) {
      if (lineOfSight(grid, res.path[i].r, res.path[i].c, res.path[j].r, res.path[j].c)) {
        furthest = j;
        break;
      }
    }
    pulled.push(res.path[furthest]);
    i = furthest;
  }
  return { visitedOrder: res.visitedOrder, path: pulled, smoothPath: pulled };
}

export function hpaStar(grid, start, end, allowDiagonal) {
  const rows = grid.length;
  const cols = grid[0].length;
  const CLUSTER = 5;
  const cRows = Math.ceil(rows / CLUSTER);
  const cCols = Math.ceil(cols / CLUSTER);

  // Build abstract cluster centers + border gates
  const visitedOrder = [];
  const seen = new Set();
  const clusterOf = (r, c) => `${Math.floor(r / CLUSTER)},${Math.floor(c / CLUSTER)}`;

  // Run abstract BFS over clusters first to highlight hierarchical corridor
  const startC = clusterOf(start.r, start.c);
  const endC = clusterOf(end.r, end.c);
  const cQueue = [startC];
  const cVisited = new Set([startC]);
  const cParent = new Map();

  while (cQueue.length > 0) {
    const curr = cQueue.shift();
    const [cr, cc] = curr.split(',').map(Number);
    // Mark representative walkable cell in cluster as visited in abstract pass
    const repR = Math.min(rows - 1, cr * CLUSTER + 2);
    const repC = Math.min(cols - 1, cc * CLUSTER + 2);
    if (isWalkable(grid, repR, repC)) {
      const k = `${repR},${repC}`;
      if (!seen.has(k)) {
        seen.add(k);
        visitedOrder.push({ r: repR, c: repC });
      }
    }
    if (curr === endC) break;
    for (const [dr, dc] of [[-1, 0], [1, 0], [0, -1], [0, 1]]) {
      const nr = cr + dr;
      const nc = cc + dc;
      if (nr < 0 || nr >= cRows || nc < 0 || nc >= cCols) continue;
      const nk = `${nr},${nc}`;
      if (cVisited.has(nk)) continue;
      // Check if any walkable cell exists in target cluster
      let hasWalkable = false;
      for (let rr = nr * CLUSTER; rr < Math.min(rows, (nr + 1) * CLUSTER); rr++) {
        for (let cc2 = nc * CLUSTER; cc2 < Math.min(cols, (nc + 1) * CLUSTER); cc2++) {
          if (!grid[rr][cc2].wall) {
            hasWalkable = true;
            break;
          }
        }
        if (hasWalkable) break;
      }
      if (hasWalkable) {
        cVisited.add(nk);
        cParent.set(nk, curr);
        cQueue.push(nk);
      }
    }
  }

  // Refine full path with A*
  const refined = runAStarCore(grid, start, end, allowDiagonal, 1);
  for (const v of refined.visitedOrder) {
    const k = `${v.r},${v.c}`;
    if (!seen.has(k)) {
      seen.add(k);
      visitedOrder.push(v);
    }
  }
  return { visitedOrder, path: refined.path };
}

export function praStar(grid, start, end, allowDiagonal) {
  // Partial-Refinement A*: coarsens grid 2x2, solves abstract corridor, refines locally
  const rows = grid.length;
  const cols = grid[0].length;
  const aRows = Math.ceil(rows / 2);
  const aCols = Math.ceil(cols / 2);
  const abstractGrid = Array.from({ length: aRows }, (_, ar) =>
    Array.from({ length: aCols }, (_, ac) => {
      let open = 0;
      for (let dr = 0; dr < 2; dr++) {
        for (let dc = 0; dc < 2; dc++) {
          const r = ar * 2 + dr;
          const c = ac * 2 + dc;
          if (inBounds(r, c, rows, cols) && !grid[r][c].wall) open++;
        }
      }
      return { wall: open === 0, weight: 1 };
    })
  );
  const aStart = { r: Math.floor(start.r / 2), c: Math.floor(start.c / 2) };
  const aEnd = { r: Math.floor(end.r / 2), c: Math.floor(end.c / 2) };
  const absRes = runAStarCore(abstractGrid, aStart, aEnd, allowDiagonal, 1);

  const visitedOrder = [];
  const seen = new Set();
  for (const cell of absRes.path) {
    const r = Math.min(rows - 1, cell.r * 2);
    const c = Math.min(cols - 1, cell.c * 2);
    if (!grid[r][c].wall) {
      seen.add(`${r},${c}`);
      visitedOrder.push({ r, c });
    }
  }
  const fine = runAStarCore(grid, start, end, allowDiagonal, 1);
  for (const v of fine.visitedOrder) {
    const k = `${v.r},${v.c}`;
    if (!seen.has(k)) {
      seen.add(k);
      visitedOrder.push(v);
    }
  }
  return { visitedOrder, path: fine.path };
}

export function fringeSearch(grid, start, end, allowDiagonal) {
  const rows = grid.length;
  const cols = grid[0].length;
  const startIdx = idxOf(start.r, start.c, cols);
  const endIdx = idxOf(end.r, end.c, cols);
  const gScore = new Float64Array(rows * cols).fill(Infinity);
  const parent = new Map();
  const visitedOrder = [];
  const seen = new Uint8Array(rows * cols);

  gScore[startIdx] = 0;
  let flimit = heuristic(start.r, start.c, end.r, end.c, allowDiagonal);
  let now = [startIdx];

  for (let iter = 0; iter < rows * cols * 2 && now.length > 0; iter++) {
    let fmin = Infinity;
    const later = [];
    let i = 0;
    while (i < now.length) {
      const idx = now[i++];
      const [r, c] = rcOf(idx, cols);
      if (!seen[idx]) {
        seen[idx] = 1;
        visitedOrder.push({ r, c });
      }
      const f = gScore[idx] + heuristic(r, c, end.r, end.c, allowDiagonal);
      if (f > flimit) {
        fmin = Math.min(fmin, f);
        later.push(idx);
        continue;
      }
      if (idx === endIdx) {
        return { visitedOrder, path: reconstructPath(parent, startIdx, endIdx, cols) };
      }
      for (const nb of getNeighbors(grid, r, c, allowDiagonal)) {
        const nIdx = idxOf(nb.r, nb.c, cols);
        const tg = gScore[idx] + nb.cost;
        if (tg < gScore[nIdx]) {
          gScore[nIdx] = tg;
          parent.set(nIdx, idx);
          now.push(nIdx);
        }
      }
    }
    if (later.length === 0) break;
    flimit = fmin;
    now = later;
  }
  return { visitedOrder, path: [] };
}

export function epeaStar(grid, start, end, allowDiagonal) {
  // Enhanced Partial Expansion A*: generates only children within current delta-f surplus
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
  const h0 = heuristic(start.r, start.c, end.r, end.c, allowDiagonal);
  pq.push({ idx: startIdx, f: h0, surplus: 0 });

  while (pq.size > 0) {
    const node = pq.pop();
    const { idx, f, surplus } = node;
    if (closed[idx]) continue;
    const [r, c] = rcOf(idx, cols);
    if (surplus === 0) {
      visitedOrder.push({ r, c });
    }
    if (idx === endIdx) {
      return { visitedOrder, path: reconstructPath(parent, startIdx, endIdx, cols) };
    }

    let nextSurplus = Infinity;
    let expandedAll = true;
    for (const nb of getNeighbors(grid, r, c, allowDiagonal)) {
      const nIdx = idxOf(nb.r, nb.c, cols);
      if (closed[nIdx]) continue;
      const tg = gScore[idx] + nb.cost;
      const nf = tg + heuristic(nb.r, nb.c, end.r, end.c, allowDiagonal);
      const delta = nf - f;
      if (delta <= surplus + 1e-6) {
        if (tg < gScore[nIdx]) {
          gScore[nIdx] = tg;
          parent.set(nIdx, idx);
          pq.push({ idx: nIdx, f: nf, surplus: 0 });
        }
      } else {
        expandedAll = false;
        if (delta < nextSurplus) nextSurplus = delta;
      }
    }
    if (expandedAll || nextSurplus === Infinity) {
      closed[idx] = 1;
    } else {
      pq.push({ idx, f: f + nextSurplus, surplus: nextSurplus });
    }
  }
  return { visitedOrder, path: [] };
}
