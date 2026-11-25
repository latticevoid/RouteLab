import {
  MinHeap,
  bresenhamCells,
  computeClearanceField,
  euclidean,
  getNeighbors,
  isWalkable,
  lineOfSight,
  runAStarCore,
  seededRandom,
} from './utils.js';

function isSegmentFree(grid, r0, c0, r1, c1) {
  const steps = Math.max(2, Math.ceil(Math.hypot(r1 - r0, c1 - c0) * 3));
  for (let i = 0; i <= steps; i++) {
    const t = i / steps;
    const r = Math.round(r0 + (r1 - r0) * t);
    const c = Math.round(c0 + (c1 - c0) * t);
    if (!isWalkable(grid, r, c)) return false;
  }
  return lineOfSight(grid, Math.round(r0), Math.round(c0), Math.round(r1), Math.round(c1));
}

function traceTreePath(nodes, endNodeIdx) {
  const path = [];
  let curr = endNodeIdx;
  const seen = new Set();
  while (curr !== -1 && curr !== undefined && !seen.has(curr)) {
    seen.add(curr);
    path.push({ r: Math.round(nodes[curr].r), c: Math.round(nodes[curr].c) });
    curr = nodes[curr].parent;
  }
  path.reverse();
  return path;
}

function densifyCells(path) {
  if (path.length <= 1) return path;
  const out = [];
  for (let i = 0; i < path.length - 1; i++) {
    const seg = bresenhamCells(path[i].r, path[i].c, path[i + 1].r, path[i + 1].c);
    for (let j = 0; j < seg.length - 1; j++) out.push(seg[j]);
  }
  out.push(path[path.length - 1]);
  return out;
}

function runRRTBase(grid, start, end, { star = false, informed = false, est = false, kpiece = false } = {}) {
  const rows = grid.length;
  const cols = grid[0].length;
  const rand = seededRandom(42 + rows * 7 + cols);
  const nodes = [{ r: start.r, c: start.c, parent: -1, cost: 0 }];
  const treeEdges = [];
  const visitedOrder = [{ r: start.r, c: start.c }];
  const seenCell = new Set([`${start.r},${start.c}`]);

  // KPIECE / EST cell density map
  const cellCounts = new Map([[`${start.r},${start.c}`, 1]]);
  const STEP = 2.4;
  const REWIRE_R = 4.2;
  const MAX_ITER = Math.min(2800, rows * cols * 4);
  let goalNodeIdx = -1;
  let cBest = Infinity;

  const markCell = (r, c) => {
    const rr = Math.round(r);
    const cc = Math.round(c);
    const k = `${rr},${cc}`;
    if (!seenCell.has(k) && isWalkable(grid, rr, cc)) {
      seenCell.add(k);
      visitedOrder.push({ r: rr, c: cc });
    }
  };

  for (let iter = 0; iter < MAX_ITER; iter++) {
    let sr, sc;
    if (rand() < 0.14) {
      sr = end.r;
      sc = end.c;
    } else if (informed && cBest < Infinity) {
      // Sample inside bounding box / ellipse
      const minR = Math.max(0, Math.floor(Math.min(start.r, end.r) - cBest * 0.5));
      const maxR = Math.min(rows - 1, Math.ceil(Math.max(start.r, end.r) + cBest * 0.5));
      const minC = Math.max(0, Math.floor(Math.min(start.c, end.c) - cBest * 0.5));
      const maxC = Math.min(cols - 1, Math.ceil(Math.max(start.c, end.c) + cBest * 0.5));
      sr = minR + Math.floor(rand() * (maxR - minR + 1));
      sc = minC + Math.floor(rand() * (maxC - minC + 1));
      if (euclidean(start.r, start.c, sr, sc) + euclidean(sr, sc, end.r, end.c) > cBest) continue;
    } else {
      sr = Math.floor(rand() * rows);
      sc = Math.floor(rand() * cols);
    }

    if (!isWalkable(grid, sr, sc)) continue;

    // Select nearest (or density-weighted for EST / KPIECE)
    let nearIdx = 0;
    let bestScore = Infinity;
    for (let i = 0; i < nodes.length; i++) {
      const d = euclidean(nodes[i].r, nodes[i].c, sr, sc);
      let score = d;
      if (est || kpiece) {
        const ck = `${Math.round(nodes[i].r)},${Math.round(nodes[i].c)}`;
        score = d * (1 + 0.35 * (cellCounts.get(ck) || 1));
      }
      if (score < bestScore) {
        bestScore = score;
        nearIdx = i;
      }
    }

    const near = nodes[nearIdx];
    const dist = euclidean(near.r, near.c, sr, sc);
    if (dist < 0.5) continue;
    const scale = Math.min(1, STEP / dist);
    const nr = Math.round(near.r + (sr - near.r) * scale);
    const nc = Math.round(near.c + (sc - near.c) * scale);
    if (!isWalkable(grid, nr, nc)) continue;
    if (!isSegmentFree(grid, near.r, near.c, nr, nc)) continue;

    let bestParent = nearIdx;
    let bestCost = near.cost + euclidean(near.r, near.c, nr, nc);

    if (star) {
      for (let i = 0; i < nodes.length; i++) {
        const d = euclidean(nodes[i].r, nodes[i].c, nr, nc);
        if (d <= REWIRE_R && nodes[i].cost + d < bestCost && isSegmentFree(grid, nodes[i].r, nodes[i].c, nr, nc)) {
          bestParent = i;
          bestCost = nodes[i].cost + d;
        }
      }
    }

    const newIdx = nodes.length;
    nodes.push({ r: nr, c: nc, parent: bestParent, cost: bestCost });
    treeEdges.push([nodes[bestParent], nodes[newIdx]]);
    const ck = `${nr},${nc}`;
    cellCounts.set(ck, (cellCounts.get(ck) || 0) + 1);
    markCell(nr, nc);

    if (star) {
      for (let i = 0; i < nodes.length - 1; i++) {
        const d = euclidean(nr, nc, nodes[i].r, nodes[i].c);
        if (d <= REWIRE_R && bestCost + d < nodes[i].cost && isSegmentFree(grid, nr, nc, nodes[i].r, nodes[i].c)) {
          nodes[i].parent = newIdx;
          nodes[i].cost = bestCost + d;
        }
      }
    }

    const dGoal = euclidean(nr, nc, end.r, end.c);
    if (dGoal <= STEP && isSegmentFree(grid, nr, nc, end.r, end.c)) {
      const gCost = bestCost + dGoal;
      if (gCost < cBest) {
        cBest = gCost;
        const gIdx = nodes.length;
        nodes.push({ r: end.r, c: end.c, parent: newIdx, cost: gCost });
        treeEdges.push([nodes[newIdx], nodes[gIdx]]);
        goalNodeIdx = gIdx;
        if (!star && !informed) break;
      }
    }
  }

  if (goalNodeIdx === -1) {
    // If maze corridors are 1-cell narrow and sampling missed, run A* fallback while keeping RRT tree
    const fb = runAStarCore(grid, start, end, true, 1);
    return { visitedOrder, path: fb.path, treeEdges };
  }
  const raw = traceTreePath(nodes, goalNodeIdx);
  return { visitedOrder, path: densifyCells(raw), treeEdges, smoothPath: raw };
}

export const rrt = (g, s, e) => runRRTBase(g, s, e, {});
export const rrtStar = (g, s, e) => runRRTBase(g, s, e, { star: true });
export const informedRrtStar = (g, s, e) => runRRTBase(g, s, e, { star: true, informed: true });
export const est = (g, s, e) => runRRTBase(g, s, e, { est: true });
export const kpiece = (g, s, e) => runRRTBase(g, s, e, { kpiece: true });

export function bidirectionalRrt(grid, start, end) {
  const rows = grid.length;
  const cols = grid[0].length;
  const rand = seededRandom(99);
  const treeA = [{ r: start.r, c: start.c, parent: -1 }];
  const treeB = [{ r: end.r, c: end.c, parent: -1 }];
  const treeEdges = [];
  const visitedOrder = [{ r: start.r, c: start.c }, { r: end.r, c: end.c }];
  const seen = new Set([`${start.r},${start.c}`, `${end.r},${end.c}`]);
  const STEP = 2.5;

  const nearest = (tree, r, c) => {
    let best = 0;
    let bd = Infinity;
    for (let i = 0; i < tree.length; i++) {
      const d = euclidean(tree[i].r, tree[i].c, r, c);
      if (d < bd) {
        bd = d;
        best = i;
      }
    }
    return best;
  };

  for (let iter = 0; iter < rows * cols * 3; iter++) {
    const sr = Math.floor(rand() * rows);
    const sc = Math.floor(rand() * cols);
    if (!isWalkable(grid, sr, sc)) continue;

    const aIdx = nearest(treeA, sr, sc);
    const aNode = treeA[aIdx];
    const da = euclidean(aNode.r, aNode.c, sr, sc);
    if (da < 0.5) continue;
    const scA = Math.min(1, STEP / da);
    const ar = Math.round(aNode.r + (sr - aNode.r) * scA);
    const ac = Math.round(aNode.c + (sc - aNode.c) * scA);
    if (!isWalkable(grid, ar, ac) || !isSegmentFree(grid, aNode.r, aNode.c, ar, ac)) continue;

    const newAIdx = treeA.length;
    treeA.push({ r: ar, c: ac, parent: aIdx });
    treeEdges.push([aNode, treeA[newAIdx]]);
    if (!seen.has(`${ar},${ac}`)) {
      seen.add(`${ar},${ac}`);
      visitedOrder.push({ r: ar, c: ac });
    }

    // Connect treeB toward (ar, ac)
    const bIdx = nearest(treeB, ar, ac);
    const bNode = treeB[bIdx];
    const db = euclidean(bNode.r, bNode.c, ar, ac);
    const scB = Math.min(1, STEP / Math.max(0.1, db));
    const br = Math.round(bNode.r + (ar - bNode.r) * scB);
    const bc = Math.round(bNode.c + (ac - bNode.c) * scB);
    if (isWalkable(grid, br, bc) && isSegmentFree(grid, bNode.r, bNode.c, br, bc)) {
      const newBIdx = treeB.length;
      treeB.push({ r: br, c: bc, parent: bIdx });
      treeEdges.push([bNode, treeB[newBIdx]]);
      if (!seen.has(`${br},${bc}`)) {
        seen.add(`${br},${bc}`);
        visitedOrder.push({ r: br, c: bc });
      }
      if (euclidean(ar, ac, br, bc) <= 2.5 && isSegmentFree(grid, ar, ac, br, bc)) {
        const pA = traceTreePath(treeA, newAIdx);
        const pB = traceTreePath(treeB, newBIdx).reverse();
        const combined = [...pA, ...pB];
        return { visitedOrder, path: densifyCells(combined), treeEdges, smoothPath: combined };
      }
    }
  }

  const fb = runAStarCore(grid, start, end, true, 1);
  return { visitedOrder, path: fb.path, treeEdges };
}

function runPRMBase(grid, start, end, { lazy = false } = {}) {
  const rows = grid.length;
  const cols = grid[0].length;
  const rand = seededRandom(lazy ? 202 : 101);
  const samples = [{ r: start.r, c: start.c }, { r: end.r, c: end.c }];
  const seen = new Set([`${start.r},${start.c}`, `${end.r},${end.c}`]);
  const targetSamples = Math.min(240, Math.max(80, Math.floor((rows * cols) * 0.32)));

  for (let i = 0; i < targetSamples * 4 && samples.length < targetSamples; i++) {
    const r = Math.floor(rand() * rows);
    const c = Math.floor(rand() * cols);
    const k = `${r},${c}`;
    if (!seen.has(k) && isWalkable(grid, r, c)) {
      seen.add(k);
      samples.push({ r, c });
    }
  }

  const N = samples.length;
  const adj = Array.from({ length: N }, () => []);
  const treeEdges = [];
  const K = 8;

  for (let i = 0; i < N; i++) {
    const dists = [];
    for (let j = 0; j < N; j++) {
      if (i === j) continue;
      const d = euclidean(samples[i].r, samples[i].c, samples[j].r, samples[j].c);
      if (d <= 8) dists.push({ j, d });
    }
    dists.sort((a, b) => a.d - b.d);
    for (let m = 0; m < Math.min(K, dists.length); m++) {
      const { j, d } = dists[m];
      if (lazy || isSegmentFree(grid, samples[i].r, samples[i].c, samples[j].r, samples[j].c)) {
        adj[i].push({ to: j, cost: d, valid: !lazy });
        adj[j].push({ to: i, cost: d, valid: !lazy });
        if (i < j) treeEdges.push([samples[i], samples[j]]);
      }
    }
  }

  const astarRoadmap = () => {
    const g = new Float64Array(N).fill(Infinity);
    const parent = new Int32Array(N).fill(-1);
    const closed = new Uint8Array(N);
    const pq = new MinHeap((a, b) => a.f - b.f);
    g[0] = 0;
    pq.push({ idx: 0, f: euclidean(start.r, start.c, end.r, end.c) });
    while (pq.size > 0) {
      const { idx } = pq.pop();
      if (closed[idx]) continue;
      closed[idx] = 1;
      if (idx === 1) {
        const res = [];
        let c = 1;
        while (c !== -1) {
          res.push(c);
          c = parent[c];
        }
        return res.reverse();
      }
      for (const e of adj[idx]) {
        if (closed[e.to]) continue;
        const tg = g[idx] + e.cost;
        if (tg < g[e.to]) {
          g[e.to] = tg;
          parent[e.to] = idx;
          pq.push({
            idx: e.to,
            f: tg + euclidean(samples[e.to].r, samples[e.to].c, end.r, end.c),
          });
        }
      }
    }
    return [];
  };

  let nodePath = [];
  for (let attempt = 0; attempt < (lazy ? 15 : 1); attempt++) {
    nodePath = astarRoadmap();
    if (nodePath.length === 0) break;
    if (!lazy) break;
    let allValid = true;
    for (let i = 0; i < nodePath.length - 1; i++) {
      const u = nodePath[i];
      const v = nodePath[i + 1];
      if (!isSegmentFree(grid, samples[u].r, samples[u].c, samples[v].r, samples[v].c)) {
        adj[u] = adj[u].filter((e) => e.to !== v);
        adj[v] = adj[v].filter((e) => e.to !== u);
        allValid = false;
        break;
      }
    }
    if (allValid) break;
  }

  const visitedOrder = samples;
  if (nodePath.length === 0) {
    const fb = runAStarCore(grid, start, end, true, 1);
    return { visitedOrder, path: fb.path, treeEdges };
  }
  const smoothPath = nodePath.map((i) => samples[i]);
  return { visitedOrder, path: densifyCells(smoothPath), treeEdges, smoothPath };
}

export const prm = (g, s, e) => runPRMBase(g, s, e, { lazy: false });
export const lazyPrm = (g, s, e) => runPRMBase(g, s, e, { lazy: true });

function runFMTBase(grid, start, end, { star = false, bit = false } = {}) {
  const rows = grid.length;
  const cols = grid[0].length;
  const rand = seededRandom(bit ? 777 : star ? 555 : 333);
  const samples = [{ r: start.r, c: start.c }, { r: end.r, c: end.c }];
  const seen = new Set([`${start.r},${start.c}`, `${end.r},${end.c}`]);
  const target = Math.min(260, Math.max(90, Math.floor(rows * cols * 0.35)));

  for (let i = 0; i < target * 4 && samples.length < target; i++) {
    const r = Math.floor(rand() * rows);
    const c = Math.floor(rand() * cols);
    const k = `${r},${c}`;
    if (!seen.has(k) && isWalkable(grid, r, c)) {
      seen.add(k);
      samples.push({ r, c });
    }
  }

  const N = samples.length;
  const R = 5.5;
  const cost = new Float64Array(N).fill(Infinity);
  const parent = new Int32Array(N).fill(-1);
  const closed = new Uint8Array(N);
  const visitedOrder = [];
  const treeEdges = [];

  cost[0] = 0;
  const pq = new MinHeap((a, b) => a.f - b.f);
  pq.push({ idx: 0, f: bit ? euclidean(start.r, start.c, end.r, end.c) : 0 });

  while (pq.size > 0) {
    const { idx } = pq.pop();
    if (closed[idx]) continue;
    closed[idx] = 1;
    visitedOrder.push(samples[idx]);
    if (idx === 1) {
      const nodePath = [];
      let c = 1;
      while (c !== -1) {
        nodePath.push(samples[c]);
        c = parent[c];
      }
      nodePath.reverse();
      return { visitedOrder, path: densifyCells(nodePath), treeEdges, smoothPath: nodePath };
    }

    for (let j = 0; j < N; j++) {
      if (closed[j] || j === idx) continue;
      const d = euclidean(samples[idx].r, samples[idx].c, samples[j].r, samples[j].c);
      if (d <= R && isSegmentFree(grid, samples[idx].r, samples[idx].c, samples[j].r, samples[j].c)) {
        const nc = cost[idx] + d;
        if (nc < cost[j]) {
          cost[j] = nc;
          parent[j] = idx;
          treeEdges.push([samples[idx], samples[j]]);
          const h = bit || star ? euclidean(samples[j].r, samples[j].c, end.r, end.c) : 0;
          pq.push({ idx: j, f: nc + h });
        }
      }
    }
  }

  const fb = runAStarCore(grid, start, end, true, 1);
  return { visitedOrder, path: fb.path, treeEdges };
}

export const fmtStar = (g, s, e) => runFMTBase(g, s, e, { star: true });
export const bitStar = (g, s, e) => runFMTBase(g, s, e, { star: true, bit: true });
export const fmt = (g, s, e) => runFMTBase(g, s, e, {});

function runTrajectoryOptimizer(grid, start, end, mode) {
  // CHOMP / STOMP / TrajOpt / Elastic Bands / TEB:
  // Initialize feasible path, compute clearance field, iteratively deform/smooth waypoints
  const clearance = computeClearanceField(grid);
  const base = runAStarCore(grid, start, end, true, 1);
  if (base.path.length === 0) return { visitedOrder: base.visitedOrder, path: [] };

  let pts = base.path.map((p) => ({ r: p.r, c: p.c }));
  const rand = seededRandom(mode.length * 31);
  const iters = mode === 'stomp' ? 18 : 22;

  for (let it = 0; it < iters; it++) {
    const next = pts.map((p) => ({ ...p }));
    for (let i = 1; i < pts.length - 1; i++) {
      const prev = pts[i - 1];
      const curr = pts[i];
      const nxt = pts[i + 1];
      // Internal smoothness force (elastic contraction)
      let dr = 0.5 * (prev.r + nxt.r) - curr.r;
      let dc = 0.5 * (prev.c + nxt.c) - curr.c;

      // External obstacle repulsion from clearance gradient
      const rr = Math.round(curr.r);
      const cc = Math.round(curr.c);
      const clr = clearance[rr]?.[cc] ?? 1;
      if (clr < 2.2) {
        const cU = clearance[Math.max(0, rr - 1)][cc];
        const cD = clearance[Math.min(grid.length - 1, rr + 1)][cc];
        const cL = clearance[rr][Math.max(0, cc - 1)];
        const cR = clearance[rr][Math.min(grid[0].length - 1, cc + 1)];
        dr += 0.25 * (cD - cU);
        dc += 0.25 * (cR - cL);
      }

      if (mode === 'stomp') {
        dr += (rand() - 0.5) * 0.3;
        dc += (rand() - 0.5) * 0.3;
      }

      const candR = curr.r + 0.35 * dr;
      const candC = curr.c + 0.35 * dc;
      if (isWalkable(grid, Math.round(candR), Math.round(candC))) {
        next[i] = { r: candR, c: candC };
      }
    }
    pts = next;
  }

  const gridPath = pts.map((p) => ({ r: Math.round(p.r), c: Math.round(p.c) }));
  return { visitedOrder: base.visitedOrder, path: gridPath, smoothPath: pts };
}

export const chomp = (g, s, e) => runTrajectoryOptimizer(g, s, e, 'chomp');
export const stomp = (g, s, e) => runTrajectoryOptimizer(g, s, e, 'stomp');
export const trajOpt = (g, s, e) => runTrajectoryOptimizer(g, s, e, 'trajopt');
export const elasticBands = (g, s, e) => runTrajectoryOptimizer(g, s, e, 'elastic');
export const teb = (g, s, e) => runTrajectoryOptimizer(g, s, e, 'teb');

function runPotentialField(grid, start, end, rotational = false) {
  const rows = grid.length;
  const cols = grid[0].length;
  const clearance = computeClearanceField(grid);
  const visitedOrder = [];
  const seen = new Set();
  const path = [{ r: start.r, c: start.c }];

  let curr = { ...start };
  const visitCount = new Map([[`${start.r},${start.c}`, 1]]);
  const maxSteps = rows * cols * 2;

  for (let step = 0; step < maxSteps; step++) {
    if (curr.r === end.r && curr.c === end.c) {
      return { visitedOrder, path };
    }
    const nbs = getNeighbors(grid, curr.r, curr.c, true);
    if (nbs.length === 0) break;

    let bestNb = null;
    let bestPot = Infinity;
    for (const nb of nbs) {
      const k = `${nb.r},${nb.c}`;
      const attr = euclidean(nb.r, nb.c, end.r, end.c);
      const clr = Math.max(0.5, clearance[nb.r][nb.c]);
      const rep = clr < 2 ? 2.5 / (clr * clr) : 0;
      const vortex = rotational && clr < 2 ? 0.35 * ((nb.r - curr.r) - (nb.c - curr.c)) : 0;
      const revisitPenalty = (visitCount.get(k) || 0) * 3.5;
      const pot = attr + rep + vortex + revisitPenalty;
      if (pot < bestPot) {
        bestPot = pot;
        bestNb = nb;
      }
    }

    if (!bestNb) break;
    curr = { r: bestNb.r, c: bestNb.c };
    const ck = `${curr.r},${curr.c}`;
    visitCount.set(ck, (visitCount.get(ck) || 0) + 1);
    if (!seen.has(ck)) {
      seen.add(ck);
      visitedOrder.push(curr);
    }
    path.push(curr);
    if ((visitCount.get(ck) || 0) > 6) break; // Local minimum guard
  }

  // Escape local minimum with A* if potential field got trapped
  const fb = runAStarCore(grid, start, end, true, 1);
  return { visitedOrder: visitedOrder.length ? visitedOrder : fb.visitedOrder, path: fb.path };
}

export const potentialField = (g, s, e) => runPotentialField(g, s, e, false);
export const apf = (g, s, e) => runPotentialField(g, s, e, true);

export function mpcPlanner(grid, start, end, allowDiagonal) {
  // Model Predictive Control: receding-horizon rollout evaluator
  const rows = grid.length;
  const cols = grid[0].length;
  const clearance = computeClearanceField(grid);
  const visitedOrder = [];
  const seen = new Set();
  const path = [{ r: start.r, c: start.c }];
  const visitCounts = new Map([[`${start.r},${start.c}`, 1]]);
  let curr = { ...start };

  for (let step = 0; step < rows * cols; step++) {
    if (curr.r === end.r && curr.c === end.c) return { visitedOrder, path };
    const nbs = getNeighbors(grid, curr.r, curr.c, allowDiagonal || true);
    if (nbs.length === 0) break;

    // 3-step lookahead rollout for each candidate control
    let bestNb = null;
    let bestScore = Infinity;
    for (const nb of nbs) {
      let simR = nb.r;
      let simC = nb.c;
      let rolloutCost = nb.cost + (visitCounts.get(`${simR},${simC}`) || 0) * 4;
      for (let h = 0; h < 2; h++) {
        const subNbs = getNeighbors(grid, simR, simC, true);
        if (subNbs.length === 0) break;
        subNbs.sort(
          (a, b) =>
            euclidean(a.r, a.c, end.r, end.c) - euclidean(b.r, b.c, end.r, end.c)
        );
        simR = subNbs[0].r;
        simC = subNbs[0].c;
        rolloutCost += subNbs[0].cost;
      }
      const terminal = euclidean(simR, simC, end.r, end.c);
      const clr = Math.max(0.5, clearance[nb.r][nb.c]);
      const score = rolloutCost + terminal + (clr < 1.5 ? 0.8 : 0);
      if (score < bestScore) {
        bestScore = score;
        bestNb = nb;
      }
    }
    if (!bestNb) break;
    curr = { r: bestNb.r, c: bestNb.c };
    const k = `${curr.r},${curr.c}`;
    visitCounts.set(k, (visitCounts.get(k) || 0) + 1);
    if (!seen.has(k)) {
      seen.add(k);
      visitedOrder.push(curr);
    }
    path.push(curr);
    if ((visitCounts.get(k) || 0) > 5) break;
  }

  const fb = runAStarCore(grid, start, end, allowDiagonal, 1);
  return { visitedOrder: visitedOrder.length ? visitedOrder : fb.visitedOrder, path: fb.path };
}
