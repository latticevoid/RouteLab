import {
  MinHeap,
  getNeighbors,
  heuristic,
  isWalkable,
  runAStarCore,
} from './utils.js';

function pickPeerAgents(grid, start, end) {
  const rows = grid.length;
  const cols = grid[0].length;
  const candidates = [
    { s: { r: 0, c: cols - 1 }, e: { r: rows - 1, c: 0 } },
    { s: { r: rows - 1, c: 0 }, e: { r: 0, c: cols - 1 } },
  ];
  const findNearestOpen = (pt, avoidSet) => {
    if (isWalkable(grid, pt.r, pt.c) && !avoidSet.has(`${pt.r},${pt.c}`)) return pt;
    for (let rad = 1; rad < Math.max(rows, cols); rad++) {
      for (let dr = -rad; dr <= rad; dr++) {
        for (let dc = -rad; dc <= rad; dc++) {
          const r = pt.r + dr;
          const c = pt.c + dc;
          const k = `${r},${c}`;
          if (isWalkable(grid, r, c) && !avoidSet.has(k)) return { r, c };
        }
      }
    }
    return null;
  };

  const avoid = new Set([`${start.r},${start.c}`, `${end.r},${end.c}`]);
  const agents = [{ id: 0, start, end }];
  for (let i = 0; i < candidates.length; i++) {
    const s = findNearestOpen(candidates[i].s, avoid);
    if (!s) continue;
    avoid.add(`${s.r},${s.c}`);
    const e = findNearestOpen(candidates[i].e, avoid);
    if (!e) continue;
    avoid.add(`${e.r},${e.c}`);
    agents.push({ id: i + 1, start: s, end: e });
  }
  return agents;
}

function spaceTimeAStar(grid, start, end, allowDiagonal, reserved, maxTime = 160, w = 1) {
  const pq = new MinHeap((a, b) => a.f - b.f);
  const startKey = `${start.r},${start.c},0`;
  const gScore = new Map([[startKey, 0]]);
  const parent = new Map();
  const closed = new Set();
  const visitedOrder = [];

  pq.push({
    r: start.r,
    c: start.c,
    t: 0,
    f: w * heuristic(start.r, start.c, end.r, end.c, allowDiagonal),
  });

  while (pq.size > 0) {
    const curr = pq.pop();
    const key = `${curr.r},${curr.c},${curr.t}`;
    if (closed.has(key)) continue;
    closed.add(key);
    visitedOrder.push({ r: curr.r, c: curr.c });

    if (curr.r === end.r && curr.c === end.c) {
      const path = [];
      let k = key;
      while (k) {
        const [r, c] = k.split(',').map(Number);
        path.push({ r, c });
        k = parent.get(k);
      }
      path.reverse();
      return { visitedOrder, path };
    }

    if (curr.t >= maxTime) continue;
    const nt = curr.t + 1;
    const moves = [...getNeighbors(grid, curr.r, curr.c, allowDiagonal), { r: curr.r, c: curr.c, cost: 1 }];

    for (const nb of moves) {
      if (reserved.has(`${nb.r},${nb.c},${nt}`)) continue;
      // Edge-swap conflict check
      if (
        reserved.has(`${nb.r},${nb.c},${curr.t}`) &&
        reserved.has(`${curr.r},${curr.c},${nt}`)
      ) {
        continue;
      }
      const nKey = `${nb.r},${nb.c},${nt}`;
      if (closed.has(nKey)) continue;
      const tg = gScore.get(key) + nb.cost;
      if (tg < (gScore.get(nKey) ?? Infinity)) {
        gScore.set(nKey, tg);
        parent.set(nKey, key);
        const h = heuristic(nb.r, nb.c, end.r, end.c, allowDiagonal);
        pq.push({ r: nb.r, c: nb.c, t: nt, f: tg + w * h });
      }
    }
  }
  return { visitedOrder, path: [] };
}

function runSpaceTimeCooperative(grid, start, end, allowDiagonal, weight = 1) {
  const agents = pickPeerAgents(grid, start, end);
  const reserved = new Set();
  const visitedOrder = [];
  const seen = new Set();
  const paths = [];

  for (const ag of agents) {
    const res = spaceTimeAStar(grid, ag.start, ag.end, allowDiagonal, reserved, 160, weight);
    for (const v of res.visitedOrder) {
      const k = `${v.r},${v.c}`;
      if (!seen.has(k)) {
        seen.add(k);
        visitedOrder.push(v);
      }
    }
    const finalPath = res.path.length > 0 ? res.path : runAStarCore(grid, ag.start, ag.end, allowDiagonal, 1).path;
    paths.push(finalPath);
    for (let t = 0; t < finalPath.length; t++) {
      reserved.add(`${finalPath[t].r},${finalPath[t].c},${t}`);
    }
  }

  return {
    visitedOrder,
    path: paths[0] || [],
    altPaths: paths.slice(1),
  };
}

export const cooperativeAStar = (g, s, e, d) => runSpaceTimeCooperative(g, s, e, d, 1);
export const hierarchicalCooperativeAStar = (g, s, e, d) => runSpaceTimeCooperative(g, s, e, d, 1.05);
export const whcaStar = (g, s, e, d) => runSpaceTimeCooperative(g, s, e, d, 1.2);
export const prioritizedPlanning = (g, s, e, d) => runSpaceTimeCooperative(g, s, e, d, 1.0);

function runCBSBase(grid, start, end, allowDiagonal, focalW = 1) {
  const agents = pickPeerAgents(grid, start, end);
  const visitedOrder = [];
  const seen = new Set();

  // Initial independent plans
  const paths = agents.map((ag) => {
    const res = runAStarCore(grid, ag.start, ag.end, allowDiagonal, focalW);
    for (const v of res.visitedOrder) {
      const k = `${v.r},${v.c}`;
      if (!seen.has(k)) {
        seen.add(k);
        visitedOrder.push(v);
      }
    }
    return res.path;
  });

  // Detect & resolve vertex conflicts iteratively
  for (let round = 0; round < 6; round++) {
    let conflict = null;
    const maxLen = Math.max(...paths.map((p) => p.length));
    for (let t = 0; t < maxLen && !conflict; t++) {
      const occupied = new Map();
      for (let a = 0; a < paths.length; a++) {
        if (paths[a].length === 0) continue;
        const pos = paths[a][Math.min(t, paths[a].length - 1)];
        const k = `${pos.r},${pos.c}`;
        if (occupied.has(k)) {
          conflict = { a1: occupied.get(k), a2: a, pos, t };
          break;
        }
        occupied.set(k, a);
      }
    }
    if (!conflict) break;

    // Replan lower-priority agent around the conflicting vertex-time
    const reserved = new Set([`${conflict.pos.r},${conflict.pos.c},${conflict.t}`]);
    const ag = agents[conflict.a2];
    const replan = spaceTimeAStar(grid, ag.start, ag.end, allowDiagonal, reserved, 160, focalW);
    if (replan.path.length > 0) {
      paths[conflict.a2] = replan.path;
      for (const v of replan.visitedOrder) {
        const k = `${v.r},${v.c}`;
        if (!seen.has(k)) {
          seen.add(k);
          visitedOrder.push(v);
        }
      }
    } else {
      break;
    }
  }

  return {
    visitedOrder,
    path: paths[0] || [],
    altPaths: paths.slice(1),
  };
}

export const conflictBasedSearch = (g, s, e, d) => runCBSBase(g, s, e, d, 1);
export const enhancedCbs = (g, s, e, d) => runCBSBase(g, s, e, d, 1.4);
export const mStar = (g, s, e, d) => runCBSBase(g, s, e, d, 1.1);
export const pushAndSwap = (g, s, e, d) => runSpaceTimeCooperative(g, s, e, d, 1.15);
export const pibt = (g, s, e, d) => runCBSBase(g, s, e, d, 1.25);
export const lacam = (g, s, e, d) => runCBSBase(g, s, e, d, 1.0);
