import {
  getNeighbors,
  heuristic,
  idxOf,
  runAStarCore,
  seededRandom,
} from './utils.js';

function guidedWalk(grid, start, end, allowDiagonal, scoreFn, rand, maxSteps) {
  const path = [{ r: start.r, c: start.c }];
  const visited = new Set([`${start.r},${start.c}`]);
  let curr = { ...start };

  for (let s = 0; s < maxSteps; s++) {
    if (curr.r === end.r && curr.c === end.c) return { path, reached: true };
    const nbs = getNeighbors(grid, curr.r, curr.c, allowDiagonal);
    if (nbs.length === 0) break;

    const unvisited = nbs.filter((nb) => !visited.has(`${nb.r},${nb.c}`));
    const pool = unvisited.length > 0 ? unvisited : nbs;

    let totalWeight = 0;
    const weights = pool.map((nb) => {
      const w = Math.max(1e-4, scoreFn(curr, nb));
      totalWeight += w;
      return w;
    });

    let pick = rand() * totalWeight;
    let chosen = pool[pool.length - 1];
    for (let i = 0; i < pool.length; i++) {
      pick -= weights[i];
      if (pick <= 0) {
        chosen = pool[i];
        break;
      }
    }
    curr = { r: chosen.r, c: chosen.c };
    visited.add(`${curr.r},${curr.c}`);
    path.push(curr);
  }
  return { path, reached: curr.r === end.r && curr.c === end.c };
}

function shortenPath(path) {
  // Remove loops from stochastic walk
  const idxMap = new Map();
  const out = [];
  for (const p of path) {
    const k = `${p.r},${p.c}`;
    if (idxMap.has(k)) {
      const prevIdx = idxMap.get(k);
      while (out.length - 1 > prevIdx) {
        const removed = out.pop();
        idxMap.delete(`${removed.r},${removed.c}`);
      }
    } else {
      idxMap.set(k, out.length);
      out.push(p);
    }
  }
  return out;
}

export function antColonyOptimization(grid, start, end, allowDiagonal) {
  const rows = grid.length;
  const cols = grid[0].length;
  const total = rows * cols;
  const pheromone = new Float64Array(total).fill(1.0);
  const rand = seededRandom(75);
  const visitedOrder = [];
  const seen = new Set();
  const maxSteps = rows * cols * 2;

  let bestPath = [];
  let bestLen = Infinity;

  for (let iter = 0; iter < 16; iter++) {
    const iterPaths = [];
    for (let ant = 0; ant < 10; ant++) {
      const res = guidedWalk(
        grid,
        start,
        end,
        allowDiagonal,
        (_, nb) => {
          const tau = pheromone[idxOf(nb.r, nb.c, cols)];
          const eta = 1 / (1 + heuristic(nb.r, nb.c, end.r, end.c, allowDiagonal));
          return tau * Math.pow(eta, 2.2);
        },
        rand,
        maxSteps
      );
      for (const p of res.path) {
        const k = `${p.r},${p.c}`;
        if (!seen.has(k)) {
          seen.add(k);
          visitedOrder.push(p);
        }
      }
      if (res.reached) {
        const clean = shortenPath(res.path);
        iterPaths.push(clean);
        if (clean.length < bestLen) {
          bestLen = clean.length;
          bestPath = clean;
        }
      }
    }
    // Evaporation
    for (let i = 0; i < total; i++) pheromone[i] = Math.max(0.2, pheromone[i] * 0.72);
    // Deposit
    for (const p of iterPaths) {
      const deposit = 15 / Math.max(1, p.length);
      for (const cell of p) {
        pheromone[idxOf(cell.r, cell.c, cols)] += deposit;
      }
    }
  }

  if (bestPath.length === 0) {
    const fb = runAStarCore(grid, start, end, allowDiagonal, 1);
    return { visitedOrder: visitedOrder.length ? visitedOrder : fb.visitedOrder, path: fb.path };
  }
  return { visitedOrder, path: bestPath };
}

function runSwarmMetaheuristic(grid, start, end, allowDiagonal, seedOffset, popSize = 14, generations = 14) {
  // Population-based metaheuristic (GA, PSO, ABC, GWO, Firefly, Cuckoo, WOA, Bat, DE):
  // Maintains a cell-attraction potential vector evolved over generations toward the best-found route.
  const rows = grid.length;
  const cols = grid[0].length;
  const total = rows * cols;
  const rand = seededRandom(seedOffset + rows * 13);
  const visitedOrder = [];
  const seen = new Set();
  const maxSteps = rows * cols * 2;

  const weights = Array.from({ length: popSize }, () => {
    const arr = new Float32Array(total);
    for (let i = 0; i < total; i++) arr[i] = 0.5 + rand();
    return arr;
  });

  let gBestPath = [];
  let gBestFit = Infinity;
  let gBestVec = weights[0];

  for (let gen = 0; gen < generations; gen++) {
    for (let p = 0; p < popSize; p++) {
      const wVec = weights[p];
      const res = guidedWalk(
        grid,
        start,
        end,
        allowDiagonal,
        (_, nb) => {
          const h = heuristic(nb.r, nb.c, end.r, end.c, allowDiagonal);
          return (wVec[idxOf(nb.r, nb.c, cols)] + 0.3) / Math.pow(1 + h, 1.8);
        },
        rand,
        maxSteps
      );
      for (const cell of res.path) {
        const k = `${cell.r},${cell.c}`;
        if (!seen.has(k)) {
          seen.add(k);
          visitedOrder.push(cell);
        }
      }
      if (res.reached) {
        const clean = shortenPath(res.path);
        if (clean.length < gBestFit) {
          gBestFit = clean.length;
          gBestPath = clean;
          gBestVec = wVec.slice();
        }
      }
    }

    // Evolve population toward gBestVec + exploration perturbation
    for (let p = 0; p < popSize; p++) {
      const wVec = weights[p];
      for (let i = 0; i < total; i++) {
        wVec[i] = Math.max(0.1, 0.55 * wVec[i] + 0.45 * gBestVec[i] + (rand() - 0.5) * 0.25);
      }
      if (gBestPath.length > 0) {
        for (const cell of gBestPath) {
          wVec[idxOf(cell.r, cell.c, cols)] += 0.6;
        }
      }
    }
  }

  if (gBestPath.length === 0) {
    const fb = runAStarCore(grid, start, end, allowDiagonal, 1);
    return { visitedOrder: visitedOrder.length ? visitedOrder : fb.visitedOrder, path: fb.path };
  }
  return { visitedOrder, path: gBestPath };
}

export const geneticAlgorithm = (g, s, e, d) => runSwarmMetaheuristic(g, s, e, d, 74, 14, 12);
export const particleSwarmOptimization = (g, s, e, d) => runSwarmMetaheuristic(g, s, e, d, 76, 14, 14);
export const artificialBeeColony = (g, s, e, d) => runSwarmMetaheuristic(g, s, e, d, 77, 12, 14);
export const greyWolfOptimizer = (g, s, e, d) => runSwarmMetaheuristic(g, s, e, d, 78, 12, 14);
export const fireflyAlgorithm = (g, s, e, d) => runSwarmMetaheuristic(g, s, e, d, 79, 12, 12);
export const cuckooSearch = (g, s, e, d) => runSwarmMetaheuristic(g, s, e, d, 80, 12, 12);
export const whaleOptimization = (g, s, e, d) => runSwarmMetaheuristic(g, s, e, d, 81, 12, 12);
export const batAlgorithm = (g, s, e, d) => runSwarmMetaheuristic(g, s, e, d, 82, 12, 12);
export const differentialEvolution = (g, s, e, d) => runSwarmMetaheuristic(g, s, e, d, 85, 14, 12);

export function simulatedAnnealing(grid, start, end, allowDiagonal) {
  const rows = grid.length;
  const cols = grid[0].length;
  const rand = seededRandom(83);
  const visitedOrder = [];
  const seen = new Set();

  let curr = { ...start };
  const path = [{ r: curr.r, c: curr.c }];
  let temp = 4.0;
  const cooling = 0.985;
  const maxSteps = rows * cols * 4;

  for (let step = 0; step < maxSteps; step++) {
    if (curr.r === end.r && curr.c === end.c) {
      return { visitedOrder, path: shortenPath(path) };
    }
    const nbs = getNeighbors(grid, curr.r, curr.c, allowDiagonal);
    if (nbs.length === 0) break;
    const cand = nbs[Math.floor(rand() * nbs.length)];
    const curH = heuristic(curr.r, curr.c, end.r, end.c, allowDiagonal);
    const nxtH = heuristic(cand.r, cand.c, end.r, end.c, allowDiagonal);
    const delta = nxtH - curH;

    if (delta <= 0 || rand() < Math.exp(-delta / Math.max(0.05, temp))) {
      curr = { r: cand.r, c: cand.c };
      const k = `${curr.r},${curr.c}`;
      if (!seen.has(k)) {
        seen.add(k);
        visitedOrder.push(curr);
      }
      path.push(curr);
    }
    temp *= cooling;
  }

  const fb = runAStarCore(grid, start, end, allowDiagonal, 1);
  return { visitedOrder: visitedOrder.length ? visitedOrder : fb.visitedOrder, path: fb.path };
}

export function tabuSearch(grid, start, end, allowDiagonal) {
  const rows = grid.length;
  const cols = grid[0].length;
  const visitedOrder = [];
  const seen = new Set();
  const path = [{ r: start.r, c: start.c }];
  const tabu = [];
  const TABU_TENURE = 18;
  let curr = { ...start };

  for (let step = 0; step < rows * cols * 3; step++) {
    if (curr.r === end.r && curr.c === end.c) {
      return { visitedOrder, path: shortenPath(path) };
    }
    const nbs = getNeighbors(grid, curr.r, curr.c, allowDiagonal);
    if (nbs.length === 0) break;
    const tabuSet = new Set(tabu);
    let nonTabu = nbs.filter((nb) => !tabuSet.has(`${nb.r},${nb.c}`));
    if (nonTabu.length === 0) nonTabu = nbs;

    nonTabu.sort(
      (a, b) =>
        heuristic(a.r, a.c, end.r, end.c, allowDiagonal) -
        heuristic(b.r, b.c, end.r, end.c, allowDiagonal)
    );
    const chosen = nonTabu[0];
    const k = `${chosen.r},${chosen.c}`;
    tabu.push(k);
    if (tabu.length > TABU_TENURE) tabu.shift();

    curr = { r: chosen.r, c: chosen.c };
    if (!seen.has(k)) {
      seen.add(k);
      visitedOrder.push(curr);
    }
    path.push(curr);
  }

  const fb = runAStarCore(grid, start, end, allowDiagonal, 1);
  return { visitedOrder: visitedOrder.length ? visitedOrder : fb.visitedOrder, path: fb.path };
}
