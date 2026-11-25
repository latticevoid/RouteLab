import * as classical from './classical.js';
import * as astarVariants from './astarVariants.js';
import * as gridBased from './gridBased.js';
import * as dynamicRealtime from './dynamicRealtime.js';
import * as robotics from './robotics.js';
import * as graphOptimization from './graphOptimization.js';
import * as natureInspired from './natureInspired.js';
import * as multiAgent from './multiAgent.js';
import * as networkRouting from './networkRouting.js';
import { ALGORITHMS } from './registry.js';
import { computePathCost } from './utils.js';

const SOLVERS = {
  ...classical,
  ...astarVariants,
  ...gridBased,
  ...dynamicRealtime,
  ...robotics,
  ...graphOptimization,
  ...natureInspired,
  ...multiAgent,
  ...networkRouting,
};

export { ALGORITHMS, CATEGORIES } from './registry.js';

export function runAlgorithm(algoId, grid, start, end, allowDiagonal = false) {
  const meta = ALGORITHMS.find((a) => a.id === algoId) || ALGORITHMS[0];
  const solverFn = SOLVERS[meta.solver] || SOLVERS.aStar;
  const t0 = performance.now();
  const res = solverFn(grid, start, end, allowDiagonal);
  const elapsedMs = Number((performance.now() - t0).toFixed(2));
  const path = res.path || [];
  return {
    visitedOrder: res.visitedOrder || [],
    path,
    altPaths: res.altPaths || [],
    treeEdges: res.treeEdges || [],
    smoothPath: res.smoothPath || null,
    cost: computePathCost(grid, path),
    elapsedMs,
    meta,
  };
}
