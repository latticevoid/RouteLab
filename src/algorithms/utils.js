export class MinHeap {
  constructor(compare = (a, b) => a.f - b.f) {
    this.data = [];
    this.compare = compare;
  }

  get size() {
    return this.data.length;
  }

  push(item) {
    this.data.push(item);
    this._bubbleUp(this.data.length - 1);
  }

  pop() {
    if (this.data.length === 0) return undefined;
    const top = this.data[0];
    const last = this.data.pop();
    if (this.data.length > 0) {
      this.data[0] = last;
      this._sinkDown(0);
    }
    return top;
  }

  peek() {
    return this.data[0];
  }

  _bubbleUp(idx) {
    const item = this.data[idx];
    while (idx > 0) {
      const parentIdx = (idx - 1) >> 1;
      const parent = this.data[parentIdx];
      if (this.compare(item, parent) >= 0) break;
      this.data[idx] = parent;
      idx = parentIdx;
    }
    this.data[idx] = item;
  }

  _sinkDown(idx) {
    const len = this.data.length;
    const item = this.data[idx];
    while (true) {
      const left = (idx << 1) + 1;
      const right = left + 1;
      let smallest = idx;
      if (left < len && this.compare(this.data[left], this.data[smallest]) < 0) {
        smallest = left;
      }
      if (right < len && this.compare(this.data[right], this.data[smallest]) < 0) {
        smallest = right;
      }
      if (smallest === idx) break;
      this.data[idx] = this.data[smallest];
      idx = smallest;
    }
    this.data[idx] = item;
  }
}

export const keyOf = (r, c) => `${r},${c}`;
export const idxOf = (r, c, cols) => r * cols + c;
export const rcOf = (idx, cols) => [Math.floor(idx / cols), idx % cols];

export const DIRS_4 = [
  [-1, 0],
  [1, 0],
  [0, -1],
  [0, 1],
];

export const DIRS_8 = [
  [-1, 0],
  [1, 0],
  [0, -1],
  [0, 1],
  [-1, -1],
  [-1, 1],
  [1, -1],
  [1, 1],
];

export function inBounds(r, c, rows, cols) {
  return r >= 0 && r < rows && c >= 0 && c < cols;
}

export function isWalkable(grid, r, c) {
  return inBounds(r, c, grid.length, grid[0].length) && grid[r][c].wall !== true;
}

export function getNeighbors(grid, r, c, allowDiagonal = false) {
  const dirs = allowDiagonal ? DIRS_8 : DIRS_4;
  const rows = grid.length;
  const cols = grid[0].length;
  const result = [];
  for (let i = 0; i < dirs.length; i++) {
    const nr = r + dirs[i][0];
    const nc = c + dirs[i][1];
    if (!inBounds(nr, nc, rows, cols) || grid[nr][nc].wall) continue;
    // Prevent corner cutting on diagonals
    if (dirs[i][0] !== 0 && dirs[i][1] !== 0) {
      if (grid[r + dirs[i][0]][c].wall && grid[r][c + dirs[i][1]].wall) continue;
    }
    const baseCost = dirs[i][0] !== 0 && dirs[i][1] !== 0 ? Math.SQRT2 : 1;
    const weight = grid[nr][nc].weight || 1;
    result.push({ r: nr, c: nc, cost: baseCost * weight });
  }
  return result;
}

export function heuristic(r1, c1, r2, c2, allowDiagonal = false) {
  const dr = Math.abs(r1 - r2);
  const dc = Math.abs(c1 - c2);
  if (!allowDiagonal) return dr + dc;
  const D = 1;
  const D2 = Math.SQRT2;
  return D * (dr + dc) + (D2 - 2 * D) * Math.min(dr, dc);
}

export function euclidean(r1, c1, r2, c2) {
  return Math.hypot(r1 - r2, c1 - c2);
}

export function reconstructPath(parentMap, startIdx, endIdx, cols) {
  const path = [];
  let curr = endIdx;
  const seen = new Set();
  while (curr !== undefined && curr !== null) {
    if (seen.has(curr)) break;
    seen.add(curr);
    const [r, c] = rcOf(curr, cols);
    path.push({ r, c });
    if (curr === startIdx) break;
    curr = parentMap.get(curr);
  }
  if (path.length === 0 || (path[path.length - 1].r * cols + path[path.length - 1].c) !== startIdx) {
    return [];
  }
  path.reverse();
  return path;
}

export function lineOfSight(grid, r0, c0, r1, c1) {
  let dr = Math.abs(r1 - r0);
  let dc = Math.abs(c1 - c0);
  let sr = r0 < r1 ? 1 : -1;
  let sc = c0 < c1 ? 1 : -1;
  let r = r0;
  let c = c0;
  let err = dr - dc;

  while (true) {
    if (!isWalkable(grid, r, c)) return false;
    if (r === r1 && c === c1) return true;
    const e2 = 2 * err;
    if (e2 > -dc && e2 < dr) {
      // Supercover check for diagonal step
      if (!isWalkable(grid, r + sr, c) && !isWalkable(grid, r, c + sc)) {
        return false;
      }
    }
    if (e2 > -dc) {
      err -= dc;
      r += sr;
    }
    if (e2 < dr) {
      err += dr;
      c += sc;
    }
  }
}

export function bresenhamCells(r0, c0, r1, c1) {
  const cells = [];
  let dr = Math.abs(r1 - r0);
  let dc = Math.abs(c1 - c0);
  let sr = r0 < r1 ? 1 : -1;
  let sc = c0 < c1 ? 1 : -1;
  let r = r0;
  let c = c0;
  let err = dr - dc;
  while (true) {
    cells.push({ r, c });
    if (r === r1 && c === c1) break;
    const e2 = 2 * err;
    if (e2 > -dc) {
      err -= dc;
      r += sr;
    }
    if (e2 < dr) {
      err += dr;
      c += sc;
    }
  }
  return cells;
}

export function computeClearanceField(grid) {
  const rows = grid.length;
  const cols = grid[0].length;
  const dist = Array.from({ length: rows }, () => new Float32Array(cols).fill(Infinity));
  const queue = [];
  for (let r = 0; r < rows; r++) {
    for (let c = 0; c < cols; c++) {
      if (grid[r][c].wall) {
        dist[r][c] = 0;
        queue.push([r, c]);
      }
    }
  }
  let head = 0;
  while (head < queue.length) {
    const [r, c] = queue[head++];
    for (const [dr, dc] of DIRS_4) {
      const nr = r + dr;
      const nc = c + dc;
      if (inBounds(nr, nc, rows, cols) && dist[nr][nc] > dist[r][c] + 1) {
        dist[nr][nc] = dist[r][c] + 1;
        queue.push([nr, nc]);
      }
    }
  }
  return dist;
}

export function computePathCost(grid, path) {
  if (!path || path.length < 2) return 0;
  let cost = 0;
  for (let i = 1; i < path.length; i++) {
    const prev = path[i - 1];
    const curr = path[i];
    const step = Math.hypot(curr.r - prev.r, curr.c - prev.c);
    const w = grid[curr.r]?.[curr.c]?.weight || 1;
    cost += step * w;
  }
  return Number(cost.toFixed(2));
}

/**
 * Fallback A* helper used inside hybrid/anytime/multi-agent/trajectory algorithms.
 */
export function runAStarCore(grid, start, end, allowDiagonal = false, weightMultiplier = 1) {
  const rows = grid.length;
  const cols = grid[0].length;
  const startIdx = idxOf(start.r, start.c, cols);
  const endIdx = idxOf(end.r, end.c, cols);
  const gScore = new Float64Array(rows * cols).fill(Infinity);
  const parent = new Map();
  const closed = new Uint8Array(rows * cols);
  const visitedOrder = [];

  gScore[startIdx] = 0;
  const pq = new MinHeap((a, b) => a.f - b.f || a.h - b.h);
  const h0 = heuristic(start.r, start.c, end.r, end.c, allowDiagonal);
  pq.push({ idx: startIdx, f: weightMultiplier * h0, h: h0 });

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
      const tg = gScore[idx] + nb.cost;
      if (tg < gScore[nIdx]) {
        gScore[nIdx] = tg;
        parent.set(nIdx, idx);
        const h = heuristic(nb.r, nb.c, end.r, end.c, allowDiagonal);
        pq.push({ idx: nIdx, f: tg + weightMultiplier * h, h });
      }
    }
  }
  return { visitedOrder, path: [] };
}

export function seededRandom(seed = 1337) {
  let s = seed >>> 0;
  return () => {
    s = (s * 1664525 + 1013904223) >>> 0;
    return s / 4294967296;
  };
}
