export function createEmptyGrid(rows, cols) {
  return Array.from({ length: rows }, () =>
    Array.from({ length: cols }, () => ({ wall: false, weight: 1 }))
  );
}

function clearStartEnd(grid, start, end) {
  const clearAround = (pt) => {
    grid[pt.r][pt.c].wall = false;
    grid[pt.r][pt.c].weight = 1;
    for (const [dr, dc] of [[-1, 0], [1, 0], [0, -1], [0, 1]]) {
      const nr = pt.r + dr;
      const nc = pt.c + dc;
      if (nr >= 0 && nr < grid.length && nc >= 0 && nc < grid[0].length) {
        // Ensure at least one neighbor is open
        if (nr === pt.r && Math.abs(nc - pt.c) === 1) {
          // Leave carving intact if already open; only ensure not completely boxed in
        }
      }
    }
  };
  clearAround(start);
  clearAround(end);

  const ensureExit = (pt) => {
    const nbs = [[-1, 0], [1, 0], [0, -1], [0, 1]]
      .map(([dr, dc]) => [pt.r + dr, pt.c + dc])
      .filter(([r, c]) => r >= 0 && r < grid.length && c >= 0 && c < grid[0].length);
    if (nbs.length > 0 && nbs.every(([r, c]) => grid[r][c].wall)) {
      const [r, c] = nbs[0];
      grid[r][c].wall = false;
    }
  };
  ensureExit(start);
  ensureExit(end);
}

export function generateRecursiveBacktrackerMaze(rows, cols, start, end) {
  const grid = Array.from({ length: rows }, () =>
    Array.from({ length: cols }, () => ({ wall: true, weight: 1 }))
  );

  const startR = start.r % 2 === 0 ? start.r : Math.max(0, start.r - 1);
  const startC = start.c % 2 === 0 ? start.c : Math.max(0, start.c - 1);
  const stack = [[startR, startC]];
  grid[startR][startC].wall = false;

  const dirs = [
    [-2, 0],
    [2, 0],
    [0, -2],
    [0, 2],
  ];

  while (stack.length > 0) {
    const [r, c] = stack[stack.length - 1];
    const shuffled = [...dirs].sort(() => Math.random() - 0.5);
    let carved = false;
    for (const [dr, dc] of shuffled) {
      const nr = r + dr;
      const nc = c + dc;
      if (nr >= 0 && nr < rows && nc >= 0 && nc < cols && grid[nr][nc].wall) {
        grid[r + dr / 2][c + dc / 2].wall = false;
        grid[nr][nc].wall = false;
        stack.push([nr, nc]);
        carved = true;
        break;
      }
    }
    if (!carved) stack.pop();
  }

  clearStartEnd(grid, start, end);
  return grid;
}

export function generateRecursiveDivisionMaze(rows, cols, start, end) {
  const grid = createEmptyGrid(rows, cols);

  // Outer border walls? Keep open or subtle; classic recursive division divides interior
  const divide = (rStart, rEnd, cStart, cEnd) => {
    const height = rEnd - rStart + 1;
    const width = cEnd - cStart + 1;
    if (height < 3 || width < 3) return;

    const horizontal = height > width ? true : width > height ? false : Math.random() < 0.5;

    if (horizontal) {
      const possibleRows = [];
      for (let r = rStart + 1; r <= rEnd - 1; r += 2) possibleRows.push(r);
      if (possibleRows.length === 0) return;
      const wallRow = possibleRows[Math.floor(Math.random() * possibleRows.length)];
      const passageCol = cStart + Math.floor(Math.random() * Math.ceil(width / 2)) * 2;

      for (let c = cStart; c <= cEnd; c++) {
        if (c !== passageCol) grid[wallRow][c].wall = true;
      }
      divide(rStart, wallRow - 1, cStart, cEnd);
      divide(wallRow + 1, rEnd, cStart, cEnd);
    } else {
      const possibleCols = [];
      for (let c = cStart + 1; c <= cEnd - 1; c += 2) possibleCols.push(c);
      if (possibleCols.length === 0) return;
      const wallCol = possibleCols[Math.floor(Math.random() * possibleCols.length)];
      const passageRow = rStart + Math.floor(Math.random() * Math.ceil(height / 2)) * 2;

      for (let r = rStart; r <= rEnd; r++) {
        if (r !== passageRow) grid[r][wallCol].wall = true;
      }
      divide(rStart, rEnd, cStart, wallCol - 1);
      divide(rStart, rEnd, wallCol + 1, cEnd);
    }
  };

  divide(0, rows - 1, 0, cols - 1);
  clearStartEnd(grid, start, end);
  return grid;
}

export function generatePrimsMaze(rows, cols, start, end) {
  const grid = Array.from({ length: rows }, () =>
    Array.from({ length: cols }, () => ({ wall: true, weight: 1 }))
  );
  const sr = start.r % 2 === 0 ? start.r : Math.max(0, start.r - 1);
  const sc = start.c % 2 === 0 ? start.c : Math.max(0, start.c - 1);
  grid[sr][sc].wall = false;

  const frontier = [];
  const addFrontier = (r, c) => {
    for (const [dr, dc] of [[-2, 0], [2, 0], [0, -2], [0, 2]]) {
      const nr = r + dr;
      const nc = c + dc;
      if (nr >= 0 && nr < rows && nc >= 0 && nc < cols && grid[nr][nc].wall) {
        frontier.push([nr, nc, r + dr / 2, c + dc / 2]);
      }
    }
  };
  addFrontier(sr, sc);

  while (frontier.length > 0) {
    const idx = Math.floor(Math.random() * frontier.length);
    const [r, c, wr, wc] = frontier.splice(idx, 1)[0];
    if (grid[r][c].wall) {
      grid[wr][wc].wall = false;
      grid[r][c].wall = false;
      addFrontier(r, c);
    }
  }
  clearStartEnd(grid, start, end);
  return grid;
}

export function generateRandomScatter(rows, cols, start, end, density = 0.28) {
  const grid = createEmptyGrid(rows, cols);
  for (let r = 0; r < rows; r++) {
    for (let c = 0; c < cols; c++) {
      if ((r === start.r && c === start.c) || (r === end.r && c === end.c)) continue;
      if (Math.random() < density) grid[r][c].wall = true;
    }
  }
  return grid;
}

export function generateWeightedTerrain(rows, cols, start, end) {
  const grid = createEmptyGrid(rows, cols);
  for (let r = 0; r < rows; r++) {
    for (let c = 0; c < cols; c++) {
      if ((r === start.r && c === start.c) || (r === end.r && c === end.c)) continue;
      const roll = Math.random();
      if (roll < 0.14) {
        grid[r][c].wall = true;
      } else if (roll < 0.48) {
        grid[r][c].weight = 2 + Math.floor(Math.random() * 6); // weights 2..7
      }
    }
  }
  return grid;
}

export const MAZE_PRESETS = [
  { id: 'backtracker', label: 'Recursive Backtracker', fn: generateRecursiveBacktrackerMaze },
  { id: 'division', label: 'Recursive Division', fn: generateRecursiveDivisionMaze },
  { id: 'prims', label: "Randomized Prim's", fn: generatePrimsMaze },
  { id: 'scatter', label: 'Random Obstacles', fn: generateRandomScatter },
  { id: 'weights', label: 'Weighted Terrain', fn: generateWeightedTerrain },
];
