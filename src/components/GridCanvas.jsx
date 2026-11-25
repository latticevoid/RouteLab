import { useEffect, useRef } from 'react';

export default function GridCanvas({
  grid,
  start,
  end,
  visitedSet,
  pathCells,
  altPaths,
  treeEdges,
  smoothPath,
  activeTool,
  darkMode,
  onCellInteract,
  onMoveEndpoint,
}) {
  const containerRef = useRef(null);
  const canvasRef = useRef(null);
  const draggingRef = useRef(null); // 'start' | 'end' | 'draw' | null
  const lastCellRef = useRef(null);
  const pathAppearRef = useRef(new Map());
  const rafRef = useRef(null);

  const rows = grid.length;
  const cols = grid[0].length;

  useEffect(() => {
    const now = performance.now();
    const map = pathAppearRef.current;
    if (!pathCells || pathCells.length === 0) {
      map.clear();
      return;
    }
    const activeKeys = new Set();
    for (const pt of pathCells) {
      const k = `${pt.r},${pt.c}`;
      activeKeys.add(k);
      if (!map.has(k)) map.set(k, now);
    }
    for (const k of map.keys()) {
      if (!activeKeys.has(k)) map.delete(k);
    }
  }, [pathCells]);

  useEffect(() => {
    const container = containerRef.current;
    const canvas = canvasRef.current;
    if (!container || !canvas) return;

    const draw = () => {
      const rect = container.getBoundingClientRect();
      if (rect.width === 0 || rect.height === 0) return;

      const dpr = window.devicePixelRatio || 1;
      const cellSize = Math.floor(Math.min(rect.width / cols, rect.height / rows));
      const drawW = Math.max(1, cellSize * cols);
      const drawH = Math.max(1, cellSize * rows);

      canvas.style.width = `${drawW}px`;
      canvas.style.height = `${drawH}px`;
      canvas.width = Math.floor(drawW * dpr);
      canvas.height = Math.floor(drawH * dpr);

      const ctx = canvas.getContext('2d');
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);

      // Tactile drafting-board palette (no neon AI-slop glow)
      const bg = darkMode ? '#0e0e11' : '#ffffff';
      const gridLine = darkMode ? 'rgba(255, 255, 255, 0.06)' : 'rgba(24, 24, 27, 0.13)';
      const wallFill = darkMode ? '#3f3f46' : '#18181b';
      const visitedFill = darkMode ? 'rgba(56, 189, 248, 0.34)' : 'rgba(37, 99, 235, 0.32)';
      const pathFill = darkMode ? '#fbbf24' : '#d97706';

      ctx.fillStyle = bg;
      ctx.fillRect(0, 0, drawW, drawH);

      const drawCellBox = (r, c, color, pad = 1.5, glow = null, scale = 1) => {
        const baseSize = Math.max(2, cellSize - pad * 2);
        const size = Math.max(2, baseSize * scale);
        const offset = (cellSize - size) / 2;
        const x = c * cellSize + offset;
        const y = r * cellSize + offset;
        const rad = Math.min(5, size * 0.22);
        ctx.save();
        if (glow) {
          ctx.shadowColor = glow;
          ctx.shadowBlur = Math.max(6, cellSize * 0.45);
        }
        ctx.fillStyle = color;
        ctx.beginPath();
        ctx.roundRect(x, y, size, size, rad);
        ctx.fill();
        ctx.restore();
      };

      // Grid lines first
      ctx.strokeStyle = gridLine;
      ctx.lineWidth = 0.75;
      ctx.beginPath();
      for (let r = 0; r <= rows; r++) {
        ctx.moveTo(0, r * cellSize);
        ctx.lineTo(drawW, r * cellSize);
      }
      for (let c = 0; c <= cols; c++) {
        ctx.moveTo(c * cellSize, 0);
        ctx.lineTo(c * cellSize, drawH);
      }
      ctx.stroke();

      // Draw cells (weights, visited, walls)
      const cellPad = Math.max(1, cellSize * 0.08);
      for (let r = 0; r < rows; r++) {
        for (let c = 0; c < cols; c++) {
          const x = c * cellSize;
          const y = r * cellSize;
          const cell = grid[r][c];
          const k = `${r},${c}`;

          if (cell.wall) {
            drawCellBox(r, c, wallFill, cellPad);
            continue;
          }

          if (cell.weight > 1) {
            const alpha = Math.min(0.48, 0.14 + cell.weight * 0.05);
            drawCellBox(
              r,
              c,
              darkMode ? `rgba(245, 158, 11, ${alpha})` : `rgba(217, 119, 6, ${alpha})`,
              cellPad
            );
          }

          const visitOrder = visitedSet.get(k);
          if (visitOrder !== undefined) {
            drawCellBox(r, c, visitedFill, cellPad);
          }

          if (cell.weight > 1 && cellSize >= 14) {
            ctx.fillStyle = darkMode ? '#fcd34d' : '#92400e';
            ctx.font = `600 ${Math.max(8, Math.floor(cellSize * 0.4))}px ui-monospace, monospace`;
            ctx.textAlign = 'center';
            ctx.textBaseline = 'middle';
            ctx.fillText(String(cell.weight), x + cellSize / 2, y + cellSize / 2);
          }
        }
      }

      // Tree / Roadmap edges (for RRT, PRM, FMT*, etc.)
      if (treeEdges && treeEdges.length > 0) {
        ctx.strokeStyle = darkMode ? 'rgba(167, 139, 250, 0.45)' : 'rgba(109, 40, 217, 0.35)';
        ctx.lineWidth = 1.2;
        ctx.beginPath();
        for (const [a, b] of treeEdges) {
          ctx.moveTo((a.c + 0.5) * cellSize, (a.r + 0.5) * cellSize);
          ctx.lineTo((b.c + 0.5) * cellSize, (b.r + 0.5) * cellSize);
        }
        ctx.stroke();
      }

      // Alternative / Multi-Agent paths
      if (altPaths && altPaths.length > 0) {
        const altColors = ['#a855f7', '#ec4899', '#14b8a6'];
        altPaths.forEach((ap, idx) => {
          if (!ap || ap.length === 0) return;
          ctx.strokeStyle = altColors[idx % altColors.length];
          ctx.lineWidth = Math.max(2, cellSize * 0.22);
          ctx.lineCap = 'round';
          ctx.lineJoin = 'round';
          ctx.beginPath();
          ap.forEach((pt, i) => {
            const px = (pt.c + 0.5) * cellSize;
            const py = (pt.r + 0.5) * cellSize;
            if (i === 0) ctx.moveTo(px, py);
            else ctx.lineTo(px, py);
          });
          ctx.stroke();
        });
      }

      // Primary Path Cells (grow + glow)
      let needsMoreFrames = false;
      const now = performance.now();
      if (pathCells && pathCells.length > 0) {
        const lastIdx = pathCells.length - 1;
        const head = pathCells[lastIdx];
        const reachedEnd = head.r === end.r && head.c === end.c;

        for (let i = 0; i < pathCells.length; i++) {
          const pt = pathCells[i];
          const isHead = !reachedEnd && i === lastIdx;
          const born = pathAppearRef.current.get(`${pt.r},${pt.c}`) ?? now;
          const age = now - born;
          let scale = 1;
          if (age < 180) {
            needsMoreFrames = true;
            const t = age / 180;
            scale = t < 0.7 ? 0.45 + (t / 0.7) * 0.65 : 1.1 - ((t - 0.7) / 0.3) * 0.1;
          }
          drawCellBox(
            pt.r,
            pt.c,
            isHead ? '#fef08a' : pathFill,
            cellPad,
            'rgba(251, 191, 36, 0.75)',
            scale
          );
        }

        if (smoothPath && smoothPath.length > 1) {
          ctx.save();
          ctx.shadowColor = 'rgba(251, 191, 36, 0.7)';
          ctx.shadowBlur = Math.max(6, cellSize * 0.4);
          ctx.strokeStyle = darkMode ? '#fde047' : '#b45309';
          ctx.lineWidth = Math.max(2, cellSize * 0.2);
          ctx.lineCap = 'round';
          ctx.lineJoin = 'round';
          ctx.beginPath();
          smoothPath.forEach((pt, i) => {
            const px = (pt.c + 0.5) * cellSize;
            const py = (pt.r + 0.5) * cellSize;
            if (i === 0) ctx.moveTo(px, py);
            else ctx.lineTo(px, py);
          });
          ctx.stroke();
          ctx.restore();
        }
      }

      // Origin (Glowing Emerald) & End (Glowing Red)
      drawCellBox(start.r, start.c, '#10b981', cellPad, 'rgba(16, 185, 129, 0.75)');
      drawCellBox(end.r, end.c, '#f43f5e', cellPad, 'rgba(244, 63, 94, 0.75)');

      if (needsMoreFrames) {
        rafRef.current = requestAnimationFrame(draw);
      }
    };

    draw();
    const ro = new ResizeObserver(draw);
    ro.observe(container);
    return () => {
      ro.disconnect();
      if (rafRef.current) cancelAnimationFrame(rafRef.current);
    };
  }, [grid, start, end, visitedSet, pathCells, altPaths, treeEdges, smoothPath, darkMode, rows, cols]);

  const getCellFromEvent = (e) => {
    const canvas = canvasRef.current;
    if (!canvas) return null;
    const rect = canvas.getBoundingClientRect();
    const cellW = rect.width / cols;
    const cellH = rect.height / rows;
    const c = Math.floor((e.clientX - rect.left) / cellW);
    const r = Math.floor((e.clientY - rect.top) / cellH);
    if (r < 0 || r >= rows || c < 0 || c >= cols) return null;
    return { r, c };
  };

  const handlePointerDown = (e) => {
    e.preventDefault();
    const cell = getCellFromEvent(e);
    if (!cell) return;
    e.currentTarget.setPointerCapture?.(e.pointerId);

    if (cell.r === start.r && cell.c === start.c) {
      draggingRef.current = 'start';
      lastCellRef.current = `${cell.r},${cell.c}`;
      return;
    }
    if (cell.r === end.r && cell.c === end.c) {
      draggingRef.current = 'end';
      lastCellRef.current = `${cell.r},${cell.c}`;
      return;
    }

    if (activeTool === 'start') {
      onMoveEndpoint('start', cell);
      draggingRef.current = 'start';
      return;
    }
    if (activeTool === 'end') {
      onMoveEndpoint('end', cell);
      draggingRef.current = 'end';
      return;
    }

    draggingRef.current = 'draw';
    lastCellRef.current = `${cell.r},${cell.c}`;
    onCellInteract(cell.r, cell.c);
  };

  const handlePointerMove = (e) => {
    if (!draggingRef.current) return;
    const cell = getCellFromEvent(e);
    if (!cell) return;
    const key = `${cell.r},${cell.c}`;
    if (key === lastCellRef.current) return;
    lastCellRef.current = key;

    if (draggingRef.current === 'start') {
      onMoveEndpoint('start', cell);
    } else if (draggingRef.current === 'end') {
      onMoveEndpoint('end', cell);
    } else if (draggingRef.current === 'draw') {
      onCellInteract(cell.r, cell.c);
    }
  };

  const handlePointerUp = () => {
    draggingRef.current = null;
    lastCellRef.current = null;
  };

  return (
    <div ref={containerRef} className="grid-canvas-container">
      <canvas
        ref={canvasRef}
        className="grid-canvas"
        onPointerDown={handlePointerDown}
        onPointerMove={handlePointerMove}
        onPointerUp={handlePointerUp}
        onPointerCancel={handlePointerUp}
      />
    </div>
  );
}
