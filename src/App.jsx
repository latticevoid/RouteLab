import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import {
  Play,
  Square,
  Trash2,
  Eraser,
  BrickWall,
  Weight,
  MapPin,
  Flag,
  Sun,
  Moon,
  SlidersHorizontal,
  RotateCcw,
  ArrowUpRight,
  ArrowLeftRight,
  Shuffle,
  PanelLeftClose,
  PanelLeftOpen,
  Wand2,
} from 'lucide-react';
import GridCanvas from './components/GridCanvas.jsx';
import AlgorithmDropdown from './components/AlgorithmDropdown.jsx';
import { runAlgorithm } from './algorithms/index.js';
import { createEmptyGrid, MAZE_PRESETS } from './utils/mazeGenerators.js';

const SPEEDS = {
  slow: { label: 'Slow', batch: 2, delay: 24 },
  normal: { label: 'Normal', batch: 8, delay: 14 },
  fast: { label: 'Fast', batch: 24, delay: 10 },
  instant: { label: 'Instant', batch: Infinity, delay: 0 },
};

function getInitialDimensions() {
  if (typeof window !== 'undefined' && window.innerWidth < 768) {
    return { rows: 17, cols: 15 };
  }
  return { rows: 21, cols: 41 };
}

export default function App() {
  const initDims = useMemo(() => getInitialDimensions(), []);
  const [rows, setRows] = useState(initDims.rows);
  const [cols, setCols] = useState(initDims.cols);

  const [start, setStart] = useState(() => ({
    r: Math.floor(initDims.rows / 2),
    c: Math.max(1, Math.floor(initDims.cols * 0.2)),
  }));
  const [end, setEnd] = useState(() => ({
    r: Math.floor(initDims.rows / 2),
    c: Math.min(initDims.cols - 2, Math.floor(initDims.cols * 0.8)),
  }));

  const [grid, setGrid] = useState(() => createEmptyGrid(initDims.rows, initDims.cols));
  const [selectedAlgoId, setSelectedAlgoId] = useState('astar');
  const [activeTool, setActiveTool] = useState('wall');
  const [allowDiagonal, setAllowDiagonal] = useState(false);
  const [speedKey, setSpeedKey] = useState('fast');
  const [darkMode, setDarkMode] = useState(() => {
    if (typeof window !== 'undefined') {
      return localStorage.getItem('routelab-theme') !== 'light';
    }
    return true;
  });

  const [showMobileOptions, setShowMobileOptions] = useState(false);
  const [sidebarCollapsed, setSidebarCollapsed] = useState(false);

  const [isRunning, setIsRunning] = useState(false);
  const [visitedSet, setVisitedSet] = useState(() => new Map());
  const [pathCells, setPathCells] = useState([]);
  const [altPaths, setAltPaths] = useState([]);
  const [treeEdges, setTreeEdges] = useState([]);
  const [smoothPath, setSmoothPath] = useState(null);

  const animTimerRef = useRef(null);

  useEffect(() => {
    document.documentElement.setAttribute('data-theme', darkMode ? 'dark' : 'light');
    localStorage.setItem('routelab-theme', darkMode ? 'dark' : 'light');
  }, [darkMode]);

  const stopAnimation = useCallback(() => {
    if (animTimerRef.current) {
      clearTimeout(animTimerRef.current);
      animTimerRef.current = null;
    }
    setIsRunning(false);
  }, []);

  const clearVisualization = useCallback(() => {
    stopAnimation();
    setVisitedSet(new Map());
    setPathCells([]);
    setAltPaths([]);
    setTreeEdges([]);
    setSmoothPath(null);
  }, [stopAnimation]);

  const handleClearBoard = useCallback(() => {
    clearVisualization();
    setGrid(createEmptyGrid(rows, cols));
  }, [clearVisualization, rows, cols]);

  const handleResizeGrid = useCallback(
    (newRows, newCols) => {
      stopAnimation();
      setRows(newRows);
      setCols(newCols);
      setStart({ r: Math.floor(newRows / 2), c: Math.max(1, Math.floor(newCols * 0.2)) });
      setEnd({ r: Math.floor(newRows / 2), c: Math.min(newCols - 2, Math.floor(newCols * 0.8)) });
      setGrid(createEmptyGrid(newRows, newCols));
      setVisitedSet(new Map());
      setPathCells([]);
      setAltPaths([]);
      setTreeEdges([]);
      setSmoothPath(null);
    },
    [stopAnimation]
  );

  const handleGenerateMaze = useCallback(
    (presetFn) => {
      clearVisualization();
      setGrid(presetFn(rows, cols, start, end));
      setShowMobileOptions(false);
    },
    [clearVisualization, rows, cols, start, end]
  );

  const handleCellInteract = useCallback(
    (r, c) => {
      if ((r === start.r && c === start.c) || (r === end.r && c === end.c)) return;
      setGrid((prev) => {
        const next = prev.map((row) => row.slice());
        const cell = { ...next[r][c] };
        if (activeTool === 'wall') {
          cell.wall = true;
          cell.weight = 1;
        } else if (activeTool === 'weight') {
          cell.wall = false;
          cell.weight = cell.weight >= 5 ? 2 : 5;
        } else if (activeTool === 'erase') {
          cell.wall = false;
          cell.weight = 1;
        }
        next[r][c] = cell;
        return next;
      });
    },
    [activeTool, start, end]
  );

  const handleMoveEndpoint = useCallback(
    (type, pt) => {
      if (type === 'start') {
        if (pt.r === end.r && pt.c === end.c) return;
        setStart(pt);
      } else {
        if (pt.r === start.r && pt.c === start.c) return;
        setEnd(pt);
      }
      clearVisualization();
      setGrid((prev) => {
        if (!prev[pt.r][pt.c].wall) return prev;
        const next = prev.map((row) => row.slice());
        next[pt.r][pt.c] = { wall: false, weight: 1 };
        return next;
      });
    },
    [start, end, clearVisualization]
  );

  const handleSwapEndpoints = useCallback(() => {
    clearVisualization();
    setStart(end);
    setEnd(start);
  }, [start, end, clearVisualization]);

  const handleRandomEndpoints = useCallback(() => {
    clearVisualization();
    const open = [];
    for (let r = 0; r < rows; r++) {
      for (let c = 0; c < cols; c++) {
        if (!grid[r][c].wall) open.push({ r, c });
      }
    }
    if (open.length < 2) return;
    const i1 = Math.floor(Math.random() * open.length);
    let i2 = Math.floor(Math.random() * open.length);
    while (i2 === i1) i2 = Math.floor(Math.random() * open.length);
    setStart(open[i1]);
    setEnd(open[i2]);
  }, [rows, cols, grid, clearVisualization]);

  const handleRun = useCallback(() => {
    if (isRunning) {
      stopAnimation();
      return;
    }
    clearVisualization();

    const result = runAlgorithm(selectedAlgoId, grid, start, end, allowDiagonal);
    const { visitedOrder, path, altPaths: resAlt, treeEdges: resEdges, smoothPath: resSmooth } = result;
    const speed = SPEEDS[speedKey];

    if (speed.batch === Infinity || visitedOrder.length === 0) {
      const vMap = new Map();
      visitedOrder.forEach((v, i) => vMap.set(`${v.r},${v.c}`, i));
      setVisitedSet(vMap);
      setPathCells(path);
      setAltPaths(resAlt);
      setTreeEdges(resEdges);
      setSmoothPath(resSmooth);
      return;
    }

    setIsRunning(true);
    setTreeEdges(resEdges);
    let idx = 0;
    const vMap = new Map();

    const stepVisit = () => {
      const limit = Math.min(visitedOrder.length, idx + speed.batch);
      for (; idx < limit; idx++) {
        const v = visitedOrder[idx];
        vMap.set(`${v.r},${v.c}`, idx);
      }
      setVisitedSet(new Map(vMap));

      if (idx < visitedOrder.length) {
        animTimerRef.current = setTimeout(stepVisit, speed.delay);
      } else {
        let pIdx = 0;
        const pathDelay = speedKey === 'slow' ? 42 : speedKey === 'normal' ? 26 : 16;
        const stepPath = () => {
          pIdx += 1;
          setPathCells(path.slice(0, pIdx));
          if (pIdx < path.length) {
            animTimerRef.current = setTimeout(stepPath, pathDelay);
          } else {
            setAltPaths(resAlt);
            setSmoothPath(resSmooth);
            setIsRunning(false);
          }
        };
        if (path.length > 0) {
          stepPath();
        } else {
          setIsRunning(false);
        }
      }
    };

    stepVisit();
  }, [isRunning, stopAnimation, clearVisualization, selectedAlgoId, grid, start, end, allowDiagonal, speedKey]);

  return (
    <div className="app-shell">
      {/* Top Bar */}
      <header className="header">
        <div className="header-left">
          <span className="wordmark">RouteLab</span>
          <span className="header-sep" />
          <AlgorithmDropdown
            selectedId={selectedAlgoId}
            onSelect={(id) => {
              setSelectedAlgoId(id);
              clearVisualization();
            }}
          />
        </div>

        <div className="header-right">
          <button
            className={`btn-run ${isRunning ? 'is-running' : ''}`}
            onClick={handleRun}
          >
            {isRunning ? <Square size={14} /> : <Play size={14} />}
            <span>{isRunning ? 'Stop' : 'Run'}</span>
          </button>

          <button className="btn-ghost hide-mobile" onClick={clearVisualization}>
            <RotateCcw size={14} />
            <span>Clear path</span>
          </button>

          <button className="btn-ghost hide-mobile" onClick={handleClearBoard}>
            <Trash2 size={14} />
            <span>Clear board</span>
          </button>
        </div>
      </header>

      {/* Studio Body: Collapsible Left Sidebar + Canvas */}
      <div className="studio-body">
        <aside
          className={`sidebar ${sidebarCollapsed ? 'collapsed' : ''}`}
          aria-label="Workspace controls"
        >
          <div className="sb-header">
            {!sidebarCollapsed && <span className="sb-header-label">Controls</span>}
            <button
              type="button"
              className="sb-collapse-btn"
              onClick={() => setSidebarCollapsed((c) => !c)}
              title={sidebarCollapsed ? 'Expand sidebar' : 'Collapse sidebar'}
              aria-label={sidebarCollapsed ? 'Expand sidebar' : 'Collapse sidebar'}
            >
              {sidebarCollapsed ? <PanelLeftOpen size={16} /> : <PanelLeftClose size={16} />}
            </button>
          </div>

          <div className="sb-section">
            {!sidebarCollapsed && <div className="sb-title">Draw</div>}
            <div className={sidebarCollapsed ? 'sb-rail-stack' : 'sb-tool-grid'}>
              <button
                type="button"
                className={`sb-btn ${activeTool === 'wall' ? 'active' : ''}`}
                onClick={() => setActiveTool('wall')}
                title="Draw walls"
              >
                <BrickWall size={16} />
                {!sidebarCollapsed && <span>Wall</span>}
              </button>
              <button
                type="button"
                className={`sb-btn ${activeTool === 'weight' ? 'active' : ''}`}
                onClick={() => setActiveTool('weight')}
                title="Paint weighted cells"
              >
                <Weight size={16} />
                {!sidebarCollapsed && <span>Weight</span>}
              </button>
              <button
                type="button"
                className={`sb-btn ${activeTool === 'erase' ? 'active' : ''}`}
                onClick={() => setActiveTool('erase')}
                title="Erase cells"
              >
                <Eraser size={16} />
                {!sidebarCollapsed && <span>Erase</span>}
              </button>
              <button
                type="button"
                className={`sb-btn ${activeTool === 'start' ? 'active' : ''}`}
                onClick={() => setActiveTool('start')}
                title="Place origin"
              >
                <MapPin size={16} />
                {!sidebarCollapsed && <span>Origin</span>}
              </button>
              <button
                type="button"
                className={`sb-btn ${activeTool === 'end' ? 'active' : ''}`}
                onClick={() => setActiveTool('end')}
                title="Place end"
              >
                <Flag size={16} />
                {!sidebarCollapsed && <span>End</span>}
              </button>
            </div>
          </div>

          <div className="sb-divider" />

          <div className="sb-section">
            {!sidebarCollapsed && <div className="sb-title">Origin & End</div>}
            <div className={sidebarCollapsed ? 'sb-rail-stack' : 'sb-row'}>
              <button
                type="button"
                className="sb-btn"
                onClick={handleSwapEndpoints}
                title="Swap Origin & End"
              >
                <ArrowLeftRight size={15} />
                {!sidebarCollapsed && <span>Swap S/E</span>}
              </button>
              <button
                type="button"
                className="sb-btn"
                onClick={handleRandomEndpoints}
                title="Randomize Origin & End"
              >
                <Shuffle size={15} />
                {!sidebarCollapsed && <span>Random</span>}
              </button>
            </div>
          </div>

          {!sidebarCollapsed && (
            <>
              <div className="sb-divider" />

              <div className="sb-section">
                <div className="sb-title">Mazes</div>
                <div className="sb-stack">
                  {MAZE_PRESETS.map((preset) => (
                    <button
                      key={preset.id}
                      type="button"
                      className="sb-list-btn"
                      onClick={() => handleGenerateMaze(preset.fn)}
                    >
                      <Wand2 size={14} />
                      <span>{preset.label}</span>
                    </button>
                  ))}
                </div>
              </div>

              <div className="sb-divider" />

              <div className="sb-section">
                <div className="sb-title">Speed</div>
                <div className="sb-segmented">
                  {Object.entries(SPEEDS).map(([k, v]) => (
                    <button
                      key={k}
                      type="button"
                      className={`sb-seg-btn ${speedKey === k ? 'active' : ''}`}
                      onClick={() => setSpeedKey(k)}
                    >
                      {v.label === 'Instant' ? 'Max' : v.label === 'Normal' ? 'Norm' : v.label}
                    </button>
                  ))}
                </div>

                <div className="sb-title" style={{ marginTop: 6 }}>Grid size</div>
                <div className="sb-segmented">
                  {[
                    { key: '15x15', label: '15×15' },
                    { key: '21x41', label: '21×41' },
                    { key: '27x51', label: '27×51' },
                  ].map((sz) => (
                    <button
                      key={sz.key}
                      type="button"
                      className={`sb-seg-btn ${`${rows}x${cols}` === sz.key ? 'active' : ''}`}
                      onClick={() => {
                        const [r, c] = sz.key.split('x').map(Number);
                        handleResizeGrid(r, c);
                      }}
                    >
                      {sz.label}
                    </button>
                  ))}
                </div>

                <button
                  type="button"
                  className={`sb-toggle ${allowDiagonal ? 'active' : ''}`}
                  onClick={() => setAllowDiagonal((d) => !d)}
                  style={{ marginTop: 6 }}
                >
                  <ArrowUpRight size={15} />
                  <span>Diagonal movement</span>
                  <span className="sb-pill">{allowDiagonal ? 'On' : 'Off'}</span>
                </button>
              </div>
            </>
          )}
        </aside>

        <main className="canvas-stage">
          <button
            className="stage-theme-btn"
            onClick={() => setDarkMode((d) => !d)}
            title={darkMode ? 'Switch to Light theme' : 'Switch to Dark theme'}
            aria-label="Toggle theme"
          >
            {darkMode ? <Sun size={16} /> : <Moon size={16} />}
          </button>

          <div className="stage-canvas-wrap">
            <GridCanvas
              grid={grid}
              start={start}
              end={end}
              visitedSet={visitedSet}
              pathCells={pathCells}
              altPaths={altPaths}
              treeEdges={treeEdges}
              smoothPath={smoothPath}
              activeTool={activeTool}
              darkMode={darkMode}
              onCellInteract={handleCellInteract}
              onMoveEndpoint={handleMoveEndpoint}
            />
          </div>

          <div className="stage-credit">by LatticeVoid</div>
        </main>
      </div>

      {/* Mobile Bottom Bar */}
      <nav className="mobile-bar">
        <button
          className={`mob-btn ${activeTool === 'wall' ? 'active' : ''}`}
          onClick={() => setActiveTool('wall')}
        >
          <BrickWall size={18} />
          <span>Wall</span>
        </button>
        <button
          className={`mob-btn ${activeTool === 'weight' ? 'active' : ''}`}
          onClick={() => setActiveTool('weight')}
        >
          <Weight size={18} />
          <span>Weight</span>
        </button>
        <button
          className={`mob-btn ${activeTool === 'erase' ? 'active' : ''}`}
          onClick={() => setActiveTool('erase')}
        >
          <Eraser size={18} />
          <span>Erase</span>
        </button>
        <button
          className={`mob-btn ${activeTool === 'start' || activeTool === 'end' ? 'active' : ''}`}
          onClick={() => setActiveTool((t) => (t === 'start' ? 'end' : 'start'))}
        >
          {activeTool === 'end' ? <Flag size={18} /> : <MapPin size={18} />}
          <span>{activeTool === 'end' ? 'End' : 'Origin'}</span>
        </button>
        <button
          className={`mob-btn ${showMobileOptions ? 'active' : ''}`}
          onClick={() => setShowMobileOptions((s) => !s)}
        >
          <SlidersHorizontal size={18} />
          <span>More</span>
        </button>
      </nav>

      {/* Mobile Sheet */}
      {showMobileOptions && (
        <div className="sheet-backdrop" onClick={() => setShowMobileOptions(false)}>
          <div className="sheet-panel" onClick={(e) => e.stopPropagation()}>
            <div className="sheet-group">
              <div className="sheet-title">Mazes</div>
              <div className="sheet-grid">
                {MAZE_PRESETS.map((preset) => (
                  <button
                    key={preset.id}
                    className="sheet-chip"
                    onClick={() => handleGenerateMaze(preset.fn)}
                  >
                    {preset.label}
                  </button>
                ))}
              </div>
            </div>

            <div className="sheet-group">
              <div className="sheet-title">Board</div>
              <div className="sheet-row">
                <button
                  className={`sheet-chip ${allowDiagonal ? 'active' : ''}`}
                  onClick={() => setAllowDiagonal((d) => !d)}
                >
                  Diagonal: {allowDiagonal ? 'On' : 'Off'}
                </button>
                <button
                  className="sheet-chip"
                  onClick={() => {
                    handleSwapEndpoints();
                    setShowMobileOptions(false);
                  }}
                >
                  Swap origin & end
                </button>
                <button
                  className="sheet-chip"
                  onClick={() => {
                    handleRandomEndpoints();
                    setShowMobileOptions(false);
                  }}
                >
                  Randomize
                </button>
                <button className="sheet-chip" onClick={() => setDarkMode((d) => !d)}>
                  {darkMode ? 'Light theme' : 'Dark theme'}
                </button>
                <button
                  className="sheet-chip"
                  onClick={() => {
                    clearVisualization();
                    setShowMobileOptions(false);
                  }}
                >
                  Clear path
                </button>
                <button
                  className="sheet-chip"
                  onClick={() => {
                    handleClearBoard();
                    setShowMobileOptions(false);
                  }}
                >
                  Clear board
                </button>
              </div>
              <div className="sheet-row" style={{ marginTop: 8 }}>
                {Object.entries(SPEEDS).map(([k, v]) => (
                  <button
                    key={k}
                    className={`sheet-chip ${speedKey === k ? 'active' : ''}`}
                    onClick={() => setSpeedKey(k)}
                  >
                    {v.label}
                  </button>
                ))}
              </div>
            </div>
          </div>
        </div>
      )}
    </div>
  );
}
