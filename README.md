<div align="center">

# RouteLab

### Interactive Pathfinding, Motion Planning & Network Routing Visualizer

Visualize **103 algorithms** across 9 categories in real time on an interactive 2D maze grid.

**[Live Demo — routelabb.netlify.app](https://routelabb.netlify.app)**

[![Netlify Status](https://api.netlify.com/api/v1/badges/f9e17b0e-cf2a-461d-925f-0e4499136e6e/deploy-status)](https://app.netlify.com/projects/routelabb/deploys)

</div>

---

## Features

- **103 Algorithms** — Searchable dropdown with category filters covering classical search, A* variants, grid-based, dynamic/real-time, robotic motion planning, graph optimization, nature-inspired metaheuristics, multi-agent pathfinding (MAPF), and network routing.
- **Interactive Canvas** — Draw walls, paint weighted terrain, erase cells, and drag or place Origin (`S`) and End (`E`) nodes with mouse or touch.
- **Maze & Terrain Generators** — Recursive Backtracker, Recursive Division, Randomized Prim's, Random Obstacles, and Weighted Terrain.
- **Movement & Speed Controls** — Toggle 4-way vs. 8-way diagonal movement and switch between `Slow`, `Normal`, `Fast`, and `Max` (instant) visualization speeds, with cell-by-cell path tracing toward the target.
- **Collapsible Sidebar & Mobile UI** — Desktop sidebar collapses to an icon rail (`252px` ↔ `56px`); mobile layout uses a compact top bar, touch-friendly bottom dock, and slide-up options sheet.
- **Dark & Light Themes** — High-contrast monochrome drafting surfaces in both Dark and Light modes.

---

## Algorithms (103)

<details>
<summary><strong>Classical Pathfinding (11)</strong></summary>

1. Dijkstra's Algorithm
2. A* (A-Star)
3. Breadth-First Search (BFS)
4. Depth-First Search (DFS)
5. Bellman-Ford Algorithm
6. Floyd-Warshall Algorithm
7. Johnson's Algorithm
8. Bidirectional Search
9. Iterative Deepening DFS (IDDFS)
10. Uniform Cost Search (UCS)
11. Greedy Best-First Search

</details>

<details>
<summary><strong>A* Variations & Heuristics (14)</strong></summary>

- Weighted A*, Anytime A*, IDA*, SMA*, RBFS, Beam Search, Jump Point Search (JPS), Theta*, Lazy Theta*, Anya, HPA*, PRA*, Fringe Search, EPEA*

</details>

<details>
<summary><strong>Grid-Based & Dynamic / Real-Time (17)</strong></summary>

- Lee Algorithm, Wavefront Algorithm, Brushfire Algorithm, D*, D* Lite, Lifelong Planning A* (LPA*), Field D*, Truncated Dijkstra, Adaptive A*, RTAA*, LRTA*, RTA*, Moving Target Search, Real-Time Dijkstra

</details>

<details>
<summary><strong>Robotic Pathfinding & Motion Planning (19)</strong></summary>

- RRT, RRT*, PRM, FMT*, Informed RRT*, Bidirectional RRT, Lazy PRM, EST, KPIECE, BIT*, FMT, CHOMP, STOMP, TrajOpt, Potential Field Method, Artificial Potential Field (APF), Elastic Bands, Timed Elastic Band (TEB), Model Predictive Control (MPC)

</details>

<details>
<summary><strong>Graph Optimization & Specialized Shortest-Path (13)</strong></summary>

- Contraction Hierarchies, ALT, Transit Node Routing, Hub Labeling, Arc Flags, Highway Hierarchies, Multi-Level Dijkstra, Yen's Algorithm, Eppstein's Algorithm, Suurballe's Algorithm, Bhandari's Algorithm, K-Shortest Paths, Viterbi Algorithm

</details>

<details>
<summary><strong>Nature-Inspired & Metaheuristic (12)</strong></summary>

- Genetic Algorithm (GA), Ant Colony Optimization (ACO), Particle Swarm Optimization (PSO), Artificial Bee Colony (ABC), Grey Wolf Optimizer (GWO), Firefly Algorithm, Cuckoo Search, Whale Optimization Algorithm (WOA), Bat Algorithm, Simulated Annealing, Tabu Search, Differential Evolution

</details>

<details>
<summary><strong>Multi-Agent Pathfinding / MAPF (10)</strong></summary>

- Cooperative A*, Hierarchical Cooperative A* (HCA*), Windowed HCA* (WHCA*), Conflict-Based Search (CBS), Enhanced CBS (ECBS), Prioritized Planning, M*, Push and Swap, PIBT, LaCAM

</details>

<details>
<summary><strong>Network Routing (7)</strong></summary>

- Link-State Routing, Distance-Vector Routing, OSPF, IS-IS, RIP, BGP Path Selection, Equal-Cost Multi-Path (ECMP)

</details>

---

## Getting Started

```bash
git clone https://github.com/LatticeVoid/RouteLab.git
cd RouteLab
npm install
npm run dev
```

### Scripts

| Command | Description |
| :--- | :--- |
| `npm run dev` | Start the Vite development server |
| `npm run build` | Build the production bundle to `dist/` |
| `npm run preview` | Preview the production build locally |
| `npm run lint` | Run `oxlint` across the project |

---

## Tech Stack

- **React 19** + **Vite 8**
- **HTML5 Canvas** (high-DPI pointer & touch rendering)
- **Lucide React** icons

---

<div align="center">

Built by **LatticeVoid** · [routelabb.netlify.app](https://routelabb.netlify.app)

</div>
