# Version 5 - FINAL: Dynamic Obstacles + Real-Time Re-planning
# AI Dynamic Pathfinding Agent
# Created by: Ans Rizwan (24F-0779)
#
# Features:
#   • GBFS and A* with Manhattan / Euclidean heuristic toggle
#   • Dynamic Mode: obstacles spawn randomly while agent moves
#   • Re-planning: if a new wall blocks the path, agent recalculates from current position
#   • Full metrics dashboard: nodes visited, path cost, time, re-plans
#   • Animated step-by-step visualization

import tkinter as tk
import heapq, random, time, math

# ─────────────────────────────────────────
#  SETTINGS  (tweak freely)
# ─────────────────────────────────────────
GRID_ROWS      = 15
GRID_COLS      = 15
CELL_SIZE      = 40
PADDING        = 10
MOVE_DELAY     = 120     # ms between each agent step
SPAWN_PROB     = 0.06    # probability a new wall spawns each agent step

COLOR_EMPTY    = "#FFFFFF"
COLOR_WALL     = "#2C3E50"
COLOR_START    = "#27AE60"
COLOR_GOAL     = "#E74C3C"
COLOR_FRONTIER = "#F1C40F"
COLOR_VISITED  = "#3498DB"
COLOR_PATH     = "#2ECC71"
COLOR_AGENT    = "#9B59B6"   # purple – current agent position


# ─────────────────────────────────────────
#  HEURISTICS
# ─────────────────────────────────────────
def manhattan(a, b):
    """D_manhattan = |x1-x2| + |y1-y2|"""
    return abs(a[0]-b[0]) + abs(a[1]-b[1])

def euclidean(a, b):
    """D_euclidean = sqrt((x1-x2)^2 + (y1-y2)^2)"""
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)


# ─────────────────────────────────────────
#  SEARCH ALGORITHMS
# ─────────────────────────────────────────
def gbfs(grid, start, goal, h_fn):
    """
    Greedy Best-First Search
    f(n) = h(n)  — only heuristic, no actual cost
    Fast, but path may not be optimal.
    """
    heap = [(h_fn(start, goal), start)]
    came_from = {start: None}
    visited   = []

    while heap:
        _, cur = heapq.heappop(heap)
        if cur == goal:
            return _reconstruct(came_from, goal), visited
        if cur in visited:
            continue
        visited.append(cur)
        for nb in _neighbors(grid, cur):
            if nb not in came_from:
                came_from[nb] = cur
                heapq.heappush(heap, (h_fn(nb, goal), nb))

    return None, visited


def astar(grid, start, goal, h_fn):
    """
    A* Search
    f(n) = g(n) + h(n)  — actual cost + heuristic
    Guaranteed to find the shortest path.
    """
    heap      = [(h_fn(start, goal), 0, start)]
    came_from = {start: None}
    g_cost    = {start: 0}
    visited   = []

    while heap:
        _, g, cur = heapq.heappop(heap)
        if cur == goal:
            return _reconstruct(came_from, goal), visited
        if cur in visited:
            continue
        visited.append(cur)
        for nb in _neighbors(grid, cur):
            ng = g + 1
            if nb not in g_cost or ng < g_cost[nb]:
                g_cost[nb]    = ng
                came_from[nb] = cur
                heapq.heappush(heap, (ng + h_fn(nb, goal), ng, nb))

    return None, visited


# ─────────────────────────────────────────
#  HELPERS
# ─────────────────────────────────────────
def _neighbors(grid, pos):
    r, c = pos
    result = []
    for dr, dc in [(-1,0),(1,0),(0,-1),(0,1)]:
        nr, nc = r+dr, c+dc
        if 0<=nr<GRID_ROWS and 0<=nc<GRID_COLS and grid[nr][nc]!="wall":
            result.append((nr, nc))
    return result

def _reconstruct(came_from, goal):
    path, node = [], goal
    while node is not None:
        path.append(node); node = came_from[node]
    path.reverse(); return path


# ─────────────────────────────────────────
#  MAIN APPLICATION
# ─────────────────────────────────────────
class PathfinderApp:

    def __init__(self, root):
        self.root  = root
        self.root.title("AI Dynamic Pathfinder  –  FINAL v5  –  24F-0785 & 24F-0779")
        self.root.resizable(False, False)

        self.grid  = [[None]*GRID_COLS for _ in range(GRID_ROWS)]
        self.start = (0, 0)
        self.goal  = (GRID_ROWS-1, GRID_COLS-1)

        # Animation / agent state
        self._cell_colors  = {}     # (r,c) → color
        self._path         = []     # remaining path for agent
        self._path_index   = 0      # next step index
        self._agent_pos    = None   # current agent cell
        self._running      = False  # is agent moving?

        # Cumulative metrics
        self._total_visited = 0
        self._total_cost    = 0
        self._total_replans = 0
        self._start_time    = 0

        self._build_ui()
        self._draw_grid()

    # ══════════════════════════════════════
    #  UI CONSTRUCTION
    # ══════════════════════════════════════
    def _build_ui(self):
        cw = GRID_COLS*CELL_SIZE + 2*PADDING
        ch = GRID_ROWS*CELL_SIZE + 2*PADDING

        # Canvas
        self.canvas = tk.Canvas(self.root, width=cw, height=ch,
                                bg="#ECF0F1", highlightthickness=0)
        self.canvas.grid(row=0, column=0, padx=10, pady=10)
        self.canvas.bind("<Button-1>", self._on_click)

        # Control panel
        ctrl = tk.Frame(self.root, bg="#ECF0F1", padx=10, pady=10)
        ctrl.grid(row=0, column=1, sticky="n")

        tk.Label(ctrl, text="AI Pathfinder", font=("Arial",15,"bold"),
                 bg="#ECF0F1", fg="#2C3E50").pack(pady=(0,4))
        tk.Label(ctrl, text="24F-0785 · 24F-0779", font=("Arial",8),
                 bg="#ECF0F1", fg="#7F8C8D").pack(pady=(0,10))

        # ── Algorithm ──
        tk.Label(ctrl, text="Algorithm", font=("Arial",10,"bold"),
                 bg="#ECF0F1").pack(anchor="w")
        self.algo_var = tk.StringVar(value="A*")
        for name in ("GBFS","A*"):
            tk.Radiobutton(ctrl, text=name, variable=self.algo_var,
                           value=name, bg="#ECF0F1", font=("Arial",9)).pack(anchor="w")

        # ── Heuristic ──
        tk.Label(ctrl, text="Heuristic", font=("Arial",10,"bold"),
                 bg="#ECF0F1").pack(anchor="w", pady=(8,0))
        self.heur_var = tk.StringVar(value="Manhattan")
        for name in ("Manhattan","Euclidean"):
            tk.Radiobutton(ctrl, text=name, variable=self.heur_var,
                           value=name, bg="#ECF0F1", font=("Arial",9)).pack(anchor="w")

        # ── Dynamic Mode ──
        tk.Label(ctrl, text="Dynamic Mode", font=("Arial",10,"bold"),
                 bg="#ECF0F1").pack(anchor="w", pady=(8,0))
        self.dynamic_var = tk.BooleanVar(value=False)
        tk.Checkbutton(ctrl, text="Enable dynamic obstacles",
                       variable=self.dynamic_var, bg="#ECF0F1",
                       font=("Arial",9)).pack(anchor="w")

        # ── Buttons ──
        tk.Button(ctrl, text="▶  Start", width=18,
                  bg="#27AE60", fg="white", font=("Arial",10,"bold"),
                  command=self._start_search).pack(pady=(12,3))
        tk.Button(ctrl, text="⏹  Stop", width=18,
                  bg="#E67E22", fg="white", font=("Arial",10,"bold"),
                  command=self._stop).pack(pady=3)
        tk.Button(ctrl, text="Random Walls", width=18,
                  bg="#8E44AD", fg="white", font=("Arial",10,"bold"),
                  command=self._random_walls).pack(pady=3)
        tk.Button(ctrl, text="Reset Grid", width=18,
                  bg="#E74C3C", fg="white", font=("Arial",10,"bold"),
                  command=self._reset_grid).pack(pady=3)

        # ── Status ──
        self.status_var = tk.StringVar(value="Set up grid and press Start.")
        tk.Label(ctrl, textvariable=self.status_var, bg="#ECF0F1",
                 font=("Arial",9), wraplength=165, justify="left",
                 fg="#2C3E50").pack(pady=(10,0))

        # ── Metrics Dashboard ──
        tk.Label(ctrl, text="📊 Metrics Dashboard", font=("Arial",11,"bold"),
                 bg="#ECF0F1").pack(pady=(16,5))

        mf = tk.Frame(ctrl, bg="#D5DBDB", relief="groove", bd=1)
        mf.pack(fill="x", padx=2)

        self.m_algo    = tk.StringVar(value="Algorithm:     —")
        self.m_heur    = tk.StringVar(value="Heuristic:     —")
        self.m_visited = tk.StringVar(value="Nodes visited: —")
        self.m_cost    = tk.StringVar(value="Path cost:     —")
        self.m_time    = tk.StringVar(value="Time (ms):     —")
        self.m_replans = tk.StringVar(value="Re-plans:      —")

        for v in (self.m_algo, self.m_heur, self.m_visited,
                  self.m_cost, self.m_time, self.m_replans):
            tk.Label(mf, textvariable=v, bg="#D5DBDB",
                     font=("Courier",9), anchor="w", padx=4).pack(fill="x")

        # ── Legend ──
        tk.Label(ctrl, text="Legend", font=("Arial",11,"bold"),
                 bg="#ECF0F1").pack(pady=(14,4))
        for color, label in [
            (COLOR_START,  "Start"),   (COLOR_GOAL,  "Goal"),
            (COLOR_WALL,   "Wall"),    (COLOR_AGENT, "Agent"),
            (COLOR_PATH,   "Path"),    (COLOR_VISITED,"Visited"),
            (COLOR_FRONTIER,"Frontier"),
        ]:
            row = tk.Frame(ctrl, bg="#ECF0F1"); row.pack(anchor="w")
            tk.Label(row, bg=color, width=2, relief="solid").pack(side="left", padx=(0,6))
            tk.Label(row, text=label, bg="#ECF0F1", font=("Arial",9)).pack(side="left")

    # ══════════════════════════════════════
    #  DRAWING
    # ══════════════════════════════════════
    def _draw_grid(self):
        self.canvas.delete("all")
        for r in range(GRID_ROWS):
            for c in range(GRID_COLS):
                self._draw_cell(r, c)

    def _draw_cell(self, r, c):
        x1=PADDING+c*CELL_SIZE; y1=PADDING+r*CELL_SIZE
        x2=x1+CELL_SIZE;        y2=y1+CELL_SIZE

        pos = (r, c)
        if pos == self._agent_pos:          color, text = COLOR_AGENT, "A"
        elif pos == self.start:             color, text = COLOR_START, "S"
        elif pos == self.goal:              color, text = COLOR_GOAL,  "G"
        elif self.grid[r][c] == "wall":     color, text = COLOR_WALL,  ""
        elif pos in self._cell_colors:      color, text = self._cell_colors[pos], ""
        else:                               color, text = COLOR_EMPTY, ""

        self.canvas.create_rectangle(x1,y1,x2,y2, fill=color, outline="#BDC3C7")
        if text:
            self.canvas.create_text((x1+x2)//2,(y1+y2)//2,
                                    text=text, font=("Arial",11,"bold"), fill="white")

    # ══════════════════════════════════════
    #  SEARCH + AGENT MOVEMENT
    # ══════════════════════════════════════
    def _get_algo_and_heuristic(self):
        h_fn    = manhattan if self.heur_var.get()=="Manhattan" else euclidean
        algo_fn = gbfs      if self.algo_var.get()=="GBFS"      else astar
        return algo_fn, h_fn

    def _start_search(self):
        """Initial search from start to goal, then begin agent movement."""
        self._running = True
        self._cell_colors.clear()
        self._total_visited = 0
        self._total_cost    = 0
        self._total_replans = 0
        self._start_time    = time.time()

        algo_fn, h_fn = self._get_algo_and_heuristic()
        path, visited = algo_fn(self.grid, self.start, self.goal, h_fn)

        if path is None:
            self.status_var.set("No path found! Add fewer walls.")
            return

        self._total_visited += len(visited)
        self._path           = path
        self._path_index     = 1          # index 0 is start
        self._agent_pos      = self.start

        # Color visited and path
        for cell in visited:
            if cell not in (self.start, self.goal):
                self._cell_colors[cell] = COLOR_VISITED
        for cell in path:
            if cell not in (self.start, self.goal):
                self._cell_colors[cell] = COLOR_PATH

        self.m_algo.set(   f"Algorithm:     {self.algo_var.get()}")
        self.m_heur.set(   f"Heuristic:     {self.heur_var.get()}")
        self._update_metrics()
        self._draw_grid()
        self.status_var.set("Agent moving...")
        self.root.after(MOVE_DELAY, self._step_agent)

    def _step_agent(self):
        """Move agent one step along the current path."""
        if not self._running:
            return

        if self._path_index >= len(self._path):
            # Reached goal
            self._agent_pos = self.goal
            elapsed = (time.time()-self._start_time)*1000
            self.m_time.set(f"Time (ms):     {elapsed:.1f}")
            self.status_var.set("✅ Goal reached!")
            self._running = False
            self._draw_grid()
            return

        # ── Dynamic: maybe spawn a new wall ──
        if self.dynamic_var.get():
            self._try_spawn_obstacle()

        # ── Check if next step is still clear ──
        next_cell = self._path[self._path_index]
        if self.grid[next_cell[0]][next_cell[1]] == "wall":
            # Path is blocked — re-plan from current position
            self._replan()
            return

        # ── Move agent ──
        prev = self._agent_pos
        self._agent_pos = next_cell
        self._path_index += 1

        # Repaint the previous cell (leave it as visited)
        if prev not in (self.start, self.goal):
            self._cell_colors[prev] = COLOR_VISITED
        self._draw_cell(*prev)
        self._draw_cell(*self._agent_pos)

        self._total_cost += 1
        self._update_metrics()
        self.root.after(MOVE_DELAY, self._step_agent)

    def _try_spawn_obstacle(self):
        """Randomly place a new wall somewhere on the grid."""
        if random.random() > SPAWN_PROB:
            return
        # Pick a random empty cell that is NOT on the current planned path or key positions
        forbidden = set(self._path) | {self.start, self.goal, self._agent_pos}
        candidates = [
            (r, c)
            for r in range(GRID_ROWS) for c in range(GRID_COLS)
            if self.grid[r][c] is None and (r, c) not in forbidden
        ]
        if candidates:
            r, c = random.choice(candidates)
            self.grid[r][c] = "wall"
            self._cell_colors.pop((r, c), None)
            self._draw_cell(r, c)

    def _replan(self):
        """Re-calculate path from agent's current position."""
        self._total_replans += 1
        algo_fn, h_fn = self._get_algo_and_heuristic()
        new_path, visited = algo_fn(self.grid, self._agent_pos, self.goal, h_fn)

        self._total_visited += len(visited)

        if new_path is None:
            self.status_var.set("⚠️ Path blocked! No re-route possible.")
            self._running = False
            return

        # Clear old path colors, draw new ones
        for cell in self._path:
            if cell not in (self.start, self.goal, self._agent_pos):
                self._cell_colors.pop(cell, None)

        for cell in new_path:
            if cell not in (self.start, self.goal):
                self._cell_colors[cell] = COLOR_PATH

        self._path       = new_path
        self._path_index = 1
        self._update_metrics()
        self._draw_grid()
        self.status_var.set(f"🔄 Re-planned! (×{self._total_replans})")
        self.root.after(MOVE_DELAY, self._step_agent)

    def _update_metrics(self):
        elapsed = (time.time()-self._start_time)*1000
        self.m_visited.set(f"Nodes visited: {self._total_visited}")
        self.m_cost.set(   f"Path cost:     {self._total_cost}")
        self.m_time.set(   f"Time (ms):     {elapsed:.1f}")
        self.m_replans.set(f"Re-plans:      {self._total_replans}")

    # ══════════════════════════════════════
    #  CONTROLS
    # ══════════════════════════════════════
    def _stop(self):
        self._running = False
        self.status_var.set("Stopped.")

    def _on_click(self, event):
        if self._running:
            return    # don't allow editing while agent is moving
        c=(event.x-PADDING)//CELL_SIZE; r=(event.y-PADDING)//CELL_SIZE
        if not(0<=r<GRID_ROWS and 0<=c<GRID_COLS): return
        if (r,c) in (self.start, self.goal): return
        self.grid[r][c] = None if self.grid[r][c]=="wall" else "wall"
        self._draw_cell(r, c)

    def _reset_grid(self):
        self._running = False
        self._agent_pos = None
        self.grid = [[None]*GRID_COLS for _ in range(GRID_ROWS)]
        self._cell_colors.clear()
        self._path = []; self._path_index = 0
        self._total_visited = self._total_cost = self._total_replans = 0
        for v, default in [
            (self.m_algo,   "Algorithm:     —"),
            (self.m_heur,   "Heuristic:     —"),
            (self.m_visited,"Nodes visited: —"),
            (self.m_cost,   "Path cost:     —"),
            (self.m_time,   "Time (ms):     —"),
            (self.m_replans,"Re-plans:      —"),
        ]:
            v.set(default)
        self.status_var.set("Grid reset.")
        self._draw_grid()

    def _random_walls(self):
        self._reset_grid()
        for r in range(GRID_ROWS):
            for c in range(GRID_COLS):
                if (r,c) not in (self.start,self.goal) and random.random()<0.25:
                    self.grid[r][c]="wall"
        self._draw_grid()


# ─────────────────────────────────────────
#  ENTRY POINT
# ─────────────────────────────────────────
if __name__ == "__main__":
    root = tk.Tk()
    PathfinderApp(root)
    root.mainloop()
