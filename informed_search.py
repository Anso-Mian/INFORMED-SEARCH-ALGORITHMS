# Version 2 - Greedy Best-First Search (GBFS) Added
# AI Dynamic Pathfinding Agent
# Created by: Ans Rizwan (24F-0785) & Muhammad Shaheer (24F-0779)
# This version adds GBFS using Manhattan heuristic

import tkinter as tk
import heapq
import random
import time

# ─────────────────────────────────────────
#  SETTINGS
# ─────────────────────────────────────────
GRID_ROWS  = 15
GRID_COLS  = 15
CELL_SIZE  = 40
PADDING    = 10
ANIM_DELAY = 30        # milliseconds between animation frames

COLOR_EMPTY    = "#FFFFFF"
COLOR_WALL     = "#2C3E50"
COLOR_START    = "#27AE60"
COLOR_GOAL     = "#E74C3C"
COLOR_FRONTIER = "#F1C40F"
COLOR_VISITED  = "#3498DB"
COLOR_PATH     = "#2ECC71"


# ─────────────────────────────────────────
#  HEURISTIC
# ─────────────────────────────────────────
def manhattan(a, b):
    """h(n) = |r1-r2| + |c1-c2|"""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


# ─────────────────────────────────────────
#  GBFS ALGORITHM
# ─────────────────────────────────────────
def gbfs(grid, start, goal):
    """
    Greedy Best-First Search
    f(n) = h(n)  — only looks at heuristic, ignores actual cost
    Returns: (path, visited_order, frontier_snapshots)
    """
    # Priority queue entries: (h_value, node)
    open_heap = []
    heapq.heappush(open_heap, (manhattan(start, goal), start))

    came_from  = {start: None}    # to reconstruct path
    visited    = []               # order of expansion (for animation)
    frontiers  = []               # snapshot of frontier at each step

    while open_heap:
        _, current = heapq.heappop(open_heap)

        if current == goal:
            # Reconstruct path
            path = []
            node = goal
            while node is not None:
                path.append(node)
                node = came_from[node]
            path.reverse()
            return path, visited, frontiers

        visited.append(current)

        # Explore 4-directional neighbors (up, down, left, right)
        r, c = current
        for dr, dc in [(-1,0),(1,0),(0,-1),(0,1)]:
            nr, nc = r + dr, c + dc
            neighbor = (nr, nc)

            # Skip out-of-bounds, walls, already visited
            if not (0 <= nr < GRID_ROWS and 0 <= nc < GRID_COLS):
                continue
            if grid[nr][nc] == "wall":
                continue
            if neighbor in came_from:
                continue

            came_from[neighbor] = current
            h = manhattan(neighbor, goal)
            heapq.heappush(open_heap, (h, neighbor))

        # Save frontier snapshot (just the nodes, not priorities)
        frontiers.append([item[1] for item in open_heap])

    return None, visited, frontiers   # No path found


# ─────────────────────────────────────────
#  GUI APPLICATION
# ─────────────────────────────────────────
class PathfinderApp:

    def __init__(self, root):
        self.root = root
        self.root.title("AI Dynamic Pathfinder  –  v2 GBFS  –  24F-0785 & 24F-0779")
        self.root.resizable(False, False)

        self.grid  = [[None]*GRID_COLS for _ in range(GRID_ROWS)]
        self.start = (0, 0)
        self.goal  = (GRID_ROWS-1, GRID_COLS-1)

        # Animation state
        self._anim_steps  = []
        self._anim_index  = 0
        self._path_cells  = []
        self._cell_colors = {}    # (r,c) -> color override

        self._build_ui()
        self._draw_grid()

    # ── UI Layout ─────────────────────────────────────────────────────────
    def _build_ui(self):
        cw = GRID_COLS * CELL_SIZE + 2 * PADDING
        ch = GRID_ROWS * CELL_SIZE + 2 * PADDING

        self.canvas = tk.Canvas(self.root, width=cw, height=ch,
                                bg="#ECF0F1", highlightthickness=0)
        self.canvas.grid(row=0, column=0, padx=10, pady=10)
        self.canvas.bind("<Button-1>", self._on_click)

        ctrl = tk.Frame(self.root, bg="#ECF0F1", padx=10, pady=10)
        ctrl.grid(row=0, column=1, sticky="n")

        tk.Label(ctrl, text="AI Pathfinder", font=("Arial", 14, "bold"),
                 bg="#ECF0F1").pack(pady=(0, 10))

        tk.Button(ctrl, text="▶  Run GBFS", width=18,
                  bg="#27AE60", fg="white", font=("Arial", 10, "bold"),
                  command=self._run_gbfs).pack(pady=4)

        tk.Button(ctrl, text="Random Walls", width=18,
                  bg="#8E44AD", fg="white", font=("Arial", 10, "bold"),
                  command=self._random_walls).pack(pady=4)

        tk.Button(ctrl, text="Reset Grid", width=18,
                  bg="#E74C3C", fg="white", font=("Arial", 10, "bold"),
                  command=self._reset_grid).pack(pady=4)

        # Status label
        self.status_var = tk.StringVar(value="Draw walls, then run GBFS.")
        tk.Label(ctrl, textvariable=self.status_var, bg="#ECF0F1",
                 font=("Arial", 9), wraplength=160, justify="left",
                 fg="#2C3E50").pack(pady=(10, 0))

        # Metrics
        tk.Label(ctrl, text="Metrics", font=("Arial", 11, "bold"),
                 bg="#ECF0F1").pack(pady=(20, 5))

        self.metric_visited  = tk.StringVar(value="Nodes visited: —")
        self.metric_cost     = tk.StringVar(value="Path cost: —")
        self.metric_time     = tk.StringVar(value="Time (ms): —")

        for var in (self.metric_visited, self.metric_cost, self.metric_time):
            tk.Label(ctrl, textvariable=var, bg="#ECF0F1",
                     font=("Arial", 9), anchor="w").pack(anchor="w")

        # Legend
        tk.Label(ctrl, text="Legend", font=("Arial", 11, "bold"),
                 bg="#ECF0F1").pack(pady=(20, 5))

        for color, label in [
            (COLOR_START,    "Start"),
            (COLOR_GOAL,     "Goal"),
            (COLOR_WALL,     "Wall"),
            (COLOR_FRONTIER, "Frontier"),
            (COLOR_VISITED,  "Visited"),
            (COLOR_PATH,     "Final path"),
        ]:
            row = tk.Frame(ctrl, bg="#ECF0F1")
            row.pack(anchor="w")
            tk.Label(row, bg=color, width=2, relief="solid").pack(side="left", padx=(0,6))
            tk.Label(row, text=label, bg="#ECF0F1", font=("Arial", 9)).pack(side="left")

    # ── Draw helpers ──────────────────────────────────────────────────────
    def _draw_grid(self):
        self.canvas.delete("all")
        for r in range(GRID_ROWS):
            for c in range(GRID_COLS):
                self._draw_cell(r, c)

    def _draw_cell(self, r, c, override_color=None):
        x1 = PADDING + c * CELL_SIZE
        y1 = PADDING + r * CELL_SIZE
        x2, y2 = x1 + CELL_SIZE, y1 + CELL_SIZE

        if override_color:
            color, text = override_color, ""
        elif (r, c) == self.start:
            color, text = COLOR_START, "S"
        elif (r, c) == self.goal:
            color, text = COLOR_GOAL, "G"
        elif self.grid[r][c] == "wall":
            color, text = COLOR_WALL, ""
        elif (r, c) in self._cell_colors:
            color, text = self._cell_colors[(r, c)], ""
        else:
            color, text = COLOR_EMPTY, ""

        self.canvas.create_rectangle(x1, y1, x2, y2,
                                     fill=color, outline="#BDC3C7")
        if text:
            self.canvas.create_text((x1+x2)//2, (y1+y2)//2,
                                    text=text, font=("Arial",12,"bold"),
                                    fill="white")

    # ── Run GBFS ──────────────────────────────────────────────────────────
    def _run_gbfs(self):
        self._cell_colors.clear()
        self._draw_grid()

        t0   = time.time()
        path, visited, frontiers = gbfs(self.grid, self.start, self.goal)
        elapsed = (time.time() - t0) * 1000

        if path is None:
            self.status_var.set("No path found!")
            return

        # Update metrics
        self.metric_visited.set(f"Nodes visited: {len(visited)}")
        self.metric_cost.set(f"Path cost: {len(path)-1}")
        self.metric_time.set(f"Time (ms): {elapsed:.2f}")
        self.status_var.set("GBFS complete!")

        # Animate visited + frontier, then draw path
        self._anim_steps = list(zip(visited, frontiers + [[]]))
        self._path_cells = path
        self._anim_index = 0
        self._animate()

    def _animate(self):
        if self._anim_index < len(self._anim_steps):
            node, frontier = self._anim_steps[self._anim_index]

            if node not in (self.start, self.goal):
                self._cell_colors[node] = COLOR_VISITED
                self._draw_cell(*node)

            for fn in frontier:
                if fn not in (self.start, self.goal) and fn not in self._cell_colors:
                    self._cell_colors[fn] = COLOR_FRONTIER
                    self._draw_cell(*fn)

            self._anim_index += 1
            self.root.after(ANIM_DELAY, self._animate)
        else:
            # Draw final path
            for cell in self._path_cells:
                if cell not in (self.start, self.goal):
                    self._cell_colors[cell] = COLOR_PATH
                    self._draw_cell(*cell)

    # ── Interaction ───────────────────────────────────────────────────────
    def _on_click(self, event):
        c = (event.x - PADDING) // CELL_SIZE
        r = (event.y - PADDING) // CELL_SIZE
        if not (0 <= r < GRID_ROWS and 0 <= c < GRID_COLS):
            return
        if (r, c) in (self.start, self.goal):
            return
        self.grid[r][c] = None if self.grid[r][c] == "wall" else "wall"
        self._draw_cell(r, c)

    def _reset_grid(self):
        self.grid = [[None]*GRID_COLS for _ in range(GRID_ROWS)]
        self._cell_colors.clear()
        self.metric_visited.set("Nodes visited: —")
        self.metric_cost.set("Path cost: —")
        self.metric_time.set("Time (ms): —")
        self.status_var.set("Grid reset.")
        self._draw_grid()

    def _random_walls(self):
        self._reset_grid()
        for r in range(GRID_ROWS):
            for c in range(GRID_COLS):
                if (r, c) not in (self.start, self.goal) and random.random() < 0.25:
                    self.grid[r][c] = "wall"
        self._draw_grid()


if __name__ == "__main__":
    root = tk.Tk()
    PathfinderApp(root)
    root.mainloop()