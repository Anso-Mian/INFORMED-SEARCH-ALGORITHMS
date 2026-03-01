# Version 1 - Grid and GUI Setup
# AI Dynamic Pathfinding Agent
# Created by: Ans Rizwan (24F-0785) & Muhammad Shaheer (24F-0779)
# This version sets up the basic grid and GUI using Tkinter

import tkinter as tk
from tkinter import ttk
import random

# ─────────────────────────────────────────
#  SETTINGS  (easy to change)
# ─────────────────────────────────────────
GRID_ROWS   = 15          # number of rows
GRID_COLS   = 15          # number of columns
CELL_SIZE   = 40          # pixel size of each cell
PADDING     = 10          # padding around the grid

# Colors for each type of cell
COLOR_EMPTY    = "#FFFFFF"   # white  – empty cell
COLOR_WALL     = "#2C3E50"   # dark   – obstacle / wall
COLOR_START    = "#27AE60"   # green  – start node
COLOR_GOAL     = "#E74C3C"   # red    – goal node
COLOR_FRONTIER = "#F1C40F"   # yellow – nodes in priority queue
COLOR_VISITED  = "#3498DB"   # blue   – already explored nodes
COLOR_PATH     = "#2ECC71"   # bright green – final path


class PathfinderApp:
    """Main application window."""

    def __init__(self, root):
        self.root = root
        self.root.title("AI Dynamic Pathfinder  –  24F-0785 & 24F-0779")
        self.root.resizable(False, False)

        # Grid state: None = empty, 'wall' = obstacle
        self.grid = [[None for _ in range(GRID_COLS)] for _ in range(GRID_ROWS)]

        # Start and goal positions (row, col)
        self.start = (0, 0)
        self.goal  = (GRID_ROWS - 1, GRID_COLS - 1)

        self._build_ui()
        self._draw_grid()

    # ── Build the full UI layout ──────────────────────────────────────────
    def _build_ui(self):
        # Left: canvas for the grid
        canvas_width  = GRID_COLS * CELL_SIZE + 2 * PADDING
        canvas_height = GRID_ROWS * CELL_SIZE + 2 * PADDING

        self.canvas = tk.Canvas(
            self.root,
            width=canvas_width,
            height=canvas_height,
            bg="#ECF0F1",
            highlightthickness=0
        )
        self.canvas.grid(row=0, column=0, padx=10, pady=10)

        # Bind mouse click to place/remove walls
        self.canvas.bind("<Button-1>", self._on_click)

        # Right: control panel
        ctrl = tk.Frame(self.root, bg="#ECF0F1", padx=10, pady=10)
        ctrl.grid(row=0, column=1, sticky="n")

        tk.Label(ctrl, text="AI Pathfinder", font=("Arial", 14, "bold"),
                 bg="#ECF0F1").pack(pady=(0, 10))

        # Reset button
        tk.Button(ctrl, text="Reset Grid", width=18, bg="#E74C3C", fg="white",
                  font=("Arial", 10, "bold"),
                  command=self._reset_grid).pack(pady=4)

        # Random walls button
        tk.Button(ctrl, text="Random Walls", width=18, bg="#8E44AD", fg="white",
                  font=("Arial", 10, "bold"),
                  command=self._random_walls).pack(pady=4)

        # Info label
        tk.Label(ctrl, text="Click cells to\ntoggle walls",
                 bg="#ECF0F1", font=("Arial", 9), fg="#7F8C8D").pack(pady=(10, 0))

        # Legend
        tk.Label(ctrl, text="Legend", font=("Arial", 11, "bold"),
                 bg="#ECF0F1").pack(pady=(20, 5))

        legends = [
            (COLOR_START,    "Start node"),
            (COLOR_GOAL,     "Goal node"),
            (COLOR_WALL,     "Wall / obstacle"),
            (COLOR_FRONTIER, "Frontier"),
            (COLOR_VISITED,  "Visited"),
            (COLOR_PATH,     "Final path"),
        ]
        for color, label in legends:
            row = tk.Frame(ctrl, bg="#ECF0F1")
            row.pack(anchor="w")
            tk.Label(row, bg=color, width=2, relief="solid").pack(side="left", padx=(0, 6))
            tk.Label(row, text=label, bg="#ECF0F1", font=("Arial", 9)).pack(side="left")

    # ── Draw every cell on the canvas ────────────────────────────────────
    def _draw_grid(self):
        self.canvas.delete("all")
        for r in range(GRID_ROWS):
            for c in range(GRID_COLS):
                self._draw_cell(r, c)

    def _draw_cell(self, r, c):
        """Draw a single cell with the correct color."""
        x1 = PADDING + c * CELL_SIZE
        y1 = PADDING + r * CELL_SIZE
        x2 = x1 + CELL_SIZE
        y2 = y1 + CELL_SIZE

        # Decide color
        if (r, c) == self.start:
            color = COLOR_START
            text  = "S"
        elif (r, c) == self.goal:
            color = COLOR_GOAL
            text  = "G"
        elif self.grid[r][c] == "wall":
            color = COLOR_WALL
            text  = ""
        else:
            color = COLOR_EMPTY
            text  = ""

        self.canvas.create_rectangle(x1, y1, x2, y2,
                                     fill=color, outline="#BDC3C7", width=1)
        if text:
            self.canvas.create_text((x1 + x2) // 2, (y1 + y2) // 2,
                                    text=text, font=("Arial", 12, "bold"),
                                    fill="white")

    # ── Mouse click: toggle wall ──────────────────────────────────────────
    def _on_click(self, event):
        c = (event.x - PADDING) // CELL_SIZE
        r = (event.y - PADDING) // CELL_SIZE

        if not (0 <= r < GRID_ROWS and 0 <= c < GRID_COLS):
            return
        if (r, c) in (self.start, self.goal):
            return

        # Toggle wall on/off
        self.grid[r][c] = None if self.grid[r][c] == "wall" else "wall"
        self._draw_cell(r, c)

    # ── Reset the entire grid ─────────────────────────────────────────────
    def _reset_grid(self):
        self.grid = [[None for _ in range(GRID_COLS)] for _ in range(GRID_ROWS)]
        self._draw_grid()

    # ── Scatter random walls ──────────────────────────────────────────────
    def _random_walls(self):
        self._reset_grid()
        for r in range(GRID_ROWS):
            for c in range(GRID_COLS):
                if (r, c) in (self.start, self.goal):
                    continue
                if random.random() < 0.25:          # 25 % chance of wall
                    self.grid[r][c] = "wall"
        self._draw_grid()


# ─────────────────────────────────────────
#  Entry point
# ─────────────────────────────────────────
if __name__ == "__main__":
    root = tk.Tk()
    app  = PathfinderApp(root)
    root.mainloop()