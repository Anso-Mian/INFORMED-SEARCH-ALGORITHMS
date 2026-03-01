# 🤖 AI Dynamic Pathfinding Agent

> A real-time grid-based pathfinding visualizer implementing Informed Search Algorithms with dynamic obstacle spawning and automatic re-planning.

**Created by:** Ans Rizwan (24F-0785) & Muhammad Shaheer (24F-0779)
**Course:** Artificial Intelligence — Assignment #01

---

## 📌 What This Does

This program visualizes how AI search algorithms find a path from a **Start node** to a **Goal node** on a 15×15 grid. You can watch the agent explore the grid step by step, toggle between algorithms and heuristics, and enable **Dynamic Mode** where new walls spawn while the agent is moving — forcing it to re-plan its route in real time.

---

## ✨ Features

- **GBFS** — Greedy Best-First Search using only the heuristic f(n) = h(n)
- **A\*** — A* Search using f(n) = g(n) + h(n) for guaranteed optimal paths
- **Manhattan & Euclidean** heuristic toggle
- **Dynamic Mode** — obstacles spawn randomly while the agent moves
- **Auto Re-planning** — agent recalculates path instantly if blocked
- **Live Metrics Dashboard** — nodes visited, path cost, time (ms), re-plan count
- **Animated visualization** — frontier (yellow), visited (blue), path (green), agent (purple)
- Click cells to manually draw or erase walls

---

## 🛠️ Installation

### Requirements

- Python 3.8 or higher
- Tkinter (comes built-in with Python — no install needed)

### Steps

**1. Clone the repository**

```bash
git clone https://github.com/Anso-Mian/THE-AI-PATHFINDER.git
cd THE-AI-PATHFINDER
```

**2. No dependencies to install!**

This project uses only Python's standard library. You do not need to run any pip install command.

If for any reason Tkinter is missing on your system (rare on Linux), install it with:

```bash
# Ubuntu / Debian
sudo apt-get install python3-tk

# Fedora
sudo dnf install python3-tkinter
```

---

## ▶️ How to Run

Run the final version of the program:

```bash
python informed_search.py
```

A window will open with the grid and control panel.

---

## 🎮 How to Use

1. **Draw walls** — click any cell on the grid to place or remove a wall
2. **Random Walls** — click the button to scatter random obstacles automatically
3. **Select Algorithm** — choose GBFS or A* using the radio buttons
4. **Select Heuristic** — choose Manhattan or Euclidean using the radio buttons
5. **Enable Dynamic Mode** — tick the checkbox if you want walls to spawn while the agent moves
6. **Press Start** — the agent will find a path and animate its movement
7. **Press Stop** — halts the agent mid-journey
8. **Press Reset Grid** — clears everything and starts fresh

---

---

## 🎨 Color Guide

| Color | Meaning |
|-------|---------|
| 🟩 Green | Start node (S) |
| 🟥 Red | Goal node (G) |
| ⬛ Dark | Wall / obstacle |
| 🟨 Yellow | Frontier — nodes in the priority queue |
| 🟦 Blue | Visited — fully expanded nodes |
| 💚 Bright Green | Final path from S to G |
| 🟣 Purple | Agent's current position |

---

## 📊 Metrics Explained

| Metric | Description |
|--------|-------------|
| Algorithm | Which algorithm is running |
| Heuristic | Manhattan or Euclidean |
| Nodes Visited | Total nodes expanded across all planning calls |
| Path Cost | Number of steps the agent has actually taken |
| Time (ms) | Wall-clock time from Start to goal reached |
| Re-plans | How many times the agent had to recalculate mid-journey |

---

## 🧠 Algorithms

**Greedy Best-First Search (GBFS)**
Uses only the heuristic to decide which node to expand next. Fast but not guaranteed to find the shortest path.

**A* Search**
Combines actual cost g(n) with heuristic h(n). Slower than GBFS but always finds the optimal (shortest) path when using an admissible heuristic.

**Manhattan Distance**
h(n) = |x1 - x2| + |y1 - y2| — best for 4-directional grid movement.

**Euclidean Distance**
h(n) = sqrt((x1-x2)^2 + (y1-y2)^2) — straight-line distance, also admissible.

---

## 👥 Authors

| Name | Student ID |
|------|-----------|
| Ans Rizwan | 24F-0779 |

---

## 📄 License

Created for educational purposes as part of an Artificial Intelligence course assignment.
