# A* vs. Beam Search (BSA): An Exploration in Grid-Based Pathfinding

This project investigates and compares two fundamental pathfinding algorithms—A* Search and a specialized Beam Search (BSA) that uses A*-style pruning—for navigating large 2D grid environments. It delves into their mechanics, performance trade-offs, and suitability for challenges like pathfinding on large maps with varied obstacle densities. The findings are particularly relevant for understanding how these algorithms can be applied in systems requiring both speed and solution quality, such as robotics or mapping applications.

**Quick Links:**
*   [What's This Project About?](#1-whats-this-project-about)
*   [The Algorithms: A Quick Look](#2-the-algorithms-a-quick-look)
*   [Key Takeaways from Our Experiments](#3-key-takeaways-from-our-experiments)
*   [Thinking Bigger: Real-World Uses](#4-thinking-bigger-real-world-uses)
*   [Dive Deeper: Technical Details](#5-dive-deeper-technical-details)

---

## 1. What's This Project About?

Finding the best route from a starting point to a destination on a map is a classic challenge. This project explores this challenge in the context of a 2D grid, which can represent anything from a game level to a segment of a real-world map. We're particularly interested in very large grids (think 500,000x500,000 cells!) where efficiency is paramount.

We compare two main approaches:
*   **A* Search:** Famous for always finding the *shortest* path.
*   **Beam Search with A*-Style Pruning (BSA):** A modified Beam Search designed to be faster by intelligently limiting its search, often finding a *good* (but not always the absolute shortest) path.

The goal is to understand when and why one might be preferred over the other.

---

## 2. The Algorithms: A Quick Look

To navigate, both algorithms rely on a few key ideas:

*   **Grid & Nodes:** The map is a grid; each cell `(row, col)` is a potential "node" or step on a path.
*   **`g(n)` - Cost from Start:** The actual cost (e.g., distance traveled) to reach node `n` from the start.
*   **`h(n)` - Heuristic Estimate to Goal:** An educated guess of the remaining cost from node `n` to the goal. We use the **Manhattan Distance** (`|Δx| + |Δy|`), which is a good estimate for grid movement.
*   **`f(n)` - Total Estimated Cost:** The sum `g(n) + h(n)`. This is the algorithms' primary guide for deciding which nodes look most promising to explore next.

### A* Search: The Optimal Pathfinder
A* meticulously explores the grid to find the guaranteed shortest path.
*   **How it Works:** It uses a priority list (Open Set) to always explore the node with the lowest `f(n)` score first. It also remembers visited nodes (Closed Set) to avoid redundant work.
*   **Pros:** Guaranteed to find the best path.
*   **Cons:** Can be slow and use a lot of memory on very large maps, as it might need to consider many nodes.

### Beam Search with A*-Style Pruning (BSA): The Efficient Approximator
BSA aims for speed by being more selective.
*   **How it Works:**
    1.  It only keeps a fixed number of the most promising paths in its "beam" (defined by `beam_width W`).
    2.  At each step, it expands nodes in the current beam, generating candidate next steps.
    3.  It uses `f(n)` to score these candidates.
    4.  **Crucially, it then prunes this list of candidates down to the best `W` to form the beam for the next step.** All other candidates are discarded.
*   **Pros:** Generally faster and uses less memory than A*, especially with smaller beam widths.
*   **Cons:** Not guaranteed to find the absolute shortest path (because it might prune the best option too early). Path quality depends on the `beam_width W` – wider beams are more thorough but slower.

---

## 3. Key Takeaways from Our Experiments

We ran A* and BSA across many scenarios (different map sizes, obstacle densities, path lengths). The detailed results can be found in `pathfinding_results.csv` and `pathfinding_verbose_log.txt`. Here are the highlights:

*   **A\* Delivers Optimality:** As expected, A\* always found the shortest path when it completed within our limits.
*   **BSA's Speed/Quality Trade-off:**
    *   With a **narrow beam** (e.g., `W=8`), BSA was often much faster than A\* but sometimes produced slightly longer paths.
    *   **Increasing the beam width** for BSA improved its path quality, often matching A\*, but made it slower. Very wide beams could even make BSA slower than A\*.
*   **Performance in Different Environments:**
    *   **Sparse Obstacles (5-20% density):** This is where BSA (small `W`) really shone. It found good paths significantly faster than A\*. For example, on a long path with 15% obstacles, BSA (W=8) was about 4x faster than A\* for a path only 4% longer.
    *   **Dense Obstacles (>40%):**
        *   If the start was completely blocked (e.g., 60-75% density), both algorithms quickly realized no path was available.
        *   In a challenging but potentially passable dense scenario (40% density), A\* searched extensively before hitting its limits. BSA failed much faster, likely pruning away complex but viable routes. This shows BSA might give up too easily in very tricky "maze-like" areas.
*   **Node Limits:** When the goal was extremely far (Scenario 10), both algorithms hit their exploration limits, with BSA often reaching it faster.

---

## 4. Thinking Bigger: Real-World Uses

The way BSA performs—especially its ability to quickly find a decent path in large, moderately cluttered environments—has interesting implications for real-world systems:

### The "Quick First Path, Then Refine" Strategy
For systems that need to be responsive (like robots or mapping apps):
1.  **Get a Fast Initial Path:** Use an algorithm like BSA (with a lean beam width) to rapidly calculate a good-enough initial route. This allows the system to start acting or provide immediate feedback.
2.  **Refine in the Background:** While the system uses this initial path, a more powerful algorithm (like A\*, or BSA with a wider beam) can work in the background to find an even better or optimal route.
3.  **Update if Better:** If the refinement process finds a superior path, the system can switch to it.

This approach balances the need for immediate action with the desire for high-quality solutions.

### Applications:
*   **Robotics (e.g., using ROS):** A robot could use BSA for quick, local navigation decisions or to get an initial global path, while a more thorough planner refines the overall strategy.
*   **Mapping Services:** While not a direct replica, the principles apply. Services dealing with continent-sized road networks use many layers of heuristics and pre-computation. BSA-like approaches could be useful for quickly finding candidate route segments.

This project's BSA demonstrates how heuristic, approximate search methods are vital for making complex pathfinding problems tractable.

---

## 5. Dive Deeper: Technical Details

### Procedural Obstacle Generation
To work with massive grid environments without needing huge amounts of memory, obstacles were generated "on-the-fly." A deterministic function decides if a cell `(row, col)` is an obstacle based on its coordinates, a unique `scenario_seed`, and an `obstacle_density` percentage. This ensures that every algorithm test for a given scenario runs on the exact same conceptual map.

### Complexity Insights (Simplified)
*   **A\* Search:**
    *   **Time:** Can be thought of as roughly proportional to `V log V` (where `V` is the number of cells A* looks at). How many cells it looks at heavily depends on how "good" its heuristic guess is.
    *   **Space:** Needs to remember many of the cells it has seen or plans to see, potentially `O(V)`. This can be a lot!
*   **Beam Search (BSA):**
    *   Let `W` = beam width, `D` = length of the path.
    *   **Time:** Roughly proportional to `D * W * (number of neighbors per cell) * log(W)`.
    *   **Space:** Needs to store the current beam (`W` nodes) and candidates. Much more memory-friendly than A* if `W` is kept small.

---

This exploration aims to provide a clear understanding of the trade-offs involved in choosing a pathfinding algorithm, with practical insights drawn from empirical results.