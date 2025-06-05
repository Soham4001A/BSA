# Grid-Based Pathfinding: A* with Beam Search - (BSA)

This project explores and compares two fundamental pathfinding algorithms, A* Search and a specialized Beam Search (which we'll call BSA - Beam Search with A*-style pruning), for navigating 2D grids. The focus is on understanding their mechanics, performance trade-offs, and potential applications, especially in scenarios mimicking real-world challenges like large maps with varying obstacle densities.

## Table of Contents
1.  [Introduction: The Pathfinding Challenge](#introduction)
2.  [Core Pathfinding Concepts](#core-concepts)
    *   [The Grid & Nodes](#the-grid--nodes)
    *   [Key Metrics: `g(n)`, `h(n)`, `f(n)`](#key-metrics-gn-hn-fn)
3.  [Algorithm Deep Dive](#algorithm-deep-dive)
    *   [A* Search: The Optimal Pathfinder](#a-search-the-optimal-pathfinder)
        *   [How it Works](#how-a-works)
        *   [Why it's Good (and its Limits)](#why-a-is-good)
    *   [Beam Search with A*-Style Pruning (BSA): The Efficient Approximator](#beam-search-with-a-style-pruning-bsa-the-efficient-approximator)
        *   [How it Works](#how-bsa-works)
        *   [The Beam Width (W) Trade-off](#the-beam-width-w-trade-off)
4.  [Interpreting Our Results: A* vs. BSA](#interpreting-our-results-a-vs-bsa)
    *   [Key Observations](#key-observations)
    *   [The "Sweet Spot": Sparsely Obstructed Large Maps](#the-sweet-spot-sparsely-obstructed-large-maps)
5.  [Real-World Implications & System Design Thoughts](#real-world-implications--system-design-thoughts)
    *   [The "Quick First Path, Refine Later" Strategy](#the-quick-first-path-refine-later-strategy)
    *   [Applications in Robotics (ROS) and Mapping](#applications-in-robotics-ros-and-mapping)
6.  [Technical Details (For the Curious)](#technical-details-for-the-curious)
    *   [Procedural Obstacle Generation](#procedural-obstacle-generation)
    *   [Complexity Analysis (Time & Space)](#complexity-analysis-time--space)

---

## 1. Introduction: The Pathfinding Challenge

Finding the best path from a start to a goal is a classic problem. In this project, our "world" is a grid, which can be vast (e.g., 500,000x500,000 cells). Obstacles can block paths, and we want to find efficient routes. We explore two main algorithms:
*   **A* Search:** Known for finding the *shortest* possible path.
*   **Beam Search with A*-Style Pruning (BSA):** A faster algorithm that finds a *good*, but not always the shortest, path by limiting its search.

---

## 2. Core Pathfinding Concepts

### The Grid & Nodes
*   **Grid:** Our map is a 2D grid. Movement is to adjacent cells (up, down, left, right).
*   **Nodes (`n`):** Each cell `(row, col)` can be a node in our search.

### Key Metrics: `g(n)`, `h(n)`, `f(n)`
To make smart decisions, our algorithms use three scores for each node `n`:

1.  **`g(n)` - Cost from Start:**
    *   The *actual* cost of the path found so far from the starting node to node `n`.
    *   `g(successor) = g(current) + cost_to_move_to_successor`

2.  **`h(n)` - Heuristic Estimate to Goal:**
    *   An *educated guess* of the cost from node `n` to the goal.
    *   It must be **admissible**: never overestimate the true cost.
    *   We use **Manhattan Distance**: `h((x₁, y₁), (x₂, y₂)) = |x₁ - x₂| + |y₁ - y₂|`
        *This is perfect for grids where you can't move diagonally.*

3.  **`f(n)` - Total Estimated Cost:**
    *   The core of A* and BSA's decision-making.
    *   **`f(n) = g(n) + h(n)`**
    *   This estimates the total cost of a path from start to goal *if it goes through node `n`*. Algorithms prioritize nodes with lower `f(n)` values.

---

## 3. Algorithm Deep Dive

### A* Search: The Optimal Pathfinder

A* is celebrated for its ability to find the shortest path if one exists.

#### How A* Works
1.  **Open Set (Priority Queue):** Keeps track of nodes discovered but not yet fully evaluated, ordered by the lowest `f(n)`.
2.  **Closed Set (Visited List):** Stores nodes already evaluated to avoid re-processing.
3.  **Loop:**
    a.  Pick the node `n` with the lowest `f(n)` from the Open Set.
    b.  If `n` is the goal, reconstruct the path (done!).
    c.  Move `n` to the Closed Set.
    d.  For each neighbor `m` of `n`:
        *   If `m` is already processed (in Closed Set) or a wall, ignore it.
        *   Calculate `f(m)`. If this path to `m` is better than any previous, update `m`'s scores and parent, then add/update `m` in the Open Set.

#### Why A* is Good (and its Limits)
*   **Optimal:** Guarantees the shortest path (with an admissible heuristic).
*   **Complete:** Will find a path if one exists (given enough resources).
*   **Limits:** Can be slow and memory-hungry on very large maps because the Open and Closed Sets can grow very large.

### Beam Search with A*-Style Pruning (BSA): The Efficient Approximator

BSA is designed for speed and efficiency, especially when an absolutely perfect path isn't critical.

#### How BSA Works
1.  **Beam (Fixed-Size List):** At each step, BSA only keeps track of a small, fixed number (`W`, the "beam width") of the most promising nodes.
2.  **A*-Style Evaluation:** It uses the same `f(n) = g(n) + h(n)` to decide which nodes are "most promising."
3.  **Level-by-Level Expansion & Pruning:**
    a.  Take all nodes currently in the beam.
    b.  Generate all their valid neighbors (these are the `candidates`).
    c.  Calculate `f(n)` for all `candidates`.
    d.  **Pruning:** Sort all `candidates` by their `f(n)`. Keep only the top `W` candidates to form the *new* beam for the next step. Discard all others.
    e.  Repeat until the goal is found or the beam becomes empty.
4.  **`visited_g_costs`:** Tracks the best `g(n)` found to reach any state by BSA, preventing cycles and redundant work on states already reached more cheaply *within the beam search context*.

#### The Beam Width (`W`) Trade-off
*   **Small `W` (e.g., 1, 8):** Very fast, low memory. More likely to find a suboptimal path or miss a path because it might prune away the *actual* best route too early.
*   **Large `W` (e.g., 32, 64):** Slower, more memory. Path quality improves, getting closer to A*. If `W` is huge, BSA starts to resemble A* but without its optimality guarantees.

---

## 4. Interpreting Our Results: A* vs. BSA

Our experiments (see `pathfinding_results.csv` and `pathfinding_verbose_log.txt`) reveal key characteristics:

### Key Observations
1.  **A\*'s Optimality Confirmed:** When A\* found a path, its "Path Score (Cost)" was consistently the best (lowest).
2.  **BSA's Speed Advantage:** For smaller beam widths (e.g., W=8), BSA was often significantly faster (lower "Wall Clock Time") and processed fewer nodes ("Nodes Expanded from Beam") than A\*, especially on longer paths (e.g., Scenarios 4, 8, 12).
3.  **The Beam Width Effect:**
    *   Increasing beam width in BSA generally improved path quality (scores got closer to A\*'s).
    *   This improvement came at the cost of increased time and nodes processed by BSA. Sometimes, a very wide beam made BSA slower than A\* (e.g., Scenario 4, W=64).
4.  **Handling Obstacles:**
    *   **Very Dense/Impassable (Scenarios 6, 7):** Both algorithms quickly (and correctly) determined no path was readily available from the start.
    *   **Challenging but Passable (Scenario 5):** A\* hit its node limit, exploring extensively. BSA (W=8, W=16) failed very quickly, suggesting it pruned away viable but complex paths. This highlights BSA's incompleteness – it might miss a solution A\* could find.
    *   **Short, Dense Path (Scenario 11):** Even with high density, both algorithms (including BSA with various widths) found the optimal short path, likely because the path wasn't complex enough to fool BSA's pruning.

### The "Sweet Spot": Sparsely Obstructed Large Maps (5-20% Density)
*   **Scenarios like #4 (15% density, long path) and #8 (5% density, very long path) are particularly interesting.**
    *   A\* found optimal paths but took considerable time/nodes.
    *   BSA (W=8) found paths that were only slightly suboptimal (1-4% longer scores) but was significantly faster (e.g., S4: 0.1s for BSA vs 0.44s for A\*; S8: 0.25s for BSA vs 1.5s for A\*).
*   This suggests that for large maps with a realistic amount of "clutter" (not completely open, not a dense maze), BSA can provide a very good initial path much quicker than A\*.

---

## 5. Real-World Implications & System Design Thoughts

### The "Quick First Path, Refine Later" Strategy
Your insight is key: the behavior of BSA, especially on large maps with moderate obstacles, aligns well with a common system design pattern:

1.  **Initial, Fast Approximation:** Use a fast algorithm like BSA (with a relatively small beam width) to get a "good enough" path very quickly. This is crucial for responsive systems.
    *   For a robot (ROS), this means it can start moving in generally the right direction almost immediately.
    *   For a mapping service, it can display an initial route quickly.
2.  **Background Refinement:** While the system is using the initial path, a more computationally intensive algorithm (like A\* or BSA with a much larger beam width, or even other specialized route optimization algorithms) can run in the background to find a truly optimal or significantly better path.
3.  **Dynamic Updates:** If a better path is found, the system can then seamlessly switch to it.

This tiered approach balances responsiveness with solution quality.

### Applications in Robotics (ROS) and Mapping
*   **Robotics (ROS):** A robot needs to react quickly to its environment. Waiting for a perfect A\* on a large, dynamic map might be too slow. BSA could provide immediate motion commands, while a global A\* planner refines the long-term route or a local planner handles immediate obstacle avoidance.
*   **Mapping Services (like Google Maps):** When you request directions, the scale is immense. They likely use hierarchical approaches, precomputed data, and very fast heuristics. An algorithm like BSA could be part of finding initial candidate routes over segments of the map, which are then stitched together and further optimized. The sheer scale means true A\* over the entire planet's road network is infeasible in real-time for every query. They employ many layers of abstraction and heuristics.

Your "BSA" (Beam Search with A*-style pruning) demonstrates a core principle: **heuristic, approximate search is vital for tractability in complex, large-scale problems.**

---

## 6. Technical Details (For the Curious)

### Procedural Obstacle Generation
To simulate vast environments without storing massive grids, obstacles are generated "on-the-fly" using a deterministic pseudo-random function based on cell coordinates and a scenario seed. This ensures consistent maps for fair algorithm comparison.
*   `is_obstacle = PRNG(hash(row, col, scenario_seed)) < obstacle_density`

### Complexity Analysis (Time & Space)
(Brief summary, see previous detailed version for full equations)

*   **A\* Search:**
    *   **Time:** Roughly `O(V log V)` in typical grid scenarios (where `V` is nodes visited). Highly dependent on heuristic quality.
    *   **Space:** `O(V)` to store Open and Closed sets. Can be prohibitive.
*   **Beam Search (BSA):**
    *   Let `W` = beam width, `b` = branching factor, `D` = path depth.
    *   **Time:** Roughly `O(D * W * b * log(W*b))`.
    *   **Space:** Roughly `O(W*b + Size_of_Visited_Set)`. Significantly more memory-efficient than A* if `W` is small.

This project highlights the practical trade-offs between optimality, speed, and resource usage in pathfinding, offering insights applicable to many real-world AI and robotics challenges.