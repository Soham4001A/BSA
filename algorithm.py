import heapq
import time
import math

# --- Grid and Node Utilities ---
class Node:
    def __init__(self, position, parent=None, g=0, h=0):
        self.position = position  # (row, col) tuple
        self.parent = parent      # Parent Node object
        self.g = g                # Cost from start to current node
        self.h = h                # Heuristic (estimated cost from current node to end)
        self.f = g + h            # Total estimated cost: f = g + h

    def __lt__(self, other):
        # For priority queue: prioritize lower f-score, then lower h-score (tie-breaker)
        if self.f == other.f:
            return self.h < other.h
        return self.f < other.f

    def __eq__(self, other):
        # Nodes are equal if their positions are equal
        return self.position == other.position

    def __hash__(self):
        # Hash based on position for use in sets
        return hash(self.position)

def heuristic_manhattan(a, b):
    """Manhattan distance heuristic: |x1-x2| + |y1-y2|"""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def reconstruct_path(node):
    """Trace back from goal node to start node to get the path"""
    path = []
    current = node
    while current:
        path.append(current.position)
        current = current.parent
    return path[::-1] # Return reversed path (start to goal)

# --- A* Algorithm ---
def a_star_search(grid, start_pos, goal_pos, heuristic_func):
    start_time = time.perf_counter()
    
    start_node = Node(start_pos, None, 0, heuristic_func(start_pos, goal_pos))
    goal_node_target_pos = goal_pos # Store the goal position tuple for comparison
    
    open_set = []  # Priority queue (min-heap) of nodes to visit
    heapq.heappush(open_set, start_node)
    
    # closed_set stores positions of nodes already evaluated
    closed_set = set() 
    
    # g_costs stores the lowest g-cost found so far to reach a position
    g_costs = {start_pos: 0}

    nodes_explored = 0 # Counter for nodes popped from open_set

    while open_set:
        current_node = heapq.heappop(open_set)
        nodes_explored += 1

        if current_node.position == goal_node_target_pos:
            path = reconstruct_path(current_node)
            end_time = time.perf_counter()
            return {
                "path": path,
                "score": current_node.g,
                "time": end_time - start_time,
                "nodes_explored": nodes_explored,
                "algorithm": "A*"
            }

        closed_set.add(current_node.position)

        # Explore neighbors (up, down, left, right)
        for dr, dc in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
            neighbor_pos = (current_node.position[0] + dr, current_node.position[1] + dc)

            # Check bounds
            if not (0 <= neighbor_pos[0] < len(grid) and 0 <= neighbor_pos[1] < len(grid[0])):
                continue
            
            # Check obstacles (1 means obstacle)
            if grid[neighbor_pos[0]][neighbor_pos[1]] == 1:
                continue

            # If neighbor is in closed_set, skip (optimal path already found for consistent heuristic)
            if neighbor_pos in closed_set:
                continue
            
            # Cost to move to neighbor is 1 (can be changed for weighted edges)
            tentative_g_score = current_node.g + 1 

            if tentative_g_score < g_costs.get(neighbor_pos, float('inf')):
                # This path to neighbor is better than any previous one found
                g_costs[neighbor_pos] = tentative_g_score
                h_score = heuristic_func(neighbor_pos, goal_node_target_pos)
                neighbor_node = Node(neighbor_pos, current_node, tentative_g_score, h_score)
                heapq.heappush(open_set, neighbor_node)
                
    end_time = time.perf_counter()
    # No path found
    return {
        "path": [],
        "score": float('inf'),
        "time": end_time - start_time,
        "nodes_explored": nodes_explored,
        "algorithm": "A*"
    }

# --- Beam Search with A*-style Pruning ---
def beam_search_astar_pruning(grid, start_pos, goal_pos, heuristic_func, beam_width):
    start_time = time.perf_counter()

    start_node = Node(start_pos, None, 0, heuristic_func(start_pos, goal_pos))
    goal_node_target_pos = goal_pos

    current_beam = [start_node] # List of nodes in the current beam
    
    # visited_g_costs stores the best g-score found to reach a position by the beam search so far
    # This prevents exploring redundant or worse paths to already visited states.
    visited_g_costs = {start_pos: start_node.g} 
    
    nodes_expanded_total = 0 # Counts nodes from which successors are generated

    # Limit search depth to avoid infinite loops in case of no solution or tricky maps
    max_depth = len(grid) * len(grid[0]) 

    for _ in range(max_depth): # Loop for each level/depth
        if not current_beam:
            break # Beam is empty, no path found

        candidates = [] # Potential nodes for the next beam
        nodes_expanded_this_step = 0

        for current_node in current_beam:
            nodes_expanded_this_step += 1

            if current_node.position == goal_node_target_pos:
                path = reconstruct_path(current_node)
                end_time = time.perf_counter()
                nodes_expanded_total += nodes_expanded_this_step # Add expansions from this final step
                return {
                    "path": path,
                    "score": current_node.g,
                    "time": end_time - start_time,
                    "nodes_explored": nodes_expanded_total,
                    "algorithm": f"Beam Search (W={beam_width})"
                }

            # Explore neighbors
            for dr, dc in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
                neighbor_pos = (current_node.position[0] + dr, current_node.position[1] + dc)

                if not (0 <= neighbor_pos[0] < len(grid) and 0 <= neighbor_pos[1] < len(grid[0])):
                    continue
                if grid[neighbor_pos[0]][neighbor_pos[1]] == 1: # Obstacle
                    continue

                tentative_g_score = current_node.g + 1
                
                # If we've found a path to this neighbor before that was equally good or better, skip.
                if tentative_g_score >= visited_g_costs.get(neighbor_pos, float('inf')):
                    continue

                # Update the best g-cost to reach this neighbor
                visited_g_costs[neighbor_pos] = tentative_g_score
                h_score = heuristic_func(neighbor_pos, goal_node_target_pos)
                neighbor_node = Node(neighbor_pos, current_node, tentative_g_score, h_score)
                candidates.append(neighbor_node)
        
        nodes_expanded_total += nodes_expanded_this_step

        if not candidates:
            break # No valid successors found from the current beam

        # Sort all candidates by f-score (A*-style evaluation)
        candidates.sort() 
        
        # Prune to beam_width
        current_beam = candidates[:beam_width]
        
        if not current_beam: # If beam becomes empty after pruning
            break
            
    end_time = time.perf_counter()
    # No path found or max depth reached
    return {
        "path": [],
        "score": float('inf'),
        "time": end_time - start_time,
        "nodes_explored": nodes_expanded_total,
        "algorithm": f"Beam Search (W={beam_width})"
    }

# --- Scenarios Definition ---
# 0: traversable, 1: obstacle
scenarios = [
    {
        "name": "Simple Case",
        "grid": [
            [0, 0, 0, 0, 0],
            [0, 1, 0, 1, 0],
            [0, 1, 0, 0, 0],
            [0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0]
        ],
        "start": (0, 0),
        "goal": (4, 4)
    },
    {
        "name": "Moderate Case (U-shape)",
        "grid": [
            [0, 0, 0, 0, 0, 0, 0],
            [0, 1, 1, 1, 1, 1, 0], # Wall
            [0, 1, 0, 0, 0, 1, 0], # Path
            [0, 1, 0, 1, 0, 1, 0], # Obstacles
            [0, 0, 0, 1, 0, 0, 0], # Start here (4,0)
            [0, 1, 1, 1, 1, 1, 0], # Wall
            [0, 0, 0, 0, 0, 0, 0]  # Goal here (6,6)
        ],
        "start": (4, 0),
        "goal": (6, 6)
    },
    {
        "name": "Large Grid with Obstacles",
        "grid": [
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0], 
            [0, 1, 1, 0, 1, 0, 1, 1, 1, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [1, 1, 1, 0, 1, 1, 1, 1, 0, 1],
            [0, 0, 0, 0, 0, 0, 0, 1, 0, 0], 
            [0, 1, 1, 1, 1, 1, 0, 1, 1, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 1, 0],
            [0, 1, 1, 0, 1, 0, 1, 0, 1, 0],
            [0, 1, 0, 0, 0, 0, 1, 0, 0, 0],
            [0, 1, 0, 0, 1, 0, 0, 0, 1, 0] 
        ],
        "start": (0, 0),
        "goal": (9,9) 
    },
    {
        "name": "No Path Case",
        "grid": [
            [0, 0, 0],
            [0, 1, 1], # Wall blocking direct access
            [0, 1, 0],
            [1, 1, 0], # Start can't reach Goal
            [0, 0, 0]  # Goal
        ],
        "start": (0,0),
        "goal": (4,2)
    }
]

# --- Run and Print Results ---
BEAM_WIDTH = 3 # Experiment with this value (e.g., 1, 2, 3, 5)

for i, scenario_data in enumerate(scenarios):
    print(f"--- Scenario {i+1}: {scenario_data['name']} ---")
    grid = scenario_data['grid']
    start = scenario_data['start']
    goal = scenario_data['goal']

    print(f"Grid Dimensions: {len(grid)}x{len(grid[0])}, Start: {start}, Goal: {goal}")
    
    # Run A*
    results_astar = a_star_search(grid, start, goal, heuristic_manhattan)
    print(f"\n[{results_astar['algorithm']} Results]")
    print(f"  Path Found: {'Yes' if results_astar['path'] else 'No'}")
    if results_astar['path']:
        print(f"  Path Score (Cost): {results_astar['score']}")
        # print(f"  Path: {results_astar['path']}") # Uncomment to see the path
    print(f"  Nodes Explored: {results_astar['nodes_explored']}")
    print(f"  Wall Clock Time: {results_astar['time']:.6f} seconds")

    # Run Beam Search
    results_beam = beam_search_astar_pruning(grid, start, goal, heuristic_manhattan, BEAM_WIDTH)
    print(f"\n[{results_beam['algorithm']} Results]")
    print(f"  Path Found: {'Yes' if results_beam['path'] else 'No'}")
    if results_beam['path']:
        print(f"  Path Score (Cost): {results_beam['score']}")
        # print(f"  Path: {results_beam['path']}") # Uncomment to see the path
    print(f"  Nodes Explored (expanded from beam): {results_beam['nodes_explored']}")
    print(f"  Wall Clock Time: {results_beam['time']:.6f} seconds")
    
    # Comparison note
    if results_astar['path'] and results_beam['path']:
        if results_astar['score'] < results_beam['score']:
            print("  Note: A* found a better (shorter/cheaper) path than Beam Search.")
        elif results_beam['score'] < results_astar['score']:
             print("  Note: Beam Search found a better path than A* (this shouldn't happen if A* is optimal and uses same heuristic).") # Should not happen
        elif results_astar['score'] == results_beam['score']:
             print("  Note: Both algorithms found paths of the same quality.")
    elif results_astar['path'] and not results_beam['path']:
        print("  Note: A* found a path, but Beam Search did not.")
    elif not results_astar['path'] and results_beam['path']:
         print("  Note: Beam Search found a path, but A* did not (this shouldn't happen if A* is complete).") # Should not happen for solvable paths

    print("\n" + "="*40 + "\n")
