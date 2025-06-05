import heapq
import time
import math
import functools # For partial application

# --- Global Limits ---
MAX_NODES_TO_EXPLORE_ASTAR = 5_000_000
MAX_NODES_TO_EXPAND_BEAM = 5_000_000 # Nodes taken from beam to generate successors
# For Beam search, it might also be beneficial to consider a max_depth if needed, but max_nodes is a good proxy.

# --- Node and Path Utilities ---
class Node:
    def __init__(self, position, parent=None, g=0, h=0):
        self.position = position
        self.parent = parent
        self.g = g
        self.h = h
        self.f = g + h

    def __lt__(self, other):
        if self.f == other.f:
            return self.h < other.h
        return self.f < other.f

    def __eq__(self, other):
        return self.position == other.position

    def __hash__(self):
        return hash(self.position)

def heuristic_manhattan(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def reconstruct_path(node):
    path = []
    current = node
    while current:
        path.append(current.position)
        current = current.parent
    return path[::-1]

# --- Procedural Obstacle Generation ---
def get_deterministic_pseudo_random_for_pos(r, c, scenario_seed):
    # LCG parameters (common ones)
    a = 1664525
    lcg_c = 1013904223
    m = 2**32

    # Slightly more robust mixing - using XOR and more prime multipliers
    # These are just examples; good hash functions are a deep topic.
    val_r = r * 2654435761 # Knuth's golden ratio multiplicative hash constant for integers
    val_c = c * 334214459 # Another prime
    
    # Combine components using XOR and addition, then ensure within range
    # The idea is to make the initial state more sensitive to small changes in r or c
    initial_state = (val_r ^ val_c ^ scenario_seed) + (val_r + scenario_seed) + (val_c + scenario_seed)
    initial_state = initial_state & (m - 1)

    # LCG step
    lcg_val = (a * initial_state + lcg_c) % m
    return lcg_val / m

def is_obstacle_procedural(position, start_pos, goal_pos, grid_dims, scenario_seed, obstacle_density):
    """
    Determines if a position is an obstacle procedurally.
    Start and goal positions are never obstacles.
    grid_dims are mainly for conceptual boundaries, not directly used in this obstacle function
    but are essential for the pathfinding algorithms' bounds checks.
    """
    if position == start_pos or position == goal_pos:
        return False

    r, c = position
    # Basic check: if outside conceptual grid dims, could be treated as obstacle implicitly by caller,
    # but this function focuses on the "terrain" type.
    # if not (0 <= r < grid_dims[0] and 0 <= c < grid_dims[1]):
    #     return True # Or handle by caller

    random_val = get_deterministic_pseudo_random_for_pos(r, c, scenario_seed)
    return random_val < obstacle_density

# --- A* Algorithm (adapted for implicit grid) ---
def a_star_search_implicit(grid_dims, start_pos, goal_pos, heuristic_func, is_obstacle_func, max_nodes_explored_limit):
    start_time = time.perf_counter()
    
    start_node = Node(start_pos, None, 0, heuristic_func(start_pos, goal_pos))
    
    open_set = []
    heapq.heappush(open_set, start_node)
    
    closed_set = set()
    g_costs = {start_pos: 0}
    nodes_explored_count = 0
    limit_reached = False

    while open_set:
        if nodes_explored_count >= max_nodes_explored_limit:
            limit_reached = True
            break
            
        current_node = heapq.heappop(open_set)
        nodes_explored_count += 1

        if current_node.position == goal_pos:
            path = reconstruct_path(current_node)
            end_time = time.perf_counter()
            return {
                "path": path,
                "score": current_node.g,
                "time": end_time - start_time,
                "nodes_explored": nodes_explored_count,
                "limit_reached": False,
                "algorithm": "A*"
            }

        closed_set.add(current_node.position)

        for dr, dc in [(0, 1), (0, -1), (1, 0), (-1, 0)]: # Neighbors
            neighbor_pos = (current_node.position[0] + dr, current_node.position[1] + dc)

            # Check bounds
            if not (0 <= neighbor_pos[0] < grid_dims[0] and 0 <= neighbor_pos[1] < grid_dims[1]):
                continue
            
            # Check obstacles using the procedural function
            if is_obstacle_func(neighbor_pos):
                continue
            
            if neighbor_pos in closed_set:
                continue
            
            tentative_g_score = current_node.g + 1

            if tentative_g_score < g_costs.get(neighbor_pos, float('inf')):
                g_costs[neighbor_pos] = tentative_g_score
                h_score = heuristic_func(neighbor_pos, goal_pos)
                neighbor_node = Node(neighbor_pos, current_node, tentative_g_score, h_score)
                # Check if neighbor_node is already in open_set with higher g_cost is implicitly handled
                # by heapq behavior or by checking g_costs, but explicit check for presence is more robust for some A* variants.
                # For this version, pushing and letting g_costs check is fine.
                heapq.heappush(open_set, neighbor_node)
                
    end_time = time.perf_counter()
    return {
        "path": [],
        "score": float('inf'),
        "time": end_time - start_time,
        "nodes_explored": nodes_explored_count,
        "limit_reached": limit_reached,
        "algorithm": "A*"
    }

# --- Beam Search (adapted for implicit grid) ---
def beam_search_astar_pruning_implicit(grid_dims, start_pos, goal_pos, heuristic_func, is_obstacle_func, beam_width, max_nodes_expanded_limit):
    start_time = time.perf_counter()

    start_node = Node(start_pos, None, 0, heuristic_func(start_pos, goal_pos))
    
    current_beam = [start_node]
    visited_g_costs = {start_pos: start_node.g} 
    nodes_expanded_total = 0
    limit_reached = False
    
    # Max depth can be an additional safeguard, e.g., sum of dimensions
    max_depth = grid_dims[0] + grid_dims[1] # A loose upper bound for path length

    for depth in range(max_depth): 
        if not current_beam:
            break
        if nodes_expanded_total >= max_nodes_expanded_limit :
            limit_reached = True
            break

        candidates = []
        nodes_expanded_this_step = 0

        for current_node in current_beam:
            if nodes_expanded_total + nodes_expanded_this_step >= max_nodes_expanded_limit:
                limit_reached = True # Check before expanding this node's children
                break # Break from processing nodes in current_beam

            nodes_expanded_this_step +=1

            if current_node.position == goal_pos:
                path = reconstruct_path(current_node)
                end_time = time.perf_counter()
                nodes_expanded_total += nodes_expanded_this_step
                return {
                    "path": path,
                    "score": current_node.g,
                    "time": end_time - start_time,
                    "nodes_explored": nodes_expanded_total, # "explored" here means nodes from which successors were generated
                    "limit_reached": False,
                    "algorithm": f"Beam Search (W={beam_width})"
                }

            for dr, dc in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
                neighbor_pos = (current_node.position[0] + dr, current_node.position[1] + dc)

                if not (0 <= neighbor_pos[0] < grid_dims[0] and 0 <= neighbor_pos[1] < grid_dims[1]):
                    continue
                if is_obstacle_func(neighbor_pos):
                    continue

                tentative_g_score = current_node.g + 1
                
                if tentative_g_score >= visited_g_costs.get(neighbor_pos, float('inf')):
                    continue

                visited_g_costs[neighbor_pos] = tentative_g_score
                h_score = heuristic_func(neighbor_pos, goal_pos)
                neighbor_node = Node(neighbor_pos, current_node, tentative_g_score, h_score)
                candidates.append(neighbor_node)
        
        nodes_expanded_total += nodes_expanded_this_step
        if limit_reached: # If limit was hit while generating successors
            break

        if not candidates:
            break

        candidates.sort()
        current_beam = candidates[:beam_width]
        
        if not current_beam:
            break
            
    end_time = time.perf_counter()
    return {
        "path": [],
        "score": float('inf'),
        "time": end_time - start_time,
        "nodes_explored": nodes_expanded_total,
        "limit_reached": limit_reached,
        "algorithm": f"Beam Search (W={beam_width})"
    }

# --- Scenarios Definition for Implicit Grids (previous ones + new test) ---
HUGE_DIM = 500_000
# MAX_NODES_TO_EXPLORE_ASTAR = 200_000 # Defined in your script
# MAX_NODES_TO_EXPAND_BEAM = 200_000  # Defined in your script

scenarios = [
    {
        "name": "Zero Obstacle Test (Derived from S1)",
        "grid_dims": (HUGE_DIM, HUGE_DIM),
        "start": (0, 0),
        "goal": (50, 50),
        "scenario_seed": 123, # Same seed as S1
        "obstacle_density": 0.0 # CRITICAL CHANGE FOR THIS TEST
    },
    {
        "name": "Relatively Short Path in Huge Grid (Original S1)",
        "grid_dims": (HUGE_DIM, HUGE_DIM),
        "start": (0, 0),
        "goal": (50, 50),
        "scenario_seed": 123,
        "obstacle_density": 0.1
    },
    {
        "name": "Medium Path, Higher Obstacle Density (Original S2)",
        "grid_dims": (HUGE_DIM, HUGE_DIM),
        "start": (1000, 1000),
        "goal": (1200, 1250),
        "scenario_seed": 456,
        "obstacle_density": 0.25
    },
    {
        "name": "Longer Path Attempt (Original S3)",
        "grid_dims": (HUGE_DIM, HUGE_DIM),
        "start": (0, 0),
        "goal": (800, 800),
        "scenario_seed": 789,
        "obstacle_density": 0.15
    },
    {
        "name": "Potentially No Path (Dense Small Area) (Original S4)",
        "grid_dims": (HUGE_DIM, HUGE_DIM),
        "start": (5000, 5000),
        "goal": (5020, 5020),
        "scenario_seed": 101,
        "obstacle_density": 0.6,
    }
]

# --- Run and Print Results ---
BEAM_WIDTH = 8

for i, scenario_data in enumerate(scenarios):
    print(f"--- Scenario {i+1}: {scenario_data['name']} ---")
    dims = scenario_data['grid_dims']
    start = scenario_data['start']
    goal = scenario_data['goal']
    seed = scenario_data['scenario_seed']
    density = scenario_data['obstacle_density']

    print(f"Grid Dimensions: {dims[0]}x{dims[1]}, Start: {start}, Goal: {goal}")
    print(f"Obstacle Density: {density*100}%, Scenario Seed: {seed}")
    print(f"A* Node Limit: {MAX_NODES_TO_EXPLORE_ASTAR}, Beam Search Node Limit: {MAX_NODES_TO_EXPAND_BEAM}, Beam Width: {BEAM_WIDTH}")

    current_is_obstacle_func = functools.partial(is_obstacle_procedural,
                                                 start_pos=start,
                                                 goal_pos=goal,
                                                 grid_dims=dims,
                                                 scenario_seed=seed,
                                                 obstacle_density=density)

    # Run A*
    results_astar = a_star_search_implicit(dims, start, goal, heuristic_manhattan, current_is_obstacle_func, MAX_NODES_TO_EXPLORE_ASTAR)
    print(f"\n[{results_astar['algorithm']} Results]")
    print(f"  Path Found: {'Yes' if results_astar['path'] else 'No'}")
    if results_astar['path']:
        print(f"  Path Score (Cost): {results_astar['score']}")
    print(f"  Nodes Explored: {results_astar['nodes_explored']}")
    print(f"  Wall Clock Time: {results_astar['time']:.6f} seconds")
    if results_astar['limit_reached']:
        print(f"  Termination: Max nodes explored limit ({MAX_NODES_TO_EXPLORE_ASTAR}) reached.")

    # Run Beam Search
    results_beam = beam_search_astar_pruning_implicit(dims, start, goal, heuristic_manhattan, current_is_obstacle_func, BEAM_WIDTH, MAX_NODES_TO_EXPAND_BEAM)
    print(f"\n[{results_beam['algorithm']} Results]")
    print(f"  Path Found: {'Yes' if results_beam['path'] else 'No'}")
    if results_beam['path']:
        print(f"  Path Score (Cost): {results_beam['score']}")
    print(f"  Nodes Expanded from Beam: {results_beam['nodes_explored']}")
    print(f"  Wall Clock Time: {results_beam['time']:.6f} seconds")
    if results_beam['limit_reached']:
        print(f"  Termination: Max nodes expanded limit ({MAX_NODES_TO_EXPAND_BEAM}) reached.")
    
    # Comparison note (same as before)
    if results_astar['path'] and results_beam['path']:
        if results_astar['score'] < results_beam['score']:
            print("  Comparison: A* found a better (shorter/cheaper) path.")
        elif results_beam['score'] < results_astar['score']:
             print("  Comparison: Beam Search found a better path (A* might have hit limit or Beam got lucky).")
        else: 
             print("  Comparison: Both algorithms found paths of the same quality (or both hit limits similarly).")
    elif results_astar['path'] and not results_beam['path']:
        print("  Comparison: A* found a path, but Beam Search did not (possibly due to pruning or hitting limit).")
    elif not results_astar['path'] and results_beam['path']:
         print("  Comparison: Beam Search found a path, but A* did not (A* might have hit its limit earlier on a wider search).")
    elif not results_astar['path'] and not results_beam['path']:
        print("  Comparison: Neither algorithm found a path (possibly no path exists, or both hit limits).")

    print("\n" + "="*50 + "\n")