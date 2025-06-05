import heapq
import time
import functools
import csv

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
        "name": "1. Zero Obstacle Test (Short Path)",
        "grid_dims": (HUGE_DIM, HUGE_DIM),
        "start": (0, 0),
        "goal": (50, 50),
        "scenario_seed": 123,
        "obstacle_density": 0.0,
        "beam_widths_to_test": [8] # Basic test, expect optimal
    },
    {
        "name": "2. Relatively Short Path, Low Density", # Was Original S1
        "grid_dims": (HUGE_DIM, HUGE_DIM),
        "start": (0, 0),
        "goal": (50, 50),
        "scenario_seed": 123, # Same map as above, but with obstacles
        "obstacle_density": 0.1,
        "beam_widths_to_test": [8, 16]
    },
    {
        "name": "3. Medium Path, Moderate Density", # Was Original S2
        "grid_dims": (HUGE_DIM, HUGE_DIM),
        "start": (1000, 1000),
        "goal": (1200, 1250), # Manhattan distance 250 + 250 = 500
        "scenario_seed": 456,
        "obstacle_density": 0.25,
        "beam_widths_to_test": [8, 16]
    },
    {
        "name": "4. Longer Path - BEAM WIDTH COMPARISON", # Was Original S3
        "grid_dims": (HUGE_DIM, HUGE_DIM),
        "start": (0, 0),
        "goal": (800, 800), # Manhattan distance 1600
        "scenario_seed": 789,
        "obstacle_density": 0.15,
        "beam_widths_to_test": [8, 16, 32, 64] # Explicitly test multiple widths
    },
    {
        "name": "5. Medium Path, Higher Density ", # Was User's S4
        "grid_dims": (HUGE_DIM, HUGE_DIM),
        "start": (2500, 2000),
        "goal": (2700, 2200), # Manhattan distance 200 + 200 = 400
        "scenario_seed": 101, # Seed previously used for dense, using again
        "obstacle_density": 0.4,
        "beam_widths_to_test": [8, 16]
    },
    {
        "name": "6. Long Path, Very High Density (Likely No Path)", # Was User's S5
        "grid_dims": (HUGE_DIM, HUGE_DIM),
        "start": (4000, 4000),
        "goal": (4500, 4500), # Manhattan distance 1000. Changed from 6020,6020 to make it a bit more reasonable to analyze if a path IS found.
        "scenario_seed": 110, # New seed for this specific high-density test
        "obstacle_density": 0.6,
        "beam_widths_to_test": [8] # High density, likely fails or is very slow
    },
    {
        "name": "7. Very Long Path, Extreme Density (Almost Certainly No Path)", # Was User's S6
        "grid_dims": (HUGE_DIM, HUGE_DIM),
        "start": (0, 1000),
        "goal": (3000, 2000), # Manhattan distance 3000 + 1000 = 4000
        "scenario_seed": 120, # New seed
        "obstacle_density": 0.75,
        "beam_widths_to_test": [8] # Expect failure or limit hit
    },
    {
        "name": "8. Very Long Sparse Path",
        "grid_dims": (HUGE_DIM, HUGE_DIM),
        "start": (0,0),
        "goal": (2000, 2000), # Manhattan distance 4000
        "scenario_seed": 201,
        "obstacle_density": 0.05, # Very sparse
        "beam_widths_to_test": [8, 16, 32] # Might need wider beam for good quality on long paths
    },
    {
        "name": "9. Moderate Path, Extremely Low Density",
        "grid_dims": (HUGE_DIM, HUGE_DIM),
        "start": (100,100),
        "goal": (1000,1000), # Manhattan distance 1800
        "scenario_seed": 301,
        "obstacle_density": 0.01, # Almost open space
        "beam_widths_to_test": [8, 16]
    },
    {
        "name": "10. Forced Node Limit Test (Diagonal Max Distance)",
        "grid_dims": (HUGE_DIM, HUGE_DIM), # This grid is too large to traverse fully
        "start": (0,0),
        "goal": (HUGE_DIM -100_000, HUGE_DIM -100_000), # Goal is very far, but not literally edge to avoid overflow/extreme H
        "scenario_seed": 401,
        "obstacle_density": 0.1, # Moderate density
        "beam_widths_to_test": [8] # Expect to hit node limits
    },
    {
        "name": "11. Short Path, High Density Challenge",
        "grid_dims": (HUGE_DIM, HUGE_DIM),
        "start": (0,0),
        "goal": (30,30), # Manhattan distance 60
        "scenario_seed": 506, # New seed
        "obstacle_density": 0.35, # Challenging density for short path
        "beam_widths_to_test": [8, 16, 32]
    },
    {
        "name": "12. Another Medium Path, Different Seed/Density",
        "grid_dims": (HUGE_DIM, HUGE_DIM),
        "start": (500, 0),
        "goal": (500, 500), # Straight line path of 500 if clear
        "scenario_seed": 601,
        "obstacle_density": 0.20,
        "beam_widths_to_test": [8, 16]
    }
]

# --- Run and Print Results ---
BEAM_WIDTH = 8

# --- Main Run and Logging Logic ---
CSV_LOG_FILE_NAME = "pathfinding_results.csv"
TEXT_LOG_FILE_NAME = "pathfinding_verbose_log.txt"

CSV_HEADER = [
    "Scenario_Name", "Grid_Dims", "Start_Pos", "Goal_Pos", "Scenario_Seed", "Obstacle_Density",
    "Algorithm", "Beam_Width", "Path_Found", "Path_Score", "Nodes_Processed", "Time_s", "Limit_Reached"
]

# Open both log files
with open(CSV_LOG_FILE_NAME, 'w', newline='') as csv_log_file, \
     open(TEXT_LOG_FILE_NAME, 'w') as text_log_file:

    csv_writer = csv.writer(csv_log_file)
    csv_writer.writerow(CSV_HEADER) # Write the CSV header

    def write_to_console_and_text_log(message):
        """Helper function to print to console and write to text log."""
        print(message)
        text_log_file.write(message + "\n")

    write_to_console_and_text_log(f"Starting experiments. CSV results logged to {CSV_LOG_FILE_NAME}, Verbose log to {TEXT_LOG_FILE_NAME}")

    for i, scenario_data in enumerate(scenarios):
        scenario_name = scenario_data['name']
        dims = scenario_data['grid_dims']
        start = scenario_data['start']
        goal = scenario_data['goal']
        seed = scenario_data['scenario_seed']
        density = scenario_data['obstacle_density']
        beam_widths_to_test = scenario_data['beam_widths_to_test']

        # --- Scenario Header ---
        scenario_header_text = f"\n--- Scenario {i+1}: {scenario_name} ---"
        write_to_console_and_text_log(scenario_header_text)
        
        details_text = (
            f"  Grid Dimensions: {dims[0]}x{dims[1]}, Start: {start}, Goal: {goal}\n"
            f"  Obstacle Density: {density*100:.1f}%, Scenario Seed: {seed}\n"
            f"  A* Node Limit: {MAX_NODES_TO_EXPLORE_ASTAR}, Beam Search Node Limit: {MAX_NODES_TO_EXPAND_BEAM}"
        ) # Note: Beam width is per-run for Beam Search
        write_to_console_and_text_log(details_text)

        current_is_obstacle_func = functools.partial(is_obstacle_procedural,
                                                     start_pos=start,
                                                     goal_pos=goal,
                                                     grid_dims=dims,
                                                     scenario_seed=seed,
                                                     obstacle_density=density)

        # --- Run A* ---
        write_to_console_and_text_log(f"\n  Running A*...")
        results_astar = a_star_search_implicit(dims, start, goal, heuristic_manhattan, current_is_obstacle_func, MAX_NODES_TO_EXPLORE_ASTAR)
        
        # Log A* results to CSV
        log_row_astar = [
            scenario_name, f"{dims[0]}x{dims[1]}", str(start), str(goal), seed, f"{density:.2f}",
            results_astar['algorithm'], "N/A",
            "Yes" if results_astar['path'] else "No",
            results_astar['score'] if results_astar['path'] else "N/A",
            results_astar['nodes_explored'],
            f"{results_astar['time']:.6f}",
            "Yes" if results_astar['limit_reached'] else "No"
        ]
        csv_writer.writerow(log_row_astar)

        # Write A* results to text log
        astar_text_log = (
            f"    [{results_astar['algorithm']} Results]\n"
            f"      Path Found: {'Yes' if results_astar['path'] else 'No'}\n"
        )
        if results_astar['path']:
            astar_text_log += f"      Path Score (Cost): {results_astar['score']}\n"
        astar_text_log += (
            f"      Nodes Explored: {results_astar['nodes_explored']}\n"
            f"      Wall Clock Time: {results_astar['time']:.6f} seconds\n"
        )
        if results_astar['limit_reached']:
            astar_text_log += f"      Termination: Max nodes explored limit ({MAX_NODES_TO_EXPLORE_ASTAR}) reached.\n"
        write_to_console_and_text_log(astar_text_log)


        # --- Run Beam Search for each specified width ---
        for beam_width_val in beam_widths_to_test:
            write_to_console_and_text_log(f"\n  Running Beam Search (W={beam_width_val})...")
            results_beam = beam_search_astar_pruning_implicit(dims, start, goal, heuristic_manhattan, current_is_obstacle_func, beam_width_val, MAX_NODES_TO_EXPAND_BEAM)
            
            # Log Beam Search results to CSV
            log_row_beam = [
                scenario_name, f"{dims[0]}x{dims[1]}", str(start), str(goal), seed, f"{density:.2f}",
                results_beam['algorithm'], 
                beam_width_val,
                "Yes" if results_beam['path'] else "No",
                results_beam['score'] if results_beam['path'] else "N/A",
                results_beam['nodes_explored'],
                f"{results_beam['time']:.6f}",
                "Yes" if results_beam['limit_reached'] else "No"
            ]
            csv_writer.writerow(log_row_beam)

            # Write Beam Search results to text log
            beam_text_log = (
                f"    [{results_beam['algorithm']} Results]\n"
                f"      Path Found: {'Yes' if results_beam['path'] else 'No'}\n"
            )
            if results_beam['path']:
                beam_text_log += f"      Path Score (Cost): {results_beam['score']}\n"
            beam_text_log += (
                f"      Nodes Expanded from Beam: {results_beam['nodes_explored']}\n"
                f"      Wall Clock Time: {results_beam['time']:.6f} seconds\n"
            )
            if results_beam['limit_reached']:
                beam_text_log += f"      Termination: Max nodes expanded limit ({MAX_NODES_TO_EXPAND_BEAM}) reached.\n"
            write_to_console_and_text_log(beam_text_log)

            # --- Comparison Note (for text log only) ---
            comparison_text = "      Comparison: "
            if results_astar['path'] and results_beam['path']:
                if results_astar['score'] < results_beam['score']:
                    comparison_text += "A* found a better (shorter/cheaper) path."
                elif results_beam['score'] < results_astar['score']:
                    comparison_text += "Beam Search found a better path (A* might have hit limit or Beam got lucky)."
                else: 
                    comparison_text += "Both algorithms found paths of the same quality (or both hit limits similarly)."
            elif results_astar['path'] and not results_beam['path']:
                comparison_text += "A* found a path, but Beam Search did not (possibly due to pruning or hitting limit)."
            elif not results_astar['path'] and results_beam['path']:
                comparison_text += "Beam Search found a path, but A* did not (A* might have hit its limit earlier on a wider search)."
            elif not results_astar['path'] and not results_beam['path']:
                comparison_text += "Neither algorithm found a path (possibly no path exists, or both hit limits)."
            write_to_console_and_text_log(comparison_text)
        
        write_to_console_and_text_log("\n" + "="*60 + "\n") # Scenario separator
        
        # Flush buffers to ensure data is written, especially for long runs
        csv_log_file.flush()
        text_log_file.flush()

    write_to_console_and_text_log(f"\nAll scenarios processed. CSV log: {CSV_LOG_FILE_NAME}, Verbose log: {TEXT_LOG_FILE_NAME}")