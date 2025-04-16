class GridGraph:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.adjacency_list = {}
        self.explored_cells = set()
        self.walked_edges = set()  # New: Track walked paths as edges between positions
        self.walked_path = []      # New: Record full path history
        
        # Directions remain the same
        self.directions = [
            Position(0, -1), Position(1, 0), Position(0, 1), Position(-1, 0)
        ]
        
        # Initialize grid (same as before)
        for x in range(width):
            for y in range(height):
                pos = Position(x, y)
                self.adjacency_list[pos] = []
                for direction in self.directions:
                    neighbor = pos + direction
                    if self.is_valid_position(neighbor):
                        self.adjacency_list[pos].append(neighbor)

    def record_movement(self, from_pos, to_pos):
        """Record a movement between two positions"""
        # Add to path history
        self.walked_path.append((from_pos, to_pos))
        
        # Add to walked edges (both directions)
        self.walked_edges.add((from_pos, to_pos))
        self.walked_edges.add((to_pos, from_pos))
        
        # Mark both positions as explored
        self.mark_explored(from_pos)
        self.mark_explored(to_pos)

    def has_been_walked(self, pos1, pos2):
        """Check if a path between two positions has been walked before"""
        return (pos1, pos2) in self.walked_edges

    def get_unwalked_neighbors(self, pos):
        """Get neighbors that haven't been walked to yet"""
        unwalked = []
        for neighbor in self.adjacency_list[pos]:
            if not self.has_been_walked(pos, neighbor):
                unwalked.append(neighbor)
        return unwalked

    def calculate_exploration_path(self, robot_pos):
        """Modified to prefer unwalked paths"""
        # First try to find completely new areas
        frontier = self.find_frontier_cells()
        
        if frontier:
            closest_frontier = self.find_closest_cell(robot_pos, frontier)
            if closest_frontier:
                return self.find_path(robot_pos, closest_frontier)
        
        # If no frontier, find least walked areas
        return self.find_least_walked_path(robot_pos)

    def find_least_walked_path(self, current_pos):
        """Find path to least frequently visited areas"""
        # Get all reachable cells
        reachable = self.find_reachable_cells(current_pos)
        
        if not reachable:
            return []
            
        # Find cell with fewest visits (count appearances in walked_path)
        visit_counts = {}
        for pos in reachable:
            visit_counts[pos] = sum(1 for (f, t) in self.walked_path if f == pos or t == pos)
        
        target = min(reachable, key=lambda p: visit_counts[p])
        return self.find_path(current_pos, target)

    def find_reachable_cells(self, start_pos):
        """Find all cells reachable from current position"""
        visited = set()
        queue = [start_pos]
        
        while queue:
            current = queue.pop(0)
            if current not in visited:
                visited.add(current)
                for neighbor in self.adjacency_list[current]:
                    if neighbor not in visited and not self.has_been_walked(current, neighbor):
                        queue.append(neighbor)
        return visited

    # ... (keep all your existing methods)

# Modified navigation function
def navigate():
    robot_pos = Position(0, 0)
    map_graph = GridGraph(12, 12)
    map_graph.mark_explored(robot_pos)
    
    for _ in range(50):  # Longer simulation
        # 1. Get sensor data and update map
        sensor_readings = [False, False, False, False]  # Replace with actual sensor data
        map_graph.update_map(robot_pos, sensor_readings)
        
        # 2. Calculate path (prefer unwalked routes)
        path = map_graph.calculate_exploration_path(robot_pos)
        
        if not path or len(path) < 2:
            print("Exploration complete or stuck!")
            break
            
        # 3. Move robot and record path
        next_pos = path[1]
        map_graph.record_movement(robot_pos, next_pos)
        robot_pos = next_pos
        
        # 4. Visualize
        print(f"Moved from {path[0]} to {path[1]}")
        print(f"Walked path length: {len(map_graph.walked_path)}")
        map_graph.print_map()
        
        # Add delay for real robot
        time.sleep(0.5)  # Adjust based on movement speed