class Position:
    """Represents a position in the grid with x, y coordinates"""

    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y

    def __eq__(self, other):
        """Enable equality comparison between positions"""
        if isinstance(other, Position):
            return self.x == other.x and self.y == other.y
        return False

    def __hash__(self):
        """Make Position hashable for use in dictionaries and sets"""
        return hash((self.x, self.y))

    def __add__(self, other):
        """Allow adding positions together"""
        return Position(self.x + other.x, self.y + other.y)

    def __repr__(self):
        """String representation for debugging"""
        return f"({self.x},{self.y})"

    # added but can be ignored     
    def __str__(self) -> str:
        """User-friendly string representation"""
        return f"({self._x},{self._y})"


class GridGraph:
    """
    Graph representation of a grid map with walls.
    Each cell is a node, and edges represent possible movements.
    Absence of an edge represents a wall.
    """

    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.adjacency_list = {}
        self.explored_cells = set()

        # Possible movement directions: Up, Right, Down, Left
        self.directions = [
            Position(0, -1), Position(1, 0), Position(0, 1), Position(-1, 0)
        ]

        # Initialize the grid with connections in all directions
        for x in range(width):
            for y in range(height):
                pos = Position(x, y)

                # Add all possible neighbors
                self.adjacency_list[pos] = []
                for direction in self.directions:
                    neighbor = pos + direction

                    # Check if neighbor is within bounds
                    if self.is_valid_position(neighbor):
                        self.adjacency_list[pos].append(neighbor)

    def is_valid_position(self, pos):
        """Check if a position is within the grid boundaries"""
        return 0 <= pos.x < self.width and 0 <= pos.y < self.height

    def add_wall(self, pos1, pos2):
        """Add a wall between two adjacent cells by removing the edge in both directions"""
        # Remove pos2 from pos1's adjacency list
        if pos2 in self.adjacency_list[pos1]:
            self.adjacency_list[pos1].remove(pos2)

        # Remove pos1 from pos2's adjacency list
        if pos1 in self.adjacency_list[pos2]:
            self.adjacency_list[pos2].remove(pos1)

        print(f"Added wall between {pos1} and {pos2}")

    def mark_explored(self, pos):
        """Mark a cell as explored"""
        self.explored_cells.add(pos)

    def is_explored(self, pos):
        """Check if a cell has been explored"""
        return pos in self.explored_cells

    def get_unexplored_neighbors(self, pos):
        """Get unexplored neighbors of a position"""
        unexplored = []

        if pos in self.adjacency_list:
            for neighbor in self.adjacency_list[pos]:
                if not self.is_explored(neighbor):
                    unexplored.append(neighbor)

        return unexplored

    def get_all_unexplored_cells(self):
        """Get all unexplored cells in the grid"""
        unexplored = []

        for x in range(self.width):
            for y in range(self.height):
                pos = Position(x, y)
                if not self.is_explored(pos):
                    unexplored.append(pos)

        return unexplored

    # BFS Pathfinding Algorithm
    def find_path(self, start, goal):
        """Find the shortest path between two positions using BFS"""
        if start == goal:
            return [start]

        queue = [start]
        visited = {start}
        came_from = {}

        while queue:
            current = queue.pop(0)  # Pop from the beginning (queue behavior)

            if current == goal:
                # Reconstruct the path
                path = []
                pos = goal

                while pos != start:
                    path.append(pos)
                    pos = came_from[pos]
                path.append(start)

                # Reverse to get path from start to goal
                path.reverse()
                return path

            if current in self.adjacency_list:
                for neighbor in self.adjacency_list[current]:
                    if neighbor not in visited:
                        queue.append(neighbor)
                        visited.add(neighbor)
                        came_from[neighbor] = current

        # No path found
        return []

    def update_map(self, current_pos, wall_sensors):
        """
        Update the map as the robot discovers walls

        Parameters:
        current_pos -- Current position of the robot
        wall_sensors -- List of booleans indicating walls in the order: Up, Right, Down, Left
        """
        # Process sensor readings
        for i, has_wall in enumerate(wall_sensors):
            if i < len(self.directions):
                neighbor_pos = current_pos + self.directions[i]

                if self.is_valid_position(neighbor_pos):
                    if has_wall:
                        # Wall detected - remove the connection if it exists
                        self.add_wall(current_pos, neighbor_pos)

        # Mark the current position as explored
        self.mark_explored(current_pos)

    def calculate_exploration_path(self, robot_pos):
        """
        Calculate the most efficient exploration path using a frontier-based approach
        Returns the path to the closest frontier cell
        """
        # If the current cell is unexplored, explore it first
        if not self.is_explored(robot_pos):
            return [robot_pos]

        # Find all frontier cells (unexplored cells adjacent to explored cells)
        frontier = set()

        for explored_cell in self.explored_cells:
            for direction in self.directions:
                neighbor = explored_cell + direction

                # Check if the neighbor is valid, unexplored, and accessible
                if (self.is_valid_position(neighbor) and
                        not self.is_explored(neighbor) and
                        neighbor in self.adjacency_list[explored_cell]):
                    frontier.add(neighbor)

        # If no frontier cells, exploration is complete
        if not frontier:
            return []

        # Find the closest frontier cell
        closest_frontier = None
        min_distance = float('inf')

        for frontier_cell in frontier:
            path = self.find_path(robot_pos, frontier_cell)

            if path and len(path) - 1 < min_distance:
                min_distance = len(path) - 1
                closest_frontier = frontier_cell

        # Return the path to the closest frontier cell
        return self.find_path(robot_pos, closest_frontier)

    def print_map(self):
        """Print the map with walls for visualization"""
        print("Map with explored cells (E) and walls:")

        # Print the top border
        print("+", end="")
        for x in range(self.width):
            print("---+", end="")
        print()

        for y in range(self.height):
            # Print cells and vertical walls
            print("|", end="")
            for x in range(self.width):
                pos = Position(x, y)
                print(" E " if self.is_explored(pos) else "   ", end="")

                # Check right wall
                right_neighbor = Position(x + 1, y)
                has_right_wall = (not self.is_valid_position(right_neighbor) or
                                  right_neighbor not in self.adjacency_list[pos])
                print("|" if has_right_wall else " ", end="")
            print()

            # Print horizontal walls
            print("+", end="")
            for x in range(self.width):
                pos = Position(x, y)
                bottom_neighbor = Position(x, y + 1)

                has_bottom_wall = (not self.is_valid_position(bottom_neighbor) or
                                   bottom_neighbor not in self.adjacency_list[pos])
                print("---" if has_bottom_wall else "   ", end="")
                print("+", end="")
            print()


def navigate():
    # Simulate robot exploration
    robot_pos = Position(0, 0)

    # Create a 12x12 grid
    map_graph = GridGraph(12, 12)

    # Add some walls
    # map_graph.add_wall(Position(3, 4), Position(3, 5))
    # map_graph.add_wall(Position(6, 7), Position(7, 7))
    # map_graph.add_wall(Position(8, 2), Position(8, 3))
    # map_graph.add_wall(Position(2, 9), Position(3, 9))
    # map_graph.add_wall(Position(5, 5), Position(5, 6))

    # Mark initial position as explored
    map_graph.mark_explored(robot_pos)

    # Repeat navigation
    for i in range(10):
        """
        1st. Robot detect surrounding to update maps for walls and paths
        2nd. Robot calculate next destination
        3rd. Robot move
        """
        # Simulate discovering walls with sensors (Up, Right, Down, Left)
        """ I suggest putting the orientation codes here together with the sensor reading part """
        sensor_readings = [False, False, False, False]
        map_graph.update_map(robot_pos, sensor_readings)

        # Calculate next exploration path
        exploration_path = map_graph.calculate_exploration_path(robot_pos)

        # Print results
        print(f"Robot is at {robot_pos}")
        print(f"Next exploration path: {exploration_path}")
        print(exploration_path[1])

        # Move robot
        next_pos = exploration_path[1]
        robot_pos = next_pos
        map_graph.mark_explored(robot_pos)

        # Print the map
        map_graph.print_map()


if __name__ == "__main__":
    navigate()

