"""A* pathfinding and path simplification algorithms for grid-based navigation."""

import heapq
import numpy as np
from scipy.interpolate import splprep, splev
from scipy.ndimage import binary_dilation
from typing import Optional, Tuple, List


class PathPlanner:
    """Static class for A* pathfinding and path optimization algorithms."""

    # Algorithm constants
    MAX_SEARCH_RADIUS = 50  # Maximum search radius in grid cells for nearest free cell
    ANGLE_THRESHOLD = 0.1   # Minimum angle change in radians for path simplification

    @staticmethod
    def astar_search(
        start_grid: Tuple[int, int],
        goal_grid: Tuple[int, int],
        grid_map: np.ndarray
    ) -> Tuple[Optional[List[Tuple[int, int]]], Optional[Tuple[int, int]]]:
        """A* pathfinding algorithm on grid map.

        Assumes goal_grid is already validated and adjusted by caller.

        Args:
            start_grid: Starting grid position (i, j)
            goal_grid: Goal grid position (i, j) - must be in free space
            grid_map: Binary occupancy grid (0=free, 1=occupied)

        Returns:
            tuple: (path, closest_point) where:
                - path: List of grid positions [(i, j), ...] from start to goal, or None if no path
                - closest_point: Closest reachable point to goal if path not found, or None
        """
        height, width = grid_map.shape

        # Check if start or goal is out of bounds
        if not (0 <= start_grid[0] < height and 0 <= start_grid[1] < width):
            return None, None
        if not (0 <= goal_grid[0] < height and 0 <= goal_grid[1] < width):
            return None, None

        # Check if start is in obstacle
        if grid_map[start_grid[0], start_grid[1]] == 1:
            return None, None

        # Check if goal is in obstacle (caller should have adjusted it)
        if grid_map[goal_grid[0], goal_grid[1]] == 1:
            return None, None

        original_goal = goal_grid
        
        def heuristic(a, b):
            """Euclidean distance heuristic."""
            return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
        
        def get_neighbors(pos):
            """Get valid neighboring cells (8-connected)."""
            neighbors = []
            for di in [-1, 0, 1]:
                for dj in [-1, 0, 1]:
                    if di == 0 and dj == 0:
                        continue
                    ni, nj = pos[0] + di, pos[1] + dj
                    if 0 <= ni < height and 0 <= nj < width:
                        if grid_map[ni, nj] != 1:  # Not an obstacle
                            # Diagonal movement cost
                            cost = np.sqrt(2) if (di != 0 and dj != 0) else 1.0
                            neighbors.append(((ni, nj), cost))
            return neighbors
        
        # A* algorithm
        open_set = []
        heapq.heappush(open_set, (0, start_grid))
        open_dict = {}
        open_dict[start_grid] = 0
        came_from = {}
        g_score = {start_grid: 0}
        f_score = {start_grid: heuristic(start_grid, goal_grid)}
        closed_set = set()
        
        # Track closest point to original goal
        closest_point = start_grid
        closest_distance = heuristic(start_grid, original_goal)
        
        while open_set:
            current = heapq.heappop(open_set)[1]
            
            if current in closed_set:
                continue

            if current in open_dict:
                del open_dict[current]
            
            closed_set.add(current)
            
            # Update closest point
            dist_to_goal = heuristic(current, original_goal)
            if dist_to_goal < closest_distance:
                closest_distance = dist_to_goal
                closest_point = current
            
            # Check if reached goal
            if current == goal_grid:
                # Reconstruct path
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start_grid)
                path.reverse()
                return path, closest_point
            
            # Explore neighbors
            for neighbor, move_cost in get_neighbors(current):
                if neighbor in closed_set:
                    continue
                
                tentative_g = g_score[current] + move_cost
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f = tentative_g + heuristic(neighbor, goal_grid)

                    # Skip if already in open_set with better or equal f_score
                    if neighbor in open_dict and f >= open_dict[neighbor]:
                        continue
                    
                    f_score[neighbor] = f
                    open_dict[neighbor] = f
                    heapq.heappush(open_set, (f, neighbor))
        
        # No path found, return path to closest point
        if closest_point != start_grid:
            path = []
            current = closest_point
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start_grid)
            path.reverse()
            return path, closest_point
        
        return None, None

    @staticmethod
    def find_nearest_axial_free_cell(
        goal_grid: Tuple[int, int],
        grid_map: np.ndarray,
        max_search_distance: Optional[int] = None
    ) -> Optional[Tuple[int, int]]:
        """Find nearest free cell along x-axis or y-axis from goal position.

        Searches in four axial directions:
        1. Keep j fixed, vary i positively (south in world coordinates)
        2. Keep j fixed, vary i negatively (north in world coordinates)
        3. Keep i fixed, vary j positively (east in world coordinates)
        4. Keep i fixed, vary j negatively (west in world coordinates)

        Returns the closest free cell among all four directions.

        Args:
            goal_grid: Goal grid position (i, j)
            grid_map: Binary occupancy grid (0=free, 1=occupied)
            max_search_distance: Maximum search distance in grid cells (default: MAX_SEARCH_RADIUS)

        Returns:
            tuple: Nearest axial free cell (i, j) or None if not found within max_search_distance
        """
        if max_search_distance is None:
            max_search_distance = PathPlanner.MAX_SEARCH_RADIUS

        height, width = grid_map.shape
        goal_i, goal_j = goal_grid
        
        best_cell = None
        best_distance = float('inf')
        
        # Search along y-axis (keep x fixed, vary y)
        for di in range(1, max_search_distance + 1):
            ni = goal_i + di
            if 0 <= ni < height and grid_map[ni, goal_j] != 1:
                distance = abs(di)
                if distance < best_distance:
                    best_distance = distance
                    best_cell = (ni, goal_j)
                break  # Found closest in this direction
        
        for di in range(1, max_search_distance + 1):
            ni = goal_i - di
            if 0 <= ni < height and grid_map[ni, goal_j] != 1:
                distance = abs(di)
                if distance < best_distance:
                    best_distance = distance
                    best_cell = (ni, goal_j)
                break
        
        # Search along x-axis (keep y fixed, vary x)
        for dj in range(1, max_search_distance + 1):
            nj = goal_j + dj
            if 0 <= nj < width and grid_map[goal_i, nj] != 1:
                distance = abs(dj)
                if distance < best_distance:
                    best_distance = distance
                    best_cell = (goal_i, nj)
                break
        
        for dj in range(1, max_search_distance + 1):
            nj = goal_j - dj
            if 0 <= nj < width and grid_map[goal_i, nj] != 1:
                distance = abs(dj)
                if distance < best_distance:
                    best_distance = distance
                    best_cell = (goal_i, nj)
                break
        
        return best_cell
    
    @staticmethod
    def bresenham_line(
        start: Tuple[int, int],
        end: Tuple[int, int]
    ) -> List[Tuple[int, int]]:
        """Bresenham's line algorithm to get all grid cells between two points.

        Args:
            start: (i, j) grid coordinates
            end: (i, j) grid coordinates

        Returns:
            list: List of (i, j) grid coordinates along the line
        """
        i0, j0 = start
        i1, j1 = end
        
        points = []
        di = abs(i1 - i0)
        dj = abs(j1 - j0)
        
        si = 1 if i0 < i1 else -1
        sj = 1 if j0 < j1 else -1
        
        err = di - dj
        i, j = i0, j0
        
        while True:
            points.append((i, j))
            
            if i == i1 and j == j1:
                break
            
            e2 = 2 * err
            if e2 > -dj:
                err -= dj
                i += si
            if e2 < di:
                err += di
                j += sj
        
        return points
    
    @staticmethod
    def has_line_of_sight(
        start: Tuple[int, int],
        end: Tuple[int, int],
        grid_map: np.ndarray
    ) -> bool:
        """Check if there's a clear line of sight between two grid points.

        Args:
            start: (i, j) grid coordinates
            end: (i, j) grid coordinates
            grid_map: Binary occupancy grid (0=free, 1=occupied)

        Returns:
            bool: True if path is clear, False if blocked by obstacle
        """
        line_points = PathPlanner.bresenham_line(start, end)
        height, width = grid_map.shape
        
        for i, j in line_points:
            # Check bounds
            if not (0 <= i < height and 0 <= j < width):
                return False
            # Check obstacle
            if grid_map[i, j] == 1:
                return False
        
        return True
    
    @staticmethod
    def simplify_path_line_of_sight(
        path_grid: List[Tuple[int, int]],
        grid_map: np.ndarray
    ) -> List[Tuple[int, int]]:
        """Simplify path using line-of-sight algorithm.

        Args:
            path_grid: List of (i, j) grid coordinates
            grid_map: Binary occupancy grid (0=free, 1=occupied)

        Returns:
            list: Simplified path as list of (i, j) grid coordinates
        """
        if len(path_grid) <= 2:
            return path_grid
        
        simplified = [path_grid[0]]
        current_idx = 0
        
        while current_idx < len(path_grid) - 1:
            # Find farthest visible point
            farthest_visible = current_idx + 1
            for i in range(len(path_grid) - 1, current_idx, -1):
                if PathPlanner.has_line_of_sight(path_grid[current_idx], path_grid[i], grid_map):
                    farthest_visible = i
                    break
            
            simplified.append(path_grid[farthest_visible])
            current_idx = farthest_visible
        
        return simplified
    
    @staticmethod
    def simplify_path_angle_filter(
        path_grid: List[Tuple[int, int]],
        angle_threshold: Optional[float] = None
    ) -> List[Tuple[int, int]]:
        """Simplify path by removing points with small angle changes.

        Args:
            path_grid: List of (i, j) grid coordinates
            angle_threshold: Minimum angle change in radians to keep a point (default: ANGLE_THRESHOLD)

        Returns:
            list: Simplified path as list of (i, j) grid coordinates
        """
        if angle_threshold is None:
            angle_threshold = PathPlanner.ANGLE_THRESHOLD
        if len(path_grid) <= 2:
            return path_grid
        
        simplified = [path_grid[0]]
        
        for i in range(1, len(path_grid) - 1):
            prev = np.array(path_grid[i - 1])
            curr = np.array(path_grid[i])
            next_pt = np.array(path_grid[i + 1])
            
            # Calculate vectors
            v1 = curr - prev
            v2 = next_pt - curr
            
            # Calculate angle between vectors
            if np.linalg.norm(v1) > 0 and np.linalg.norm(v2) > 0:
                cos_angle = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
                cos_angle = np.clip(cos_angle, -1.0, 1.0)
                angle = np.arccos(cos_angle)
                
                # Keep point if angle is significant
                if angle > angle_threshold:
                    simplified.append(path_grid[i])
        
        simplified.append(path_grid[-1])

        return simplified
    
    @staticmethod
    def smooth_path_bspline(
        path_grid: List[Tuple[float, float]], 
        smoothing: float = 0.5
    ) -> List[Tuple[float, float]]:
        """Smooth path using B-spline interpolation.
    
        Args:
            path_grid: List of (x, y) coordinates
            smoothing: Smoothing factor (0=exact fit, higher=smoother)
    
        Returns:
            list: Smoothed path as list of (x, y) tuples
        """
        if len(path_grid) < 3:
            return path_grid
        
        path = np.array(path_grid)
        x = path[:, 0]
        y = path[:, 1]
        
        # Remove duplicate consecutive points
        dist = np.sqrt(np.diff(x)**2 + np.diff(y)**2)
        dist = np.concatenate(([1.0], dist))
        keep_indices = dist > 1e-6
        x = x[keep_indices]
        y = y[keep_indices]
        
        if len(x) < 3:
            return path_grid
        
        try:
            # Fit B-spline curve
            tck, u = splprep([x, y], s=smoothing, k=2)
            
            # Generate smoothed path with more points
            num_points = len(path_grid) * 3
            u_new = np.linspace(0, 1, num_points)
            x_new, y_new = splev(u_new, tck)
            
            return list(zip(x_new, y_new))
        except Exception:
            # If B-spline fitting fails, return original path
            return path_grid

    @staticmethod
    def inflate_obstacles(
        grid_map: np.ndarray,
        robot_radius: float,
        grid_size: float = 0.1
    ) -> np.ndarray:
        """Inflate obstacles in grid map by robot radius for collision-free path planning.

        Args:
            grid_map: Binary occupancy grid (0=free, 1=occupied)
            robot_radius: Robot footprint radius in meters
            grid_size: Size of each grid cell in meters

        Returns:
            np.ndarray: Inflated grid map with same shape as input
        """
        # Calculate inflation radius in grid cells
        inflation_cells = int(np.ceil(robot_radius / grid_size))

        # Create circular structuring element
        y, x = np.ogrid[-inflation_cells:inflation_cells+1, -inflation_cells:inflation_cells+1]
        structure = x**2 + y**2 <= inflation_cells**2

        # Dilate obstacles
        inflated_map = binary_dilation(grid_map, structure=structure).astype(np.uint8)

        return inflated_map


class GridMapUtils:
    """Utility class for grid map coordinate conversions."""

    @staticmethod
    def world_to_grid(
        world_pos: Tuple[float, float],
        floor_pos: np.ndarray,
        grid_map_shape: Tuple[int, int],
        grid_size: float = 0.1
    ) -> Tuple[int, int]:
        """Convert world position [x, y] to grid indices [i, j].

        Args:
            world_pos: World position (x, y) in meters
            floor_pos: Floor center position [x, y, z] in world coordinates
            grid_map_shape: Shape of grid map (height, width)
            grid_size: Size of each grid cell in meters

        Returns:
            tuple: Grid indices (i, j)
        """
        height, width = grid_map_shape
        floor_half_x = (width * grid_size) / 2
        floor_half_y = (height * grid_size) / 2
        floor_min_x = floor_pos[0] - floor_half_x
        floor_max_y = floor_pos[1] + floor_half_y

        grid_j = int(np.floor((world_pos[0] - floor_min_x) / grid_size))
        grid_i = int(np.floor((floor_max_y - world_pos[1]) / grid_size))

        return grid_i, grid_j

    @staticmethod
    def grid_to_world(
        grid_pos: Tuple[int, int],
        floor_pos: np.ndarray,
        grid_map_shape: Tuple[int, int],
        grid_size: float = 0.1
    ) -> np.ndarray:
        """Convert grid indices [i, j] to world position [x, y].

        Args:
            grid_pos: Grid indices (i, j)
            floor_pos: Floor center position [x, y, z] in world coordinates
            grid_map_shape: Shape of grid map (height, width)
            grid_size: Size of each grid cell in meters

        Returns:
            np.ndarray: World position [x, y] in meters
        """
        height, width = grid_map_shape
        floor_half_x = (width * grid_size) / 2
        floor_half_y = (height * grid_size) / 2
        floor_min_x = floor_pos[0] - floor_half_x
        floor_max_y = floor_pos[1] + floor_half_y

        world_x = floor_min_x + (grid_pos[1] + 0.5) * grid_size
        world_y = floor_max_y - (grid_pos[0] + 0.5) * grid_size

        return np.array([world_x, world_y])
