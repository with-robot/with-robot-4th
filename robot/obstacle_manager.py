#!/usr/bin/env python3
"""
Obstacle Detection and Avoidance Manager
Simulates obstacle detection and path replanning
"""

import numpy as np
from typing import List, Tuple, Optional


class Obstacle:
    """Simple obstacle representation"""

    def __init__(self, x: float, y: float, radius: float = 0.3):
        """
        Args:
            x: X position of obstacle center
            y: Y position of obstacle center
            radius: Obstacle radius (meters)
        """
        self.x = x
        self.y = y
        self.radius = radius

    def is_blocking(self, x: float, y: float, safety_margin: float = 0.2) -> bool:
        """
        Check if a point is blocked by this obstacle

        Args:
            x, y: Point to check
            safety_margin: Additional safety distance

        Returns:
            True if point is blocked
        """
        distance = np.sqrt((x - self.x)**2 + (y - self.y)**2)
        return distance < (self.radius + safety_margin)

    def __repr__(self):
        return f"Obstacle(x={self.x:.2f}, y={self.y:.2f}, r={self.radius:.2f})"


class ObstacleManager:
    """Manages obstacles and provides path planning assistance"""

    def __init__(self):
        self.obstacles: List[Obstacle] = []
        self.detected_obstacles: List[Obstacle] = []

    def add_obstacle(self, x: float, y: float, radius: float = 0.3):
        """Add an obstacle to the environment"""
        obstacle = Obstacle(x, y, radius)
        self.obstacles.append(obstacle)
        print(f"✅ Obstacle added at ({x:.2f}, {y:.2f}), radius={radius:.2f}")
        return obstacle

    def clear_obstacles(self):
        """Remove all obstacles"""
        self.obstacles.clear()
        self.detected_obstacles.clear()
        print("✅ All obstacles cleared")

    def check_path(self, x: float, y: float) -> Optional[Obstacle]:
        """
        Check if a point is blocked by any obstacle

        Args:
            x, y: Point to check

        Returns:
            Blocking obstacle if found, None otherwise
        """
        for obstacle in self.obstacles:
            if obstacle.is_blocking(x, y):
                if obstacle not in self.detected_obstacles:
                    self.detected_obstacles.append(obstacle)
                    print(f"\n🚧 OBSTACLE DETECTED at ({obstacle.x:.2f}, {obstacle.y:.2f})!")
                return obstacle
        return None

    def suggest_detour(
        self,
        current_x: float,
        current_y: float,
        target_x: float,
        target_y: float,
        obstacle: Obstacle
    ) -> List[Tuple[float, float]]:
        """
        Suggest waypoints to avoid obstacle

        Args:
            current_x, current_y: Current position
            target_x, target_y: Target position
            obstacle: Blocking obstacle

        Returns:
            List of waypoints [(x1, y1), (x2, y2), ...]
        """
        # Simple detour: go around obstacle
        # Calculate perpendicular direction
        dx = target_x - current_x
        dy = target_y - current_y
        distance = np.sqrt(dx**2 + dy**2)

        if distance < 0.01:
            return []

        # Normalize direction
        dx /= distance
        dy /= distance

        # Perpendicular direction (rotate 90 degrees)
        perp_x = -dy
        perp_y = dx

        # Calculate detour distance
        detour_distance = obstacle.radius + 0.5  # Extra safety margin

        # Generate waypoints around obstacle
        waypoints = []

        # Waypoint 1: Move perpendicular to avoid obstacle
        wp1_x = obstacle.x + perp_x * detour_distance
        wp1_y = obstacle.y + perp_y * detour_distance
        waypoints.append((wp1_x, wp1_y))

        # Waypoint 2: Move past obstacle
        wp2_x = obstacle.x + dx * (obstacle.radius + 0.3) + perp_x * detour_distance
        wp2_y = obstacle.y + dy * (obstacle.radius + 0.3) + perp_y * detour_distance
        waypoints.append((wp2_x, wp2_y))

        # Final: Original target
        waypoints.append((target_x, target_y))

        print(f"📍 Detour waypoints generated: {len(waypoints)} points")
        for i, (x, y) in enumerate(waypoints, 1):
            print(f"   WP{i}: ({x:.2f}, {y:.2f})")

        return waypoints

    def get_obstacle_info(self) -> str:
        """Get human-readable obstacle information"""
        if not self.obstacles:
            return "No obstacles in environment"

        info = f"Obstacles: {len(self.obstacles)} total"
        for i, obs in enumerate(self.obstacles, 1):
            info += f"\n  {i}. ({obs.x:.2f}, {obs.y:.2f}) radius={obs.radius:.2f}"

        if self.detected_obstacles:
            info += f"\n\nDetected: {len(self.detected_obstacles)} obstacles"

        return info


# Global obstacle manager instance
obstacle_manager = ObstacleManager()


# Test scenario
if __name__ == "__main__":
    print("="*60)
    print("🚧 Obstacle Detection System Test")
    print("="*60)

    # Create test scenario
    manager = ObstacleManager()

    # Add obstacles
    print("\n1. Adding obstacles...")
    manager.add_obstacle(0.5, 0.5, radius=0.3)  # Center obstacle
    manager.add_obstacle(1.5, 1.0, radius=0.2)  # Upper obstacle

    print(f"\n{manager.get_obstacle_info()}")

    # Test path checking
    print("\n2. Testing path checks...")

    test_points = [
        (0.3, 0.3, "Near obstacle 1"),
        (0.5, 0.5, "At obstacle 1 center"),
        (1.0, 1.0, "Safe zone"),
        (1.5, 1.0, "At obstacle 2 center"),
    ]

    for x, y, desc in test_points:
        obstacle = manager.check_path(x, y)
        status = "🚧 BLOCKED" if obstacle else "✅ CLEAR"
        print(f"   ({x:.2f}, {y:.2f}) - {desc}: {status}")

    # Test detour generation
    print("\n3. Generating detour...")
    obstacle = manager.obstacles[0]
    waypoints = manager.suggest_detour(
        current_x=0.0, current_y=0.0,
        target_x=1.0, target_y=1.0,
        obstacle=obstacle
    )

    print("\n" + "="*60)
    print("✅ Test complete!")
    print("="*60)
