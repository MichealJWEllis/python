"""
Obstacle Robot Simulator
------------------------
A mecanum drive robot that can detect and avoid obstacles inside the arena,
not just walls.

This builds on the mecanum robot by adding:
- Circular obstacles defined by center point and radius
- Ray-circle intersection for obstacle detection
- Visualization of obstacles on the plot

Real-world applications:
- Warehouse robots navigating around shelving and other robots
- Autonomous vehicles detecting pedestrians and other cars
- Vacuum robots avoiding furniture

The key new concept is ray casting against circles. Instead of just checking
"how far to the wall?", we now ask "how far to the nearest thing?" — where
"thing" can be a wall OR an obstacle.

This is Challenge 4 in the robotics learning path.
"""

import math
import matplotlib.pyplot as plt


class ObstacleRobot:
    """
    A mecanum drive robot with obstacle detection capabilities.
    
    Inherits all the holonomic movement abilities of the mecanum robot:
    - Move forward/backward (vx)
    - Strafe left/right (vy)
    - Rotate in place (omega)
    
    Adds:
    - Storage for circular obstacles
    - Ray-circle intersection algorithm for sensing obstacles
    - Modified sense_distance() that checks both walls AND obstacles
    
    Obstacles are represented as circles because:
    1. Simple to define: just (center_x, center_y, radius)
    2. Easy to compute ray intersection (quadratic formula)
    3. Good approximation for many real objects (barrels, posts, other robots)
    
    Rectangle obstacles are possible but require more complex intersection math.
    """
    
    def __init__(self, x=0, y=0, theta=0, obstacles=None, bounds=None):
        """
        Initialize an obstacle-aware robot.
        
        :param x: Starting x position in world coordinates (default: 0)
        :param y: Starting y position in world coordinates (default: 0)
        :param theta: Starting orientation in radians (default: 0, facing right)
        :param obstacles: List of circular obstacles, each defined as (cx, cy, r)
                         where cx, cy is the center and r is the radius.
                         Default: empty list (no obstacles)
        :param bounds: Optional tuple (width, height) defining arena size.
                      Arena spans from (0,0) to (width, height).
                      If None, robot exists in infinite space (but can still
                      detect obstacles).
        """
        # Position in world coordinates
        self.x = x
        self.y = y
        
        # Orientation in radians (0 = facing right)
        self.theta = theta
        
        # Store obstacles as a list
        # If None was passed, use empty list to avoid errors when iterating
        # This pattern: "value if condition else default" is a ternary expression
        self.obstacles = obstacles if obstacles is not None else []
        
        # Arena boundaries (can be None for infinite space)
        self.bounds = bounds
        
        # Velocity commands (mecanum drive)
        self.vx = 0     # Forward/backward velocity
        self.vy = 0     # Strafe velocity (left/right)
        self.omega = 0  # Rotational velocity
    
    def set_velocity(self, vx, vy, omega):
        """
        Set the robot's velocity commands.
        
        All velocities are in the robot's LOCAL coordinate frame.
        The coordinate transformation to world frame happens in update().
        
        :param vx: Forward/backward velocity (positive = forward)
        :param vy: Strafe velocity (positive = left)
        :param omega: Rotational velocity (positive = counterclockwise)
        """
        self.vx = vx
        self.vy = vy
        self.omega = omega
    
    def update(self, dt):
        """
        Update the robot's position based on velocities over time interval dt.
        
        Transforms local velocities to world frame using rotation matrix,
        then updates position. Same as mecanum robot.
        
        :param dt: Time step in seconds
        
        The Math:
        ---------
        Local frame: vx = forward, vy = left (relative to robot)
        World frame: x = right, y = up (fixed coordinates)
        
        To convert, we rotate the velocity vector by theta:
            world_vx = vx * cos(theta) - vy * sin(theta)
            world_vy = vx * sin(theta) + vy * cos(theta)
        
        This is a 2D rotation matrix applied to (vx, vy).
        """
        # Transform local velocities to world frame
        world_vx = self.vx * math.cos(self.theta) - self.vy * math.sin(self.theta)
        world_vy = self.vx * math.sin(self.theta) + self.vy * math.cos(self.theta)
        
        # Update position: velocity * time = displacement
        self.x += world_vx * dt
        self.y += world_vy * dt
        
        # Update orientation: angular_velocity * time = angle change
        self.theta += self.omega * dt
    
    def get_pose(self):
        """
        Get the robot's current pose (position + orientation).
        
        :return: Tuple of (x, y, theta) in world coordinates
        """
        return (self.x, self.y, self.theta)
    
    def sense_distance(self):
        """
        Sense distance to the nearest object (wall OR obstacle) in facing direction.
        
        This is the key method that changed from the mecanum robot. It now:
        1. Checks distance to all obstacles using ray-circle intersection
        2. Checks distance to all walls using ray-line intersection
        3. Returns the minimum (closest thing the robot would hit)
        
        :return: Distance to nearest object in facing direction,
                or infinity if nothing is in the way
        
        Why check both?
        ---------------
        The robot doesn't care WHAT it's about to hit — it just needs to know
        IF something is close. By combining wall and obstacle distances into
        one list and taking the minimum, we get simple avoidance behavior
        that works for any type of object.
        """
        # Start with empty list of distances
        # We'll add distances to obstacles and walls, then return the minimum
        distances = []
        
        # ==================== CHECK OBSTACLES ====================
        # Loop through each obstacle and calculate ray-circle intersection
        for obstacle in self.obstacles:
            # Unpack obstacle tuple into center coordinates and radius
            cx, cy, r = obstacle
            
            # Calculate distance using ray-circle intersection
            dist = self.distance_to_obstacle(cx, cy, r)
            
            # Only add if we actually hit the obstacle (not infinity)
            if dist < float('inf'):
                distances.append(dist)
        
        # ==================== CHECK WALLS ====================
        # Only check walls if bounds exist
        if self.bounds is not None:
            width, height = self.bounds
            
            # RIGHT WALL (x = width)
            # Only relevant if robot is facing rightward (cos > 0)
            if math.cos(self.theta) > 0:
                distance_to_right = (width - self.x) / math.cos(self.theta)
                distances.append(distance_to_right)
            
            # LEFT WALL (x = 0)
            # Only relevant if robot is facing leftward (cos < 0)
            if math.cos(self.theta) < 0:
                distance_to_left = (0 - self.x) / math.cos(self.theta)
                distances.append(distance_to_left)
            
            # TOP WALL (y = height)
            # Only relevant if robot is facing upward (sin > 0)
            if math.sin(self.theta) > 0:
                distance_to_top = (height - self.y) / math.sin(self.theta)
                distances.append(distance_to_top)
            
            # BOTTOM WALL (y = 0)
            # Only relevant if robot is facing downward (sin < 0)
            if math.sin(self.theta) < 0:
                distance_to_bottom = (0 - self.y) / math.sin(self.theta)
                distances.append(distance_to_bottom)
        
        # ==================== RETURN CLOSEST ====================
        # If distances list is empty (no bounds, no obstacles), return infinity
        if len(distances) == 0:
            return float('inf')
        
        # Return the closest thing (smallest distance)
        return min(distances)
    
    def distance_to_obstacle(self, cx, cy, r):
        """
        Calculate distance from robot to edge of a circular obstacle using ray casting.
        
        This is the new math for this challenge. We're solving:
        "Where does a ray from the robot intersect a circle?"
        
        :param cx: X coordinate of circle center
        :param cy: Y coordinate of circle center
        :param r: Radius of circle
        :return: Distance to circle edge, or infinity if ray misses
        
        The Math (Ray-Circle Intersection):
        ------------------------------------
        
        A ray can be described as: P(t) = origin + t * direction
        Where:
        - origin = (self.x, self.y) — robot position
        - direction = (cos(theta), sin(theta)) — unit vector in facing direction
        - t = distance along the ray (what we're solving for)
        
        Any point on the ray is:
            point_x = self.x + t * cos(theta)
            point_y = self.y + t * sin(theta)
        
        A point is ON the circle if its distance from center equals radius:
            (point_x - cx)² + (point_y - cy)² = r²
        
        Substituting the ray equation into the circle equation and simplifying
        gives us a quadratic equation: at² + bt + c = 0
        
        Where:
            a = dx² + dy² = 1 (since direction is unit length)
            b = 2 * (fx * dx + fy * dy)
            c = fx² + fy² - r²
            
            fx, fy = vector from circle center to robot = (self.x - cx, self.y - cy)
            dx, dy = direction = (cos(theta), sin(theta))
        
        Solving with quadratic formula: t = (-b ± √(b² - 4ac)) / 2a
        
        The discriminant (b² - 4ac) tells us:
        - Negative: ray misses circle entirely (no real solutions)
        - Zero: ray grazes circle tangentially (one solution)
        - Positive: ray passes through circle (two solutions — entry and exit)
        
        We want the CLOSER intersection (smaller positive t).
        Negative t means the intersection is BEHIND the robot.
        """
        # Direction vector — unit vector pointing where robot faces
        # cos(theta) = x component, sin(theta) = y component
        dx = math.cos(self.theta)
        dy = math.sin(self.theta)
        
        # Vector from circle center to robot position
        # This is used to determine if robot is inside/outside circle
        # and which direction the circle is relative to the ray
        fx = self.x - cx
        fy = self.y - cy
        
        # Quadratic coefficients for at² + bt + c = 0
        # a = direction dot direction = 1 for unit vectors
        # b = 2 * (center-to-robot dot direction)
        # c = |center-to-robot|² - radius²
        a = dx * dx + dy * dy  # Always 1 for unit direction, but written explicitly
        b = 2 * (fx * dx + fy * dy)
        c = (fx * fx + fy * fy) - r * r
        
        # Discriminant determines number of intersections
        # b² - 4ac from quadratic formula
        discriminant = b * b - 4 * a * c
        
        if discriminant < 0:
            # Negative discriminant = no real solutions = ray misses circle
            # The ray passes by the circle without touching it
            return float('inf')
        
        # Two solutions from quadratic formula: (-b ± √discriminant) / 2a
        # t1 uses minus (closer intersection)
        # t2 uses plus (farther intersection)
        t1 = (-b - math.sqrt(discriminant)) / (2 * a)
        t2 = (-b + math.sqrt(discriminant)) / (2 * a)
        
        # We want the closest POSITIVE intersection
        # Positive t = in front of robot
        # Negative t = behind robot (we ignore these)
        if t1 > 0:
            # t1 is the entry point (closer), and it's in front of us
            return t1
        elif t2 > 0:
            # t1 was behind us, but t2 is in front
            # This happens when robot is INSIDE the circle
            return t2
        else:
            # Both intersections are behind the robot
            # Circle is completely behind us
            return float('inf')
    
    def step(self):
        """
        Execute one step of autonomous obstacle avoidance behavior.
        
        Uses the same logic as mecanum robot, but now sense_distance()
        also detects obstacles, so the robot avoids both walls AND obstacles.
        
        Behavior:
        - If something is within 2.0 units: strafe right to avoid
        - Otherwise: drive forward
        
        The beauty of abstraction:
        -------------------------
        Notice that step() didn't change AT ALL from mecanum robot.
        We only changed sense_distance() to also check obstacles.
        
        Because step() just asks "is something close?" and doesn't care
        WHAT that something is, it automatically works for obstacles too.
        This is good software design — each method has one job.
        """
        if self.sense_distance() <= 2.0:
            # Something is close! Strafe right to avoid it.
            # Whether it's a wall or obstacle, the behavior is the same.
            self.set_velocity(0, -1, 0)
            self.update(0.1)
        else:
            # Path is clear, drive forward
            self.set_velocity(1, 0, 0)
            self.update(0.1)
    
    def run(self, max_steps):
        """
        Run the autonomous behavior for multiple steps.
        
        :param max_steps: How many sense-decide-act cycles to run
        :return: List of poses for visualization
        """
        steps_list = []
        for i in range(max_steps):
            self.step()
            steps_list.append(self.get_pose())
        return steps_list


def visualize(poses, bounds, obstacles=[]):
    """
    Draw the robot's path through the arena, including obstacles.
    
    This extends the previous visualize() by adding obstacle rendering.
    Obstacles are drawn as filled gray circles.
    
    :param poses: List of (x, y, theta) tuples from robot.run()
    :param bounds: Tuple (width, height) defining arena size
    :param obstacles: List of (cx, cy, r) tuples defining circular obstacles.
                     Default is empty list (no obstacles to draw).
    
    Matplotlib Circle Objects:
    --------------------------
    plt.Circle((x, y), radius) creates a circle patch.
    plt.gca() gets the current axes (where we're drawing).
    add_patch() adds the circle to the plot.
    
    We need add_patch() because circles aren't simple x,y data points —
    they're shapes that need to be drawn specially.
    """
    # Unpack arena dimensions
    width, height = bounds
    
    # Extract x and y coordinates from poses
    x_values = [pose[0] for pose in poses]
    y_values = [pose[1] for pose in poses]
    
    # Create a new figure
    plt.figure()
    
    # Plot the robot's path as a blue line
    plt.plot(x_values, y_values)
    
    # Mark start (green) and end (red) positions
    plt.scatter(poses[0][0], poses[0][1], color="green")
    plt.scatter(poses[-1][0], poses[-1][1], color="red")
    
    # Set axis limits to show full arena
    plt.xlim(0, width)
    plt.ylim(0, height)
    
    # Force equal axis scaling (so circles look like circles, not ovals)
    plt.axis('equal')
    
    # Draw each obstacle as a filled gray circle
    for obstacle in obstacles:
        cx, cy, r = obstacle  # Unpack center and radius
        
        # Create a Circle patch
        # (cx, cy) = center position
        # r = radius
        # color='gray' = fill color
        # fill=True = solid circle (not just outline)
        circle = plt.Circle((cx, cy), r, color='gray', fill=True)
        
        # Add the circle to the current axes
        # gca() = "get current axes" — the plotting area
        # add_patch() adds a shape (patch) to the plot
        plt.gca().add_patch(circle)
    
    # Display the plot
    plt.show()


# This block only runs when executing this file directly
if __name__ == '__main__':
    # ==================== TEST 1: Basic obstacle detection ====================
    # Robot at (2, 5), obstacle at (7, 5) with radius 1
    # Robot facing right (theta=0), directly at the obstacle
    # Expected distance: 7 - 2 - 1 = 4
    # (5 units to center, minus 1 unit radius = 4 units to edge)
    
    robot = ObstacleRobot(
        x=2, y=5, theta=0,
        obstacles=[(7, 5, 1)],
        bounds=(10, 10)
    )
    print("Distance to obstacle:", robot.sense_distance())
    
    # ==================== TEST 2: Autonomous obstacle avoidance ====================
    # Robot starts at (1, 5), obstacle in the middle at (5, 5)
    # Robot should:
    # 1. Drive toward obstacle
    # 2. Detect it at distance 2.0
    # 3. Strafe down to avoid it
    # 4. Continue past the obstacle
    # 5. Eventually hit the right wall and strafe down more
    
    obstacles = [(5, 5, 1)]
    robot = ObstacleRobot(
        x=1, y=5, theta=0,
        obstacles=obstacles,
        bounds=(10, 10)
    )
    poses = robot.run(200)
    visualize(poses, (10, 10), obstacles)