"""
Mecanum Drive Robot Simulator
-----------------------------
A robot with four mecanum wheels that can move in any direction without rotating.

Mecanum wheels have rollers mounted at 45° angles around the wheel's circumference.
When the wheel spins, the rollers deflect the force at an angle instead of pushing
straight forward like a normal wheel.

By combining four mecanum wheels (two with rollers angled one way, two the other),
the robot can:
- Drive forward/backward (all wheels same direction)
- Strafe left/right (wheels in X pattern)
- Rotate in place (wheels in O pattern)
- Any combination of the above simultaneously

Real-world examples:
- Warehouse robots that need to navigate tight aisles
- Competition robots (FIRST Robotics teams love these)
- Industrial AGVs (Automated Guided Vehicles)
- Some wheelchairs for tight maneuvering

Key advantage over differential drive: Can move sideways without rotating first.
Key disadvantage: More complex mechanically, less traction, doesn't work well on
uneven surfaces (the rollers can slip).

This is Challenge 3 in the robotics learning path.
"""

# math module gives us cos(), sin(), and pi for trigonometry
import math

# matplotlib is a plotting library - pyplot is the part we use to draw graphs
import matplotlib.pyplot as plt


class MecanumRobot:
    """
    A mecanum drive robot that can move in any direction.
    
    Unlike differential drive (which can only go forward/backward and rotate),
    mecanum can strafe sideways. This is called "holonomic" motion - the robot
    can move in any direction independent of its orientation.
    
    The robot is controlled by three velocity commands:
    - vx: forward/backward velocity (in robot's local frame)
    - vy: left/right strafe velocity (in robot's local frame)
    - omega: rotational velocity
    
    These get translated to world coordinates based on the robot's current
    orientation (theta).
    
    Coordinate system (same as other robots):
    - x increases to the right (world frame)
    - y increases upward (world frame)
    - theta = 0 means facing right (positive x direction)
    - theta increases counterclockwise
    """
    
    def __init__(self, x=0, y=0, theta=0, bounds=None):
        """
        Initialize a mecanum drive robot.
        
        :param x: Starting x position in world coordinates (default: 0)
        :param y: Starting y position in world coordinates (default: 0)
        :param theta: Starting orientation in radians (default: 0, facing right)
        :param bounds: Optional tuple (width, height) defining arena size.
                      Arena spans from (0,0) to (width, height).
                      If None, robot exists in infinite space.
        """
        # Position in world coordinates - changes as robot moves
        self.x = x
        self.y = y
        
        # Orientation in radians - changes as robot rotates
        # theta = 0 means facing right (world +x direction)
        self.theta = theta
        
        # Arena boundaries for collision detection and sensing
        self.bounds = bounds
        
        # Velocity commands in robot's LOCAL frame
        # These are set by set_velocity() and used by update()
        # They represent what the robot is trying to do, not where it is
        self.vx = 0     # Forward/backward velocity (positive = forward)
        self.vy = 0     # Strafe velocity (positive = left in standard convention)
        self.omega = 0  # Rotational velocity (positive = counterclockwise)
    
    def set_velocity(self, vx, vy, omega):
        """
        Set the robot's velocity commands.
        
        This doesn't move the robot - it just stores what velocities the robot
        should try to achieve. The actual movement happens in update().
        
        All velocities are in the robot's LOCAL coordinate frame:
        - vx positive = move in the direction the robot is facing
        - vy positive = move to the robot's left
        - omega positive = rotate counterclockwise
        
        :param vx: Forward/backward velocity (units per second)
        :param vy: Strafe left/right velocity (units per second)  
        :param omega: Rotational velocity (radians per second)
        
        Examples:
        - set_velocity(1, 0, 0)    → drive forward
        - set_velocity(-1, 0, 0)   → drive backward
        - set_velocity(0, 1, 0)    → strafe left
        - set_velocity(0, -1, 0)   → strafe right
        - set_velocity(0, 0, 1)    → spin counterclockwise
        - set_velocity(1, 0, 1)    → drive forward while turning left (circle)
        - set_velocity(1, 1, 0)    → drive diagonally forward-left
        """
        self.vx = vx
        self.vy = vy
        self.omega = omega
    
    def update(self, dt):
        """
        Update the robot's position based on velocities over time interval dt.
        
        This is where the coordinate transformation happens. The robot thinks
        in its local frame (forward, left, rotate), but the world uses fixed
        coordinates (x, y). We need to convert.
        
        :param dt: Time step in seconds (e.g., 0.1 = 100 milliseconds)
        
        The Coordinate Transformation:
        ------------------------------
        
        The robot's local "forward" direction depends on which way it's facing.
        If theta = 0 (facing right), local forward = world +x.
        If theta = π/2 (facing up), local forward = world +y.
        
        To convert local velocities (vx, vy) to world velocities, we use a
        rotation matrix. This rotates the velocity vector by angle theta.
        
        The formula:
            world_vx = vx * cos(theta) - vy * sin(theta)
            world_vy = vx * sin(theta) + vy * cos(theta)
        
        Concrete Example:
        -----------------
        Robot facing up (theta = π/2), commanded to go "forward" (vx=1, vy=0):
        
            world_vx = 1 * cos(π/2) - 0 * sin(π/2) = 1 * 0 - 0 * 1 = 0
            world_vy = 1 * sin(π/2) + 0 * cos(π/2) = 1 * 1 + 0 * 0 = 1
        
        So the robot moves in world +y direction (up), which is correct -
        "forward" for a robot facing up is the up direction.
        
        Another Example:
        ----------------
        Robot facing up (theta = π/2), commanded to strafe left (vx=0, vy=1):
        
            world_vx = 0 * cos(π/2) - 1 * sin(π/2) = 0 - 1 = -1
            world_vy = 0 * sin(π/2) + 1 * cos(π/2) = 0 + 0 = 0
        
        So the robot moves in world -x direction (left), which is correct -
        "left" for a robot facing up is the left direction in world space.
        
        Why This Matters:
        -----------------
        This transformation is why mecanum is powerful. You can say "strafe right"
        and the robot figures out which world direction that means based on its
        current orientation. The control logic stays simple (think in robot terms),
        and the math handles the conversion.
        """
        # Transform local velocities (vx, vy) to world frame
        # This is a 2D rotation matrix applied to the velocity vector
        # cos(theta) and sin(theta) encode the robot's current heading
        world_vx = self.vx * math.cos(self.theta) - self.vy * math.sin(self.theta)
        world_vy = self.vx * math.sin(self.theta) + self.vy * math.cos(self.theta)
        
        # Update position using world-frame velocities
        # velocity * time = distance moved
        self.x += world_vx * dt
        self.y += world_vy * dt
        
        # Update orientation
        # omega * time = angle rotated
        # Rotation doesn't need coordinate transformation - it's the same in
        # both local and world frames
        self.theta += self.omega * dt
    
    def get_pose(self):
        """
        Get the robot's current pose (position + orientation).
        
        :return: Tuple of (x, y, theta) in world coordinates
        """
        return (self.x, self.y, self.theta)
    
    def sense_distance(self):
        """
        Simulate a forward-facing distance sensor.
        
        Returns the distance to the nearest wall in the direction the robot
        is currently facing. Uses ray casting - shooting an imaginary ray
        forward and measuring where it hits.
        
        This is identical to the point robot and differential drive robot.
        The drive system doesn't affect sensing - a sensor is a sensor.
        
        :return: Distance to nearest wall in facing direction, or infinity if no bounds
        
        The Math (ray casting to axis-aligned walls):
        ---------------------------------------------
        For each wall, we calculate how far along our facing direction we'd
        travel before hitting it.
        
        For the right wall (x = width):
            - Horizontal gap: width - self.x
            - How much of our direction is horizontal: cos(theta)
            - Distance along our path: gap / cos(theta)
            - Only valid if cos(theta) > 0 (we're facing rightward)
        
        Similar logic for left, top, bottom walls.
        We return the minimum (closest wall we're facing).
        """
        # No boundaries = infinite distance in all directions
        if self.bounds is None:
            return float('inf')
        
        width, height = self.bounds
        distances = []
        
        # RIGHT WALL (x = width)
        # Only relevant if facing rightward (cos > 0)
        if math.cos(self.theta) > 0:
            distance_to_right = (width - self.x) / math.cos(self.theta)
            distances.append(distance_to_right)
        
        # LEFT WALL (x = 0)
        # Only relevant if facing leftward (cos < 0)
        if math.cos(self.theta) < 0:
            # (0 - x) is negative, cos is negative, result is positive
            distance_to_left = (0 - self.x) / math.cos(self.theta)
            distances.append(distance_to_left)
        
        # TOP WALL (y = height)
        # Only relevant if facing upward (sin > 0)
        if math.sin(self.theta) > 0:
            distance_to_top = (height - self.y) / math.sin(self.theta)
            distances.append(distance_to_top)
        
        # BOTTOM WALL (y = 0)
        # Only relevant if facing downward (sin < 0)
        if math.sin(self.theta) < 0:
            distance_to_bottom = (0 - self.y) / math.sin(self.theta)
            distances.append(distance_to_bottom)
        
        # Return the closest wall we're facing
        return min(distances)
    
    def step(self):
        """
        Execute one step of autonomous wall-avoidance behavior.
        
        This showcases mecanum's unique ability: when it sees a wall ahead,
        it strafes sideways instead of rotating. The robot keeps facing
        the same direction while sliding to avoid the obstacle.
        
        Behavior:
        - If wall within 2.0 units: strafe right (vy = -1)
        - Otherwise: drive forward (vx = 1)
        - Always update by 0.1 seconds
        
        Comparison to other drive systems:
        - Point robot: Would instantly rotate, then move forward
        - Differential drive: Would spin in place, then drive forward
        - Mecanum: Slides sideways without changing orientation
        
        This is useful in tight spaces where rotating would cause collisions,
        or when the robot needs to keep a sensor/camera pointed at something
        while moving.
        """
        if self.sense_distance() <= 2.0:
            # Wall ahead! Strafe right to avoid it.
            # vy = -1 moves right (negative = robot's right)
            # Notice: omega = 0, so theta doesn't change
            self.set_velocity(0, -1, 0)
            self.update(0.1)
        else:
            # Path is clear, drive forward
            # vx = 1 moves in facing direction
            self.set_velocity(1, 0, 0)
            self.update(0.1)
    
    def run(self, max_steps):
        """
        Run the autonomous behavior for multiple steps.
        
        :param max_steps: How many sense-decide-act cycles to run
        :return: List of poses [(x1,y1,theta1), (x2,y2,theta2), ...]
        """
        steps_list = []
        for i in range(max_steps):
            self.step()
            steps_list.append(self.get_pose())
        return steps_list


def visualize(poses, bounds):
    """
    Draw the robot's path through the arena.
    
    :param poses: List of (x, y, theta) tuples from robot.run()
    :param bounds: Tuple (width, height) defining arena size for axis limits
    """
    # Unpack arena dimensions
    width, height = bounds
    
    # Extract x coordinates from all poses
    # List comprehension: [expression for item in list]
    # pose[0] gets the first element (x) from each (x, y, theta) tuple
    x_values = [pose[0] for pose in poses]
    
    # Extract y coordinates (second element, index 1)
    y_values = [pose[1] for pose in poses]
    
    # Create a new figure (drawing canvas)
    plt.figure()
    
    # Plot the path as a connected line
    # matplotlib connects all points in order with a line
    plt.plot(x_values, y_values)
    
    # Mark starting position with green dot
    # poses[0] = first pose, [0] and [1] extract x and y
    plt.scatter(poses[0][0], poses[0][1], color="green")
    
    # Mark ending position with red dot
    # poses[-1] = last pose (Python negative indexing)
    plt.scatter(poses[-1][0], poses[-1][1], color="red")
    
    # Set axis limits to show full arena
    # Without this, matplotlib auto-scales to just where the robot went
    plt.xlim(0, width)
    plt.ylim(0, height)
    
    # Force equal scaling on both axes
    # Without this, circles look like eggs because the plot window
    # might be wider than it is tall (or vice versa)
    plt.axis('equal')
    
    # Display the plot window
    # Execution pauses here until you close the window
    plt.show()


def drive_circle(robot, steps):
    """
    Drive the robot in a circle by combining forward motion with rotation.
    
    A circle happens when you move forward while constantly turning.
    Think of driving a car with the steering wheel held at a fixed angle.
    
    The radius of the circle depends on the ratio of vx to omega:
    - Higher omega (faster turning) = tighter circle
    - Higher vx (faster forward) = larger circle
    
    :param robot: A MecanumRobot instance
    :param steps: How many update steps to run
    :return: List of poses tracing the circular path
    
    Math insight:
    - vx = 0, vy = 1, omega = 2 makes the robot strafe while rotating
    - The strafe keeps it moving sideways while it spins, creating a circle
    - You could also do vx = 1, vy = 0, omega = 1 for a different circle
    """
    poses = []
    # Strafe left (vy=1) while rotating counterclockwise (omega=2)
    # This creates a circular path
    robot.set_velocity(0, 1, 2)
    for i in range(steps):
        robot.update(0.1)
        poses.append(robot.get_pose())
    return poses


def drive_square(robot, steps_per_side):
    """
    Drive the robot in a square WITHOUT rotating.
    
    This is something ONLY mecanum/holonomic robots can do. A differential
    drive robot would have to stop and rotate 90° at each corner.
    
    The square is traced by:
    1. Moving forward (robot's forward direction)
    2. Strafing right (robot's right direction)
    3. Moving backward (opposite of forward)
    4. Strafing left (opposite of right)
    
    Throughout all four sides, theta stays constant at 0. The robot
    always faces the same direction - it just slides around.
    
    :param robot: A MecanumRobot instance
    :param steps_per_side: How many update steps per side of the square
    :return: List of poses tracing the square path
    
    Real-world application:
    - Warehouse robots moving between shelves in tight aisles
    - Camera robots that need to keep pointing at a subject while moving
    - Assembly line robots that need precise lateral positioning
    """
    poses = []
    
    # Side 1: Move forward (in robot's facing direction)
    # Robot faces right (theta=0), so this moves in +x direction
    robot.set_velocity(1, 0, 0)
    for i in range(steps_per_side):
        robot.update(0.1)
        poses.append(robot.get_pose())
    
    # Side 2: Strafe right (robot's right is world's -y... wait, check this)
    # Actually with vy=1, robot moves to its left, which is world +y
    # So vy=1 here moves up, not right. Let me trace through...
    # Robot at theta=0: vy=1 means move in robot's left = world +y (up)
    robot.set_velocity(0, 1, 0)
    for i in range(steps_per_side):
        robot.update(0.1)
        poses.append(robot.get_pose())
    
    # Side 3: Move backward (opposite of side 1)
    # vx=-1 moves in world -x direction (left)
    robot.set_velocity(-1, 0, 0)
    for i in range(steps_per_side):
        robot.update(0.1)
        poses.append(robot.get_pose())
    
    # Side 4: Strafe left (opposite of side 2)
    # vy=-1 moves in robot's right = world -y (down)
    robot.set_velocity(0, -1, 0)
    for i in range(steps_per_side):
        robot.update(0.1)
        poses.append(robot.get_pose())
    
    return poses


# This block only runs when executing this file directly
if __name__ == "__main__":
    # ========== Basic Movement Tests ==========
    
    # Test 1: Move forward
    # Robot at (5,5) facing right, move forward for 1 second
    # Should end at (6, 5) - moved 1 unit in x direction
    robot = MecanumRobot(x=5, y=5, bounds=(10, 10))
    print('Test-1: Forward movement')
    robot.set_velocity(vx=1, vy=0, omega=0)
    for i in range(10):
        robot.update(0.1)
    print("After forward:", robot.get_pose())
    
    # Test 2: Strafe (sideways movement)
    # Robot at (5,5) facing right, strafe left for 1 second
    # Should end at (5, 6) - moved 1 unit in y direction
    # (Robot's left is world's up when facing right)
    print('Test-2: Strafe movement')
    robot = MecanumRobot(x=5, y=5, bounds=(10, 10))
    robot.set_velocity(vx=0, vy=1, omega=0)
    for i in range(10):
        robot.update(0.1)
    print("After strafe left:", robot.get_pose())
    
    # Test 3: Strafe while rotated
    # Robot at (5,5) facing UP (theta=π/2), strafe left for 1 second
    # Robot's left when facing up is world's left (-x direction)
    # Should end at (4, 5) - moved 1 unit in -x direction
    print('Test-3: Strafe while rotated')
    robot = MecanumRobot(x=5, y=5, theta=math.pi/2, bounds=(10, 10))
    robot.set_velocity(vx=0, vy=1, omega=0)
    for i in range(10):
        robot.update(0.1)
    print("Rotated + strafe left:", robot.get_pose())
    
    # ========== Shape Tests ==========
    
    # Test 4: Autonomous wall-following behavior
    # Robot starts at (1, 5), runs toward right wall, then strafes down
    # Creates an L-shaped path
    print('Test-4: Autonomous wall avoidance')
    robot = MecanumRobot(x=1, y=5, bounds=(10, 10))
    poses = robot.run(200)
    visualize(poses, (10, 10))
    
    # Test 5: Circle - combining forward/strafe with rotation
    # Demonstrates smooth circular motion
    print('Test-5: Circle')
    robot = MecanumRobot(x=8, y=8, bounds=(20, 20))
    poses = drive_circle(robot, 200)
    visualize(poses, (20, 20))
    
    # Test 6: Square without rotation
    # Demonstrates mecanum's unique ability to strafe
    # Theta stays at 0 throughout - robot never turns!
    print('Test-6: Square without rotation')
    robot = MecanumRobot(x=4, y=4, bounds=(20, 20))
    poses = drive_square(robot, 10)
    print("Final pose:", robot.get_pose())
    print("Theta changed?", robot.theta != 0)
    visualize(poses, (20, 20))