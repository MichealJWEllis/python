"""
Differential Drive Robot Simulator
----------------------------------
A robot with two independently controlled wheels. Unlike the point robot
that could magically rotate in place and move forward, this robot moves
by controlling the speed of its left and right wheels.

Real-world examples of differential drive robots:
- Roomba vacuum cleaners
- Warehouse robots (like Amazon Kiva)
- Many research robots
- Tank-style vehicles

Key concept: The robot steers by varying the relative speeds of its wheels,
not by having a separate steering mechanism like a car.
"""

# math module gives us cos(), sin(), and pi for trigonometry
import math

# matplotlib is a plotting library - pyplot is the part we use to draw graphs
import matplotlib.pyplot as plt


class DiffDriveRobot:
    """
    A differential drive robot with two wheels.
    
    The robot's motion is determined entirely by the speeds of its two wheels:
    - Both wheels same speed → straight line
    - Right wheel faster → curves left
    - Left wheel faster → curves right
    - Wheels opposite speeds → spins in place
    
    Coordinate system (same as point robot):
    - x increases to the right
    - y increases upward
    - theta = 0 means facing right (positive x)
    - theta increases counterclockwise
    """
    
    def __init__(self, wheel_base, x=0, y=0, theta=0, bounds=None):
        """
        Initialize a differential drive robot.
        
        :param wheel_base: Distance between the two wheels (in same units as x, y).
                          This affects turning behavior - wider robots turn slower.
                          A typical small robot might have wheel_base of 0.2-0.5 meters.
                          We use 1.0 for simplicity.
        :param x: Starting x position (default: 0, at origin)
        :param y: Starting y position (default: 0, at origin)
        :param theta: Starting orientation in radians (default: 0, facing right)
        :param bounds: Optional tuple (width, height) defining arena size.
                      Arena spans from (0,0) to (width, height).
                      If None, robot exists in infinite space.
        """
        # Position and orientation - these change as robot moves
        self.x = x
        self.y = y
        self.theta = theta
        
        # Physical property of the robot - doesn't change after construction
        # Wider wheel_base = slower turning for same wheel speed difference
        self.wheel_base = wheel_base
        
        # Arena boundaries for collision detection and sensing
        self.bounds = bounds
        
        # Wheel speeds - start at zero (robot is stationary)
        # These are set by set_wheel_speeds() and used by update()
        # Units are distance per second (e.g., meters/second)
        self.left = 0   # Left wheel speed
        self.right = 0  # Right wheel speed
    
    def set_wheel_speeds(self, left=0, right=0):
        """
        Set the speed of each wheel.
        
        This doesn't move the robot - it just stores the commanded speeds.
        The robot actually moves when update() is called.
        
        Think of this like pressing the gas pedal - you're commanding
        a speed, but movement happens over time.
        
        :param left: Left wheel speed (positive = forward, negative = backward)
        :param right: Right wheel speed (positive = forward, negative = backward)
        
        Examples:
        - set_wheel_speeds(2, 2)   → both wheels forward, same speed
        - set_wheel_speeds(1, 3)   → right faster, will curve left
        - set_wheel_speeds(-1, 1)  → wheels opposite, spins in place
        - set_wheel_speeds(-2, -2) → both backward, reverses straight
        """
        self.left = left
        self.right = right
    
    def update(self, dt):
        """
        Update the robot's position based on wheel speeds over time interval dt.
        
        This is where the physics happens. Given the current wheel speeds,
        calculate how far the robot moves and rotates in dt seconds.
        
        :param dt: Time step in seconds (e.g., 0.1 = 100 milliseconds)
                  Smaller dt = more accurate simulation but more computation
                  Typical values: 0.01 to 0.1 seconds
        
        The Math Explained:
        -------------------
        
        1. LINEAR VELOCITY (how fast the robot moves forward/backward):
        
           linear_velocity = (right + left) / 2
           
           This is the average of both wheel speeds. Why?
           - If both wheels go forward at speed 2, the robot goes forward at speed 2
           - If left=1 and right=3, the center of the robot moves at speed 2
             (while also turning, handled separately)
           - If left=-2 and right=2, average is 0 - no forward movement, just spinning
        
        2. ANGULAR VELOCITY (how fast the robot rotates):
        
           angular_velocity = (right - left) / wheel_base
           
           This is the difference in wheel speeds, scaled by how far apart they are.
           
           Why the difference?
           - If both wheels go the same speed, difference is 0, no rotation
           - If right wheel is faster, the robot curves left (positive rotation)
           - If left wheel is faster, the robot curves right (negative rotation)
           
           Why divide by wheel_base?
           - Wider robots need more wheel speed difference to turn at the same rate
           - Imagine a shopping cart vs a bus - same wheel speed difference,
             very different turning rates
        
        3. POSITION UPDATE:
        
           x += linear_velocity * cos(theta) * dt
           y += linear_velocity * sin(theta) * dt
           
           Same trig as the point robot:
           - cos(theta) gives the x-component of the direction
           - sin(theta) gives the y-component of the direction
           - Multiply by dt because velocity is "per second" but we're only
             simulating a fraction of a second
        
        4. ORIENTATION UPDATE:
        
           theta += angular_velocity * dt
           
           Angular velocity is "radians per second", so multiply by dt
           to get how many radians to rotate in this time step.
        """
        # Calculate linear velocity: average of both wheels
        # Units: if wheel speeds are in m/s, this is also m/s
        linear_velocity = (self.right + self.left) / 2
        
        # Calculate angular velocity: difference divided by wheel spacing
        # Units: radians per second
        # Positive = counterclockwise (turning left)
        # Negative = clockwise (turning right)
        angular_velocity = (self.right - self.left) / self.wheel_base
        
        # Update x position
        # linear_velocity * dt = distance traveled this time step
        # cos(theta) = fraction of that distance in the x direction
        self.x += linear_velocity * math.cos(self.theta) * dt
        
        # Update y position
        # sin(theta) = fraction of that distance in the y direction
        self.y += linear_velocity * math.sin(self.theta) * dt
        
        # Update orientation
        # angular_velocity * dt = angle rotated this time step (in radians)
        self.theta += angular_velocity * dt
    
    def get_pose(self):
        """
        Get the robot's current pose (position + orientation).
        
        "Pose" in robotics specifically means position AND orientation together.
        This is different from just "position" which would only be (x, y).
        
        :return: Tuple of (x, y, theta) representing full pose
        """
        return (self.x, self.y, self.theta)
    
    def sense_distance(self):
        """
        Simulate a forward-facing distance sensor (identical to point robot).
        
        Returns the distance to the nearest wall in the direction the robot
        is currently facing. This uses ray casting - imagine shooting a laser
        forward and measuring where it hits.
        
        The robot's drive system (differential vs point) doesn't affect sensing.
        The sensor just measures distance in the facing direction.
        
        :return: Distance to nearest wall in facing direction, or infinity if no bounds
        
        The Math (same as point robot):
        -------------------------------
        For each wall, calculate: (gap to wall) / (component of direction toward wall)
        
        Only count walls we're moving toward (positive result).
        Return the minimum (closest wall).
        """
        # No boundaries = infinite distance (nothing to sense)
        if self.bounds is None:
            return float('inf')
        
        width, height = self.bounds
        distances = []
        
        # RIGHT WALL (x = width)
        # cos(theta) > 0 means we're facing rightward (toward this wall)
        if math.cos(self.theta) > 0:
            # (width - self.x) = gap between robot and right wall
            # cos(theta) = how directly we're facing that wall
            # Division gives the distance along our facing direction
            distance_to_right = (width - self.x) / math.cos(self.theta)
            distances.append(distance_to_right)
        
        # LEFT WALL (x = 0)
        # cos(theta) < 0 means we're facing leftward
        if math.cos(self.theta) < 0:
            # (0 - self.x) is negative (we're to the right of the wall)
            # cos(theta) is negative (we're facing left)
            # Negative / negative = positive distance
            distance_to_left = (0 - self.x) / math.cos(self.theta)
            distances.append(distance_to_left)
        
        # TOP WALL (y = height)
        # sin(theta) > 0 means we're facing upward
        if math.sin(self.theta) > 0:
            distance_to_top = (height - self.y) / math.sin(self.theta)
            distances.append(distance_to_top)
        
        # BOTTOM WALL (y = 0)
        # sin(theta) < 0 means we're facing downward
        if math.sin(self.theta) < 0:
            distance_to_bottom = (0 - self.y) / math.sin(self.theta)
            distances.append(distance_to_bottom)
        
        # Return distance to the closest wall we're facing
        return min(distances)
    
    def step(self):
        """
        Execute one step of autonomous wall-avoidance behavior.
        
        This is the "sense-decide-act" cycle:
        1. SENSE: Check distance to nearest wall
        2. DECIDE: Too close? Spin. Otherwise? Drive forward.
        3. ACT: Set wheel speeds and update position
        
        Behavior:
        - If wall is within 2.0 units: spin in place to find clear direction
        - Otherwise: drive straight forward
        
        Unlike the point robot's step() which called rotate() or move_forward(),
        this uses set_wheel_speeds() to achieve the same behaviors:
        - Spinning: opposite wheel speeds (-1, 1) gives zero linear velocity
        - Driving: equal wheel speeds (1, 1) gives straight line motion
        """
        if self.sense_distance() <= 2.0:
            # Wall is close! Spin in place to find a clear direction.
            # Wheels at opposite speeds: linear_velocity = (-1+1)/2 = 0 (no forward motion)
            # angular_velocity = (1-(-1))/wheel_base = 2/wheel_base (spinning)
            self.set_wheel_speeds(-1, 1)
            self.update(0.1)
        else:
            # Path is clear. Drive forward.
            # Equal wheel speeds: linear_velocity = (1+1)/2 = 1 (forward motion)
            # angular_velocity = (1-1)/wheel_base = 0 (no turning)
            self.set_wheel_speeds(1, 1)
            self.update(0.1)
    
    def run(self, max_steps):
        """
        Run the autonomous behavior for multiple steps.
        
        Each step, the robot senses walls, decides what to do, and moves.
        We record the pose after each step to track the robot's path.
        
        :param max_steps: How many sense-decide-act cycles to run
        :return: List of poses [(x1,y1,theta1), (x2,y2,theta2), ...]
                 Can be passed to visualize() to see the path
        """
        steps_list = []
        
        for i in range(max_steps):
            # Execute one sense-decide-act cycle
            self.step()
            # Record where we ended up
            steps_list.append(self.get_pose())
        
        return steps_list


def visualize(poses, bounds):
    """
    Draw the robot's path through the arena.
    
    This is a standalone function (not a class method) because it doesn't
    need access to the robot's internal state - it just takes data and draws.
    
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
    
    # Display the plot window
    # Execution pauses here until you close the window
    plt.show()


# This block only runs when executing this file directly
# It won't run if you import this file as a module
if __name__ == "__main__":
    # Create a differential drive robot
    # - wheel_base=1.0: wheels are 1 unit apart
    # - starting at (1, 1): not in a corner, has room to move
    # - bounds=(10, 10): 10x10 arena
    robot = DiffDriveRobot(wheel_base=1.0, x=1, y=1, bounds=(10, 10))
    
    # Run autonomous behavior for 300 steps
    # Each step is 0.1 seconds, so this simulates 30 seconds of movement
    poses = robot.run(300)
    
    # Visualize the path
    # Should see curves (not sharp corners like point robot)
    # Robot curves smoothly when turning, then drives straight
    visualize(poses, (10, 10))