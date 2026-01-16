"""
Point Robot Simulator
---------------------
A simple robot that exists in a 2D coordinate space. The robot has a position
(x, y) and an orientation (theta, in radians). It can move forward and rotate.

This is a "point robot" - it has no physical size, just a location and direction.
It can magically rotate in place and move in perfectly straight lines. Real robots
don't work this way, but point robots are the simplest model for learning
fundamental robotics concepts like:

- Pose (position + orientation)
- Trigonometry for movement
- Boundary detection
- Distance sensing via ray casting
- Reactive autonomous behavior (sense-decide-act)

This is Challenge 1 in the robotics learning path.
Challenge 2 (diff_drive_robot.py) adds realistic two-wheel physics.
"""

# math module gives us cos(), sin(), and pi for trigonometry
import math

# matplotlib is a plotting library - pyplot is the part we use to draw graphs
import matplotlib.pyplot as plt


class Robot:
    """
    A 2D point robot that can move, rotate, sense walls, and navigate autonomously.
    
    The robot uses a coordinate system where:
    - x increases to the right
    - y increases upward
    - theta = 0 means facing right (positive x direction)
    - theta = pi/2 means facing up (positive y direction)
    - theta = pi means facing left (negative x direction)
    - theta increases counterclockwise
    """
    
    def __init__(self, x=0, y=0, theta=0, bounds=None):
        """
        Initialize a new Robot.
        
        :param x: Starting x position (default 0, the origin)
        :param y: Starting y position (default 0, the origin)
        :param theta: Starting orientation in radians (default 0, facing right)
        :param bounds: Optional tuple (width, height) defining arena size.
        
        Arena spans from (0,0) to (width, height).
        If None, robot can move infinitely in any direction.
        """
        # Store the robot's position and orientation as instance attributes
        # These will change as the robot moves and rotates
        self.x = x
        self.y = y
        self.theta = theta
        self.bounds = bounds
        
        # Validate bounds if provided - negative arena size makes no sense
        if self.bounds is not None:
            width, height = self.bounds  # Unpack tuple into two variables
            if width < 0 or height < 0:
                # Raise an error immediately - fail fast principle
                raise ValueError("bounds must be (width, height) with non-negative values")
    
    def move_forward(self, distance):
        """
        Move the robot forward in the direction it's currently facing.
        
        The key insight here is trigonometry:
        - cos(theta) gives the x-component of the direction vector
        - sin(theta) gives the y-component of the direction vector
        
        Example: if theta=0 (facing right), cos(0)=1 and sin(0)=0
        so movement is purely in the x direction
        
        Example: if theta=pi/2 (facing up), cos(pi/2)=0 and sin(pi/2)=1
        so movement is purely in the y direction
        
        Example: if theta=pi/4 (facing 45° up-right), cos and sin are both ~0.707
        so movement is split equally between x and y
        
        :param distance: How far to move (in the same units as x, y)
        :return: True if move succeeded, False if blocked by boundary
        """
        # Calculate where the robot WOULD end up if it moves
        # Don't actually move yet - check boundaries first
        new_x = self.x + distance * math.cos(self.theta)
        new_y = self.y + distance * math.sin(self.theta)
        
        # If there are boundaries, check if the new position is valid
        if self.bounds is not None:
            width, height = self.bounds
            
            # Check all four boundary conditions:
            # - new_x >= 0 (not past left wall)
            # - new_x <= width (not past right wall)
            # - new_y >= 0 (not past bottom wall)
            # - new_y <= height (not past top wall)
            # The expression "0 <= new_x <= width" is Python shorthand for all of these
            if not (0 <= new_x <= width and 0 <= new_y <= height):
                return False  # Move would go out of bounds, don't do it
        
        # Move is valid (or no bounds exist), so update position
        self.x = new_x
        self.y = new_y
        return True
    
    def rotate(self, angle):
        """
        Rotate the robot by a given angle.
        
        Positive angles rotate counterclockwise (standard math convention).
        Negative angles rotate clockwise.
        
        Example: rotate(math.pi/2) turns the robot 90° counterclockwise
        Example: rotate(-math.pi/4) turns the robot 45° clockwise
        
        Note: theta can grow beyond 2*pi or go negative. That's fine -
        cos() and sin() handle any angle correctly.
        
        :param angle: Rotation amount in radians
        """
        # Simply add the angle to current orientation
        # += is shorthand for: self.theta = self.theta + angle
        self.theta += angle
    
    def get_pose(self):
        """
        Get the robot's current pose (position + orientation).
        
        In robotics, "pose" specifically means the combination of position 
        and orientation. This is different from just "position" which would
        only be (x, y).
        
        :return: Tuple of (x, y, theta)
        """
        return (self.x, self.y, self.theta)
    
    def sense_distance(self):
        """
        Simulate a forward-facing distance sensor.
        
        Returns the distance from the robot to the nearest wall in the 
        direction it's currently facing. This is called "ray casting" -
        we're shooting an imaginary ray forward and seeing where it hits.
        
        The math:
        - For horizontal walls (left/right), we care about x-component of movement
        - For vertical walls (top/bottom), we care about y-component of movement
        - distance = (gap to wall) / (component of direction toward that wall)
        
        Example: Robot at (3, 5) facing right (theta=0) in 10x10 arena
        - Gap to right wall: 10 - 3 = 7
        - x-component of direction: cos(0) = 1
        - Distance to right wall: 7 / 1 = 7
        
        We only consider walls we're moving TOWARD (positive result).
        Walls behind us give negative results and are ignored.
        
        :return: Distance to nearest wall, or infinity if no bounds
        """
        # No boundaries means infinite distance in all directions
        if self.bounds is None:
            return float('inf')  # Python's way of representing infinity
        
        width, height = self.bounds
        
        # We'll collect distances to all walls we're facing, then take minimum
        distances = []
        
        # Check RIGHT wall (x = width)
        # Only relevant if we're moving rightward (cos > 0)
        if math.cos(self.theta) > 0:
            # Gap to wall divided by how fast we're approaching it
            distance_to_right = (width - self.x) / math.cos(self.theta)
            distances.append(distance_to_right)
        
        # Check LEFT wall (x = 0)
        # Only relevant if we're moving leftward (cos < 0)
        if math.cos(self.theta) < 0:
            # Note: (0 - self.x) is negative, cos(theta) is negative
            # Negative / negative = positive distance
            distance_to_left = (0 - self.x) / math.cos(self.theta)
            distances.append(distance_to_left)
        
        # Check TOP wall (y = height)
        # Only relevant if we're moving upward (sin > 0)
        if math.sin(self.theta) > 0:
            distance_to_top = (height - self.y) / math.sin(self.theta)
            distances.append(distance_to_top)
        
        # Check BOTTOM wall (y = 0)
        # Only relevant if we're moving downward (sin < 0)
        if math.sin(self.theta) < 0:
            distance_to_bottom = (0 - self.y) / math.sin(self.theta)
            distances.append(distance_to_bottom)
        
        # Return the closest wall
        return min(distances)
    
    def step(self):
        """
        Execute one step of the robot's behavior algorithm.
        
        This implements simple reactive behavior:
        - If wall is close (< 1.0 units), turn right 45°
        - Otherwise, move forward 0.5 units
        
        This is a "sense-decide-act" cycle:
        1. SENSE: Check distance to wall
        2. DECIDE: Too close? Turn. Otherwise? Go forward.
        3. ACT: Execute the chosen action
        
        The robot has no memory of past positions - it only reacts to 
        current sensor readings. This is called "reactive behavior".
        
        :return: String indicating what action was taken
        """
        if self.sense_distance() < 1.0:
            # Wall is close! Turn right (negative angle = clockwise)
            # Wait, we're using positive pi/4... that's counterclockwise (left)
            # This makes the robot turn left when it sees a wall
            self.rotate(math.pi / 4)
            return 'rotated'
        else:
            # Path is clear, move forward
            self.move_forward(0.5)
            return 'moved'
    
    def run(self, max_steps):
        """
        Run the robot's behavior for multiple steps.
        
        Each step, the robot senses and acts, then we record its pose.
        This builds up a history of everywhere the robot has been.
        
        :param max_steps: Maximum number of steps to take
        :return: List of poses [(x1,y1,theta1), (x2,y2,theta2), ...]
        """
        steps_list = []  # Will hold all poses
        
        # Loop max_steps times
        # We use 'i' as the loop variable but don't actually use it
        # We just need to repeat the action max_steps times
        for i in range(max_steps):
            self.step()  # Execute one sense-decide-act cycle
            steps_list.append(self.get_pose())  # Record where we ended up
        
        return steps_list


def visualize(poses, bounds):
    """
    Draw the robot's path through the arena.
    
    This is a standalone function (not a method) because it doesn't need
    access to the robot's internal state - it just needs the recorded poses.
    
    :param poses: List of (x, y, theta) tuples from robot.run()
    :param bounds: Tuple (width, height) defining arena size for axis limits
    """
    # Unpack bounds into width and height
    width, height = bounds
    
    # Extract x coordinates from all poses
    # List comprehension: for each pose tuple, grab the first element (index 0)
    # poses = [(1,2,0), (3,4,0.5), ...] becomes x_values = [1, 3, ...]
    x_values = [pose[0] for pose in poses]
    
    # Same for y coordinates (index 1)
    y_values = [pose[1] for pose in poses]
    
    # Create a new figure (drawing canvas)
    plt.figure()
    
    # Plot the path as a connected line
    # plt.plot connects all the points in order
    plt.plot(x_values, y_values)
    
    # Mark the starting position with a green dot
    # poses[0] is the first pose, [0] and [1] extract x and y
    plt.scatter(poses[0][0], poses[0][1], color="green")
    
    # Mark the ending position with a red dot
    # poses[-1] is the last pose (Python negative indexing)
    plt.scatter(poses[-1][0], poses[-1][1], color="red")
    
    # Set axis limits to show the full arena
    # Without this, matplotlib would auto-scale to just the path
    plt.xlim(0, width)
    plt.ylim(0, height)
    
    # Display the plot window
    # Code pauses here until you close the window
    plt.show()


# This block only runs when you execute this file directly
# It won't run if you import this file as a module from another file
if __name__ == "__main__":
    
    # === Test 1: Basic movement with bounds ===
    robot = Robot(0, 0, 0, bounds=(10, 10))
    # Robot at origin, facing right, in 10x10 arena
    # Move forward 5 units - should succeed and end at (5, 0)
    print(robot.get_pose(), robot.move_forward(5))
    
    # === Test 2: Movement blocked by boundary ===
    robot = Robot(9, 5, 0, bounds=(10, 10))
    # Robot at (9, 5), facing right
    # Move forward 3 would put it at x=12, past the wall at x=10
    # Should return False and position should stay at (9, 5)
    print(robot.get_pose(), robot.move_forward(3))
    
    # === Test 3: Movement blocked by left boundary ===
    robot = Robot(0, 0, math.pi, bounds=(10, 10))
    # Robot at origin, facing LEFT (theta = pi)
    # Moving forward would put it at negative x
    # Should return False
    print(robot.get_pose(), robot.move_forward(1))
    
    # === Test 4: Distance sensor facing right ===
    robot = Robot(5, 5, 0, bounds=(10, 10))
    # Robot in center, facing right
    # Distance to right wall (x=10) should be 10-5 = 5.0
    print(robot.sense_distance())  # Should be 5.0
    
    # === Test 5: Distance sensor facing left ===
    robot = Robot(5, 5, math.pi, bounds=(10, 10))
    # Robot in center, facing left
    # Distance to left wall (x=0) should be 5-0 = 5.0
    print(robot.sense_distance())  # Should be 5.0
    
    # === Test 6: Distance sensor with no bounds ===
    robot = Robot(0, 0, 0, bounds=None)
    # No arena walls = infinite distance
    print(robot.sense_distance())  # Should be inf
    
    # === Test 7: Run behavior and print poses ===
    robot = Robot(1, 1, 0, bounds=(10, 10))
    poses = robot.run(20)  # Run for 20 steps
    for pose in poses:
        print(pose)  # Watch the robot move and turn
    
    # === Test 8: Visualize 100 steps ===
    robot = Robot(1, 1, 0, bounds=(10, 10))
    poses = robot.run(100)
    visualize(poses, (10, 10))  # Opens a plot window
    
    # === Test 9: Start in corner, visualize 200 steps ===
    robot = Robot(0.5, 0.5, 0, bounds=(10, 10))
    poses = robot.run(200)
    visualize(poses, (10, 10))  # More steps = more bouncing around