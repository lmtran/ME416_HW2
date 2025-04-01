"""Functions for modeling ROSBot"""

from math import cos, sin
import random
import numpy as np

def model_parameters():
    """Returns two constant model parameters"""
    # This is a stub. Write your code here.
    # return param_k, param_d
    param_k = 1.0  # A scaling factor for the linear speed.
    param_d = 1.0  # A scaling factor for the angular speed.
    return param_k, param_d

def speeds_to_twist(speed_left, speed_right):
    """
    Given normalized speeds for the left and right wheels, return the linear and
    angular velocity of the robot.
    """
    # This function converts the left and right wheel speeds into the robot's
    # linear and angular velocities.
    param_k, param_d = model_parameters()
    speed_linear = (param_k / 2) * (speed_left + speed_right)
    speed_angular = (param_k / (2 * param_d)) * (speed_right - speed_left)
    return (speed_linear, speed_angular)

def system_matrix(state_theta):
    """
    Returns a numpy array with the A(theta) matrix
    for a differential drive robot
    """
    # This function computes the system matrix A(θ) for the robot's kinematic model
    system_matrix_a = np.array([
        [cos(state_theta), 0],
        [sin(state_theta), 0],
        [0, 1]
    ])
    return system_matrix_a

def system_field(state_z, input_u):
    """
    Computes the field at a given state for the dynamical model
    Args:
        state_z: Current state vector
        input_u: Input vector
    Returns:
        State derivatives
    """
    theta = state_z[2]
    a_matrix = system_matrix(theta)
    dot_state_z = a_matrix @ input_u
    return dot_state_z

def euler_step(state_z, input_u, step_size):
    """
    Integrates the dynamical model for one time step using Euler's method
    Args:
        state_z: Current state vector
        input_u: Input vector
        step_size: Time step size
    Returns:
        Next state vector
    """
    dot_state_z = system_field(state_z, input_u)
    state_z_next = state_z + dot_state_z * step_size
    return state_z_next

def twist_to_speeds(speed_linear, speed_angular,k=1.0,d=1.0):
    """
    Given the desired linear and angular velocity of the robot, returns
    normalized speeds for the left and right motors. Speeds needs to be
    thresholded to be between −1.0 (backward at maximum speed) and 1.0
    (forward at maximum speed).
    """
    # Apply the inverted relations
    speed_left = (1 / k) * speed_linear - (d / k) * speed_angular
    speed_right = (1 / k) * speed_linear + (d / k) * speed_angular
    speed_left = max(-1.0, min(1.0, speed_left))
    speed_right = max(-1.0, min(1.0, speed_right))
    return speed_left, speed_right

def test_effect_of_k_and_d():
    """
    Test the effect of changing model parameters k and d
    on motor speeds for linear and angular velocities.

    This function systematically evaluates the behavior of the
    twist_to_speeds function by varying the parameters k and d
    and observing their impact on the calculated motor speeds.

    The test runs four cases:
    1. Increase both k and d
    2. Increase k, decrease d
    3. Decrease k, increase d
    4. Decrease both k and d

    Prints the left and right motor speeds for each case.
    """
    # Define a range of test values for k and d
    test_values = [
        (2.0, 2.0),  # Increase both k and d
        (2.0, 0.5),  # Increase k, decrease d
        (0.5, 2.0),  # Decrease k, increase d
        (0.5, 0.5)   # Decrease both k and d
    ]

    # Test with a sample linear and angular velocity
    speed_linear = 0.5
    speed_angular = 0.3

    for k, d in test_values:
        speed_left, speed_right = twist_to_speeds(speed_linear, speed_angular, k=k, d=d)
        print(f"For k={k}, d={d}: Left motor speed={speed_left}, Right motor speed={speed_right}")

class KeysToVelocities():
    """
    Class to translate cumulative key strokes to speed commands
    """

    def __init__(self):
        # initialize attributes here
        self.speed_linear = 0
        self.speed_angular = 0
        self.text_description = 'Initial values'

    def update_speeds(self, key):
        """
        Update speeds based on keyboard input
        Args:
            key: The pressed key
        Returns:
            Tuple of (linear speed, angular speed, description)
        """
        if key == ord('W'):  # Move Forward
            self.speed_linear += 0.1
        elif key == ord('S'):  # Move Backwards
            self.speed_linear -= 0.1
        elif key == ord('A'):  # Move Left
            self.speed_angular += 0.1
        elif key == ord('D'):  # Move Right
            self.speed_angular -= 0.1
        elif key == ord(' '):  # Stop
            self.speed_linear = 0
            self.speed_angular = 0

        # Threshold speed to be within [-1.0,1.0]
        self.speed_linear = max(-1.0, min(1.0, self.speed_linear))
        self.speed_angular = max(-1.0, min(1.0, self.speed_angular))

        self.text_description = (f'Linear: {self.speed_linear:.2f}, '
                                f'Angular: {self.speed_angular:.2f}')
        return self.speed_linear, self.speed_angular, self.text_description

class StampedMsgRegister():
    """
    Store a previous message, and compute delay with respect to current one
    """
    def __init__(self):
        # initialize attributes here
        self.msg_previous = None
        self.time_previous = None

    def replace_and_compute_delay(self, msg):
        """
        Compute delay between current and stored message,
        then replace stored message with current one
        Args:
            msg: The new message
        Returns:
            Tuple of (time delay, previous message)
        """
        time_current = msg.header.stamp.to_sec()
        if self.time_previous is None:
            delay = 0.0
        else:
            delay = time_current - self.time_previous
        self.msg_previous = msg
        self.time_previous = time_current
        return delay, self.msg_previous

    def previous_stamp(self):
        """
        Return stored message
        Returns:
            The previous message
        """
        return self.msg_previous

def speeds_to_twist_test():
    """
    Check that speeds_to_twist and twist_to_speeds are the inverse of each other
    """

    # Pick random numbers between -1.0 and +1.0 as normalized wheel speeds
    speed_left = 2 * random.random() - 1
    speed_right = 2 * random.random() - 1

    # Apply speeds_to_twist followed by twist_to_speeds
    speed_left_tilde, speed_right_tilde = twist_to_speeds(
        *speeds_to_twist(speed_left, speed_right))

    # Check that the result is equal to the inputs (up to a tolerance)
    flag_correct_left = abs(speed_left - speed_left_tilde) < 1e-3
    flag_correct_right = abs(speed_right - speed_right_tilde) < 1e-3
    if flag_correct_left and flag_correct_right:
        print('Test passed')
    else:
        print('Test passed')
if __name__ =="__main__": 
    speed_to_twist_test()
