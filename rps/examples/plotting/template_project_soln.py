# Simulator Skeleton File - Project 1
# This file provides the bare - bones requirements for interacting with the Robotarium.
# Note that this code won't actually run.  You'll have to insert your own algorithm!
# If you want to see some working code, check out the 'examples' folder.
import math

import numpy as np
from scipy.io import savemat

import rps.robotarium as robotarium
from rps.utilities.barrier_certificates import create_unicycle_barrier_certificate_with_boundary

# Get Robotarium object used to communicate with the robots / simulator
N = 1
r = robotarium.Robotarium(number_of_robots=N, show_figure=True)
data = []

# Select the number of iterations for the experiment.
iterations = 3000 # Do not change

# Create a boundary barrier
uni_barrier_certificate = create_unicycle_barrier_certificate_with_boundary()

# Other important variables
target1 = np.array([[-0.5, 0, 0]]).T
target2 = np.array([[0.5, 0, 0]]).T
k = 1

# ######################### Place Static Variables Here ##################
# ############### Do not modify anything outside this area ###############
# var = 0
position_error = 0.01 # accuracy for position for closed loop control (fixed)
rotation_error = 0.1 # accuracy for orientation for closed loop control (fixed)
constant_velocity = 0.08 # constant velocity for both controls
time_per_iteration = 0.033 # time taken for each control loop

state = 1 # starting state of finite state machine
k = 4 # use a higher gain for closed loop
controller_state = 0

# draw markers for targets and the path between them
# viscircles([target1(1:2) target2(1:2) target1(1:2) target2(1:2)]',[0.01;0.01; 0.07;0.07]);
# line([target1(1) target2(1)] , [target1(2) target2(2)]);

# ############### Do not modify anything outside this area ###############
# ########################################################################

# ######################## Place Helper Functions Here ##################
# ############## Do not modify anything outside this area ###############
# def foo(b)

# uses concepts from example controller in simulator
def parking_controller(pose, target, current_state = 0):
    # control parameters
    distance_error = 0.005
    distance_epsilon = 0.001
    orientation_error = 0.02
    angle_epsilon = 0.001
    approach_angle_gain = 1
    desired_angle_gain = 2.7
    rotation_error_gain = 1

    u = np.array([[0.0, 0.0]]).T
    done = False

    translate = target[:2] - pose[:2]
    distance = np.linalg.norm(translate, axis=0)
    orientation = wrap(pose[2] - target[2])

    if current_state == 0 and distance > distance_error - distance_epsilon:
        rotation = wrap(np.arctan2(translate[1], translate[0]) - target[2])
        approach_angle = wrap(rotation - orientation)
        if abs(approach_angle) < angle_epsilon:
            u = np.array([[approach_angle_gain * distance, 0]]).T
        else:
            ca = np.cos(approach_angle)
            sa = np.sin(approach_angle)
            u[0] = approach_angle_gain * distance * ca
            u[1] = (desired_angle_gain * approach_angle
                    + approach_angle_gain * ((ca * sa) / approach_angle)
                    * (approach_angle + rotation_error_gain * rotation))
    elif abs(orientation) > orientation_error:
        current_state = 1
        if (distance > distance_error):
            current_state = 0
        u[1] = -2 * orientation
    elif current_state > 3:
        done = True
        current_state = 0
    else:
        current_state = current_state + 1
    return u, done, current_state

# moves robot a fixed number of steps at a constant speed with no angular velocity
def open_loop_controller(steps, velocity, step_count = 0):
    u = np.array([[0.0, 0.0]]).T
    done = False

    if step_count < steps:
        step_count += 1
        u = np.array([[velocity, 0.0]]).T
    else:
        done = True
        step_count = 0
    return u, done, step_count


# moves robot to target at constant speed and adjusting angle based on feedback
def closed_loop_controller(pose, target, constant_velocity, gain, distance_error, orientation_error, current_state = 0):
    desired_angle_gain = 2

    u = np.array([[0.0, 0.0]]).T
    done = False

    pose_diff = target[:2] - pose[:2]
    distance = np.linalg.norm(pose_diff, axis=0)
    desired_theta = np.arctan2(pose_diff[1], pose_diff[0])
    alpha = wrap(desired_theta - pose[2]).item()
    orientation = wrap(target[2] - pose[2]).item()
    if current_state == 0 and distance > distance_error:
        u = np.array([[constant_velocity, gain * alpha]]).T
    elif abs(orientation) > orientation_error:
        u = np.array([[0, desired_angle_gain * orientation]]).T
    else:
        done = True
        current_state = 0
    return u, done, current_state


# rotates robot to point to a target
def direction_controller(pose, target, current_state = 0):
    rotation_error = 0.02
    desired_angle_gain = 2

    u = np.array([[0.0, 0.0]]).T
    done = False

    translate = target[:2] - pose[:2]
    target_angle = wrap(np.arctan2(translate[1], translate[0]))
    desired_angle = wrap(target_angle - pose[2])
    if (abs(desired_angle) > rotation_error):
        current_state = 0
        u = np.array([[0, desired_angle_gain * desired_angle]]).T
    elif current_state > 2:
        done = True
        current_state = 0
    else:
        current_state += 1

    return u, done, current_state


def number_of_iterations(pose, target, velocity, time_per_iteration):
    distance = math.dist(pose[:2], target[:2])
    return round(distance / (velocity * time_per_iteration))


def wrap(theta):
    return np.arctan2(np.sin(theta), np.cos(theta))

# ############## Do not modify anything outside this area ###############
# #######################################################################



# Iterate for the previously specified number of iterations
for t in range(1, iterations+1):
    # Retrieve the most recent poses from the Robotarium.  The time delay is
    # approximately 0.033 seconds
    p = r.get_poses()

    # ######################## Place Algorithm Here #########################
    # ############## Do not modify anything outside this area ###############
    # u = ???;
    u = np.array([[0.0, 0.0]]).T
    done = 0 # signals the current state has ended and transition to next
    if state in [1, 4, 6]:
        # command robot to target 1
        u, done, controller_state = parking_controller(p, target1, controller_state)
    elif state in [2]:
        # task 3 part a - point robot to target 2
        u, done, controller_state = direction_controller(p, target2, controller_state)
    elif state in [3]:
        # task 3 part b - move at constant velocity without feedback
        u, done, step_count = open_loop_controller(steps, constant_velocity, step_count)
    elif state in [5]:
        # task 5 - move at constant velocity using feedback
        u, done, controller_state = closed_loop_controller(p, target2, constant_velocity, k, position_error, rotation_error, controller_state)

    if done:
        print(f'State {state} completed at iteration {t}. Pose {p.T}')
        state = state + 1
        if state == 3:
            steps = number_of_iterations(p, target2, constant_velocity, time_per_iteration)
            step_count = 0
        controller_state = 0

    # You  can try with u = np.array([[0.1], [0]]) and np.array([[0], [1]]) first.
    # Observe what happens to get a sense of how it works.

    # You should think about implementing a finite-state machine. How many
    # states are there? What are the transitions? What signals the transition
    # from one state to another?

    # ############## Do not modify anything outside this area ###############
    # #######################################################################

    # Send velocities to agents

    # Apply the barrier to the velocities
    u = uni_barrier_certificate(u, p)

    # Set velocities of agents 1,...,N
    r.set_velocities(list(range(1, N+1)), u) # u is the input, a 2x1 vector for 1 robot

    data.append(np.concatenate((p, u)).tolist())
    # Send the previously set velocities to the agents.  This function must be called!
    r.step()

savemat('data.mat', {'data':data})
# Call at end of script to print debug information and for your script to run on the Robotarium server properly
r.call_at_scripts_end()

