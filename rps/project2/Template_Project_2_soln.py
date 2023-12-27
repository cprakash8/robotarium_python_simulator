## Simulator Skeleton File - Project 2 

# This file provides the bare-bones requirements for interacting with the
# Robotarium.  Note that this code won't actually run.  You'll have to
# insert your own algorithm!  If you want to see some working code, check
# out the 'examples' folder.

import rps.robotarium as robotarium
import numpy as np
import math
from scipy.io import savemat
import matplotlib.pyplot as plt

N = 3
data = []

# Select the number of iterations for the experiment.
iterations = 1750 # Do not change

# Other important variables
initpos1 = np.array([[.5],
                    [.1],
                    [math.pi/2]])
initpos2 = np.array([[0],
                     [.3],
                     [math.pi-0.2]])
initpos3 = np.array([[.1],
                     [-.3],
                     [math.pi-0.1]])
target = np.array([[.3, .2, .1],
                   [0, 0, 0]])
targetalt = np.array([[.3, .1, .2],
                   [0, 0, 0]])

initial_conditions = np.hstack((initpos1, initpos2, initpos3))

# Get Robotarium object used to communicate with the robots/simulator
r = robotarium.Robotarium(number_of_robots=N, show_figure=True, initial_conditions=initial_conditions, sim_in_real_time=False)

######################## Place Static Variables Here ##################
############## Do not modify anything outside this area ###############    
# var = 0;

waypoints1 = [[ 0.40,  0.21],[-0.16, 0.21],[-0.21, 0.04],[0.12, 0.04],[0.3, 0.00]]
waypoints2 = [[-0.48,  0.30],[-0.50, -0.10],[-0.34,  -0.2],[-0.55, -0.1],[-0.3,  -0.04],[0.12,  -0.04],[0.20, 0.00]]
waypoints3 = [[0.45, -0.15],[0.50, 0.25],[-0.472, 0.3],[-0.3, 0.04],[-0.25,0.04],[0.1, 0.00]]

state1 = 0
state2 = 0
state3 = 0

target1 = np.array(waypoints1[state1]).T
target2 = np.array(waypoints2[state2]).T
target3 = np.array(waypoints3[state3]).T

numsteps1 = 0
numsteps2 = 0
numsteps3 = 0
    
############## Do not modify anything outside this area ###############
#######################################################################


######################## Place Helper Functions Here ##################
############## Do not modify anything outside this area ###############

# def foo(b)

def wrap(theta):
    return np.arctan2(np.sin(theta), np.cos(theta))
def get_zone(x, y):
    z = 'X'
    if 0.1 < x <= 0.3 and -0.05 < y < 0.05:
        z = 'T'
    elif -0.2 < x <= 0.1 and -0.05 < y < 0.05:
        z = 'R'
    elif -0.2 == x and -0.05 < y < 0.05:
        z = 'S'
    elif -0.2 <= x <= 0.35 and -0.1 <= y <= 0.1:
        z = 'X'
    elif -0.3 < x < 0.45 and -0.2 < y < 0.2:
        z = 'S'
    elif -0.6 < x < 0.6 and -0.35 < y < 0.35:
        z = 'F'
    else:
        z = 'X'
    return z

def get_zone_limits(z):
    assert z in 'FSRT', 'Invalid zone'
    if z == 'F':
        return math.pi/4, 0.05, 0.08
    elif z == 'S':
        return math.pi / 6, 0.03, 0.04
    elif z in 'RT':
        return math.pi / 2, 0, 0.02
def closed_loop_controller(pose, target):
    z = get_zone(pose[0], pose[1])
    wmax, vmin, vmax = get_zone_limits(z)

    distance_error = 0.005;
    if z in 'SF':
        distance_error = 0.05

    gain = 1.55

    u = np.array([vmax, 0])
    done = False

    pose_diff = target[:2] - pose[:2]
    distance = np.linalg.norm(pose_diff)
    desired_theta = np.arctan2(pose_diff[1], pose_diff[0])
    alpha = wrap(desired_theta - pose[2]).item()

    u[1] = gain * alpha
    if u[1] < -wmax:
        u[1] = -wmax
    elif u[1] > wmax:
        u[1] = wmax

    if distance < distance_error:
        done = True
        if z == 'T':
            u[0] = vmin

    if abs(alpha) > math.pi / 2 and z in  'FS':
        u[0] = vmin

    return u, done

def validate(p, u):
    zones = ['X', 'X', 'X']
    for i in range(3):
        zones[i] = get_zone(p[0, i], p[1, i])

    # R3: Do not fly out of the simulation boundary.
    # X1: Entry is prohibited at any time
    for i in range(3):
        if zones[i] == 'X':
            return True, 'prohibited region violated'
    # R1: Minimum separation of aircrafts in the air and on the runway: 0.25 m
    # R2: R1 does not apply to aircrafts on the taxiway.
    for i in range(3):
        if zones[i] != 'T':
            for j in range(3):
                if i != j and  zones[j] != 'T':
                    dist = np.linalg.norm(p[:2, i] - p[:2,j])
                    if dist < 0.251:
                        return True, 'minimum distance violated between {} and {}'.format((i, j))

    # speed and rotation limits
    for i in range(3):
        z = zones[i]
        wmax, vmin, vmax = get_zone_limits(z)
        if u[0, i] < vmin or u[0, i] > vmax or abs(u[1, i]) > wmax:
            return True, 'speed limitation violated'

    return False, ""


############## Do not modify anything outside this area ###############
#######################################################################


# Iterate for the previously specified number of iterations
for t in range(1, 1+iterations):
    
    # Retrieve the most recent poses from the Robotarium.  The time delay is
    # approximately 0.033 seconds
    p = r.get_poses()
    
    # Plot the traces every 20 iterations
    if t % 20 == 0:
        plt.plot(p[0,0],p[1,0],'k.',p[0,1],p[1,1],'m.',p[0,2],p[1,2],'b.')
    
    # Success check with position tolerance embedded
    if (p[:2, :].round(2) == target).all() or (p[:2, :].round(2) == targetalt).all():
        print('Success! The final iteration number is {}.'.format(t))
        break

    ######################## Place Algorithm Here #########################
    ############## Do not modify anything outside this area ###############
    # u = ???;
    
    # You can try with u = np.array([[0.03, 0.02, 0.01],[0, 0, 0]]) first. Observe what happens to
    # get a sense of how it works.
    #u = np.array([[0.03, 0.02, 0.01], [0, 0, 0]])
    u = np.zeros((2,3))

    u[:, 0], done = closed_loop_controller(p[:,0], target1)
    if done:
        if state1 < len(waypoints1) - 1:
            state1 = state1 + 1
            target1 = np.array(waypoints1[state1]).T
        elif numsteps1 == 0:
            numsteps1 = t

    u[:, 1], done = closed_loop_controller(p[:, 1], target2)
    if done:
        if state2 < len(waypoints2) - 1:
            state2 = state2 + 1
            target2 = np.array(waypoints2[state2]).T
        elif numsteps2 == 0:
            numsteps2 = t

    u[:, 2], done = closed_loop_controller(p[:, 2], target3)
    if done:
        if state3 < len(waypoints3) - 1:
            state3 = state3 + 1
            target3 = np.array(waypoints3[state3]).T
        elif numsteps3 == 0:
            numsteps3 = t

    violation, message = validate(p,u)
    if violation:
        print("At step {},  {}".format(t, message))
        print(p, u)

    ############## Do not modify anything outside this area ###############
    #######################################################################
    
    # Send velocities to agents
    r.set_velocities(np.arange(N), u) # u is the input, a 2x3 vector for 3 robots
    data.append(np.hstack((p[:,0], u[:, 0], p[:, 1], u[:, 1], p[:, 2], u[:, 2])))
    # Iterate the simulation
    r.step()

# Call at end of script to print debug information and for your script to run on the Robotarium server properly
r.call_at_scripts_end()

# save data and figure
savemat('data.mat', {'data':np.vstack(tuple(data)).T})
plt.savefig("plot.pdf")