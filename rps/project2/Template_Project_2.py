# Simulator Skeleton File - Project 2
# This file provides the bare - bones requirements for interacting with the Robotarium.
# Note that this code won't actually run.  You'll have to insert your own algorithm!
# If you want to see some working code, check out the 'examples' folder.

import numpy as np
from scipy.io import savemat

import rps.robotarium as robotarium
from rps.utilities.barrier_certificates import create_unicycle_barrier_certificate_with_boundary
from rps.utilities.misc import *
from matplotlib.patches import Rectangle


# Get Robotarium object used to communicate with the robots / simulator
N=3
initial = np.array([[.5, 0, .1],
                       [.1, .3, -.3],
                       [np.pi/2, np.pi-.2, np.pi-0.1]])

r = robotarium.Robotarium(number_of_robots=N, show_figure=True, initial_conditions=initial)

#################################################
#gt_img = plt.imread('GTLogo.png')
#gt_img_handle = r.axes.imshow(gt_img, extent=(-1, 1, -1, 1))

# Select the number of iterations for the experiment.
iterations = 1750 # Do not change

# Other important variables
target = np.array([[.3, .2, .1], [0, 0, 0]])
targetalt = np.array([[.3, .1, .2], [0, 0, 0]])
data = []

# ######################### Place Static Variables Here ##################
# ############### Do not modify anything outside this area ###############
# var = 0;

# ############### Do not modify anything outside this area ###############
# ########################################################################


# ######################## Place Helper Functions Here ##################
# ############## Do not modify anything outside this area ###############
# def foo(b)

# ############## Do not modify anything outside this area ###############
# #######################################################################

# Iterate for the previously specified number of iterations
for t in range(iterations):
    # Retrieve the most recent poses from the Robotarium.  The time delay is
    # approximately 0.033 seconds
    p = r.get_poses()

    #Plot path traces every 20 iterations
    if np.mod(t,20) == 0:
        plt.plot(p[0][0], p[1][0], 'k.',  # Black dot
        p[0][1], p[1][1], 'm.',  # Magenta dot
        p[0][2], p[1][2], 'b.')  # Blue dot
    
    #Success check with position tolerance embedded
    p_rounded = np.round(p[:2, :], decimals=2)
    if np.array_equal(p_rounded, target) or np.array_equal(p_rounded, targetalt):
        print(f'Success! The final iteration number is {t}.\n')
        break

    # ######################## Place Algorithm Here #########################
    # ############## Do not modify anything outside this area ###############
    u = np.array([[.1, .2, .3], [.1, .2, .3]])
    # In u the first array covers velocity and the second array
    # covers angular velocity the affinity is as follows 
    # np.array([[1, 2, 3], [1, 2, 3]])
    # Observe what happens to get a sense of how it works.

    # You should think about implementing a finite-state machine. How many
    # states are there? What are the transitions? What signals the transition
    # from one state to another?

    # ############## Do not modify anything outside this area ###############
    # #######################################################################

    # Send velocities to agents

    # Set velocities of agents 1,...,N
    r.set_velocities(np.arange(N), u) # u is the input, a 2x1 vector for 1 robot

    data.append(np.vstack((p, u)))
    # Send the previously set velocities to the agents.  This function must be called!
    r.step()

savemat('data.mat', {'data':np.hstack(tuple(data))})
savemat(__file__ + "_autograder.mat", {'data':np.hstack(tuple(data))})
# Call at end of script to print debug information and for your script to run on the Robotarium server properly
r.call_at_scripts_end()
