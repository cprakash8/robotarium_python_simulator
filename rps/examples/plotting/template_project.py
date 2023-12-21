# Simulator Skeleton File - Project 1
# This file provides the bare - bones requirements for interacting with the Robotarium.
# Note that this code won't actually run.  You'll have to insert your own algorithm!
# If you want to see some working code, check out the 'examples' folder.

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
target1 = np.array([-0.5, 0, 0]).transpose()
target2 = np.array([0.5, 0, 0]).transpose()
k = 1

# ######################### Place Static Variables Here ##################
# ############### Do not modify anything outside this area ###############
# var = 0;

# ############### Do not modify anything outside this area ###############
# ########################################################################


# Iterate for the previously specified number of iterations
for t in range(1, iterations+1):
    # Retrieve the most recent poses from the Robotarium.  The time delay is
    # approximately 0.033 seconds
    p = r.get_poses()

    # ######################## Place Algorithm Here #########################
    # ############## Do not modify anything outside this area ###############
    # u = ???;

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

# ######################## Place Helper Functions Here ##################
# ############## Do not modify anything outside this area ###############
# def foo(b)

# ############## Do not modify anything outside this area ###############
# #######################################################################
