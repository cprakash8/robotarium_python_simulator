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
  
    
    
############## Do not modify anything outside this area ###############
#######################################################################

######################## Place Helper Functions Here ##################
############## Do not modify anything outside this area ###############

# def foo(b)


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
    u = np.array([[0.03, 0.02, 0.01], [0, 0, 0]])
    
    
    
    
    ############## Do not modify anything outside this area ###############
    #######################################################################
    
    # Send velocities to agents
    r.set_velocities(np.arange(N), u) # u is the input, a 2x3 vector for 3 robots
    data.append(np.hstack((p[:,0], u[:, 0], p[:, 1], u[:, 1], p[:, 2], u[:, 2])))
    # Iterate the simulation
    r.step()

savemat('data.mat', {'data':np.vstack(tuple(data)).T})

# Call at end of script to print debug information and for your script to run on the Robotarium server properly
r.call_at_scripts_end()