import time
import numpy as np
import elcomat2log
import JPE_cryopos
import math
import matplotlib.pyplot as plt

# Timing the execution of the script
start_time = time.time()

# Reading position from "elcomat2log" collimator script
def readpos():
    try:
        b, x, y = elcomat2log.get_elcomat_xy()
        success = True
    except:
        success = False
        print("Failed to read position. Exiting.")
        exit()
    if not b:
        print("Autocollimator is in relative mode. Exiting.")
        exit()
    if x == 0 or y == 0:
        print("Autocollimator outside range. Exiting.")
        exit()
    return x, y


# Motion step resolution/gains:
# Arcsec moved per actuator step in [+,-] x- or y-direction.
def get_gains(dir_x, dir_y):
    G_xx = [ 0.076, 0.091][dir_x]
    G_xy = [0.003, 0.003][dir_x]
    G_yy = [ 0.085, 0.095][dir_y]
    G_yx = [0.000024,0.000015][dir_y]
    return G_xx, G_xy, G_yy, G_yx


# Solving for correct step count with analytical model
def solve_steps(Delta_x, Delta_y, G_xx, G_xy, G_yy, G_yx):
    b = (Delta_y / G_yy) - (G_xy*Delta_x)/(G_yy*G_xx)
    c = (G_xy*G_yx)/(G_yy*G_xx)
    Steps_y = b / (1-c)
    Steps_x = (Delta_x - Steps_y * G_yx) / G_xx
    return Steps_x, Steps_y
    

# Current position
x, y = readpos()

# Default position
target_x = 937
target_y = 653

# Defining different global gains: 
# First one full step, then scale down to avoid overshooting from hysteresis
global_gain_first = 1
global_gain_else = 0.8
global_gain_shift = 0.4

# Avoid too large steps - clip is max on actuator step count
clip = 1000

# Activate "JPE_cryopos" actuator script
actuator = JPE_cryopos.JPEcryopos()

# Desired change in position
Delta_x = target_x-x
print("Desired change in position X: %6.2f arcsec" % Delta_x)
Delta_y = target_y-y
print("Desired change in position Y: %6.2f arcsec" % Delta_y)

# Calculate distance from default position
dist = np.sqrt(Delta_x**2 + Delta_y**2)

# Precision/image stabilization (min desired distances from default position)
goal_dist = 0.2 # FCS req. 0.06

# Parameters used for loop underneath
X = []
Y = []
rounds = 0
old_x_move = 0
old_y_move = 0

# Moving the grating unit back to default position
while dist > goal_dist:

    # Guessing first on positive movement - [+,-][0] -> [+]
    direction_x = 0 
    direction_y = 0
    G_xx, G_xy, G_yy, G_yx = get_gains(direction_x, direction_y)
    Steps_x, Steps_y = solve_steps(Delta_x, Delta_y, G_xx, G_xy, G_yy, G_yx)

    # Taking into account negative movement - [+,-][1] -> [-]
    if Steps_x < 0:
        direction_x = 1
    if Steps_y < 0:
        direction_y = 1
    G_xx, G_xy, G_yy, G_yx = get_gains(direction_x, direction_y)
    Steps_x, Steps_y = solve_steps(Delta_x, Delta_y, G_xx, G_xy, G_yy, G_yx)


    # First amount of steps moved
    if rounds == 0:
        if abs(Steps_x) > 0:
            x_move = int(Steps_x*global_gain_first)
            print("Moving X: %6d steps with initial gain of %3.2f" % (x_move, global_gain_first))
        if abs(Steps_y) > 0:
            y_move = int(Steps_y*global_gain_first)
            print("Moving Y: %6d steps with initial gain of %3.2f" % (y_move, global_gain_first))
            
    # Then scaled down step count
    else:
        if abs(int(Steps_x*global_gain_else)) > 0:
            x_move = int(Steps_x*global_gain_else)
            print("Moving X: %6d steps with gain of %3.2f" % (x_move, global_gain_else))
        if abs(int(Steps_y*global_gain_else)) > 0:
            y_move = int(Steps_y*global_gain_else)
            print("Moving Y: %6d steps with gain of %3.2f" % (y_move, global_gain_else))
    
   # Scaling the step count even more down if there is a change in [+,-] direction        
    if old_x_move * x_move < 0:
        x_move = int(Steps_x*global_gain_shift)
        print("Moving X: %6d steps with reverse direction gain of %3.2f" % (x_move, global_gain_shift))
    if old_y_move * y_move < 0:
        y_move = int(Steps_y*global_gain_shift)
        print("Moving Y: %6d steps with reverse direction gain of %3.2f" % (y_move, global_gain_shift))
        
    # Stops movement if execution takes too many loops/steps (begins to overshoot)
    if rounds > 15:
        print("Overshooting")
        exit()

    # The actual moving 
    if x_move != 0:
    	actuator.move(1, x_move, relative_step_size = 50)
    if y_move != 0:
    	actuator.move(2, y_move, relative_step_size = 50)

    # Loop parameters
    old_x_move = x_move
    old_y_move = y_move
    rounds += 1
    X.append(x_move)
    Y.append(y_move)
	
    # Pause between each movement
    pause = max(1 + abs(Steps_x) / 500, 1 + abs(Steps_y) / 500)
    print("pause: %6.2f s" % pause)
    time.sleep(pause)

    # Reading new position and calculating new distance to default after each movement
    x, y = readpos()

    # Desired change in position
    Delta_x = target_x-x
    print("Desired change in position X: %6.2f arcsec" % Delta_x)
    Delta_y = target_y-y
    print("Desired change in position Y: %6.2f arcsec" % Delta_y)

    dist = np.sqrt(Delta_x** 2 + Delta_y** 2)
    
    print()
    print("_____New loop_____")
    print()
    
print(f"Target distance {dist:6.2f} arcsec")
print("Done!")

# Record the end time of script execution and calculate elapsed time
end_time = time.time()
elapsed_time = end_time - start_time

# Plot the step counts vs. loops
run = np.linspace(1, rounds, rounds)
plt.plot(X, run, marker='o', linestyle='--', color='orange', label='X-steps')
plt.plot(Y, run, marker='o', linestyle='--', color='green', label='Y-steps')
plt.text(np.min(X)+5, np.max(run)*0.9, f'Execution Time: {elapsed_time:.1f} s',
            fontsize=9, ha='left', va='top', color='red')
plt.grid()
plt.xlabel('Step-size [steps/"]')
plt.ylabel('Loops')
plt.legend()
plt.show()
