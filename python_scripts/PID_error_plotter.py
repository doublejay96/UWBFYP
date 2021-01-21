#PID error plotter, from super_logs.csv
import numpy as np
import matplotlib.pyplot as plt

curr_run = "run33"
prev_run = "run31"

super_logs = np.loadtxt(curr_run+'/super_logs.csv', delimiter=',',skiprows=1,usecols=(0,12,13,15,16))
super_logs_prev = np.loadtxt(prev_run+'/super_logs.csv', delimiter=',',skiprows=1,usecols=(0,12,13,15,16))
with open(curr_run+'/PID_constants.txt') as file:
    print("Current run PID constants:")
    for line in file:
        print(line)
with open(prev_run+'/PID_constants.txt') as file:
    print("Previous run PID constants:")
    for line in file:
        print(line)
plt.figure()
plt.plot(super_logs[:,0], super_logs[:,1], label="PID_x_error (m)", color='b')
plt.plot(super_logs[:,0], super_logs[:,3], label="Offb_x_velocity (m/s)", color='r')
plt.plot(super_logs_prev[:,0], super_logs_prev[:,1], label="PID_x_error (previous) (m)", color='g')
plt.plot(super_logs_prev[:,0], super_logs_prev[:,3], label="Offb_x_velocity (previous) (m/s)", color='y')
plt.legend(loc="upper right")
plt.title("PID error and offboard velocity in X (forward/backward)")
plt.figure()
plt.plot(super_logs[:,0], super_logs[:,2], label="PID_y_error (m)", color='b')
plt.plot(super_logs[:,0], super_logs[:,4], label="Offb_y_velocity (m/s)", color='r')
plt.plot(super_logs_prev[:,0], super_logs_prev[:,2], label="PID_y_error (previous) (m)", color='g')
plt.plot(super_logs_prev[:,0], super_logs_prev[:,4], label="Offb_y_velocity (previous) (m/s)", color='y')
plt.legend(loc="upper right")
plt.title("PID error and offboard velocity in Y (left/right)")
plt.show()
