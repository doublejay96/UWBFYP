#UWB error plotter, from super_logs.csv
import numpy as np
import matplotlib.pyplot as plt

curr_run = "run40"

super_logs = np.loadtxt(curr_run+'/super_logs.csv', delimiter=',',skiprows=1,usecols=(0,1,2,3,4,5,6,7,8,9,10))
with open(curr_run+'/PID_constants.txt') as file:
    print("Current run PID constants:")
    for line in file:
        print(line)
VICON_leader_x_relative = (super_logs[:,4] - super_logs[:,1]) * 100 #in cm
VICON_leader_y_relative = (super_logs[:,5] - super_logs[:,2]) * 100
VICON_leader_z_relative = (super_logs[:,6] - super_logs[:,3]) * 100

UWB_x_error = super_logs[:,7] - VICON_leader_y_relative ##the UWB X minus VICON y
UWB_y_error = super_logs[:,8] - VICON_leader_x_relative ##the UWB Y minus VICON x
filtered_UWB_x_error = super_logs[:,9] - VICON_leader_y_relative
filtered_UWB_y_error = super_logs[:,10] - VICON_leader_x_relative

UWB_x_error_mean = np.mean(UWB_x_error)
UWB_y_error_mean = np.mean(UWB_y_error)
filtered_UWB_x_error_mean = np.mean(filtered_UWB_x_error)
filtered_UWB_y_error_mean = np.mean(filtered_UWB_y_error)
print("UWB_x_error_mean: {:.3f}, UWB_y_error_mean: {:.3f}".format(UWB_x_error_mean, UWB_y_error_mean))
print("filtered_UWB_x_error_mean: {:.3f}, filtered_UWB_y_error_mean: {:.3f}".format(filtered_UWB_x_error_mean, filtered_UWB_y_error_mean))


plt.figure()
plt.plot(super_logs[:,0], VICON_leader_y_relative, label="VICON_y (cm)", color='b')
plt.plot(super_logs[:,0], super_logs[:,7], label="UWB_x (cm)", color='r')
plt.plot(super_logs[:,0], super_logs[:,9], label="filtered_UWB_x (cm)", color='g')
plt.plot(super_logs[:,0], UWB_x_error, label="UWB_x_error (cm)", color='y')
plt.plot(super_logs[:,0], filtered_UWB_x_error, label="filtered_UWB_x_error (cm)", color='k')
plt.legend(loc="upper right")
plt.title("UWB and errors in X (left/right)")
plt.figure()
plt.plot(super_logs[:,0], VICON_leader_x_relative, label="VICON_x (cm)", color='b')
plt.plot(super_logs[:,0], super_logs[:,8], label="UWB_y (cm)", color='r')
plt.plot(super_logs[:,0], super_logs[:,10], label="filtered_UWB_y (cm)", color='g')
plt.plot(super_logs[:,0], UWB_y_error, label="UWB_y_error (cm)", color='y')
plt.plot(super_logs[:,0], filtered_UWB_y_error, label="filtered_UWB_y_error (cm)", color='k')
plt.legend(loc="upper right")
plt.title("UWB and errors in Y (forward/backward)")
plt.figure()
plt.hist(filtered_UWB_x_error, bins=20)
plt.xlabel("Error (cm)")
plt.title("Histogram of filtered UWB x error (left/right)")
plt.figure()
plt.hist(filtered_UWB_y_error, bins=20)
plt.xlabel("Error (cm)")
plt.title("Histogram of filtered UWB y error (forward/backward)")


plt.show()
