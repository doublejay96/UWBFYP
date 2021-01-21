#Flight path plotter/comparison, from super_logs.csv
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
curr_run = "run40"

super_logs = np.loadtxt(curr_run+'/super_logs.csv', delimiter=',',skiprows=1,usecols=(0,1,2,3,4,5,6))
with open(curr_run+'/PID_constants.txt') as file:
    print("Current run PID constants:")
    for line in file:
        print(line)
VICON_follower_x = super_logs[:,1] * 100 #in cm
VICON_follower_y = super_logs[:,2] * 100
VICON_follower_z = super_logs[:,3] * 100
VICON_leader_x = super_logs[:,4] * 100
VICON_leader_x_offset = (super_logs[:,4] * 100) - 200
VICON_leader_y = super_logs[:,5] * 100
VICON_leader_z = super_logs[:,6] * 100

plt.figure()
plt.title("2-D flight paths in VICON reference frame (cm)")
plt.plot(VICON_follower_x, VICON_follower_y, label="Follower position (cm)", color='b')
plt.plot(VICON_leader_x, VICON_leader_y, label="Leader position (cm)", color='r')
plt.plot(VICON_leader_x_offset, VICON_leader_y, label="Leader position offset (cm)", color='g')
plt.legend(loc="upper right")

fig = plt.figure()
plt.subplots_adjust(bottom=0.15) #bottom to give some allowance to the slider
ax = fig.add_subplot(111, projection='3d')
ax.set_title("3-D Flight Path Visualiser in VICON reference frame (cm)")
l1, = ax.plot(VICON_follower_x, VICON_follower_y, VICON_follower_z, label="Follower position (cm)", color='b')
#l2, = ax.plot(VICON_leader_x, VICON_leader_y, VICON_leader_z, label="Leader position (cm)", color='r')
l3, = ax.plot(VICON_leader_x_offset, VICON_leader_y, VICON_leader_z, label="Leader position offset (cm)", color='g')
ax.set_zlim3d(bottom=0) #to ensure it is scaled appropriately in z (otherwise v small range in z compared to x,y)
#set a display for the time in seconds
time_display = fig.text(0.91, 0.115, str("{:.2f}").format(super_logs[-1,0]))
plt.legend(loc="upper right")

#make a separate axes object to put the slider in, then make the slider
ax_slider = plt.axes([0.2, 0.1, 0.65, 0.03])
slider_time = Slider(ax_slider, 'Time', 1, super_logs.shape[0]-1, valinit=super_logs.shape[0]-1, valstep = 1)
def updateTimeDisplayed(val):
    l1.set_data_3d(VICON_follower_x[:slider_time.val], VICON_follower_y[:slider_time.val], VICON_follower_z[:slider_time.val])
    #l2.set_data_3d(VICON_leader_x[:slider_time.val], VICON_leader_y[:slider_time.val], VICON_leader_z[:slider_time.val])
    l3.set_data_3d(VICON_leader_x_offset[:slider_time.val], VICON_leader_y[:slider_time.val], VICON_leader_z[:slider_time.val])
    time_display.set_text(str("{:.2f}").format(super_logs[slider_time.val,0]))
    fig.canvas.draw_idle()
slider_time.on_changed(updateTimeDisplayed)

plt.show()
