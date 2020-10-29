#A script to read PDoA data and translate it into control inputs to the offboard control.
import serial
import json
import numpy as np
import math
import asyncio
from mavsdk import System
from mavsdk.offboard import (OffboardError, VelocityBodyYawspeed)

##Variables to be set
pdoa_port = serial.Serial('/dev/ttyACM0', timeout=1) #open the serial port (for PDoA)
MAX_I = 5 #the number of values to take a running average of
TARGET_X = 0 #the X-distance in cm to maintain the tag position at
TARGET_Y = 50 #the Y-distance in cm to maintain the tag position at
VELOCITY_DIVIDER = 100 #distance divided by this to set target velocity. Distance in cm, velocity in m/s. Fine-tune?
YAW_VELOCITY_DIVIDER = 5 #current azimuth angle divided by this to set target yaw velocity. Both azimuth and yaw velocity in degrees, deg/sec. Fine-tune?
##Other global variables
running_D = np.zeros(MAX_I) #make an array for D values
running_Xcm = np.zeros(MAX_I) #make an array for Xcm values
running_Ycm = np.zeros(MAX_I) #make an array for Ycm values
running_P = np.zeros(MAX_I) #make an array for P values
D = 0 #the mean D, Xcm, Ycm, P
Xcm = 0
Ycm = 0
P = 0
i = 0 #increment counter

async def main():
    initialize_matrices()
    #drone = System(mavsdk_server_address='localhost', port=50051) #connect to MAVSDK server (start process first)
    #await drone.connect(system_address="udp://:14540") #address to pixhawk?
    #await drone.action.arm() #arm the drone
    #create initial setpoint, set initial velocities to 0 (forward, right, down, yaw angular rate)
    #await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0,0.0,0.0,0.0)) #set initial velocities to 0 in body reference frame
    print("--Starting offboard mode")
    #try:
    #    await drone.offboard.start() #start offboard mode
    #except OffboardError as error:
    #    print(f"Starting offboard mode failed with error code: {error._result.result}")
    #    print("--Disarming")
    #    await drone.action.disarm()
    #    return
    while True:
        await asyncio.gather(update_UWB_position(), send_control_input())
    print("--Stopping offboard")
    #try:
    #    await drone.offboard.stop() #stop offboard mode
    #except OffboardError as error:
    #    print(f"Stopping offboard mode failed with error code: {error._result.result}")
    return #final return out of main

def initialize_matrices(): #opens the serial port to PDoA, initialize position matrices
    global running_D, running_Xcm, running_Ycm, running_p, D, Xcm, Ycm, P
    pdoa_port.readline().decode('UTF-8') #remove the first message, usually messed up
    datastring = pdoa_port.readline().decode('UTF-8') #read second message until '\n' from port, decode to string
    if datastring:
        dataJSON = json.loads(datastring[datastring.index('{'):]) #remove JSxxxx prefix to make it readable by loads
    running_D = np.full(MAX_I, dataJSON["TWR"]["D"]) #initialise the matrices with the first value read
    running_Xcm = np.full(MAX_I, dataJSON["TWR"]["Xcm"])
    running_Ycm = np.full(MAX_I, dataJSON["TWR"]["Ycm"])
    running_P = np.full(MAX_I, dataJSON["TWR"]["P"])
    D = dataJSON["TWR"]["D"]
    Xcm = dataJSON["TWR"]["Xcm"]
    Ycm = dataJSON["TWR"]["Ycm"]
    P = dataJSON["TWR"]["P"]

#note: +ve y-direction is ahead of the node, +ve x-direction is left of the node
async def update_UWB_position(): #get the position of tag relative to node from the PDoA node
    global i, D, Xcm, Ycm
    datastring = pdoa_port.readline().decode('UTF-8') #read until '\n' from port, decode to string
    if datastring:
        dataJSON = json.loads(datastring[datastring.index('{'):]) #remove JSxxxx prefix to make it readable by loads
        #dataJSON is a dict with key "TWR", mapped to another dict with various keys
        running_D[i] = dataJSON["TWR"]["D"] #enter the new value into the arrays, overwriting the oldest one
        running_Xcm[i] = dataJSON["TWR"]["Xcm"]
        running_Ycm[i] = dataJSON["TWR"]["Ycm"]
        running_P[i] = dataJSON["TWR"]["P"]
        i = (i+1)%MAX_I #increment i and wraparound
        D = running_D.mean() #take the average of the array as D
        Xcm = running_Xcm.mean() #take the average of the array as Xcm
        Ycm = running_Ycm.mean() #take the average of the array as Ycm
        P = running_P.mean() #take the average of the array as P
        print("5-average of D is {} cm, X is {} cm, Y is {} cm, P is {} deg".format(D,Xcm,Ycm,P))

def calculate_yaw_rate(Yin, Xin): #calculate tangent (angle to yaw to), then returns body yaw velocity accordingly to turn straight-on. DO NOT SET YET
    angle = math.tanh(Xin/Yin) #calculate angle of node in degree. 0 deg is straight ahead, +ve is to the left anticlockwise
    yaw_vel = -angle/YAW_VELOCITY_DIVIDER #get the velocity to change yaw angle. (for +ve yaw angle (to left), turn left (-ve yaw velocity))
    return yaw_vel

async def send_control_input(): #send the appropriate control inputs over MAVlink to PX4
    Y_disp = Ycm - TARGET_Y #get the distance to move in Y
    X_disp = Xcm - TARGET_X #get the distance to move in X
    Y_vel = Y_disp/VELOCITY_DIVIDER #get the velocity to move forward
    X_vel = -X_disp/VELOCITY_DIVIDER #get the velocity to move right (-ve as X_disp is +ve left)
    yaw_vel = calculate_yaw_rate(Ycm, Xcm) #calculate appropriate yaw velocity (to tag, not to target position)
    #await drone.offboard.set_velocity_body(VelocityBodyYawspeed(Y_vel, X_vel, 0, 0)) #send the set body velocities to pixhawk
    print("set forward velocity {:.3f} m/s, right velocity {:.3f} m/s, down velocity {} m/s, yawspeed {:.3f} deg/s".format(Y_vel, X_vel, 0, yaw_vel))

if __name__ == "__main__":
    asyncio.run(main()) #run the main function
