#A script to check how many updates per second we get from the PDoA module
import serial
import json
import time

pdoa_port = serial.Serial('/dev/ttyACM0', timeout=1) #open the serial port (for PDoA)
print("Reading from " + pdoa_port.name)
pdoa_port.readline().decode('UTF-8') #remove the first message, usually messed up
print("Wait...")
num_updates = 0
now = time.time()
while (time.time() - now < 10): #keep reading until 10 seconds have passed
    datastring = pdoa_port.readline().decode('UTF-8') #read until '\n' from port, decode to string
    num_updates = num_updates + 1
print("number of updates in 10 seconds is {}".format(num_updates))
print("update frequency is {:.3f} Hz".format(num_updates/10))
