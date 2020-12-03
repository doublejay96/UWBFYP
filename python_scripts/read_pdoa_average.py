#A script that takes the D, Xcm, Ycm measurements from the PDoA node and does a simple moving-average filter on them to smooth out the noise.
import serial
import json
import numpy as np

pdoa_port = serial.Serial('/dev/ttyACM0', timeout=1) #open the serial port (for PDoA)
print("Reading from " + pdoa_port.name)
pdoa_port.readline().decode('UTF-8') #remove the first message, usually messed up
MAX_I = 6 #the number of values to take a running average of
datastring = pdoa_port.readline().decode('UTF-8') #read until '\n' from port, decode to string
if datastring:
    dataJSON = json.loads(datastring[datastring.index('{'):]) #remove JSxxxx prefix to make it readable by loads
running_D = np.full(MAX_I, dataJSON["TWR"]["D"]) #initialise the matrices with the first value read
running_Xcm = np.full(MAX_I, dataJSON["TWR"]["Xcm"])
running_Ycm = np.full(MAX_I, dataJSON["TWR"]["Ycm"])
running_P = np.full(MAX_I, dataJSON["TWR"]["P"])
i = 0
while True:
    datastring = pdoa_port.readline().decode('UTF-8') #read until '\n' from port, decode to string
    if datastring:
        dataJSON = json.loads(datastring[datastring.index('{'):]) #remove JSxxxx prefix to make it readable by loads
        #dataJSON is a dict with key "TWR", mapped to another dict with various keys
        running_D[i] = dataJSON["TWR"]["D"] #enter the new value into the array, overwriting the oldest one
        running_Xcm[i] = dataJSON["TWR"]["Xcm"]
        running_Ycm[i] = dataJSON["TWR"]["Ycm"]
        running_P[i] = dataJSON["TWR"]["P"]
        i = (i+1)%MAX_I #increment i and wraparound
        print("{}-average of D is {:.2f} cm, X is {:.2f} cm, Y is {:.2f} cm, P is {:.3f} deg".format(MAX_I, running_D.mean(),running_Xcm.mean(),running_Ycm.mean(),running_P.mean()))
