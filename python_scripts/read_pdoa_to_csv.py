#A script that takes the D, Xcm, Ycm, P measurements from the PDoA node and outputs them to CSV.
import serial
import json
import numpy as np
import time
import csv

pdoa_port = serial.Serial('/dev/ttyACM0', timeout=1) #open the serial port (for PDoA)
print("Reading from " + pdoa_port.name)
pdoa_port.readline().decode('UTF-8') #remove the first message, usually messed up
datastring = pdoa_port.readline().decode('UTF-8') #read until '\n' from port, decode to string
if datastring:
    dataJSON = json.loads(datastring[datastring.index('{'):]) #remove JSxxxx prefix to make it readable by loads
writer = csv.writer(open("pdoa_output_100sec_60_10.csv","w"), delimiter=",") #start the csv writer with the opened csv file
print("Recording for 100 seconds...")
now = time.time() #start the time here
while (time.time() - now < 100): #keep running until 10 second have passed
    datastring = pdoa_port.readline().decode('UTF-8') #read until '\n' from port, decode to string
    if datastring:
        dataJSON = json.loads(datastring[datastring.index('{'):]) #remove JSxxxx prefix to make it readable by loads
        #dataJSON is a dict with key "TWR", mapped to another dict with various keys
        D = dataJSON["TWR"]["D"] #enter the new value into the array, overwriting the oldest one
        Xcm = dataJSON["TWR"]["Xcm"]
        Ycm = dataJSON["TWR"]["Ycm"]
        P = dataJSON["TWR"]["P"]
        writer.writerow([D, Xcm, Ycm, P]) #write the values into a row
