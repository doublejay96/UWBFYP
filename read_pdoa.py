#A very simple script to see what we get from the USB/UART serial output from the PDoA module
import serial
import json

pdoa_port = serial.Serial('/dev/ttyACM0') #open the serial port (for PDoA)
print("Reading from " + pdoa_port.name)
pdoa_port.readline().decode('UTF-8') #remove the first message, usually messed up
while True:
    datastring = pdoa_port.readline().decode('UTF-8') #read until '\n' from port, decode to string
    print(datastring)
    #if datastring:
    #    dataJSON = json.loads(datastring[datastring.index('{'):]) #remove JSxxxx prefix to make it readable by loads
