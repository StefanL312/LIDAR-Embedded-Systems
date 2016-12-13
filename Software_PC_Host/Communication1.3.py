"""
Author: S.A.Lung
Project: LIDAR
Description: program used to communicate with Microcontroller
"""
import math
import matplotlib.pyplot as plt
import numpy as np
import time
import serial
import hashlib
#class foo(object):
GlobalACK = False

#########Dataframe
DestAddr = 'S'
SourceAddr = 'M'
Command = ' '
FrameSize = ' '
Data = chr(0)
StateFlags = chr(0)
CRC = chr(0x5B)
line = ' '
###########

PkgSize = 0

#def commdis(COM):

def Checksum(DestAddr, SourceAddr, Command, Framesize, StateFlags, Data):
    
    summation = ord(DestAddr)^ord(SourceAddr)^ord(Command)^ord(Framesize)^ord(StateFlags)^ord(Data)
    return summation
    

def commsetup(COM):
    global Sensor
    Sensor = serial.Serial(COM, 115200,bytesize =8, parity = serial.PARITY_ODD, timeout=2)
    line = Sensor.readline()
    

def Send(Data):
    global Sensor
    Sensor.write(Data)
    if(GlobalACK):
        ReceiveACK()
    return 0

def ReceivePackageInfo():
    global Sensor, SourceAddr, DestAddr,line, PkgSize
    line = Sensor.readline(12)
    print("Pkgzinfo: ", line)
    if(line[:2] == SourceAddr+DestAddr):
        if(line[2:3] in 'R'):
            PkgSize = line[9:10]
            print("Pkg: ", PkgSize)
            return True
        else:
            return False

def ReceiveScanPackage():
    global Sensor, SourceAddr, DestAddr,line, PkgSize
    line = Sensor.readline(11)
    if(line[:2] == SourceAddr+DestAddr):
        if(line[2:3] in 'R'):
            PkgSize = line[8:9]
            return True
        else:
            return False
        
def ReceiveLastPackage():
    global Sensor, SourceAddr, DestAddr,line, PkgSize
    line = Sensor.readline(10)
    if(line[:2] == SourceAddr+DestAddr):
        if(line[2:3] in 'R'):
            PkgSize = line[6:7]
            return True
        else:
            return False

def ReceiveEOFACK():
    global Sensor, SourceAddr, DestAddr,line
    line = Sensor.readline()
    print("new: ",line)
    if(line[:2] == SourceAddr+DestAddr):
        if(line[2:3] in 'E'):
            return True
        else:
            return False
    
def ReceiveACK():
    global Sensor, SourceAddr, DestAddr, line
    line = Sensor.readline()
    print("new: ",line)
    
    
    if(line[:2] == SourceAddr+DestAddr):
        if(line[2:3] == 'Y'):
            return True
        else:
            return False
    else:
        return False

def ReceiveSYNACK(message):
    global Sensor, SourceAddr, DestAddr, line
    line = Sensor.readline()
    
    print("Acknowledge: ",line)
    if(line[:2] == SourceAddr+DestAddr):
        if(line[2:] == line[2:]):
            return True
        else:
            return False
    else:
        return False
        
def Receive_raw():
    global Sensor
    line = Sensor.readline()
    return line
while(True):
    command = raw_input("Give command: ")

    if("connect COM" in command):
        COM = command[8:]
        commsetup(COM)
        Command = 'C'
        Data = chr(0)
        FrameSize = chr(0x06)
        CRC = Checksum(DestAddr, SourceAddr, Command, FrameSize, StateFlags, Data)
        message = DestAddr+SourceAddr+Command+FrameSize+StateFlags+chr(CRC)
        print(message)
        Send(message)
        if(ReceiveSYNACK(message)):
            print("Connection established")
        else:
            print("No connection")
            Sensor.close()
        print("done")        
        
    elif("stop" in command):
        Command = 'S'
        FrameSize = '0'
        Send(DestAddr+SourceAddr+Command+FrameSize+Data+StateFlags+CRC)
        ReceiveACK()
        break
    
    elif("scan" in command):
        Command = "A"
        FrameSize = chr(0x06)
        CRC = Checksum(DestAddr, SourceAddr, Command, FrameSize, StateFlags, Data)
        message = DestAddr+SourceAddr+Command+FrameSize+StateFlags+chr(CRC)
        Send(message)
        if(ReceiveEOFACK()):
            print("ACK")
        else:
            print("no ACK")
    
    elif("request data" in command):
        Command = 'R'
        FrameSize = chr(0x06)
        CRC = Checksum(DestAddr, SourceAddr, Command, FrameSize, StateFlags, Data)
        message = DestAddr+SourceAddr+Command+FrameSize+StateFlags+chr(CRC)
        Send(message)
        if(ReceivePackageInfo()):
            while(1):
                ReceiveScanPackage()
                if(PkgSize!= '\x00'):
                    print(line)
                else:
                    break
            if(ReceiveLastPackage()):
                print(line)

            if(ReceiveEOFACK()):
                print("Finished")
            
            
                
                
            print("Ready")
            

                
        
    elif("live scan" in command):
        Command = 'R'
        FrameSize = '0'
        Send(DestAddr+SourceAddr+Command+FrameSize+Data+StateFlags+CRC)
        ReceiveACK()

        while(True):
            value = Receive_raw()
            if(value == 'E'):
                break
            
    elif("disconnect" in command):
        print("going to disconnect")
        Command = 'D'
        Data = chr(0)
        FrameSize = chr(0x06)
        CRC = Checksum(DestAddr, SourceAddr, Command, FrameSize, StateFlags, Data)
        message = DestAddr+SourceAddr+Command+FrameSize+StateFlags+chr(CRC)
        print(message)
        Send(message)
        if(ReceiveSYNACK(message)):
            print("Close connection")
            Sensor.close()
        else:
            print("Say WHut??")
            


            
        ##put other commands
        
