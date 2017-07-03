import serial
import struct
import time
import RobotRaconteur as RR
import thread
import threading
import numpy
import traceback
import socket
import time
import sys
import signal
import argparse

RRN = RR.RobotRaconteurNode.s

# catch control c to allow for use of the same port
def signal_handler(signal, frame):
        print('Ctrl+C shutdown!')
        RRN.Shutdown()
        sys.exit(0)

class NrfImuInterface(object):
    def __init__(self,  serialPort):
        self._lock=threading.RLock()
        self._serial=serial.Serial(serialPort,baudrate=57600)

    def IMU1_read(self):
        with self._lock:
            dat=struct.pack("BB",50,0)
            self._serial.write(dat)
            raw = self._serial.read(size = 14)
            unConverted = struct.unpack( "hhhhhhh", raw )
            converted = [unConverted[0]/16384.0, unConverted[1]/16384.0,unConverted[2]/16384.0,unConverted[3]/131.0,unConverted[4]/131.0,unConverted[5]/131.0]
            return converted

    def IMU2_read(self):
        with self._lock:
            dat=struct.pack("BB",50,1)
            self._serial.write(dat)
            raw = self._serial.read(size = 14)
            unConverted = struct.unpack( "hhhhhhh", raw )
            converted = [unConverted[0]/16384.0, unConverted[1]/16384.0,unConverted[2]/16384.0,unConverted[3]/131.0,unConverted[4]/131.0,unConverted[5]/131.0]
            return converted
		     

def main():
    # accept arguments to command line call
    parser = argparse.ArgumentParser()
    parser.add_argument("--serverPort", help="port to start this server on")
    parser.add_argument("--roboclawUSBPort", help="USB port that the roboclaw is on")
    args = parser.parse_args()
    port = args.serverPort
    usbPort = args.roboclawUSBPort
    if port == None:
        port = 10001
    
    if usbPort == None:
        usbPort = '/dev/ttyACM1'
        
    t1 = RR.LocalTransport()
    t1.StartServerAsNodeName("arduinoIMU")
    RRN.RegisterTransport(t1)

    t2 = RR.TcpTransport()
    t2.EnableNodeAnnounce()
    t2.StartServer(port)
    RRN.RegisterTransport(t2)

    with open('nrfIMU.robodef', 'r') as f:
        service_def = f.read()
    
    myGateway = NrfImuInterface(usbPort)

    RRN.RegisterServiceType(service_def)
    RRN.RegisterService("arduinoIMUData", "arduinoIMU.NrfImuInterface", myGateway)
    print "Conect string: tcp://localhost:" + str(port) + "/arduinoIMU/arduinoIMUData"
    raw_input("Press any key to end")

    RRN.Shutdown()
  


if __name__ == '__main__':
    main()
