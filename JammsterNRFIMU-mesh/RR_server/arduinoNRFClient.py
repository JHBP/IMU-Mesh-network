import RobotRaconteur as RR
RRN = RR.RobotRaconteurNode.s

def main():
    t1 = RR.LocalTransport()
    RRN.RegisterTransport(t1)
    

    t2 = RR.TcpTransport()
    RRN.RegisterTransport(t2)

    myGateway = RRN.ConnectService('tcp://localhost:10001/arduinoIMU/arduinoIMUData')

    print myGateway.IMU1_read() 
    print myGateway.IMU2_read() 

    RRN.Shutdown()
  

if __name__ == '__main__':
    main()

