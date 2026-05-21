import struct
import socket
import numpy as np
import multiprocessing, ctypes
from time import sleep
import random

RESPONS_SZ = 36
FORCE_DIV  =  10000.0 # -------------- Default Force  divide value
TORQUE_DIV = 100000.0 # -------------- Default Torque divide value


class OptoForceCmd:
    """ Container class for OptoForce commands """
    
    ## Datagrams ##
    DG_cmd = struct.Struct('! 2H I') #- 2x uint8, 1x uint32, Big-endian (Network)
    DG_res = struct.Struct('! 3I 6i') # 3x uint32, 6x int32, Big-endian (Network)
    COMMANDS = {
            'set_filter_0' : DG_cmd.pack( 0x1234 , #- Header
                                          0x0081 , #- Set filter
                                               0 ), # No filter
            'set_bias_0' : DG_cmd.pack( 0x1234 , #- Header
                                        0x0042 , #- Set bias
                                             0 ), # No bias
            'set_bias_1' : DG_cmd.pack( 0x1234 , #- Header
                                        0x0042 , #- Set bias
                                             1 ), # Yes bias
            'send_01' : DG_cmd.pack( 0x1234 , #- Header
                                     0x0002 , #- Data Request
                                          1 ), # Number of samples
            'stop_data' : DG_cmd.pack( 0x1234 , #- Header
                                       0x0000 , #- Data Request
                                           10 ), # Number of samples
        }

    # Discrete rates the sensor actually honours (empirically verified).
    # Maps Hz → speed_reg value (speed_reg = 1000 / Hz).
    VALID_RATES = {500: 2, 250: 4, 100: 10, 50: 20, 20: 50, 10: 100, 5: 200}

    @classmethod
    def make_set_speed(cls, hz: int) -> bytes:
        """Build a set-speed command, snapping to the nearest supported rate.

        Supported rates: 5, 10, 20, 50, 100, 250, 500 Hz (empirically verified).
        """
        nearest = min(cls.VALID_RATES, key=lambda v: abs(v - hz))
        if nearest != hz:
            print(f"Warning: {hz} Hz not a valid sensor rate; "
                  f"using {nearest} Hz instead. "
                  f"Valid rates: {sorted(cls.VALID_RATES)}")
        speed_reg = cls.VALID_RATES[nearest]
        return cls.DG_cmd.pack(0x1234, 0x0082, speed_reg)
                
    @staticmethod
    def unpack_response( res ):
        """ Unpack the response into a python list """
        return list( OptoForceCmd.DG_res.unpack( res )[3:] ) # Trim off the header


class OptoForce:
    def __init__(self, ip_address: str = "192.168.0.5", port: int = 49152, poll_rate: int = 100):
        self.sensorAddr = (ip_address, port)
        self.sock_r = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.cmd = OptoForceCmd()
        self.poll_rate = poll_rate
        self.poll_rate_cmd = OptoForceCmd.make_set_speed(poll_rate)
        
    def connect(self):
        """ Connect to the sensor and start streaming """
        self.sock_r.settimeout(5)
        self.sock_r.connect(self.sensorAddr)
        self.prime_sensor()
        self.flush()  # clear config-response packets before starting read loop

    def prime_sensor(self):
        """ Send configuration commands to start the stream """
        self.send_datagram(self.poll_rate_cmd)
        self.send_datagram(self.cmd.COMMANDS['set_filter_0'])
        self.send_datagram(self.cmd.COMMANDS['set_bias_1'])

    def send_datagram(self, commandBytes, wait_s=0.020):
        """ Send a configuration command and wait for it to take effect """
        self.sock_r.send(commandBytes)
        sleep(wait_s)

    def flush(self):
        """Drain any packets already buffered in the OS socket buffer.

        Useful after a burst or reconnect to clear stale responses before
        starting a fresh timed read loop.
        """
        self.sock_r.settimeout(0)
        try:
            while True:
                self.sock_r.recv(RESPONS_SZ)
        except (BlockingIOError, socket.error):
            pass
        finally:
            self.sock_r.settimeout(0.05)

    def recv_datum(self):
        """Request and read one sample (request-response mode).

        The sensor responds to individual send_01 requests; it does not stream
        continuously. set_speed (0x0082) sets the max rate but does not trigger
        autonomous streaming.

        Returns a list of 6 floats [Fx, Fy, Fz, Tx, Ty, Tz] in N / N·m,
        or an empty list on timeout.
        """
        self.sock_r.send(self.cmd.COMMANDS['send_01'])
        try:
            data, _ = self.sock_r.recvfrom(RESPONS_SZ)
        except socket.timeout:
            return []
        if len(data) != RESPONS_SZ:
            return []
        rtnDat = self.cmd.unpack_response(data)
        for i in range(3):
            rtnDat[i  ] /= FORCE_DIV
            rtnDat[i+3] /= TORQUE_DIV
        return rtnDat

    def close(self):
        """ Close the socket """
        self.sock_r.close()

