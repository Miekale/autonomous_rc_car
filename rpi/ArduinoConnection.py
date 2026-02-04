import serial
import time

class ArduinoConnection:
    HANDSHAKE_INIT = b'\x16'
    HANDSHAKE_ACK = b'\x06'


    def __init__(self, port, baud = 115200, timeout = 1.0):
        self.serial = serial.Serial(port, baud, timeout)
        self.handshakeConnection()

    def handshakeConnection(self): 
        """
        Waits on the arduino until a connection handshake is completed. 
        Waits for the byte HANDSHAKE_INIT to be sent from the arduino.
        """
        self.serial.reset_input_buffer()

        while True:
            # wait until we receive a byte from arduino that it has started
            if self.serial.in_waiting > 0:
                byte = self.serial.read(1)
                if byte == self.HANDSHAKE_INIT:
                    self.serial.write(self.HANDSHAKE_ACK)
                    return
            time.sleep(0.01)

    def sendCommand(self, *args):
        # construct packet
        pass
    
    def closeConnection(self):
        self.serial.close()
    
