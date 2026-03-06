import serial
import time
import struct

class ArduinoConnection:
    HANDSHAKE_INIT = b'\x16'
    HANDSHAKE_ACK = b'\x06'


    def __init__(self, port, baud = 115200, timeout = 1.0):
        self.serial = serial.Serial(port, baud, timeout = timeout)
        self.handshake()

    def read_byte(self):
        data = self.serial.read(1)
        if data:
            return data[0]  # returns int 0-255
        return None
    
    def wait_for_ack(self, timeout_ms: int) -> bool:
        """Wait for an ACK byte (let's say 0xAA) within timeout in ms"""
        start = time.time()
        timeout_s = timeout_ms / 1000.0
        while (time.time() - start) < timeout_s:
            b = self.read_byte()
            if b is not None and b == 0xAA:
                return True
            time.sleep(0.005)
        return False

    def handshake(self):
        if not self.serial.is_open:
            print("Failed to open serial port!")
            return

        ready = False
        start = time.time()
        while (time.time() - start) < 3.0:  # 3 seconds timeout
            b = self.read_byte()
            if b is not None and b == 0x55:
                print("Arduino READY received")
                ready = True
                break
            time.sleep(0.005)

        if not ready:
            print("Handshake timeout (3s). Continuing anyway.")

        # Send handshake command 0x08 with empty payload
        self.write_data(0x08, [])

        if self.wait_for_ack(200):
            print("HANDSHAKE STUPID")

    def sendCommand(self, *args):
        # construct packet
        pass

    def write_data(self, cmd_byte: int, payload: list[float] = None):
        """
        Send a packet with header, length, payload, and XOR checksum.
        Payload can be empty or contain any number of floats.
        """
        header = 0xAA
        payload_bytes = bytes([cmd_byte])

        if payload:
            payload_bytes += b''.join(struct.pack('<f', f) for f in payload)

        length = len(payload_bytes)
        checksum = self._xor_checksum(header, length, payload_bytes)
        packet = bytes([header, length]) + payload_bytes + bytes([checksum])
        self.serial.write(packet)

    def _xor_checksum(self, header: int, length: int, payload: bytes) -> int:
        x = header ^ length
        for b in payload:
            x ^= b
        return x & 0xFF

    def closeConnection(self):
        self.serial.close()
    
