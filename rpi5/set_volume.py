import sys
import serial
import struct
import time

# Packet constants from comm_packet.hpp
REQUEST_PACKET = 0xAA
MCU_ADDR = 0x01
CMD_VOL = 0x04

SERIAL_PORT = '/dev/ttyAMA1'
BAUD_RATE = 2000000

if len(sys.argv) < 2:
    print("Usage: python3 set_volume.py <volume>")
    print("  volume: float value 50 (dB)")
    sys.exit(1)

volume = float(sys.argv[1])

# Build packet: header (4 bytes) + float volume (4 bytes)
# len field = 4 (size of float payload in bytes)
header = struct.pack('BBBB', REQUEST_PACKET, MCU_ADDR, CMD_VOL, 4)
payload = struct.pack('<f', volume)
packet = header + payload

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(0.1)
    ser.write(packet)
    ser.flush()
    print(f"Sent volume command: {volume}")
    ser.close()
except serial.SerialException as e:
    print(f"Serial error: {e}")
except Exception as e:
    print(f"Error: {e}")
