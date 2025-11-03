#!/usr/bin/env python3
import serial
import time

# Adjust baudrate to match your ESP32-P4 configuration (e.g., 115200, 3000000, etc.)
BAUDRATE = 3000000  
DEVICE   = "/dev/cu.usbmodem5A671689901"

def main():
    try:
        # Open serial port
        ser = serial.Serial(
            port=DEVICE,
            baudrate=BAUDRATE,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1  # seconds
        )

        print(f"Opened {DEVICE} at {BAUDRATE} baud")

        # Example: write repeatedly
        for i in range(10):
            msg = f"Hello ESP32-P4 {i}\r\n"
            ser.write(msg.encode("utf-8"))
            print(f"Sent: {msg.strip()}")
            time.sleep(0.5)

        ser.close()
        print("Closed port")

    except serial.SerialException as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()
