#!/usr/bin/env python3
import serial
import threading

BAUDRATE = 115200            # match your ESP32-P4 UART setting
DEVICE   = "/dev/tty.usbmodem5A671689901"

def reader(ser):
    while True:
        line = ser.read()
        if line:
            # print(f"[RX] {line.decode(errors='ignore').strip()}")
            print(line)

def main():
    ser = serial.Serial(DEVICE, BAUDRATE, timeout=1)
    print(f"Connected to {DEVICE} at {BAUDRATE} baud")

    # Start reader thread
    t = threading.Thread(target=reader, args=(ser,), daemon=True)
    t.start()

    # Writer loop
    try:
        while True:
            msg = "aa"
            ser.write((msg + "\r\n").encode("utf-8"))
            
    except KeyboardInterrupt:
        print("\nClosing...")
        ser.close()

if __name__ == "__main__":
    main()
