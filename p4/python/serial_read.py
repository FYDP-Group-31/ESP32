#!/usr/bin/env python3
import serial

BAUDRATE = 3000000  # Match this to your ESP32-P4 UART config
DEVICE   = "/dev/cu.usbmodem5A671689901"

def main():
    try:
        ser = serial.Serial(
            port=DEVICE,
            baudrate=BAUDRATE,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1  # seconds; blocks until timeout or data
        )

        print(f"Listening on {DEVICE} at {BAUDRATE} baud... (Ctrl+C to quit)")

        while True:
            line = ser.readline()  # read until newline or timeout
            if line:
                print(f"Received: {line.decode(errors='ignore').strip()}")

    except KeyboardInterrupt:
        print("\nExiting")
    except serial.SerialException as e:
        print(f"Serial error: {e}")
    finally:
        try:
            ser.close()
        except:
            pass

if __name__ == "__main__":
    main()
