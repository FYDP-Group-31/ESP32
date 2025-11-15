import serial
import threading
import time
import signal

write_lock = threading.Lock()
run_thread = True

threads: list[threading.Thread] = []

def periodic_ping(ser: serial.Serial):
  while run_thread:
    with write_lock:
      ser.write(b'\x00\x00\x01')  # RPI5_ADDR, CMD_PING
    time.sleep(1)

def serial_read(ser: serial.Serial):
  while run_thread:
    line = ser.read()
    if line:
      print(f"({time.time()}){line}")

def signal_handler(sig, frame):
  global run_thread
  run_thread = False
  for t in threads:
    t.join()

def main():
  signal.signal(signal.SIGINT, signal_handler)
  ser = serial.Serial("/dev/ttyACM1", baudrate=2_000_000, timeout=1)
  ser.reset_output_buffer()

  threads.append(threading.Thread(target=periodic_ping, args=(ser,), daemon=False))
  threads.append(threading.Thread(target=serial_read, args=(ser,), daemon=False))
  for t in threads:
    t.start()


if __name__ == "__main__":
  main()