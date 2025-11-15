import serial
import threading
import time
import signal

REQUEST_PACKET = 0xAA
RESPONSE_PACKET = 0x55

RPI5_ADDR = 0x00
MCU_ADDR = 0x01
INVALID_ADDR = 0xFF

CMD_PING = 0x00
CMD_AUDIO_DATA = 0x01
CMD_INVALID = 0xFF

class CommPacketHeader(bytearray):
  def __init__(self, type: int, addr: int, cmd: int, length: int):
    super().__init__((
      type,
      addr,
      cmd,
      length
    ))

class CommPacketPayloadPing(bytearray):
  def __init__(self, msg: int, seq: int):
    super().__init__((
      msg,
      seq
    ))

class CommPacket(bytearray):
  def __init__(self, addr: int, cmd: int, data: bytearray):
    super().__init__()
    self.extend(CommPacketHeader(REQUEST_PACKET, addr, cmd, len(data)))
    self.extend(data)

write_lock = threading.Lock()
run_thread = True

threads: list[threading.Thread] = []

def periodic_ping(ser: serial.Serial):
  seq = 0
  while run_thread:
    pass
    with write_lock:
      ping = CommPacket(MCU_ADDR, CMD_PING, CommPacketPayloadPing(0x42, seq))
      ser.write(ping)
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