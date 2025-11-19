import serial
import threading
import time
import signal
from enum import Enum

REQUEST_PACKET = 0xAA
RESPONSE_PACKET = 0x55

RPI5_ADDR = 0x00
MCU_ADDR = 0x01
INVALID_ADDR = 0xFF

CMD_PING = 0x00
CMD_AUDIO_DATA = 0x01
CMD_RESET = 0x02
CMD_INVALID = 0xFF

class CommPacketHeader(bytearray):
  def __init__(self, type: int, addr: int, cmd: int, length: int):
    super().__init__((
      type,
      addr,
      cmd,
      length & 0xFF,
      (length >> 8) & 0xFF # forces length to be 2 bytes
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

class ReadState(Enum):
  READ_STATE_IDLE = 0
  READ_STATE_CHECK_TYPE = 1
  READ_STATE_CHECK_ADDR = 2
  READ_STATE_CHECK_CMD = 3
  READ_STATE_CHECK_LENGTH = 4
  READ_STATE_READ_PAYLOAD = 5
  READ_STATE_VERIFY_CRC = 6
  READ_STATE_HANDLE_ERROR = 7

class ESP32_Comms:
  def __init__(self, control_port: str, audio_port: str, baudrate: int):
    self.ser = serial.Serial(control_port, baudrate=baudrate, timeout=1)
    self.ser.reset_output_buffer()

    self.read_state = ReadState.READ_STATE_IDLE
    self.threads: list[threading.Thread] = [
      threading.Thread(target=self._serial_read_thread, daemon=False),
      threading.Thread(target=self._periodic_ping_thread, daemon=False)
    ]
    self.write_lock = threading.Lock()
    self.run_thread = False

  def start_threads(self):
    self.run_thread = True
    for t in self.threads:
      t.start()

  def stop_threads(self):
    self.run_thread = False
    for t in self.threads:
      t.join()

  def _serial_read_thread(self):
    while self.run_thread:
      line = self.ser.read_all()
      if line:
        for byte in line:
          match self.read_state:
            case ReadState.READ_STATE_IDLE:
              if byte == RESPONSE_PACKET:
                self.read_state = ReadState.READ_STATE_CHECK_ADDR
              elif byte == REQUEST_PACKET:
                self.read_state = ReadState.READ_STATE_CHECK_ADDR
              else:
                pass
            case ReadState.READ_STATE_CHECK_ADDR:
              if byte == RPI5_ADDR:
                self.read_state = ReadState.READ_STATE_CHECK_CMD
              else:
                self.read_state = ReadState.READ_STATE_IDLE
            case ReadState.READ_STATE_CHECK_CMD:
              if byte == CMD_PING:
                print("Ping")
                self.read_state = ReadState.READ_STATE_CHECK_LENGTH
              elif byte == CMD_AUDIO_DATA:
                print("Audio Data")
                self.read_state = ReadState.READ_STATE_CHECK_LENGTH
              elif byte == CMD_RESET:
                print("Reset")
                self.read_state = ReadState.READ_STATE_CHECK_LENGTH
              else:
                self.read_state = ReadState.READ_STATE_IDLE
            case ReadState.READ_STATE_CHECK_LENGTH:
              print(f"Lenth: {byte}")
              self.read_state = ReadState.READ_STATE_IDLE
            case _:
              self.read_state = ReadState.READ_STATE_IDLE

  def _periodic_ping_thread(self):
    seq = 0
    while self.run_thread:
      with self.write_lock:
        ping = CommPacket(MCU_ADDR, CMD_PING, CommPacketPayloadPing(0x42, seq))
        print(f"{time.time()} Pinging MCU ({seq})")
        self.ser.write(ping)
        seq += 2
        if seq >= 256:
          seq = 0
      time.sleep(1)
  
  def _signal_handler(self, sig, frame) -> None:
    self.stop_threads()



def main():
  esp32_comms = ESP32_Comms(control_port="/dev/ttyAMA0", audio_port="/dev/ttyAMA1", baudrate=2_000_000)
  signal.signal(signal.SIGINT, esp32_comms._signal_handler)
  esp32_comms.start_threads()



if __name__ == "__main__":
  main()