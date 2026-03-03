import sys
import numpy as np
import matplotlib.pyplot as plt
from scipy.io import wavfile
import serial
import struct
import time

# Number of loops from command line argument, default to 1
num_loops = int(sys.argv[1]) if len(sys.argv) > 1 else 1

# Read the WAV file
# sample_rate, audio_data = wavfile.read('M1F1-int16-AFsp.wav')
sample_rate, audio_data = wavfile.read('we-are-charlie-kirk.wav')

# Create time axis
duration = len(audio_data) / sample_rate
time_axis = np.linspace(0, duration, len(audio_data))

# Create the plot
plt.figure(figsize=(12, 6))

# If stereo, plot both channels
if len(audio_data.shape) == 2:
    plt.subplot(3, 1, 1)
    plt.plot(time_axis, audio_data[:, 0])
    plt.title('Left Channel')
    plt.xlabel('Time (seconds)')
    plt.ylabel('Amplitude')
    plt.grid(True)
    
    plt.subplot(3, 1, 2)
    plt.plot(time_axis, audio_data[:, 1])
    plt.title('Right Channel')
    plt.xlabel('Time (seconds)')
    plt.ylabel('Amplitude')
    plt.grid(True)
    
    # Combine channels into mono
    mono_data = audio_data.mean(axis=1).astype(np.int16)
    plt.subplot(3, 1, 3)
    plt.plot(time_axis, mono_data)
    plt.title('Combined Mono Channel')
    plt.xlabel('Time (seconds)')
    plt.ylabel('Amplitude')
    plt.grid(True)
else:
    # Mono audio
    mono_data = audio_data.astype(np.int16)
    plt.plot(time_axis, audio_data)
    plt.title(f'Audio Waveform - Sample Rate: {sample_rate} Hz')
    plt.xlabel('Time (seconds)')
    plt.ylabel('Amplitude')
    plt.grid(True)

plt.tight_layout()
plt.savefig('audio_waveform.png', dpi=150, bbox_inches='tight')
print("Saved waveform plot to audio_waveform.png")
plt.close()

# Send audio data over serial
print(f"Sending {len(mono_data)} samples to /dev/ttyAMA0")

# Packet constants from comm_packet.hpp
REQUEST_PACKET = 0xAA
MCU_ADDR = 0x01
CMD_AUDIO_DATA = 0x01
SAMPLES_PER_PACKET = 128

try:
    # Open serial port
    ser = serial.Serial('/dev/ttyAMA0', 2000000, timeout=1)
    time.sleep(0.1)  # Wait for serial to initialize
    
    total_packets = (len(mono_data) + SAMPLES_PER_PACKET - 1) // SAMPLES_PER_PACKET

    for loop_count in range(1, num_loops + 1):
        print(f"--- Loop {loop_count}/{num_loops} ---")

        for i in range(0, len(mono_data), SAMPLES_PER_PACKET):
            # Get chunk of audio samples
            chunk = mono_data[i:i+SAMPLES_PER_PACKET]
            
            # Pad with zeros if last packet is incomplete
            if len(chunk) < SAMPLES_PER_PACKET:
                chunk = np.pad(chunk, (0, SAMPLES_PER_PACKET - len(chunk)), 'constant')
            
            # Create packet header (4 bytes)
            # len field is the number of samples (128), not bytes
            header = struct.pack('BBBB', REQUEST_PACKET, MCU_ADDR, CMD_AUDIO_DATA, SAMPLES_PER_PACKET)
            
            # Convert audio samples to little-endian
            audio_bytes = b''.join(struct.pack('<h', sample) for sample in chunk)
            
            # Send packet
            packet = header + audio_bytes
            ser.write(packet)
            ser.flush()
            
            packet_num = i // SAMPLES_PER_PACKET + 1
            
            if packet_num % 100 == 0:
                print(f"Sent packet {packet_num}/{total_packets}")
            time.sleep(0.002)
        
        print(f"Completed loop {loop_count} ({total_packets} packets, {len(mono_data)} samples)")
    
except KeyboardInterrupt:
    print("\nStopped by user")
except serial.SerialException as e:
    print(f"Error opening serial port: {e}")
except Exception as e:
    print(f"Error: {e}")
finally:
    if 'ser' in dir() and ser.is_open:
        ser.close()
