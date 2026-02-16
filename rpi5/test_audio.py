import numpy as np
import matplotlib.pyplot as plt
from scipy.io import wavfile
import serial
import struct
import time

# Read the WAV file
sample_rate, audio_data = wavfile.read('M1F1-int16-AFsp.wav')

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
print(f"Sending {len(mono_data)} samples to /dev/ttyACM1...")

# Packet constants from comm_packet.hpp
REQUEST_PACKET = 0xAA
MCU_ADDR = 0x01
CMD_AUDIO_DATA = 0x01
SAMPLES_PER_PACKET = 128

try:
    # Open serial port
    ser = serial.Serial('/dev/ttyACM1', 2000000, timeout=1)
    time.sleep(0.1)  # Wait for serial to initialize
    
    # Open packet log file
    with open('packet_log.txt', 'w') as log_file:
        log_file.write("Audio Packet Transmission Log\n")
        log_file.write("=" * 80 + "\n\n")
        
        # Send audio data in packets
        total_packets = (len(mono_data) + SAMPLES_PER_PACKET - 1) // SAMPLES_PER_PACKET
        
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
            
            packet_num = i // SAMPLES_PER_PACKET + 1
            
            # Log packet details
            log_file.write(f"Packet #{packet_num}/{total_packets}\n")
            log_file.write(f"  Header: [0x{REQUEST_PACKET:02X}, 0x{MCU_ADDR:02X}, 0x{CMD_AUDIO_DATA:02X}, 0x{SAMPLES_PER_PACKET:02X}]\n")
            log_file.write(f"  Packet size: {len(packet)} bytes (4 header + 256 payload)\n")
            log_file.write(f"  Sample range: {i} to {i + len(chunk) - 1}\n")
            log_file.write(f"  First 5 samples: {chunk[:5].tolist()}\n")
            log_file.write(f"  Last 5 samples: {chunk[-5:].tolist()}\n")
            log_file.write(f"  First 10 payload bytes (hex): {' '.join(f'{b:02X}' for b in audio_bytes[:10])}\n")
            log_file.write("\n")
            
            if packet_num % 100 == 0:
                print(f"Sent packet {packet_num}/{total_packets}")
        
        print(f"Successfully sent {total_packets} packets ({len(mono_data)} samples)")
        print(f"Packet log saved to packet_log.txt")
    
    ser.close()
    
except serial.SerialException as e:
    print(f"Error opening serial port: {e}")
except Exception as e:
    print(f"Error: {e}")
