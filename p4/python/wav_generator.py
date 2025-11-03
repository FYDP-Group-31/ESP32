from scipy.io.wavfile import write
import numpy as np
import matplotlib.pyplot as plt

SAMPLE_RATE = 48000
PORT = "/dev/cu.usbmodem5A671689901"

class Tone:
    def __init__(self, frequency: int | float):
        self.frequency = frequency

class MonoToneGenerator:
    def __init__(self, sample_rate: int, frequency: int | float, periods: int):
        self.sample_rate = sample_rate
        self.frequency = frequency
        self.periods = periods
        self.duration = periods / frequency

        t = np.linspace(0, self.duration, int(sample_rate * self.duration))
        waveform = 0.5 * np.sin(2 * np.pi * frequency * t)

        plt.figure(figsize=(10, 4))
        plt.plot(t, waveform)
        plt.title(f"{frequency} Hz Sine Wave ({periods} periods)")
        plt.xlabel("Time [s]")
        plt.ylabel("Amplitude")
        plt.grid(True)
        plt.savefig(f"{frequency}hz_{sample_rate}.png", dpi=3000, bbox_inches="tight")
        plt.close()
        


def main():
    MonoToneGenerator(
        sample_rate=48000,
        frequency=440,
        periods=1000
    )

if __name__ == "__main__":
    main()