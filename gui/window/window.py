import tkinter as tk
from tkinter import ttk

import sys
import os
import threading
import subprocess
import time

class Window(tk.Tk):
    def __init__(self):
        self.run_event_loop : bool = True
        self.esp32_port : str = None

        super().__init__()
        self.attributes(
            "-fullscreen", True
        )

        # Get information on display
        self.width = self.winfo_screenwidth()
        self.height = self.winfo_screenheight()

        # Device Menu
        self.dev_menu = tk.Frame(self, bg="lightblue", width=self.width*0.3, height=self.height)
        self.dev_menu.pack(side="left", fill="both", expand=True)

        # Diagnostics Menu
        self.diagnostics_menu = tk.Frame(self, bg="lightgreen", width=self.width*0.7, height=self.height)
        self.diagnostics_menu.pack(side="right", fill="both", expand=True)


        self.threads : list[threading.Thread] = [
            threading.Thread(target=self.init_peripherals, name="INIT"),
            threading.Thread(target=self.event_loop, name="EVENTLOOP")
        ]

        for thread in self.threads:
            thread.start()

        # Get information on platform
        self.platform = sys.platform
        if self.platform == "darwin":
            pass
        elif self.platform.startswith("linux"):
            pass
        else:
            print(f"Unknown platform {self.platform}")

        self.protocol("WM_DELETE_WINDOW", self.close_window)
        self.mainloop()

    def _select_esp32(self, event):
        self.esp32_port = self.dev_select.get()
    
    def init_peripherals(self):
        output = subprocess.check_output("ls /dev", shell=True, text=True)
        devices = output.splitlines()

        self.dev_select = ttk.Combobox(self.dev_menu, values=devices, state="readonly")  # "normal" to allow typing
        self.dev_select.set("Select ESP32")  # default selection
        self.dev_select.pack(padx=1, pady=12)
        self.dev_select.bind("<<ComboboxSelected>>", self._select_esp32)

        self.uart_select = ttk.Combobox(self.dev_menu, values=devices, state="readonly")  # "normal" to allow typing
        self.uart_select.set("Select ESP32 UART")  # default selection
        self.uart_select.pack(padx=1, pady=12)
        self.uart_select.bind("<<ComboboxSelected>>", self._select_esp32)

    def event_loop(self):
        while self.run_event_loop:
            pass

    def close_window(self):
        self.run_event_loop = False
        for thread in self.threads:
            thread.join()
        self.destroy()

def main():
    window = Window()

if __name__ == "__main__":
    main()