import serial
import threading
import tkinter as tk
from tkinter import ttk

class SerialReaderGUI:
    def __init__(self, port='COM6', baudrate=115200):
        self.root = tk.Tk()
        self.root.title("Slėgio Duomenys")
        self.root.geometry("400x200")

        # Etiketės
        self.label_sl = ttk.Label(self.root, text="Sl: ---", font=("Arial", 14))
        self.label_min = ttk.Label(self.root, text="Min: ---", font=("Arial", 12))
        self.label_max = ttk.Label(self.root, text="Max: ---", font=("Arial", 12))
        self.label_status = ttk.Label(self.root, text="Statusas: ---", font=("Arial", 14, 'bold'))

        self.label_sl.pack(pady=5)
        self.label_min.pack(pady=5)
        self.label_max.pack(pady=5)
        self.label_status.pack(pady=10)

        # Serial
        self.ser = serial.Serial(port, baudrate, timeout=1)

        # Gijų paleidimas
        self.running = True
        threading.Thread(target=self.read_serial, daemon=True).start()

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.mainloop()

    def read_serial(self):
        while self.running:
            try:
                line = self.ser.readline().decode("utf-8").strip()
                if line:
                    # Pvz.: "Sl: 101.000 | Min: 100.000 | Max: 105.000 | IN RANGE"
                    self.update_labels(line)
            except:
                continue

    def update_labels(self, line):
        parts = line.split("|")
        if len(parts) == 4:
            sl = parts[0].strip()
            min_val = parts[1].strip()
            max_val = parts[2].strip()
            status = parts[3].strip()

            self.label_sl.config(text=sl)
            self.label_min.config(text=min_val)
            self.label_max.config(text=max_val)
            self.label_status.config(text=f"Statusas: {status}")

            if "IN RANGE" in status:
                self.label_status.config(foreground="green")
            else:
                self.label_status.config(foreground="red")

    def on_close(self):
        self.running = False
        self.ser.close()
        self.root.destroy()

if __name__ == "__main__":
    SerialReaderGUI(port="COM6")  # ← pakeisk, jei kita COM
