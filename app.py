import tkinter as tk
from tkinter import ttk
import threading
import time
import csv
import os
import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from tenmaDcLib import Tenma72_13360
try:
    import adafruit_max31865 as max31865
except ModuleNotFoundError:
    print("Error: The module 'adafruit_max31865' is not installed. Please install it using 'pip install adafruit-circuitpython-max31865'.")
    exit()
import board
import busio
import digitalio

# PID Controller for Adjusting Power Supply Output
class PIDController:
    def __init__(self, kp=1.5, ki=0.1, kd=0.05):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.previous_error = 0
        self.integral = 0

    def compute(self, setpoint, current_temp):
        error = setpoint - current_temp
        self.integral += error
        derivative = error - self.previous_error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.previous_error = error
        return max(0, min(output, 3))  # Clamp output to 0-3A

# Main Tkinter Application
class HeaterControlApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Heater Control")
        self.root.geometry("500x500")
        self.running = False

        # Check SPI availability
        try:
            spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
            cs = digitalio.DigitalInOut(board.D5)
            self.sensor = max31865.MAX31865(spi, cs, wires=2, ref_resistor=430.0)
        except Exception as e:
            print(f"Error initializing MAX31865 sensor: {e}")
            self.sensor = None

        # Initialize power supply communication
        self.power_supply1, self.power_supply2 = self.initialize_power_supplies()

        self.pid = PIDController(kp=1.2, ki=0.2, kd=0.1)

        # Tkinter Variables
        self.current_temp = tk.DoubleVar(value=0.0)
        self.set_temp = tk.DoubleVar(value=30.0)
        self.current_output = tk.DoubleVar(value=0.0)
        self.light_intensity = tk.DoubleVar(value=0.0)

        # GUI Components
        self.setup_gui()

        self.data_log_file = "temperature_log.csv"
        self.setup_logging()

        self.time_stamps = []
        self.temperatures = []
        self.start_time = time.time()

    def initialize_power_supplies(self):
        try:
            serial_port1 = serial.Serial('/dev/ttyACM0', baudrate=9600, timeout=1)
            serial_port2 = serial.Serial('/dev/ttyUSB1', baudrate=9600, timeout=1)
            return Tenma72_13360(serial_port1), Tenma72_13360(serial_port2)
        except Exception as e:
            print(f"Error connecting to power supply: {e}")
            return None, None

    def setup_gui(self):
        ttk.Label(self.root, text="Current Temperature (°C):").pack(pady=5)
        ttk.Label(self.root, textvariable=self.current_temp, font=("Arial", 14)).pack()

        ttk.Label(self.root, text="Set Temperature (°C):").pack(pady=5)
        ttk.Entry(self.root, textvariable=self.set_temp, font=("Arial", 14)).pack()

        ttk.Label(self.root, text="Light Intensity (%):").pack(pady=5)
        ttk.Entry(self.root, textvariable=self.light_intensity, font=("Arial", 14)).pack()

        ttk.Button(self.root, text="Start", command=self.start_control).pack(pady=10)
        ttk.Button(self.root, text="Stop", command=self.stop_control).pack(pady=5)

        ttk.Label(self.root, text="Output Current (A):").pack(pady=5)
        ttk.Label(self.root, textvariable=self.current_output, font=("Arial", 14)).pack()

        ttk.Button(self.root, text="Show Plot", command=self.show_plot).pack(pady=5)

    def show_plot(self):
        plt.figure()
        plt.xlabel("Time (s)")
        plt.ylabel("Temperature (°C)")
        plt.title("Temperature vs. Time")
        plt.grid()
        plt.plot(self.time_stamps, self.temperatures, label="Temperature")
        plt.legend()
        plt.show()

    def setup_logging(self):
        if not os.path.exists(self.data_log_file):
            with open(self.data_log_file, "w", newline="") as file:
                writer = csv.writer(file)
                writer.writerow(["Time (s)", "Set Temperature (°C)", "Current Temperature (°C)", "Output Current (A)", "Light Intensity (%)"])

    def start_control(self):
        self.running = True
        threading.Thread(target=self.control_loop, daemon=True).start()

    def stop_control(self):
        self.running = False
        if self.power_supply1:
            self.power_supply1.setCurrent(0)
        if self.power_supply2:
            self.power_supply2.setCurrent(0)

    def control_loop(self):
        while self.running:
            temp = self.sensor.temperature if self.sensor else 0
            self.current_temp.set(temp)
            setpoint = self.set_temp.get()
            output_current = self.pid.compute(setpoint, temp)
            self.current_output.set(output_current)
            time.sleep(1)

if __name__ == "__main__":
    root = tk.Tk()
    app = HeaterControlApp(root)
    root.protocol("WM_DELETE_WINDOW", app.stop_control)
    root.mainloop()
