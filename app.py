import tkinter as tk
from tkinter import ttk
import threading
import time
import csv
import os
import serial
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from tenmaDcLib import Tenma72_2540

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
    def __init__(self, kp=0.5, ki=0.01, kd=0.05, integral_limit=100):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.previous_error = 0
        self.integral = 0
        self.integral_limit = integral_limit  # Limit the integral term

    def compute(self, setpoint, current_temp):
        error = setpoint - current_temp
        self.integral += error
        self.integral = max(-self.integral_limit, min(self.integral, self.integral_limit))
        derivative = error - self.previous_error
        proportional = self.kp * error
        integral = self.ki * self.integral
        derivative_term = self.kd * derivative
        output = (abs(proportional + integral + derivative_term)) / 10
        self.previous_error = error

        print(f"Error: {error}, P: {proportional}, I: {integral}, D: {derivative_term}, Output: {output}")

        return max(0, min(output, 5.1))  # Clamp output to 0-5.1A for Tenma 72-2540

# Main Tkinter Application
class HeaterControlApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Temperature Control System")
        self.root.geometry("1000x800")
        self.running = False
        self.is_logged_in = False

        try:
            spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
            cs = digitalio.DigitalInOut(board.D5)
            self.sensor = max31865.MAX31865(spi, cs, wires=2, ref_resistor=430.0)
        except Exception as e:
            print(f"Error initializing MAX31865 sensor: {e}")
            self.sensor = None

        self.power_supply1 = self.initialize_power_supply()
        self.pid = PIDController(kp=0.5, ki=0.01, kd=0.05)

        self.current_temp = tk.DoubleVar(value=0.0)
        self.set_temp = tk.DoubleVar(value=30.0)
        self.current_output = tk.DoubleVar(value=0.0)
        self.light_intensity = tk.DoubleVar(value=0.0)
        self.connection_status = tk.BooleanVar(value=False)
        self.error_message = tk.StringVar(value="")
        self.temp_profiles = []
        self.initial_temp = tk.DoubleVar(value=25.0)
        self.is_processing = tk.BooleanVar(value=False)
        self.heat_sink_temp = tk.DoubleVar(value=0.0)
        self.filtered_temp = 0.0
        self.alpha = 0.2

        self.setup_login_gui()
        self.data_log_file = "temperature_log.csv"
        self.setup_logging()
        self.time_stamps = []
        self.temperatures = []
        self.start_time = time.time()

    def initialize_power_supply(self):
        try:
            serial_port1 = serial.Serial('/dev/ttyUSB0', baudrate=9600, timeout=1)
            return Tenma72_2540(serial_port1)
        except Exception as e:
            print(f"Error connecting to power supply: {e}")
            return None

    def control_loop(self):
        for profile in self.temp_profiles:
            if not self.is_processing.get():
                break
            self.set_temp.set(profile["T2"])
            start_time = time.time()
            while time.time() - start_time < profile["Stay Time"] * 60:
                if not self.is_processing.get():
                    break
                temp = self.sensor.temperature if self.sensor else 0
                self.filtered_temp = self.alpha * temp + (1 - self.alpha) * self.filtered_temp
                self.current_temp.set(self.filtered_temp)
                output_current = self.pid.compute(self.set_temp.get(), self.filtered_temp)
                self.current_output.set(output_current)
                if self.power_supply1:
                    self.power_supply1.setCurrent(1, int(output_current * 1000))  # Convert A to mA
                self.time_stamps.append(time.time() - self.start_time)
                self.temperatures.append(self.filtered_temp)
                self.update_plot()
                time.sleep(1)

    def setup_logging(self):
        if not os.path.exists(self.data_log_file):
            with open(self.data_log_file, "w", newline="") as file:
                writer = csv.writer(file)
                writer.writerow(["Time (s)", "Set Temperature (°C)", "Current Temperature (°C)", "Output Current (A)", "Light Intensity (%)"])

if __name__ == "__main__":
    root = tk.Tk()
    app = HeaterControlApp(root)
    root.mainloop()
