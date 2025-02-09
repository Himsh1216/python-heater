import tkinter as tk
from tkinter import ttk
import threading
import time
import csv
import os
import serial
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
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
        # Anti-windup: Limit the integral term
        self.integral = max(-self.integral_limit, min(self.integral, self.integral_limit))
        derivative = error - self.previous_error
        proportional = self.kp * error
        integral = self.ki * self.integral
        derivative_term = self.kd * derivative
        output = (abs(proportional + integral + derivative_term))/10
        self.previous_error = error

        # Log PID terms for debugging
        print(f"Error: {error}, P: {proportional}, I: {integral}, D: {derivative_term}, Output: {output}")

        return max(0, min(output, 3))  # Clamp output to 0-3A

# Main Tkinter Application
class HeaterControlApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Temperature Control System")
        self.root.geometry("1000x800")
        self.running = False
        self.is_logged_in = False

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

        self.pid = PIDController(kp=0.5, ki=0.01, kd=0.05)  # Updated PID values

        # Tkinter Variables
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
        self.filtered_temp = 0.0  # Initialize filtered temperature
        self.alpha = 0.2  # Smoothing factor (0 < alpha < 1)

        # GUI Components
        self.setup_login_gui()

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

    def setup_login_gui(self):
        self.login_frame = ttk.Frame(self.root)
        self.login_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        ttk.Label(self.login_frame, text="Login", font=("Arial", 16, "bold")).pack(pady=10)

        self.username_var = tk.StringVar()
        self.password_var = tk.StringVar()

        ttk.Label(self.login_frame, text="Username:").pack()
        ttk.Entry(self.login_frame, textvariable=self.username_var).pack(pady=5)

        ttk.Label(self.login_frame, text="Password:").pack()
        ttk.Entry(self.login_frame, textvariable=self.password_var, show="*").pack(pady=5)

        ttk.Button(self.login_frame, text="Login", command=self.handle_login).pack(pady=10)

    def handle_login(self):
        username = self.username_var.get()
        password = self.password_var.get()

        # Simple authentication (replace with your own logic)
        if username == "admin" and password == "password":
            self.is_logged_in = True
            self.login_frame.pack_forget()
            self.setup_gui()
        else:
            self.error_message.set("Invalid username or password")

    def setup_gui(self):
        # Create a canvas and a vertical scrollbar
        self.canvas = tk.Canvas(self.root)
        self.scrollbar = ttk.Scrollbar(self.root, orient="vertical", command=self.canvas.yview)
        self.scrollable_frame = ttk.Frame(self.canvas)

        # Configure the canvas
        self.scrollable_frame.bind(
            "<Configure>",
            lambda e: self.canvas.configure(
                scrollregion=self.canvas.bbox("all")
            )
        )
        self.canvas.create_window((0, 0), window=self.scrollable_frame, anchor="nw")
        self.canvas.configure(yscrollcommand=self.scrollbar.set)

        # Pack the canvas and scrollbar
        self.canvas.pack(side="left", fill="both", expand=True)
        self.scrollbar.pack(side="right", fill="y")

        # Main container inside the scrollable frame
        main_container = ttk.Frame(self.scrollable_frame)
        main_container.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # Title and error message
        title_label = ttk.Label(main_container, text="Temperature Control System", font=("Arial", 16, "bold"))
        title_label.pack(pady=10)

        error_label = ttk.Label(main_container, textvariable=self.error_message, foreground="red")
        error_label.pack()

        # Current Readings
        current_readings_frame = ttk.LabelFrame(main_container, text="Current Readings")
        current_readings_frame.pack(fill=tk.X, pady=10)

        ttk.Label(current_readings_frame, text="Current Temperature:").pack()
        ttk.Label(current_readings_frame, textvariable=self.current_temp, font=("Arial", 14)).pack()

        ttk.Label(current_readings_frame, text="Heat Sink Temperature:").pack()
        ttk.Label(current_readings_frame, textvariable=self.heat_sink_temp, font=("Arial", 14)).pack()

        # Connection Status
        connection_frame = ttk.Frame(main_container)
        connection_frame.pack(fill=tk.X, pady=10)

        connect_button = ttk.Button(connection_frame, text="Connect Device", command=self.connect_device)
        connect_button.pack(side=tk.LEFT, padx=5)

        connection_status_label = ttk.Label(connection_frame, textvariable=self.connection_status)
        connection_status_label.pack(side=tk.LEFT, padx=5)

        # Temperature Profile
        profile_frame = ttk.LabelFrame(main_container, text="Temperature Profile")
        profile_frame.pack(fill=tk.BOTH, expand=True, pady=10)

        self.profile_table = ttk.Treeview(profile_frame, columns=("T1", "T2", "Stay Time", "Light Intensity"), show="headings")
        self.profile_table.heading("T1", text="T1 (°C)")
        self.profile_table.heading("T2", text="T2 (°C)")
        self.profile_table.heading("Stay Time", text="Stay Time (min)")
        self.profile_table.heading("Light Intensity", text="Light Intensity (%)")
        self.profile_table.pack(fill=tk.BOTH, expand=True)

        # Input fields for profile data
        input_frame = ttk.Frame(profile_frame)
        input_frame.pack(pady=10)

        ttk.Label(input_frame, text="T2 (°C):").grid(row=0, column=0, padx=5)
        self.t2_entry = ttk.Entry(input_frame)
        self.t2_entry.grid(row=0, column=1, padx=5)

        ttk.Label(input_frame, text="Stay Time (min):").grid(row=0, column=2, padx=5)
        self.stay_time_entry = ttk.Entry(input_frame)
        self.stay_time_entry.grid(row=0, column=3, padx=5)

        ttk.Label(input_frame, text="Light Intensity (%):").grid(row=0, column=4, padx=5)
        self.light_intensity_entry = ttk.Entry(input_frame)
        self.light_intensity_entry.grid(row=0, column=5, padx=5)

        add_reading_button = ttk.Button(input_frame, text="Add Profile", command=self.add_reading)
        add_reading_button.grid(row=0, column=6, padx=5)

        # Start and Stop Process Buttons
        button_frame = ttk.Frame(profile_frame)
        button_frame.pack(pady=10)

        start_process_button = ttk.Button(button_frame, text="Start Process", command=self.start_process)
        start_process_button.pack(side=tk.LEFT, padx=5)

        stop_process_button = ttk.Button(button_frame, text="Stop Process", command=self.stop_process)
        stop_process_button.pack(side=tk.LEFT, padx=5)

        # Temperature vs Time Plot
        plot_frame = ttk.LabelFrame(main_container, text="Temperature vs Time")
        plot_frame.pack(fill=tk.BOTH, expand=True, pady=10)

        self.fig, self.ax = plt.subplots()
        self.canvas_plot = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas_plot.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def connect_device(self):
        try:
            self.connection_status.set(True)
            self.error_message.set("")
        except Exception as e:
            self.error_message.set(f"Failed to connect to the device: {e}")

    def add_reading(self):
        try:
            t1_value = self.initial_temp.get() if not self.temp_profiles else self.temp_profiles[-1]["T2"]
            t2_value = float(self.t2_entry.get())
            stay_time = float(self.stay_time_entry.get())
            light_intensity = float(self.light_intensity_entry.get())

            new_profile = {"T1": t1_value, "T2": t2_value, "Stay Time": stay_time, "Light Intensity": light_intensity}
            self.temp_profiles.append(new_profile)
            self.update_profile_table()

            # Clear input fields
            self.t2_entry.delete(0, tk.END)
            self.stay_time_entry.delete(0, tk.END)
            self.light_intensity_entry.delete(0, tk.END)
        except ValueError:
            self.error_message.set("Invalid input. Please enter numeric values.")

    def update_profile_table(self):
        for row in self.profile_table.get_children():
            self.profile_table.delete(row)
        for profile in self.temp_profiles:
            self.profile_table.insert("", tk.END, values=(profile["T1"], profile["T2"], profile["Stay Time"], profile["Light Intensity"]))

    def start_process(self):
        if not self.connection_status.get():
            self.error_message.set("Device not connected")
            return
        if not self.temp_profiles:
            self.error_message.set("No temperature profiles added")
            return
        self.is_processing.set(True)
        self.error_message.set("")
        self.time_stamps = []
        self.temperatures = []
        threading.Thread(target=self.control_loop, daemon=True).start()

    def stop_process(self):
        self.is_processing.set(False)

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
                # Apply low-pass filter to temperature readings
                self.filtered_temp = self.alpha * temp + (1 - self.alpha) * self.filtered_temp
                self.current_temp.set(self.filtered_temp)
                output_current = self.pid.compute(self.set_temp.get(), self.filtered_temp)
                self.current_output.set(output_current)
                self.time_stamps.append(time.time() - self.start_time)
                self.temperatures.append(self.filtered_temp)
                self.update_plot()
                time.sleep(1)

    def update_plot(self):
        self.ax.clear()
        self.ax.plot(self.time_stamps, self.temperatures, label="Temperature")
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("Temperature (°C)")
        self.ax.legend()
        self.canvas_plot.draw()

    def setup_logging(self):
        if not os.path.exists(self.data_log_file):
            with open(self.data_log_file, "w", newline="") as file:
                writer = csv.writer(file)
                writer.writerow(["Time (s)", "Set Temperature (°C)", "Current Temperature (°C)", "Output Current (A)", "Light Intensity (%)"])

if __name__ == "__main__":
    root = tk.Tk()
    app = HeaterControlApp(root)
    root.mainloop()