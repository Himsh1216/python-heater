import tkinter as tk
from tkinter import ttk
import threading
import time
import csv
import os
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from tenmaDcLib import instantiate_tenma_class_from_device_response

try:
    import adafruit_max31865 as max31865
except ModuleNotFoundError:
    print("Error: The module 'adafruit_max31865' is not installed. "
          "Please install it using 'pip install adafruit-circuitpython-max31865'.")
    exit()

import board
import busio
import digitalio


# ---------------------------------------------------
# Simple PID Controller
# ---------------------------------------------------
class PIDController:
    """
    A simple PID controller using Kp, Ki, Kd, and integral clamping.
    """
    def __init__(self, Kp=1.5, Ki=0.1, Kd=0.13, integral_limit=10000):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral_limit = integral_limit
        
        self.previous_error = 0.0
        self.integral = 0.0

    def compute(self, setpoint, current_temp):
        """
        Compute the PID output.
        :param setpoint: Desired temperature
        :param current_temp: Current measured temperature
        :return: (output_clamped, p_term, i_term, d_term)
                 output_clamped is in [0, 3000] (0–3 A scaled by 1000).
        """
        error = setpoint - current_temp

        # Integrate error
        self.integral += error
        # Clamp the integral to avoid wind-up
        self.integral = max(-self.integral_limit, min(self.integral, self.integral_limit))

        # Derivative
        derivative = error - self.previous_error

        # PID calculations
        p_term = self.Kp * error
        i_term = self.Ki * self.integral
        d_term = self.Kd * derivative
        output = p_term + i_term + d_term

        # Update previous error
        self.previous_error = error

        # Scale from "PID units" to 0–3000 mA
        output_scaled = output * 10

        # Clamp output to [0, 3000] => 0–3 A
        output_clamped = max(0, min(output_scaled, 3000))

        # Debug print
        print(f"PID Debug -> Error: {error:.2f}, P: {p_term:.2f}, I: {i_term:.2f}, "
              f"D: {d_term:.2f}, Output (clamped): {output_clamped:.2f}")

        return output_clamped, p_term, i_term, d_term


# ---------------------------------------------------
# Heater Control Application
# ---------------------------------------------------
class HeaterControlApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Temperature Control System")
        self.root.geometry("1000x800+600+600")
        self.running = False
        self.is_logged_in = False

        # Initialize sensor (MAX31865)
        try:
            spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
            cs = digitalio.DigitalInOut(board.D5)
            self.sensor = max31865.MAX31865(spi, cs, wires=2, ref_resistor=430.0)
        except Exception as e:
            print(f"Error initializing MAX31865 sensor: {e}")
            self.sensor = None

        # We'll instantiate self.power_supply once the user clicks "Connect Device"
        self.power_supply = None

        # PID controller
        self.pid = PIDController(Kp=2.0, Ki=0.1, Kd=0.13)

        # Variables
        self.current_temp = tk.DoubleVar(value=0.0)
        self.set_temp = tk.DoubleVar(value=30.0)
        self.current_output = tk.DoubleVar(value=0.0)
        self.connection_status = tk.BooleanVar(value=False)
        self.error_message = tk.StringVar(value="")
        self.temp_profiles = []
        self.initial_temp = tk.DoubleVar(value=25.0)
        self.is_processing = tk.BooleanVar(value=False)
        self.heat_sink_temp = tk.DoubleVar(value=0.0)  # if you have a second sensor

        # Setup login GUI
        self.setup_login_gui()

        # CSV logging
        self.data_log_file = "temperature_log.csv"
        self.setup_logging()

        # For plotting
        self.time_stamps = []
        self.temperatures = []
        self.start_time = time.time()

    # ------------------------------------------------
    # Login GUI
    # ------------------------------------------------
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

        if username == "admin" and password == "password":
            self.is_logged_in = True
            self.login_frame.pack_forget()
            self.setup_gui()
        else:
            self.error_message.set("Invalid username or password")

    # ------------------------------------------------
    # Main GUI
    # ------------------------------------------------
    def setup_gui(self):
        self.canvas = tk.Canvas(self.root)
        self.scrollbar = ttk.Scrollbar(self.root, orient="vertical", command=self.canvas.yview)
        self.scrollable_frame = ttk.Frame(self.canvas)

        self.scrollable_frame.bind(
            "<Configure>",
            lambda e: self.canvas.configure(scrollregion=self.canvas.bbox("all"))
        )
        self.canvas.create_window((0, 0), window=self.scrollable_frame, anchor="nw")
        self.canvas.configure(yscrollcommand=self.scrollbar.set)

        self.canvas.pack(side="left", fill="both", expand=True)
        self.scrollbar.pack(side="right", fill="y")

        main_container = ttk.Frame(self.scrollable_frame)
        main_container.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        title_label = ttk.Label(main_container, text="Temperature Control System", font=("Arial", 16, "bold"))
        title_label.pack(pady=10)

        error_label = ttk.Label(main_container, textvariable=self.error_message, foreground="red")
        error_label.pack()

        # Current readings
        current_readings_frame = ttk.LabelFrame(main_container, text="Current Readings")
        current_readings_frame.pack(fill=tk.X, pady=10)

        ttk.Label(current_readings_frame, text="Current Temperature:").pack()
        ttk.Label(current_readings_frame, textvariable=self.current_temp, font=("Arial", 14)).pack()

        ttk.Label(current_readings_frame, text="Heat Sink Temperature:").pack()
        ttk.Label(current_readings_frame, textvariable=self.heat_sink_temp, font=("Arial", 14)).pack()

        # Connection frame
        connection_frame = ttk.Frame(main_container)
        connection_frame.pack(fill=tk.X, pady=10)

        connect_button = ttk.Button(connection_frame, text="Connect Device", command=self.connect_device)
        connect_button.pack(side=tk.LEFT, padx=5)

        connection_status_label = ttk.Label(connection_frame, textvariable=self.connection_status)
        connection_status_label.pack(side=tk.LEFT, padx=5)

        # Profile frame
        profile_frame = ttk.LabelFrame(main_container, text="Temperature Profile")
        profile_frame.pack(fill=tk.BOTH, expand=True, pady=10)

        self.profile_table = ttk.Treeview(profile_frame, columns=("T1", "T2", "Stay Time"), show="headings")
        self.profile_table.heading("T1", text="T1 (°C)")
        self.profile_table.heading("T2", text="T2 (°C)")
        self.profile_table.heading("Stay Time", text="Stay Time (min)")
        self.profile_table.pack(fill=tk.BOTH, expand=True)

        input_frame = ttk.Frame(profile_frame)
        input_frame.pack(pady=10)

        ttk.Label(input_frame, text="T2 (°C):").grid(row=0, column=0, padx=5)
        self.t2_entry = ttk.Entry(input_frame)
        self.t2_entry.grid(row=0, column=1, padx=5)

        ttk.Label(input_frame, text="Stay Time (min):").grid(row=0, column=2, padx=5)
        self.stay_time_entry = ttk.Entry(input_frame)
        self.stay_time_entry.grid(row=0, column=3, padx=5)

        add_reading_button = ttk.Button(input_frame, text="Add Profile", command=self.add_reading)
        add_reading_button.grid(row=0, column=4, padx=5)

        button_frame = ttk.Frame(profile_frame)
        button_frame.pack(pady=10)

        start_process_button = ttk.Button(button_frame, text="Start Process", command=self.start_process)
        start_process_button.pack(side=tk.LEFT, padx=5)

        stop_process_button = ttk.Button(button_frame, text="Stop Process", command=self.stop_process)
        stop_process_button.pack(side=tk.LEFT, padx=5)

        # Plot frame
        plot_frame = ttk.LabelFrame(main_container, text="Temperature vs Time")
        plot_frame.pack(fill=tk.BOTH, expand=True, pady=10)

        self.fig, self.ax = plt.subplots()
        self.canvas_plot = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas_plot.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    # ------------------------------------------------
    # Connect Power Supply Device
    # ------------------------------------------------
    def connect_device(self):
        """
        Attempt to connect to the Tenma power supply on a specific port.
        If successful, assign the instantiated Tenma object to self.power_supply
        and set self.connection_status to True.
        """
        try:
            port = '/dev/ttyACM0'  # Change if your device is on a different port
            self.power_supply = instantiate_tenma_class_from_device_response(port)
            self.connection_status.set(True)
            self.error_message.set("Connected to the Tenma power supply successfully.")
        except Exception as e:
            self.connection_status.set(False)
            self.error_message.set(f"Failed to connect to the device: {e}")

    def add_reading(self):
        """
        Adds a new temperature step to the profile. T1 is either the initial
        temperature or the previous step's T2.
        """
        try:
            t1_value = self.initial_temp.get() if not self.temp_profiles else self.temp_profiles[-1]["T2"]
            t2_value = float(self.t2_entry.get())
            stay_time = float(self.stay_time_entry.get())

            new_profile = {
                "T1": t1_value,
                "T2": t2_value,
                "Stay Time": stay_time
            }
            self.temp_profiles.append(new_profile)
            self.update_profile_table()

            self.t2_entry.delete(0, tk.END)
            self.stay_time_entry.delete(0, tk.END)
        except ValueError:
            self.error_message.set("Invalid input. Please enter numeric values for T2 and stay time.")

    def update_profile_table(self):
        for row in self.profile_table.get_children():
            self.profile_table.delete(row)

        for profile in self.temp_profiles:
            self.profile_table.insert(
                "",
                tk.END,
                values=(profile["T1"], profile["T2"], profile["Stay Time"])
            )

    # ------------------------------------------------
    # Start / Stop Process
    # ------------------------------------------------
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
        self.start_time = time.time()

        # Launch the control loop in a separate thread
        threading.Thread(target=self.control_loop, daemon=True).start()

    def stop_process(self):
        self.is_processing.set(False)

    # ------------------------------------------------
    # Control Logic with sub-step (T2 - 5)
    # ------------------------------------------------
    def control_loop(self):
        """
        For each profile:
          1. If T2 > T1, first ramp to (T2 - 5), dwell 40s, then ramp to T2 and dwell for 'Stay Time'.
          2. If T2 <= T1, ramp directly to T2 and dwell.
        """
        if self.power_supply:
            # Reset the Tenma power supply to 0 A at the start of the process
            self.power_supply.OFF()
            self.power_supply.setVoltage(1, 30000)  # 30 V (adjust as needed)
            self.power_supply.setCurrent(1, 2500)   # Start with some safe current limit
            self.power_supply.ON()

        for profile in self.temp_profiles:
            if not self.is_processing.get():
                break

            T1 = profile["T1"]
            T2 = profile["T2"]
            stay_time = profile["Stay Time"]

            # --------------------------------------------
            # If T2 > T1, do sub-step to (T2 - 5)
            # --------------------------------------------
            if T2 > T1:
                T2_sub = T2 - 5

                # If T2_sub is below T1, clamp or skip
                if T2_sub < T1:
                    T2_sub = T1

                # ---- Sub-step A: Ramp T1 -> T2_sub ----
                if abs(T2_sub - T1) > 0.5:  # Only if there's an actual gap
                    self.set_temp.set(T2_sub)
                    while self.is_processing.get():
                        temp = self.sensor.temperature if self.sensor else T1
                        self.current_temp.set(temp)

                        # PID output
                        output_current, p_term, i_term, d_term = self.pid.compute(self.set_temp.get(), temp)
                        if self.power_supply:
                            try:
                                self.power_supply.setCurrent(1, int(output_current))
                            except Exception as e:
                                print(f"Warning: Could not set power supply current: {e}")

                        # Log for plotting & CSV
                        elapsed_time = time.time() - self.start_time
                        self.time_stamps.append(elapsed_time)
                        self.temperatures.append(temp)
                        self.log_data(
                            elapsed_time,
                            self.set_temp.get(),
                            temp,
                            p_term,
                            i_term,
                            d_term,
                            output_current
                        )
                        self.update_plot()

                        # Check if we've reached T2_sub within +/- 0.5°C
                        if temp >= (T2_sub - 0.5):
                            break
                        time.sleep(1)

                    # ---- Sub-step A dwell: hold T2_sub for 40 seconds ----
                    dwell_start = time.time()
                    while self.is_processing.get() and (time.time() - dwell_start < 40):
                        temp = self.sensor.temperature if self.sensor else T2_sub
                        self.current_temp.set(temp)

                        # PID to hold T2_sub
                        output_current, p_term, i_term, d_term = self.pid.compute(T2_sub, temp)
                        if self.power_supply:
                            try:
                                self.power_supply.setCurrent(1, int(output_current))
                            except Exception as e:
                                print(f"Warning: Could not set power supply current: {e}")

                        # Log and plot
                        elapsed_time = time.time() - self.start_time
                        self.time_stamps.append(elapsed_time)
                        self.temperatures.append(temp)
                        self.log_data(
                            elapsed_time,
                            T2_sub,
                            temp,
                            p_term,
                            i_term,
                            d_term,
                            output_current
                        )
                        self.update_plot()
                        time.sleep(0.2)

                # ---- Sub-step B: Now ramp T2_sub -> T2 ----
                self.set_temp.set(T2)
                while self.is_processing.get():
                    temp = self.sensor.temperature if self.sensor else T2_sub
                    self.current_temp.set(temp)

                    # PID output
                    output_current, p_term, i_term, d_term = self.pid.compute(T2, temp)
                    if self.power_supply:
                        try:
                            self.power_supply.setCurrent(1, int(output_current))
                        except Exception as e:
                            print(f"Warning: Could not set power supply current: {e}")

                    # Log for plotting & CSV
                    elapsed_time = time.time() - self.start_time
                    self.time_stamps.append(elapsed_time)
                    self.temperatures.append(temp)
                    self.log_data(
                        elapsed_time,
                        T2,
                        temp,
                        p_term,
                        i_term,
                        d_term,
                        output_current
                    )
                    self.update_plot()

                    # Check if we've reached T2 within +/- 0.5°C
                    if temp >= (T2 - 0.5):
                        break
                    time.sleep(1)

                # ---- Sub-step B dwell: hold at T2 for stay_time ----
                stay_start = time.time()
                while self.is_processing.get() and (time.time() - stay_start < stay_time * 60):
                    temp = self.sensor.temperature if self.sensor else T2
                    self.current_temp.set(temp)

                    # PID to hold T2
                    output_current, p_term, i_term, d_term = self.pid.compute(T2, temp)
                    if self.power_supply:
                        try:
                            self.power_supply.setCurrent(1, int(output_current))
                        except Exception as e:
                            print(f"Warning: Could not set power supply current: {e}")

                    # Log
                    elapsed_time = time.time() - self.start_time
                    self.time_stamps.append(elapsed_time)
                    self.temperatures.append(temp)
                    self.log_data(
                        elapsed_time,
                        T2,
                        temp,
                        p_term,
                        i_term,
                        d_term,
                        output_current
                    )
                    self.update_plot()
                    time.sleep(0.3)

            else:
                # ---------------------------------------------------------
                # If T2 <= T1, do the "normal" ramp & dwell logic
                # ---------------------------------------------------------
                self.set_temp.set(T2)

                # Ramp until T2 is reached within +/- 0.5°C
                while self.is_processing.get():
                    temp = self.sensor.temperature if self.sensor else T1
                    self.current_temp.set(temp)

                    output_current, p_term, i_term, d_term = self.pid.compute(T2, temp)
                    if self.power_supply:
                        try:
                            self.power_supply.setCurrent(1, int(output_current))
                        except Exception as e:
                            print(f"Warning: Could not set power supply current: {e}")

                    # Log for plotting & CSV
                    elapsed_time = time.time() - self.start_time
                    self.time_stamps.append(elapsed_time)
                    self.temperatures.append(temp)
                    self.log_data(
                        elapsed_time,
                        T2,
                        temp,
                        p_term,
                        i_term,
                        d_term,
                        output_current
                    )
                    self.update_plot()

                    # Check if we've reached T2 within +/- 0.5°C
                    if temp <= (T2 + 0.5):
                        break
                    time.sleep(1)

                # Dwell for the specified 'Stay Time'
                stay_start = time.time()
                while self.is_processing.get() and (time.time() - stay_start < stay_time * 60):
                    temp = self.sensor.temperature if self.sensor else T2
                    self.current_temp.set(temp)
                    output_current, p_term, i_term, d_term = self.pid.compute(T2, temp)

                    if self.power_supply:
                        try:
                            self.power_supply.setCurrent(1, int(output_current))
                        except Exception as e:
                            print(f"Warning: Could not set power supply current: {e}")

                    # Log
                    elapsed_time = time.time() - self.start_time
                    self.time_stamps.append(elapsed_time)
                    self.temperatures.append(temp)
                    self.log_data(
                        elapsed_time,
                        T2,
                        temp,
                        p_term,
                        i_term,
                        d_term,
                        output_current
                    )
                    self.update_plot()
                    time.sleep(0.3)

        # After all profiles or if stopped, turn off the supply
        if self.power_supply:
            self.power_supply.setCurrent(1, 0)
            self.power_supply.OFF()

    # ------------------------------------------------
    # CSV Logging
    # ------------------------------------------------
    def setup_logging(self):
        """
        Sets up a CSV logging file with headers if it doesn't exist.
        We add columns for Time, Set Temperature, Current Temperature,
        Proportional, Integral, Derivative, and Output Current.
        """
        if not os.path.exists(self.data_log_file):
            with open(self.data_log_file, "w", newline="") as file:
                writer = csv.writer(file)
                writer.writerow([
                    "Time (s)",
                    "Set Temperature (°C)",
                    "Current Temperature (°C)",
                    "Proportional",
                    "Integral",
                    "Derivative",
                    "Output Current (A)"
                ])

    def log_data(self, time_s, set_temp, current_temp, p, i, d, output_current):
        """
        Append a single row of data to the CSV log.
        """
        with open(self.data_log_file, "a", newline="") as file:
            writer = csv.writer(file)
            writer.writerow([time_s, set_temp, current_temp, p, i, d, output_current])

    # ------------------------------------------------
    # Plotting
    # ------------------------------------------------
    def update_plot(self):
        self.ax.clear()
        self.ax.plot(self.time_stamps, self.temperatures, label="Temperature")
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("Temperature (°C)")
        self.ax.legend()
        self.canvas_plot.draw()


if __name__ == "__main__":
    root = tk.Tk()
    app = HeaterControlApp(root)
    root.mainloop()
