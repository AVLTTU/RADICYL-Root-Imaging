"""
rotate_capture.py

Rotates the stage in user-defined degree increments, captures an image at each step,
and saves images to a user-specified folder with a common prefix.

Dependencies:
  - nidaqmx
  - pypylon (Basler Pylon Python API)
  - Pillow (for saving images)
  - tkinter (for GUI)
"""

import os
import math
import threading
import tkinter as tk
from tkinter import filedialog, messagebox

import nidaqmx
from nidaqmx.constants import AcquisitionType, Level
from pypylon import pylon
from PIL import Image

# ————— GLOBAL HARDWARE CONFIG —————
DEVICE        = "cDAQ1"                     # NI-MAX device name
COUNTER       = f"{DEVICE}Mod1/ctr1"        # CTR1 → pin 25 → STEP+
DIR_LINE      = f"{DEVICE}Mod1/port0/line1" # DIO1  → pin 4 → DIR+
MICROSTEPS    = 1600                        # STR3 DIP SW5–8 = OFF,OFF,OFF,ON
GEAR_RATIO    = 16                          # motor revs per stage rev
RPM_DEFAULT   = 120                         # moderate default speed
# ——————————————————————————————————

def rotate_degrees(degrees: float, rpm: float, clockwise: bool = True):
    """
    Rotate the stage by a specified number of degrees at given RPM.
    Assumes EN is floating (enabled).

    :param degrees: Degrees to rotate (0 < degrees <= 360)
    :param rpm: Speed in RPM (motor side, before gear ratio)
    :param clockwise: True to rotate CW (DIR low), False for CCW.
    """
    # Compute motor-side revolutions needed
    stage_revs = degrees / 360.0
    motor_revs = stage_revs * GEAR_RATIO
    total_steps = int(MICROSTEPS * motor_revs + 0.5)

    # Pulse frequency for motor microsteps
    freq_hz = rpm * MICROSTEPS / 60.0

    # Compute move duration and timeout
    move_time = motor_revs * 60.0 / rpm           # seconds
    timeout = move_time + 2.0                     # add margin

    # 1) Set direction
    with nidaqmx.Task() as dir_task:
        dir_task.do_channels.add_do_chan(DIR_LINE)
        dir_task.write(False if clockwise else True)

    # 2) Generate hardware-timed pulse train on CTR1 → STEP+
    with nidaqmx.Task() as step_task:
        step_task.co_channels.add_co_pulse_chan_freq(
            counter=COUNTER,
            freq=freq_hz,
            duty_cycle=0.5,
            idle_state=Level.LOW
        )
        step_task.timing.cfg_implicit_timing(
            sample_mode=AcquisitionType.FINITE,
            samps_per_chan=total_steps
        )
        step_task.start()
        step_task.wait_until_done(timeout=timeout)

def capture_image(camera, save_path: str):
    """
    Grab one image from the Basler camera and save it to disk.

    :param camera: An instantiated pylon.InstantCamera object, already opened.
    :param save_path: Full file path where image will be saved (e.g., ".../prefix_1.png").
    """
    grab_result = camera.GrabOne(5000)  # 5-second timeout
    if grab_result.GrabSucceeded():
        array = grab_result.GetArray()
        img = Image.fromarray(array)
        img.save(save_path)
        grab_result.Release()
    else:
        grab_result.Release()
        raise RuntimeError(f"Camera grab failed for {save_path}")

class RotateCaptureGUI:
    def __init__(self, master):
        self.master = master
        master.title("Rotate & Capture Setup")

        # X-degree increment
        tk.Label(master, text="Degree increment (factor of 360):").grid(row=0, column=0, sticky="e", padx=5, pady=5)
        self.degree_var = tk.StringVar(value="5")
        tk.Entry(master, textvariable=self.degree_var, width=10).grid(row=0, column=1, padx=5, pady=5)

        # Prefix for filenames
        tk.Label(master, text="Image prefix:").grid(row=1, column=0, sticky="e", padx=5, pady=5)
        self.prefix_var = tk.StringVar(value="sample_test")
        tk.Entry(master, textvariable=self.prefix_var, width=20).grid(row=1, column=1, padx=5, pady=5)

        # Output directory
        tk.Label(master, text="Output directory:").grid(row=2, column=0, sticky="e", padx=5, pady=5)
        self.output_var = tk.StringVar()
        tk.Entry(master, textvariable=self.output_var, width=30).grid(row=2, column=1, padx=5, pady=5)
        tk.Button(master, text="Browse...", command=self.browse_directory).grid(row=2, column=2, padx=5, pady=5)

        # Start button
        self.start_button = tk.Button(master, text="Start", command=self.start_process)
        self.start_button.grid(row=3, column=1, pady=10)

    def browse_directory(self):
        directory = filedialog.askdirectory(title="Select Output Directory")
        if directory:
            self.output_var.set(directory)

    def start_process(self):
        try:
            x_degree = float(self.degree_var.get())
        except ValueError:
            messagebox.showerror("Input Error", "Degree increment must be a number.")
            return

        if 360 % x_degree != 0:
            messagebox.showerror("Input Error", f"{x_degree} is not a factor of 360.")
            return

        prefix = self.prefix_var.get().strip()
        if not prefix:
            messagebox.showerror("Input Error", "Image prefix cannot be empty.")
            return

        output_dir = self.output_var.get().strip()
        if not output_dir:
            messagebox.showerror("Input Error", "Please select an output directory.")
            return

        # Disable UI while running
        self.start_button.config(state="disabled")

        # Run the rotation & capture in a separate thread to keep GUI responsive
        threading.Thread(
            target=self.rotate_and_capture,
            args=(x_degree, prefix, output_dir),
            daemon=True
        ).start()

    def rotate_and_capture(self, x_degree, prefix, output_dir):
        try:
            # 1) Create subfolder
            folder_name = f"{prefix}_images"
            folder_path = os.path.join(output_dir, folder_name)
            os.makedirs(folder_path, exist_ok=True)

            # 2) Initialize Basler camera
            tl_factory = pylon.TlFactory.GetInstance()
            devices = tl_factory.EnumerateDevices()
            if not devices:
                raise RuntimeError("No Basler camera found.")
            camera = pylon.InstantCamera(tl_factory.CreateDevice(devices[0]))
            camera.Open()

            # 3) Capture iteration count
            steps = int(360 / x_degree)
            # 4) Initial capture at 0°
            save_path = os.path.join(folder_path, f"{prefix}_1.png")
            capture_image(camera, save_path)

            # 5) Rotate and capture loop
            for i in range(1, steps):
                # Rotate by x_degree CW
                rotate_degrees(x_degree, RPM_DEFAULT, clockwise=True)

                # Save image i+1
                save_path = os.path.join(folder_path, f"{prefix}_{i+1}.png")
                capture_image(camera, save_path)

            # 6) Final rotate to return to 0° if desired
            rotate_degrees(x_degree, RPM_DEFAULT, clockwise=True)

            camera.Close()
            messagebox.showinfo("Done", f"Captured {steps} images in:\n{folder_path}")
        except Exception as e:
            messagebox.showerror("Error", str(e))
        finally:
            # Re-enable Start button
            self.start_button.config(state="normal")

if __name__ == "__main__":
    root = tk.Tk()
    app = RotateCaptureGUI(root)
    root.mainloop()
