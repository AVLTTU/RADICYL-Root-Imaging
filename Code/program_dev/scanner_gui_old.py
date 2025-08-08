"""
scanner_gui.py

A streamlined GUI for high-throughput plant imaging and SLEAP annotation:

Features:
  1. Scan a plant by rotating in user-specified degree increments,
     capturing images at each step, and saving them as PNGs and an HDF5 stack
     (shape: frames × height × width × 1, with fps, height, width attrs).
  2. Maintain a folder hierarchy under a user-chosen "Scans" root.
  3. Display status of each plant folder (scanned vs. annotated).
  4. Always-available "Open SLEAP" button to launch SLEAP; users then manually
     import the HDF5 via File ▸ Import Video ▸ From HDF5.

Dependencies (install into your Python environment):
  - nidaqmx
  - pypylon
  - Pillow
  - h5py
  - tkinter (usually included with Python)

Requires a conda environment "sleap" with SLEAP installed.
"""

import os
import threading
import subprocess
import traceback
import tkinter as tk
from tkinter import ttk, filedialog, messagebox

import h5py
import numpy as np
from PIL import Image
import nidaqmx
from nidaqmx.constants import AcquisitionType, Level
from pypylon import pylon

# --------------------------- HARDWARE / GLOBAL CONFIG ---------------------------

DEVICE       = "cDAQ1"                     # As shown in NI-MAX
COUNTER      = f"{DEVICE}Mod1/ctr1"        # CTR1 → pin 25 → STEP+
DIR_LINE     = f"{DEVICE}Mod1/port0/line1" # DIO1  → pin 4  → DIR+
MICROSTEPS   = 1600                        # DIP SW5–8 = OFF,OFF,OFF,ON
GEAR_RATIO   = 16                          # motor revs per stage rev
RPM_DEFAULT  = 30                         # default RPM for scanning

# -------------------------------------------------------------------------------

def rotate_degrees(degrees: float, rpm: float, clockwise: bool = True):
    stage_revs = degrees / 360.0
    motor_revs = stage_revs * GEAR_RATIO
    total_steps = int(MICROSTEPS * motor_revs + 0.5)
    freq_hz = rpm * MICROSTEPS / 60.0
    move_time = motor_revs * 60.0 / rpm
    timeout   = move_time + 2.0

    with nidaqmx.Task() as dir_task:
        dir_task.do_channels.add_do_chan(DIR_LINE)
        dir_task.write(False if clockwise else True)
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

def capture_image(camera: pylon.InstantCamera, save_path: str):
    grab = camera.GrabOne(5000)
    if grab.GrabSucceeded():
        img = Image.fromarray(grab.GetArray())
        img.save(save_path)
        grab.Release()
    else:
        grab.Release()
        raise RuntimeError(f"Camera grab failed for {save_path}")

def save_h5_stack(png_folder: str, h5_path: str):
    pngs = sorted(f for f in os.listdir(png_folder) if f.lower().endswith(".png"))
    if not pngs:
        raise RuntimeError("No PNGs found for HDF5 stack.")
    first = np.array(Image.open(os.path.join(png_folder, pngs[0])))
    H, W = first.shape
    N = len(pngs)
    stack = np.zeros((N, H, W, 1), dtype=first.dtype)
    for i, fname in enumerate(pngs):
        arr = np.array(Image.open(os.path.join(png_folder, fname)))
        if arr.shape != (H, W):
            raise RuntimeError(f"Size mismatch in {fname}")
        stack[i, :, :, 0] = arr
    with h5py.File(h5_path, "w") as f:
        dset = f.create_dataset(
            "images", data=stack,
            compression="gzip", compression_opts=1
        )
        dset.attrs["fps"]    = 10.0
        dset.attrs["height"] = H
        dset.attrs["width"]  = W

def scan_folder_status(root_dir: str):
    status = {}
    for child in sorted(os.listdir(root_dir)):
        path = os.path.join(root_dir, child)
        if os.path.isdir(path):
            files = os.listdir(path)
            has_h5 = any(f.lower().endswith(".h5") and not f.lower().endswith(".predictions.h5")
                         for f in files)
            has_slp = any(f.lower().endswith(".slp") for f in files)
            status[child] = {"h5": has_h5, "slp": has_slp}
    return status

class RotateCaptureDialog(tk.Toplevel):
    def __init__(self, master, scans_root, on_complete):
        super().__init__(master)
        self.title("Scan New Plant")
        self.scans_root = scans_root
        self.on_complete = on_complete

        # Variables
        self.degree_var = tk.StringVar(value="5")
        self.prefix_var = tk.StringVar()
        self.dir_var    = tk.StringVar(value=scans_root)

        # UI layout
        ttk.Label(self, text="Degree increment (°):").grid(row=0, column=0, sticky="e", padx=5, pady=5)
        ttk.Entry(self, textvariable=self.degree_var, width=10).grid(row=0, column=1, padx=5)
        ttk.Label(self, text="Image prefix:").grid(row=1, column=0, sticky="e", padx=5, pady=5)
        ttk.Entry(self, textvariable=self.prefix_var, width=20).grid(row=1, column=1, padx=5)
        ttk.Label(self, text="Scans root:").grid(row=2, column=0, sticky="e", padx=5, pady=5)
        ttk.Entry(self, textvariable=self.dir_var, width=30, state="readonly").grid(row=2, column=1, padx=5)
        ttk.Button(self, text="Browse...", command=self.choose_scans_root).grid(row=2, column=2, padx=5)
        self.start_button = ttk.Button(self, text="Start Scan", command=self.start_scan)
        self.start_button.grid(row=3, column=1, pady=10)
        self.progress = ttk.Progressbar(self, mode="determinate")
        self.progress.grid(row=4, column=0, columnspan=3, sticky="we", padx=5, pady=5)

    def choose_scans_root(self):
        d = filedialog.askdirectory(title="Select Scans root")
        if d: self.dir_var.set(d)

    def start_scan(self):
        try:
            x_deg = float(self.degree_var.get())
        except ValueError:
            return messagebox.showerror("Error", "Degree must be a number.")
        if 360 % x_deg != 0:
            return messagebox.showerror("Error", "Must divide 360 evenly.")
        prefix = self.prefix_var.get().strip()
        if not prefix:
            return messagebox.showerror("Error", "Prefix required.")
        root = self.dir_var.get()
        if not os.path.isdir(root):
            return messagebox.showerror("Error", "Invalid Scans root.")

        self.start_button.config(state="disabled")
        threading.Thread(
            target=self._do_scan,
            args=(root, prefix, x_deg),
            daemon=True
        ).start()

    def _do_scan(self, root, prefix, x_deg):
        try:
            plant = os.path.join(root, prefix)
            raw   = os.path.join(plant, "raw")
            os.makedirs(raw, exist_ok=True)

            tl = pylon.TlFactory.GetInstance()
            devs = tl.EnumerateDevices()
            if not devs:
                raise RuntimeError("No Basler camera found.")
            cam = pylon.InstantCamera(tl.CreateDevice(devs[0]))
            cam.Open()

            steps = int(360 / x_deg)
            for i in range(steps):
                self.progress["maximum"] = steps
                self.progress["value"]   = i
                fname = f"{prefix}_{i+1:03d}.png"
                capture_image(cam, os.path.join(raw, fname))
                if i < steps - 1:
                    rotate_degrees(x_deg, RPM_DEFAULT, clockwise=True)
            cam.Close()

            h5_path = os.path.join(plant, f"{prefix}.h5")
            save_h5_stack(raw, h5_path)

            self.on_complete(prefix)
            messagebox.showinfo("Done", f"{prefix}: {steps} images + HDF5 saved.")
        except Exception as e:
            traceback.print_exc()
            messagebox.showerror("Scan Error", str(e))
        finally:
            self.start_button.config(state="normal")
            self.destroy()

class PlantManagerApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("RADICYL Scan Manager")
        self.geometry("800x500")

        self.scans_root = ""
        self.folder_status = {}

        tb = ttk.Frame(self); tb.pack(fill="x", pady=5, padx=5)
        ttk.Button(tb, text="Select Scans Root", command=self.ask_scans_root).pack(side="left", padx=2)
        self.scan_btn = ttk.Button(tb, text="Scan New Plant", command=self.open_scan, state="disabled")
        self.scan_btn.pack(side="left", padx=2)
        ttk.Button(tb, text="Refresh List", command=self.populate_tree).pack(side="left", padx=2)
        # always-enabled Open SLEAP button
        ttk.Button(tb, text="Open SLEAP", command=self.open_sleap).pack(side="right", padx=2)

        paned = ttk.PanedWindow(self, orient="horizontal"); paned.pack(fill="both", expand=True, pady=5, padx=5)
        tree_frame = ttk.Frame(paned)
        self.tree = ttk.Treeview(tree_frame, show="tree")
        vsb = ttk.Scrollbar(tree_frame, orient="vertical", command=self.tree.yview)
        self.tree.configure(yscrollcommand=vsb.set)
        self.tree.pack(side="left", fill="both", expand=True); vsb.pack(side="right", fill="y")
        paned.add(tree_frame, weight=3)

        self.status_var = tk.StringVar()
        ttk.Label(self, textvariable=self.status_var, anchor="w").pack(fill="x", padx=5)

        self.tree.bind("<<TreeviewSelect>>", lambda e: self.on_tree_select())
        self.ask_scans_root()

    def ask_scans_root(self):
        d = filedialog.askdirectory(title="Select Scans root")
        if not d: return self.destroy()
        self.scans_root = d
        self.scan_btn.config(state="normal")
        self.populate_tree()

    def populate_tree(self):
        self.tree.delete(*self.tree.get_children())
        self.folder_status = scan_folder_status(self.scans_root)
        for fld, st in self.folder_status.items():
            flag = "✅" if st["slp"] else ("🟡" if st["h5"] else "❌")
            self.tree.insert("", "end", iid=fld, text=f"{flag}  {fld}")
        self.status_var.set(f"Scans root: {self.scans_root}")

    def get_selected(self):
        sel = self.tree.selection()
        return sel[0] if sel else None

    def on_tree_select(self):
        fld = self.get_selected()
        if fld:
            self.status_var.set(f"Selected: {fld}")

    def open_scan(self):
        RotateCaptureDialog(self, self.scans_root, on_complete=lambda f: (
            self.populate_tree(), self.status_var.set(f"Scanned {f} successfully.")))

    def open_sleap(self):
        try:
            subprocess.Popen(["conda", "run", "-n", "sleap", "sleap-label"])
            self.status_var.set("SLEAP launched — Import HDF5 via File ▸ Import Video ▸ From HDF5.")
        except Exception as e:
            messagebox.showerror("Error", str(e))

if __name__ == "__main__":
    PlantManagerApp().mainloop()
