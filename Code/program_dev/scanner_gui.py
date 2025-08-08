"""
scanner_gui.py

A GUI for high-throughput plant imaging and SLEAP annotation,
with barcode-driven workflow and species management.

Features:
  1. Define species and scan plants via barcode.
  2. Scan a plant by rotating in degree increments, capturing images, saving PNGs + HDF5.
  3. Two-level tree: species → barcoded plants, with status icons.
  4. Shared per-species .slp files (primary.slp, lateral.slp).
  5. Always-available Open SLEAP button for manual import.

Dependencies:
  - nidaqmx
  - pypylon
  - Pillow
  - h5py
  - tkinter
Requires a conda environment "sleap" with SLEAP installed.
"""

import os
import threading
import subprocess
import traceback
import tkinter as tk
from tkinter import ttk, filedialog, messagebox, simpledialog

import h5py
import numpy as np
from PIL import Image
import nidaqmx
from nidaqmx.constants import AcquisitionType, Level
from pypylon import pylon

# --------------------------- HARDWARE / GLOBAL CONFIG ---------------------------
DEVICE       = "cDAQ1"
COUNTER      = f"{DEVICE}Mod1/ctr1"
DIR_LINE     = f"{DEVICE}Mod1/port0/line1"
MICROSTEPS   = 1600
GEAR_RATIO   = 16
RPM_DEFAULT  = 30
# -------------------------------------------------------------------------------

def rotate_degrees(degrees: float, rpm: float, clockwise: bool = True):
    stage_revs = degrees / 360.0
    motor_revs = stage_revs * GEAR_RATIO
    steps = int(MICROSTEPS * motor_revs + 0.5)
    freq = rpm * MICROSTEPS / 60.0
    timeout = motor_revs * 60.0 / rpm + 2.0

    with nidaqmx.Task() as t:
        t.do_channels.add_do_chan(DIR_LINE)
        t.write(False if clockwise else True)
    with nidaqmx.Task() as t:
        t.co_channels.add_co_pulse_chan_freq(counter=COUNTER,
                                             freq=freq,
                                             duty_cycle=0.5,
                                             idle_state=Level.LOW)
        t.timing.cfg_implicit_timing(sample_mode=AcquisitionType.FINITE,
                                     samps_per_chan=steps)
        t.start()
        t.wait_until_done(timeout=timeout)

def capture_image(cam: pylon.InstantCamera, path: str):
    res = cam.GrabOne(5000)
    if res.GrabSucceeded():
        Image.fromarray(res.GetArray()).save(path)
        res.Release()
    else:
        res.Release()
        raise RuntimeError(f"Grab failed for {path}")

def save_h5_stack(png_folder: str, h5_path: str):
    pngs = sorted(f for f in os.listdir(png_folder) if f.lower().endswith(".png"))
    if not pngs:
        raise RuntimeError("No PNGs to stack.")
    arr0 = np.array(Image.open(os.path.join(png_folder, pngs[0])))
    H, W = arr0.shape
    N = len(pngs)
    stack = np.zeros((N, H, W, 1), dtype=arr0.dtype)
    for i, f in enumerate(pngs):
        arr = np.array(Image.open(os.path.join(png_folder, f)))
        if arr.shape != (H, W):
            raise RuntimeError(f"Size mismatch in {f}")
        stack[i, :, :, 0] = arr
    with h5py.File(h5_path, "w") as hf:
        d = hf.create_dataset("images", data=stack,
                              compression="gzip", compression_opts=1)
        d.attrs["fps"]    = 10.0
        d.attrs["height"] = H
        d.attrs["width"]  = W

def get_species_list(root: str):
    return [d for d in sorted(os.listdir(root))
            if os.path.isdir(os.path.join(root, d))]

def scan_folder_status(root: str):
    status = {}
    for species in get_species_list(root):
        spath = os.path.join(root, species)
        slps = [f.lower() for f in os.listdir(spath)]
        primary = "primary.slp" in slps
        lateral = "lateral.slp" in slps
        plants = {}
        for plant in sorted(os.listdir(spath)):
            ppath = os.path.join(spath, plant)
            if os.path.isdir(ppath):
                raw = os.path.join(ppath, "raw")
                has_raw = os.path.isdir(raw) and bool(os.listdir(raw))
                has_h5  = os.path.exists(os.path.join(ppath, plant + ".h5"))
                plants[plant] = {"raw": has_raw, "h5": has_h5}
        status[species] = {
            "primary": primary,
            "lateral": lateral,
            "plants": plants
        }
    return status

class RotateCaptureDialog(tk.Toplevel):
    def __init__(self, master, scans_root, on_done, default_barcode=None):
        super().__init__(master)
        self.scans_root = scans_root
        self.on_done    = on_done
        self.title("Scan New Plant")

        # Variables
        self.degree = tk.StringVar(self, value="5")
        self.barcode = tk.StringVar(self, value=default_barcode or "")
        self.species = tk.StringVar(self)

        # UI
        ttk.Label(self, text="Barcode:").grid(row=0, column=0, sticky="e", padx=5, pady=5)
        ttk.Entry(self, textvariable=self.barcode, width=20).grid(row=0, column=1, padx=5)
        ttk.Label(self, text="Species:").grid(row=1, column=0, sticky="e", padx=5, pady=5)
        cb = ttk.Combobox(self, textvariable=self.species,
                          values=["New Species"] + get_species_list(self.scans_root))
        cb.grid(row=1, column=1, padx=5)
        cb.set("")  # Blank by default
        ttk.Label(self, text="Degree (°):").grid(row=2, column=0, sticky="e", padx=5, pady=5)
        ttk.Entry(self, textvariable=self.degree, width=10).grid(row=2, column=1, padx=5)
        self.start_btn = ttk.Button(self, text="Start Scan", command=self.start_scan)
        self.start_btn.grid(row=3, column=1, pady=10)
        self.prog = ttk.Progressbar(self, mode="determinate")
        self.prog.grid(row=4, column=0, columnspan=2, sticky="we", padx=5, pady=5)

    def start_scan(self):
        code = self.barcode.get().strip()
        sp   = self.species.get().strip()
        try:
            d = float(self.degree.get())
        except ValueError:
            return messagebox.showerror("Error", "Invalid degree")
        if not code:
            return messagebox.showerror("Error", "Barcode required")
        if not sp:
            return messagebox.showerror("Error", "Species required")
        if sp == "New Species":
            sp = simpledialog.askstring("New Species", "Enter species name:")
            if not sp:
                return
            spath = os.path.join(self.scans_root, sp)
            os.makedirs(spath, exist_ok=True)
        if 360 % d != 0:
            return messagebox.showerror("Error", "Degree must divide 360")

        self.start_btn.config(state="disabled")
        threading.Thread(target=self._scan_thread, args=(sp, code, d), daemon=True).start()

    def _scan_thread(self, species, barcode, x_deg):
        try:
            spath = os.path.join(self.scans_root, species)
            plant = os.path.join(spath, barcode)
            raw   = os.path.join(plant, "raw")
            os.makedirs(raw, exist_ok=True)

            tl = pylon.TlFactory.GetInstance()
            devs = tl.EnumerateDevices()
            if not devs:
                raise RuntimeError("No camera found")
            cam = pylon.InstantCamera(tl.CreateDevice(devs[0]))
            cam.Open()

            steps = int(360 / x_deg)
            for i in range(steps):
                self.prog["maximum"] = steps
                self.prog["value"]   = i
                self.update_idletasks()
                fname = f"{barcode}_{i+1:03d}.png"
                capture_image(cam, os.path.join(raw, fname))
                if i < steps - 1:
                    rotate_degrees(x_deg, RPM_DEFAULT)
            cam.Close()

            h5_path = os.path.join(plant, f"{barcode}.h5")
            save_h5_stack(raw, h5_path)

            self.on_done(species, barcode)
            messagebox.showinfo("Scan Complete", f"Scanned {barcode} under {species}")
        except Exception as e:
            traceback.print_exc()
            messagebox.showerror("Scan Error", str(e))
        finally:
            self.start_btn.config(state="normal")
            self.destroy()

class PlantManagerApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("RADICYL Scan Manager")
        self.geometry("800x500")

        self.scans_root = ""
        self.status     = {}

        tb = ttk.Frame(self); tb.pack(fill="x", pady=5, padx=5)
        ttk.Button(tb, text="Select Scans Root", command=self.ask_root).pack(side="left")
        ttk.Button(tb, text="New Species", command=self.new_species).pack(side="left", padx=2)
        self.scan_btn = ttk.Button(tb, text="Scan New Plant",
                                   command=self.manual_scan, state="disabled")
        self.scan_btn.pack(side="left", padx=2)
        ttk.Button(tb, text="Refresh List", command=self.populate_tree).pack(side="left", padx=2)
        ttk.Button(tb, text="Open SLEAP", command=self.open_sleap).pack(side="right")

        # Barcode entry (hidden, always in focus)
        self.barcode_var = tk.StringVar()
        self.barcode_entry = ttk.Entry(tb, textvariable=self.barcode_var, width=1)
        self.barcode_entry.pack(side="right")
        self.barcode_entry.bind("<Return>", lambda e: self.on_barcode_scan())

        self.tree = ttk.Treeview(self)
        self.tree.pack(fill="both", expand=True, padx=5, pady=5)

        self.status_var = tk.StringVar()
        ttk.Label(self, textvariable=self.status_var, anchor="w").pack(fill="x", padx=5)

        self.ask_root()

    def ask_root(self):
        d = filedialog.askdirectory(title="Select Scans root")
        if not d:
            self.destroy()
            return
        self.scans_root = d
        self.scan_btn.config(state="normal")
        self.populate_tree()
        self.barcode_entry.focus_set()

    def new_species(self):
        name = simpledialog.askstring("New Species", "Species name:")
        if not name:
            return
        path = os.path.join(self.scans_root, name)
        if os.path.exists(path):
            return messagebox.showerror("Error", "Species already exists")
        os.makedirs(path)
        self.populate_tree()
        self.barcode_entry.focus_set()

    def populate_tree(self):
        self.tree.delete(*self.tree.get_children())
        self.status = scan_folder_status(self.scans_root)
        for sp, st in self.status.items():
            slp_count = int(st["primary"]) + int(st["lateral"])
            sp_flag = "🔴" if slp_count == 0 else ("🟠" if slp_count == 1 else "🟢")
            sp_id = self.tree.insert("", "end", text=f"{sp_flag}  {sp}")
            for plant, pst in st["plants"].items():
                p_flag = "❌" if not pst["raw"] else ("🟡" if not pst["h5"] else "🟢")
                self.tree.insert(sp_id, "end", text=f"{p_flag}  {plant}")
        self.status_var.set(f"Scans root: {self.scans_root}")
        self.barcode_entry.focus_set()

    def manual_scan(self):
        RotateCaptureDialog(self, self.scans_root, self.on_scan_done)
        self.barcode_entry.focus_set()

    def on_barcode_scan(self):
        code = self.barcode_var.get().strip()
        self.barcode_var.set("")
        if not code:
            return
        # Search for existing plant across species
        found_sp = None
        for sp, st in self.status.items():
            if code in st["plants"]:
                found_sp = sp
                break
        if found_sp:
            # Select and open folder
            sp_id = self.tree.get_children()[list(self.status).index(found_sp)]
            plant_id = [c for c in self.tree.get_children(sp_id) if self.tree.item(c, "text").split()[-1] == code][0]
            self.tree.selection_set(plant_id)
            self.tree.see(plant_id)
            os.startfile(os.path.join(self.scans_root, found_sp, code))
            self.status_var.set(f"Selected & opened existing {code} under {found_sp}")
        else:
            # Auto-open dialog for new
            RotateCaptureDialog(self, self.scans_root, self.on_scan_done, default_barcode=code)
        self.barcode_entry.focus_set()

    def on_scan_done(self, species, barcode):
        self.populate_tree()
        self.status_var.set(f"Scanned new plant {barcode} under {species}")
        self.barcode_entry.focus_set()

    def open_sleap(self):
        try:
            subprocess.Popen(["conda", "run", "-n", "sleap", "sleap-label"])
            self.status_var.set("SLEAP launched — import HDF5 via File ▸ Import Video ▸ From HDF5.")
        except Exception as e:
            messagebox.showerror("Error", str(e))
        self.barcode_entry.focus_set()

if __name__ == "__main__":
    PlantManagerApp().mainloop()