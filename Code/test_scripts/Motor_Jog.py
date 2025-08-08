"""
spin_cw_ccw.py
Rotates the stage exactly 360° clockwise, then 360° counter-clockwise
using CTR1 → pin 25 (STEP+) and DIO1 → pin 4 (DIR+).
"""

import nidaqmx
from nidaqmx.constants import AcquisitionType, Level

# ————— USER CONFIG —————
DEVICE       = "cDAQ1"                     # match NI-MAX
COUNTER      = f"{DEVICE}Mod1/ctr1"        # CTR1 drives STEP+
DIR_LINE     = f"{DEVICE}Mod1/port0/line1" # DIO1 → DIR+

MICROSTEPS_PER_MOTOR_REV = 1600            # SW5–8 = OFF,OFF,OFF,ON
GEAR_RATIO               = 16             # motor revs per stage rev
RPM                      = 300            # step rate ≈ 8 kHz
REV_STAGE                = 1              # one full stage revolution
# ——————————————————————

# Derived values
STEPS_PER_STAGE_REV = int(MICROSTEPS_PER_MOTOR_REV * GEAR_RATIO)
PULSE_FREQ_HZ       = RPM * MICROSTEPS_PER_MOTOR_REV / 60
TOTAL_STEPS         = STEPS_PER_STAGE_REV * REV_STAGE
MOVE_TIME_S         = REV_STAGE * 60 / RPM
TIMEOUT_S           = MOVE_TIME_S + 5     # safety margin

def rotate_stage(clockwise: bool):
    """Rotate stage one full turn CW (True) or CCW (False)."""
    # 1) set direction
    with nidaqmx.Task() as dir_task:
        dir_task.do_channels.add_do_chan(DIR_LINE)
        dir_task.write(False if clockwise else True)
    # 2) generate step pulses
    with nidaqmx.Task() as step_task:
        step_task.co_channels.add_co_pulse_chan_freq(
            counter=COUNTER,
            freq=PULSE_FREQ_HZ,
            duty_cycle=0.5,
            idle_state=Level.LOW
        )
        step_task.timing.cfg_implicit_timing(
            sample_mode=AcquisitionType.FINITE,
            samps_per_chan=TOTAL_STEPS
        )
        print(f"{'CW' if clockwise else 'CCW'} 360° @ {RPM} RPM …")
        step_task.start()
        step_task.wait_until_done(timeout=TIMEOUT_S)
        print("Done.")

if __name__ == "__main__":
    # Clockwise then counter-clockwise
    rotate_stage(clockwise=True)
    rotate_stage(clockwise=False)
    print("\nCompleted: 360° CW → 360° CCW")
