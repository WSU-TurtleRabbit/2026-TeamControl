import os
import time
import random
import math
from datetime import datetime
import matplotlib.pyplot as plt
from TeamControl.harness.harness import Harness
from TeamControl.harness.constants import (
    ROBOT_ID, IS_YELLOW, DELAY,
    STEPS, BRAKE_STEPS, FREQ,
    NUM_RUNS, SETTLE_MM,
    VX_MAX, VY_MAX, VW_MAX,
)
"""
Program description:
1. Drive phase: random velocity for ~2s, logged to CSV
2. Brake phase: zero velocity for ~0.5s, position sampled in memory → compute overshoot + settling time
3. Repeat for next run
4. After the whole loop, Plot overshooting + settling time side-by-side

"""
PLOTS_DIR = os.path.join(os.path.dirname(__file__), "plots")
os.makedirs(PLOTS_DIR, exist_ok=True)

h = Harness(robot_id=ROBOT_ID, is_yellow=IS_YELLOW)
path = h.start("overshoot_test")
print(f"Logging to: {path}")
time.sleep(DELAY)

overshoots = []
settle_times = []

for run in range(NUM_RUNS):
    vx = random.uniform(-VX_MAX, VX_MAX)
    vy = random.uniform(-VY_MAX, VY_MAX)
    w  = random.uniform(-VW_MAX, VW_MAX)
    print(f"\nRun {run + 1}/{NUM_RUNS}: vx={vx:.2f}  vy={vy:.2f}  w={w:.2f}")

    # Drive phase — logged to CSV
    h.set_logging(True)
    for _ in range(STEPS):
        h.send(vx=vx, vy=vy, w=w)
        time.sleep(1 / FREQ)
    h.set_logging(False)

    brake_pos = h.read_position()
    t_brake = time.monotonic()

    # Brake phase — collect position samples for metric computation
    brake_samples = []
    for _ in range(BRAKE_STEPS):
        h.send(vx=0.0, vy=0.0, w=0.0)
        time.sleep(1 / FREQ)
        pos = h.read_position()
        if pos is not None:
            t_ms = (time.monotonic() - t_brake) * 1000
            brake_samples.append((t_ms, pos[0], pos[1]))

    if brake_pos is None or not brake_samples:
        print("  No vision data — skipping.")
        continue

    bx, by, _ = brake_pos
    dists = [math.hypot(px - bx, py - by) for _, px, py in brake_samples]

    overshoot = max(dists)

    settle_t = None
    for (t_ms, _, _), d in zip(brake_samples, dists):
        if d < SETTLE_MM:
            settle_t = t_ms
            break
    if settle_t is None:
        settle_t = brake_samples[-1][0]  # never settled within brake phase

    overshoots.append(overshoot)
    settle_times.append(settle_t)
    print(f"  Overshoot: {overshoot:.1f} mm  |  Settle time: {settle_t:.1f} ms")

h.stop()

# Scatter plots
runs = list(range(1, len(overshoots) + 1))
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 4))

ax1.plot(runs, overshoots, color="steelblue", marker="o")
ax1.set_xlabel("Run")
ax1.set_ylabel("Overshoot (mm)")
ax1.set_title("Overshoot Distance per Run")
ax1.set_xticks(runs)

ax2.plot(runs, settle_times, color="darkorange", marker="o")
ax2.set_xlabel("Run")
ax2.set_ylabel("Settling Time (ms)")
ax2.set_title("Settling Time per Run")
ax2.set_xticks(runs)

plt.tight_layout()
max_overshoot = max(overshoots) if overshoots else 0.0
ts = datetime.now().strftime("%Y%m%d_%H%M%S")
plot_path = os.path.join(PLOTS_DIR, f"{ts}_max{max_overshoot:.1f}mm.png")
plt.savefig(plot_path)
plt.show()
print(f"\nPlot saved to {plot_path}")
