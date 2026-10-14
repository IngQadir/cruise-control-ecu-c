"""Plot the CSV written by the cruise control demo.

Usage:  python3 tools/plot_demo.py cruise_demo.csv
Output: cruise_demo.png
"""
import csv
import sys

import matplotlib
matplotlib.use("Agg")  # no window needed (works in WSL)
import matplotlib.pyplot as plt

path = sys.argv[1] if len(sys.argv) > 1 else "cruise_demo.csv"

t, speed, throttle, state = [], [], [], []
with open(path, newline="") as f:
    for row in csv.DictReader(f):
        t.append(float(row["time_s"]))
        speed.append(float(row["speed_kmh"]))
        throttle.append(float(row["throttle_pct"]))
        state.append(row["state"])

fig, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex=True, figsize=(10, 8))

ax1.plot(t, speed)
ax1.set_ylabel("Speed [km/h]")
ax1.grid(True)

ax2.plot(t, throttle, color="tab:orange")
ax2.set_ylabel("Throttle request [%]")
ax2.grid(True)

levels = {"OFF": 0, "STANDBY": 1, "ACTIVE": 2, "FAULT": 3}
ax3.step(t, [levels[s] for s in state], where="post", color="tab:green")
ax3.set_yticks(list(levels.values()))
ax3.set_yticklabels(list(levels.keys()))
ax3.set_ylabel("State")
ax3.set_xlabel("Time [s]")
ax3.grid(True)

fig.suptitle("Cruise control demo")
fig.tight_layout()
fig.savefig("cruise_demo.png", dpi=150)
print("saved cruise_demo.png")
