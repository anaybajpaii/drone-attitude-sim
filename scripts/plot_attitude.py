import csv
import matplotlib.pyplot as plt

time = []
roll = []
pitch = []
yaw = []

with open("data/drift.csv", "r") as f:
    reader = csv.DictReader(f)
    for row in reader:
        time.append(float(row["time"]))
        roll.append(float(row["roll"]))
        pitch.append(float(row["pitch"]))
        yaw.append(float(row["yaw"]))

plt.figure(figsize=(10, 6))
plt.plot(time, roll, label="Roll")
plt.plot(time, pitch, label="Pitch")
plt.plot(time, yaw, label="Yaw")
plt.xlabel("Time (s)")
plt.ylabel("Angle (deg)")
plt.title("Uncontrolled Drone Attitude Drift")
plt.legend()
plt.grid(True)
plt.savefig("data/drift_plot.png")
plt.show()