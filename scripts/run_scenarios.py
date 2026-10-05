import csv
import subprocess
import matplotlib.pyplot as plt

scenarios = [
    ("Low Kd (underdamped)", [15, 0, 0, 2.0, 0.1, 0.1, 2.0, 0.1, 0.1, 2.0, 0.1, 0.1]),
    ("Baseline Kd", [15, 0, 0, 2.0, 0.1, 0.5, 2.0, 0.1, 0.5, 2.0, 0.1, 0.5]),
    ("High Kd (overdamped)", [15, 0, 0, 2.0, 0.1, 1.5, 2.0, 0.1, 1.5, 2.0, 0.1, 1.5]),
]

plt.figure(figsize=(10, 6))

for name, params in scenarios:
    output_path = f"data/scenario_{name.split()[0].lower()}.csv"
    args = ["drone_sim.exe"] + [str(p) for p in params] + [output_path]
    subprocess.run(args, check=True)

    time = []
    roll = []
    with open(output_path, "r") as f:
        reader = csv.DictReader(f)
        for row in reader:
            time.append(float(row["time"]))
            roll.append(float(row["roll"]))

    plt.plot(time, roll, label=name)

plt.xlabel("Time (s)")
plt.ylabel("Roll (deg)")
plt.title("Effect of Derivative Gain on Roll Settling Behavior")
plt.legend()
plt.grid(True)
plt.savefig("data/scenario_comparison.png")
plt.show()