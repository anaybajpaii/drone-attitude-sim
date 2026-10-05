# Drone Attitude Simulator

This is a C++ simulator that models 3-axis (roll, pitch, yaw) drone attitude control using an independent PID controller per axis, converting axis corrections into motor thrust values via a standard quadcopter motor mixer.

## Overview

The simulator models a drone's orientation drifting away from level flight under a constant disturbance on each axis. It includes two modes:

* Uncontrolled model — disturbance only, demonstrating the drone drifting off-level on its own.
* Controlled model — a PID controller per axis reads the current tilt each timestep and applies a corrective torque to hold the drone level.

Axis corrections are combined with a base throttle and converted into four individual motor thrust values. Simulation data can be exported to CSV and plotted using Python.

## Physics

Each axis's state is described by its angle (rad, tilt from level) and angular velocity (rad/s). A constant disturbance produces an angular acceleration on each axis, causing it to drift over time if left uncorrected.

The controller adds a corrective torque using the full PID law:

$correction = K_p·error + K_i·∑(error·dt) + K_d·(error − previousError)/dt$

where error is the difference between the setpoint (0, level) and the current angle. The total angular acceleration each timestep is:

$angularAcceleration = disturbance + correction$

A semi-implicit Euler integration method then updates angular velocity and angle at each fixed timestep using this acceleration, producing an approximate trajectory of the drone's orientation over time.

This project is a direct follow-up to [stabilization-sim](https://github.com/anaybajpaii/stabilization-sim), which found that a proportional-only controller can prevent a system from falling but can't settle it to rest. The derivative term added here is what fixes that.

## Features

* Uncontrolled 3-axis drift physics (constant disturbance)
* Full PID controller per axis with configurable gains
* Motor mixer converting axis corrections into 4 motor thrust values
* Runtime CLI input for initial tilt and per-axis PID gains
* CSV trajectory data output
* Python plotting script for visualizing single and multi-gain comparisons
* Unit tests for controller correctness

## Project Structure

```
├── src/
│   ├── main.cpp
│   ├── DroneState.h
│   ├── DroneState.cpp
│   ├── PIDController.h
│   ├── PIDController.cpp
│   ├── MotorMixer.h
│   ├── MotorMixer.cpp
│   └── DataLogger.cpp
├── data/
├── scripts/
│   ├── plot_attitude.py
│   └── run_scenarios.py
├── docs/
│   ├── control_notes.md
│   └── screenshots/
├── tests/
│   └── test_controller.cpp
├── Makefile
├── README.md
├── LICENSE
└── .gitignore
```

## Tools

* C++ — simulation and control calculations
* Python — data visualization
* Matplotlib — trajectory plots
* Git/GitHub — version control

## Build

```
g++ src/main.cpp src/DroneState.cpp src/DataLogger.cpp src/PIDController.cpp src/MotorMixer.cpp -o drone_sim
./drone_sim
```

Run with custom initial tilt and gains:

```
./drone_sim <rollDeg> <pitchDeg> <yawDeg> <rollKp> <rollKi> <rollKd> <pitchKp> <pitchKi> <pitchKd> <yawKp> <yawKi> <yawKd>
```

## Plotting

```
pip install matplotlib
python scripts/plot_attitude.py
```

Compare multiple PID gain scenarios:

```
python scripts/run_scenarios.py
```

## Testing

```
g++ tests/test_controller.cpp src/PIDController.cpp -o test_controller
./test_controller
```

## Documentation

See `docs/control_notes.md` for the mathematical model, gain comparison findings, the limitations of the current approach, and its connection to real drone firmware. See `docs/screenshots/` for plotted output.

## License

MIT
