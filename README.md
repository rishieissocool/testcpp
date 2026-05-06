# Stripped C++ Latency Server

This is a minimal server-side sender + visualizer for hardware latency testing.

## What it does

- Sends **only** robot velocity commands over UDP to the hardware format used by `RobotFramework`.
- Drives a robot in a clean circle (`vx`, `vy`, `w=0`) at a fixed high send rate.
- Shows a lightweight Win32 GUI for visual comparison.
  - Green marker: commanded circular position.
  - Orange marker: optional vision/observed position from UDP.

## Protocol compatibility

This sends exactly the same command shape expected by the robot receiver:

`id vx vy w kick dribble timestamp`

UDP destination defaults:

- command send to robot: `robot_ip:50514`
- telemetry receive from robot: `0.0.0.0:50513`
- optional vision pose receive: `0.0.0.0:50515`

## Build and run

Open `testcpp.slnx` in Visual Studio, then run `testcpp` (x64 recommended).

CLI arguments:

`testcpp.exe [robot_ip] [robot_id] [send_hz] [radius_m] [omega_rad_s]`

Example:

`testcpp.exe 172.20.10.2 1 200 0.6 1.0`

## Linux build

Linux uses a stripped terminal visual/status app (`testcpp/main_linux.cpp`) and the provided build script.

```bash
cd testcpp
chmod +x build_linux.sh
./build_linux.sh
./build/testcpp_latency 172.20.10.2 1 200 0.6 1.0
```

Press `Ctrl+C` to stop.

## Optional vision feed input

To compare a measured robot location against command, send UDP packets to port `50515`:

`x y [timestamp]`

Example payload:

`0.21 -0.37 1714283000123`

Where `x` and `y` are in meters (same world frame you want to compare against).
