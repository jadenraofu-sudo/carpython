# Mecanum Wheel Robot Control System

A Python-based control system for a 4-wheel mecanum robot with encoder feedback and PID speed control. Supports both hardware control on Raspberry Pi and simulation on macOS.

## Overview

This project implements:
- **PID-based motor speed control** for precise velocity regulation
- **Encoder-based feedback** for accurate odometry
- **Mecanum kinematics** for omnidirectional movement (forward, backward, lateral sliding, rotation)
- **Path planning** with line and semi-circle arc movements
- **Dual-mode operation**: Hardware control on Raspberry Pi or simulation visualization on macOS

## Project Structure

### Core Modules

- **`moves.py`** - Hardware control layer (Raspberry Pi)
  - `Encoder` class: Reads quadrature encoder signals
  - `Wheel` class: Controls individual motor with PID speed regulation
  - Motor and encoder initialization via PCA9685 PWM controller and GPIO
  - Car-level movement functions (`go_ahead_pid`, `go_back_pid`, etc.)

- **`helper.py`** - Kinematics and motion planning
  - `wheelspeeds()`: Converts body-frame velocity to individual wheel speeds
  - `wheelhelper()`: Transforms world-frame velocity to robot frame and computes wheel speeds
  - `linehelper()`: Plans linear motion between two points
  - `semihelper()`: Plans semi-circular arc motion
  - `tankturnhelper()`: Plans rotation-in-place maneuvers

- **`plot.py`** - Visualization and simulation (macOS)
  - Simulates robot kinematics and renders path
  - Displays individual wheel contact velocities and spin rates
  - Interactive matplotlib-based visualization

- **`main.py`** - Command scheduler
  - Parses instruction files for timed movement commands
  - Supports both relative and absolute time scheduling
  - Executes planned movements on schedule

- **`config.py`** - Configuration parameters
  - Environment setting (`'mac'` or `'car'`)
  - Robot dimensions (wheel radius `R`, distances `LX`, `LY`)
  - Time step configuration

## Hardware Setup (Raspberry Pi)

### Motor Control
- **PCA9685 PWM Controller** (I2C address) via `busio` and `adafruit_pca9685`
- **4 DC motors** with direction control (IN1, IN2) and speed control (EN)
- **Motor channels**:
  - Front-Left (FL): EN=4, IN1=5, IN2=6
  - Front-Right (FR): EN=0, IN1=1, IN2=2
  - Rear-Left (RL): EN=12, IN1=13, IN2=14
  - Rear-Right (RR): EN=8, IN1=9, IN2=10

### Encoders
- **Quadrature encoders** on each wheel (2 channels per motor)
- **GPIO pins**:
  - FL: GPIO 22, 10
  - FR: GPIO 17, 27
  - RL: GPIO 5, 6
  - RR: GPIO 9, 11
- **Encoder resolution**: 750 clicks per revolution (default)
- **Interrupt-driven**: `GPIO.add_event_detect()` triggers encoder reads

### Communication
- **I2C**: SCL=GPIO3, SDA=GPIO2 (for PCA9685)
- **GPIO**: BCM numbering mode

## Installation & Dependencies

### macOS (Simulation)
```bash
pip install numpy matplotlib
```

### Raspberry Pi (Hardware)
```bash
pip install numpy simple-pid adafruit-pca9685 Adafruit-Blinka RPi.GPIO busio
```

## Usage

### Configuration

Edit `config.py` to set:
```python
environment = 'mac'  # 'mac' for simulation, 'car' for hardware
R = 0.03  # wheel radius (meters)
LX = 0.1  # half-width of robot (meters)
LY = 0.1  # half-length of robot (meters)
SEC_PER_STEP = 0.02  # time step for motion planning
```

### Instruction File Format

Create an instruction file (default: `instructions.txt`):
```
relativeTime(True)
0.5 line([2.0, 1.0], v=0.5, w=0.0)
1.0 semi([3.0, 2.0], dir=1, v=0.5)
0.5 line([0.0, 0.0], v=0.5, w=0.0)
```

**Commands**:
- `relativeTime(True/False)` - Toggle relative vs. absolute time scheduling (immediate)
- `setpoints([x, y])` - Define path points (immediate)
- `startQ()` - Begin execution (immediate)
- `line(point, v, w, align)` - Linear motion
  - `point`: `[x, y]` target
  - `v`: velocity (m/s, default 1.0)
  - `w`: angular velocity (deg/s, default 0.0)
  - `align`: auto-rotate to face target direction (default True)
- `semi(point, dir, v, w, align, tangential)` - Semi-circular arc
  - `point`: `[x, y]` endpoint
  - `dir`: 1 or -1 (which side of circle)
  - `v`, `w`, `align`: same as `line()`
  - `tangential`: auto-compute angular velocity for circular motion (default True)

### Running

**macOS (simulation)**:
```bash
python main.py instructions.txt
```

**Raspberry Pi (hardware)**:
```bash
python main.py instructions.txt
```

## PID Tuning

The `Wheel` class uses default PID gains:
- **Kp** = 1.5 (proportional)
- **Ki** = 0.8 (integral)
- **Kd** = 0.05 (derivative)

Adjust at runtime:
```python
wheel.tune(kp=2.0, ki=1.0, kd=0.1)
```

Or modify `Wheel.DEFAULT_KP/KI/KD` constants.

## Motor Control API

### Open-Loop (Original)
```python
wheel.move(power)  # -100 to 100, disables PID
```

### Closed-Loop (PID)
```python
wheel.set_target_speed(target_rps)  # set desired speed in rev/s
wheel.update()  # call in main loop to tick PID (every ~20 ms)
wheel.speed_rps  # read current measured speed
```

### Car-Level
```python
go_ahead_pid(target_rps)  # forward
go_back_pid(target_rps)   # backward
stop_car()  # brake and reset encoders
update_all()  # update all 4 PID controllers
```

## Coordinate Frame

- **World frame**: X right, Y forward
- **Robot frame**: Local coordinate system aligned with robot heading (angle in radians)
- **Wheel speeds**: Computed independently for omnidirectional motion

## Testing

Run `test_encoders.py` to verify encoder functionality and calibrate encoder resolution.

## Troubleshooting

- **Encoders not reading**: Check GPIO connections and `perRev` calibration in `moves.py`
- **Motor not moving**: Verify PCA9685 I2C address and PWM frequency
- **Uneven speed**: Tune PID gains per-wheel or verify motor power matching
- **Velocity tracking error**: Increase `Kp` or `Ki`, check encoder resolution

## References

- Mecanum kinematics: Velocity decomposition into individual wheel commands
- PID control: `simple-pid` library documentation
- Raspberry Pi GPIO: `RPi.GPIO` documentation
