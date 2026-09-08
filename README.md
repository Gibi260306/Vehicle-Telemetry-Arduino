# Arduino Vehicle Telemetry and Safety System

A bench-top vehicle telemetry and safety prototype combining Arduino firmware, simulated vehicle controls, obstacle sensing, a TFT interface and a Python telemetry dashboard.

## Toolchain

* Arduino C/C++
* Python 3
* Arduino IDE
* PySerial
* Matplotlib
* Adafruit GFX and ST7735 libraries
* SPI serial communication

## Hardware

* Arduino-compatible development board
* ST7735 TFT display
* HC-SR04 ultrasonic sensor
* Potentiometers for throttle, brake and steering inputs
* Push button for system control

## System Design

The Arduino reads the simulated vehicle controls and ultrasonic distance sensor, processes the measurements and updates a safety state machine. The current values and system state are displayed locally and transmitted to the Python dashboard over serial.

Separate update intervals are used for control sampling, distance measurement, display updates and telemetry transmission.

## Embedded Firmware

* Converts throttle and brake inputs into percentages
* Maps steering input between `-35°` and `+35°`
* Applies smoothing to control inputs
* Uses median filtering for distance measurements
* Detects invalid ultrasonic sensor readings
* Uses warning hysteresis to prevent repeated state switching
* Provides live status and measurements through the TFT display
* Uses an interrupt-driven button with software debounce

## Safety States

| State          | Condition                              |
| -------------- | -------------------------------------- |
| `OFF`          | System disabled                        |
| `RUNNING`      | Normal operation                       |
| `WARNING`      | Obstacle below 20 cm                   |
| `CRASHED`      | Obstacle below 10 cm                   |
| `SENSOR_FAULT` | No valid ultrasonic reading for 500 ms |

A warning remains active until the measured distance rises above 23 cm.

## Telemetry Dashboard

The Python dashboard supports both live serial data and replay from recorded CSV logs.

It provides:

* Live throttle, brake, steering and distance graphs
* Packet validation and malformed-packet rejection
* Lost, duplicate and out-of-order packet tracking
* Arduino timestamp rollover handling
* System-state and fault reporting
* CSV telemetry recording
* Time-correct replay of previous sessions

## Telemetry Protocol

Telemetry is transmitted at `115200` baud using a custom comma-separated packet format:

```text
VTP1,timestamp,sequence,throttle,brake,steering,distance,state,faults
```

Sequence numbers allow the dashboard to detect communication errors, while fault flags identify invalid sensor data.

## CPU Integration

Recorded telemetry from this project is also used by the companion [8-bit CPU project](https://github.com/Gibi260306/CPU-Design).

A Python converter transforms selected CSV records into memory fixtures, allowing telemetry safety logic to execute as an assembly program on the custom Verilog processor. This provides an end-to-end connection between the embedded vehicle prototype, software tooling and FPGA-based CPU design.
