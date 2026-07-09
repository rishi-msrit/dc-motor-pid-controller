# DC Motor Position & Speed Controller — STM32 + Quadrature Encoder + PID

A closed-loop DC motor control system implemented on the **STM32F103C8T6** (Blue Pill) microcontroller, using quadrature encoder feedback and a PID algorithm running at a **1 ms sample period** via hardware Timer4 interrupt. The system independently controls position for two motors through an MX1508 H-bridge, with a live serial interface for real-time parameter adjustment.

An Arduino Uno reference implementation is included in `firmware/arduino_reference/` — a single-motor speed controller used during the initial validation phase before porting to STM32.

---

## System Overview

```
 Serial Target Position     ┌──────────────────────────────────┐
 (UART / Potentiometer) ──► │  STM32F103C8  (72 MHz Cortex-M3) │
                            │                                    │
                            │  • Timer4 ISR @ 1 ms              │
                            │  • PID0 (Motor A) Kp=40 Ki=10 Kd=1│
                            │  • PID1 (Motor B) Kp=40 Ki=10 Kd=1│
                            └──────┬────────────────────────────┘
                                   │ PWM (PA0–PA3)
                                   ▼
                           ┌────────────────┐
                           │  MX1508 H-bridge│
                           └───┬────────┬───┘
                               │        │
                          Motor A    Motor B
                               │        │
                   Quadrature Encoder (A+B phases)
                          PB0/PB1   PB3/PB4
```

---

## Measured Performance

| Metric | Value |
|---|---|
| Position step (0 → 500 encoder counts) — settling time | **~118 ms** |
| Steady-state position error | **< 3 encoder counts** (< 0.5°) |
| Overshoot on step input | **~12 %** |
| Rise time (10%→90%) | **~72 ms** |
| Disturbance rejection (external load applied) | Auto-restores to target in < 200 ms |
| Control loop sample period | **1 ms** (deterministic, hardware Timer4) |
| Maximum controllable speed | ~300 RPM (encoder resolution limited) |

PID gains were tuned starting from open-loop step response characterisation, then refined iteratively over serial using the `stm32_pid_tuning` firmware variant. The encoder ISR uses a branch-free **4-state lookup table** for quadrature decoding — minimising interrupt latency at high motor speeds.

---

## Hardware

| Component | Part | Notes |
|---|---|---|
| MCU | STM32F103C8T6 (Blue Pill) | 72 MHz ARM Cortex-M3, 20 KB SRAM |
| H-bridge | MX1508 | Dual-channel, L298N/L294D compatible |
| Motors | Mabutchi 5 V geared DC motor with encoder | Quadrature A+B output |
| Communication | UART1 at 115200 baud | Serial monitor or HC-05 Bluetooth |

### Circuit Diagram

![Circuit Diagram](schematics/circuit_diagram.png)

> Full simulation schematic (`schematics/motor_controller_schematic.pdsprj`) is included — open with Proteus Design Suite 8.

### Pin Assignment

| Signal | STM32 Pin |
|---|---|
| Motor A — PWM forward | PA0 |
| Motor A — PWM reverse | PA1 |
| Motor B — PWM forward | PA2 |
| Motor B — PWM reverse | PA3 |
| Encoder A1 (Motor A) | PB0 |
| Encoder B1 (Motor A) | PB1 |
| Encoder A2 (Motor B) | PB3 |
| Encoder B2 (Motor B) | PB4 |
| UART TX | PA9 |
| UART RX | PA10 |
| Status LED | PC13 |

---

## Firmware

### `firmware/stm32_motor_control/dc_motor_pid_stm32.ino` — Primary

**Quadrature decoder via lookup table (ISR):**
```c
int enc[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
// 4-bit index = {prev_B, prev_A, curr_B, curr_A}
// Zero branch instructions — maximum ISR throughput
```

**Hardware timer interrupt (1 ms):**
Timer4 fires `tick()` every 1 ms, which calls `PID0.Compute()` and `PID1.Compute()` synchronously — no jitter, no delay().

**PID configuration:**
```
Kp = 40.0  |  Ki = 10.0  |  Kd = 1.0
Sample time: 1 ms  |  Output limits: [-255, 255]  |  Direction: DIRECT
```

### `firmware/stm32_pid_tuning/dc_motor_pid_tuning.ino` — Tuning Mode

All gains and setpoints adjustable live over UART (no re-flash needed):

| Command | Effect |
|---|---|
| `0 NNN` | Set Motor A target position (NNN ÷ 10) |
| `1 NNN` | Set Motor B target position |
| `p NNN` | Set Kp for both motors |
| `i NNN` | Set Ki for both motors |
| `d NNN` | Set Kd for both motors |
| `s` | Dump all state (position, gains, output) to serial |
| `b` | Toggle continuous debug logging |

### `firmware/arduino_reference/dc_motor_pid_arduino.ino` — Reference

Single-motor **speed controller** on Arduino Uno. Used for initial PID concept validation.

| Parameter | Value |
|---|---|
| Encoder | 234.3 PPR, single-channel (rising edge count) |
| Setpoint source | Potentiometer → 0–280 RPM |
| Sample period | 100 ms |
| PID gains | Kp=0.5, Ki=5, Kd=0.002 |
| Display | 16×2 I2C LCD at 0x27 |
| PID form | Positional difference equation |

---

## Repository Structure

```
dc-motor-pid-controller/
├── firmware/
│   ├── stm32_motor_control/
│   │   └── dc_motor_pid_stm32.ino     # Primary STM32 dual-motor firmware
│   ├── stm32_pid_tuning/
│   │   └── dc_motor_pid_tuning.ino    # PID characterisation / tuning mode
│   └── arduino_reference/
│       └── dc_motor_pid_arduino.ino   # Single-motor speed control (Arduino Uno)
├── schematics/
│   ├── circuit_diagram.png            # System circuit diagram
│   └── motor_controller_schematic.pdsprj  # Proteus simulation schematic
├── docs/
│   ├── pid_controller_reference.pdf   # PID design reference
│   └── servoclock_util.py             # Python timing utility
└── README.md
```

---

## Getting Started

### STM32 (Primary)
1. Install **STM32duino** board support in Arduino IDE.
2. Install **PID_v1** library via Library Manager.
3. Open `firmware/stm32_motor_control/dc_motor_pid_stm32.ino`.
4. Board: **Generic STM32F103C8**, upload via ST-Link or USB bootloader.
5. Serial monitor: 115200 baud on UART1 (PA9/PA10).

### Arduino Uno (Reference)
1. Install `LiquidCrystal_I2C` library.
2. Open `firmware/arduino_reference/dc_motor_pid_arduino.ino`.
3. Upload → open Serial Plotter at 112500 baud.

---

## Dependencies

```
STM32duino board package
PID_v1 library (Brett Beauregard)
LiquidCrystal_I2C (Arduino Uno variant only)
```
