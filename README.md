# STM32 Quadcopter Flight Controller

A bare-metal quadcopter flight controller firmware built on an STM32 microcontroller using STM32 HAL. The system handles IMU data acquisition via DMA, sensor fusion, digital filtering, and PID-based attitude stabilization — all within a 1 kHz control loop.

---

## Features

- **1 kHz control loop** driven by a hardware timer interrupt (TIM3)
- **DMA-based SPI** for non-blocking MPU6500 sensor reads
- **Madgwick AHRS** filter for roll/pitch estimation from quaternions
- **IIR low-pass filters** (bilinear transform) on both accelerometer and gyroscope axes
- **PID controllers** with derivative filtering and dynamic anti-windup for roll and pitch
- **Gyroscope bias calibration** on startup (1000-sample average)
- **State machine** flight sequencer: IDLE → CLIMB → HOVER → LAND
- **Safety cutoff**: instant motor kill if roll or pitch exceeds ±30°
- **PWM ESC output** on TIM2 (4 channels, 1000–2000 µs pulse range)

---

## Hardware

| Component | Details |
|-----------|---------|
| MCU | STM32F4xx (HAL-based) |
| IMU | MPU6500 (SPI1, DMA) |
| ESC interface | TIM2 CH1–CH4 (PWM, 50 Hz) |
| Control loop timer | TIM3 (1 kHz interrupt) |
| CS Pin | GPIOA |

### Motor Mixing (X-frame)

```
        Front
   M1 (FL)  M2 (FR)
       \      /
        \    /
   M4 (RL)  M3 (RR)
        Back
```

| Motor | Roll | Pitch |
|-------|------|-------|
| M1 Front-Left  | + | − |
| M2 Front-Right | − | − |
| M3 Rear-Right  | − | + |
| M4 Rear-Left   | + | + |

---

## Software Architecture

### Control Loop Flow

```
TIM3 IRQ (1 kHz)
    └─► MPU6500_ReadRaw_DMA()          // Kick off SPI DMA transfer

SPI DMA Complete IRQ
    └─► MPU6500_DMA_Complete_Callback() // Parse raw bytes
    └─► Convert to g / rad·s⁻¹ + subtract gyro bias
    └─► IIR_Update() × 6               // Filter accel & gyro (30 Hz cutoff)
    └─► MadgwickAHRSupdateIMU()        // Quaternion integration
    └─► Euler angle extraction (roll, pitch)
    └─► PID_Update() × 2               // Roll & pitch PID
    └─► Motor mixing
    └─► set_esc_speed() × 4            // PWM output
```

### Source Files

| File | Description |
|------|-------------|
| `main.c` | Entry point, peripheral init, ISR callbacks, state machine, ESC control |
| `mpu6500.c` | MPU6500 driver: SPI read/write, DMA transfer, calibration, data conversion |
| `MadgwickAHRS.c` | Madgwick IMU/AHRS quaternion filter |
| `iir.c` | First-order IIR low-pass filter (bilinear transform) |
| `pid.c` | PID controller with derivative filter and dynamic anti-windup |

---

## Configuration

Key parameters in `main.c`:

```c
#define sampling_freq   1000.0f   // Control loop rate (Hz)
#define cutoff_freq       30.0f   // IIR filter cutoff (Hz)
#define ESC_MIN_PULSE     1000    // 0% throttle (µs)
#define ESC_MAX_PULSE     2000    // 100% throttle (µs)
#define HOVER_THROTTLE  1400.0f   // Hover throttle — tune for your AUW
#define CLIMB_RATE         0.3f   // Throttle ramp-up rate per 1 ms tick
#define LAND_RATE          0.2f   // Throttle ramp-down rate per 1 ms tick
```

PID gains (tune these for your frame):

```c
pid_roll.Kp  = 1.4153f;
pid_roll.Ki  = 0.0f;
pid_roll.Kd  = 0.0f;
pid_roll.Tau = 0.02f;       // Derivative filter time constant
pid_roll.maxlim =  400.0f;
pid_roll.minlim = -400.0f;
// pitch uses identical defaults
```

Madgwick filter gain in `MadgwickAHRS.c`:

```c
#define betaDef  0.1f   // Higher = faster convergence, more noise
```

---

## Flight Sequence

After power-on the firmware waits **10 seconds** (time to step back) before the first flight, then executes:

1. **IDLE** – Motors at minimum (1000 µs), ESCs armed
2. **CLIMB** – Throttle ramps up at `CLIMB_RATE` until `HOVER_THROTTLE`
3. **HOVER** – Holds `HOVER_THROTTLE` for 15 seconds with PID stabilization
4. **LAND** – Throttle ramps down at `LAND_RATE` until motors stop
5. Back to **IDLE** — waits 10 s before next cycle

> **Safety**: If |roll| > 30° or |pitch| > 30° at any point, throttle is immediately set to 1000 µs and the firmware halts.

---

## Getting Started

### Prerequisites

- STM32CubeIDE (or any ARM GCC toolchain)
- STM32 HAL drivers for your target (F4 series)
- 4-channel ESC + brushless motors wired to TIM2 CH1–CH4

### Build & Flash

1. Clone this repository and open the project in STM32CubeIDE.
2. Verify the pin assignments in `MX_GPIO_Init()` match your hardware (`CS_Pin`, `Debug_Pin`).
3. Build (Ctrl+B) and flash to your board.
4. On first power-on, the IMU calibration runs for ~3 seconds — keep the drone **stationary and level**.

### Tuning Tips

- Start with `HOVER_THROTTLE` at a value that lifts your drone about 50% — typically between 1300–1550 µs depending on motor KV and prop size.
- Increase `Kp` gradually while the drone is tethered or held by hand until oscillation begins, then back off ~20%.
- Add `Kd` (with `Tau` ~0.01–0.05) to damp oscillations before enabling `Ki`.
- Lower `betaDef` (e.g. 0.04) if the attitude estimate feels jittery; raise it if it lags.

---

## Dependencies

- [STM32 HAL](https://github.com/STMicroelectronics/STM32CubeF4) — peripheral abstraction
- [Madgwick AHRS](http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/) — Sebastian Madgwick's open-source orientation filter

---

## License

This project is provided as-is for educational and personal use. The Madgwick AHRS implementation retains its original open-source license. STM32 HAL code is © STMicroelectronics under their standard license terms.
