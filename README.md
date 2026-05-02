# STM32F407 — Dual BNO085 IMU on Shared SPI Bus

**RACLAB Rover Navigation System** | STM32F407VG Discovery + 2× BNO085 | Stable 100 Hz Dual-IMU Driver

---

## Overview

This project implements a robust driver for two BNO085 9-DOF IMU sensors operating on a **single shared SPI bus** over an STM32F407VG Discovery board. Both sensors share the same SCK/MISO/MOSI lines; bus arbitration is handled entirely in software through dedicated Chip Select and Interrupt pins.

The core engineering challenge — and the solution documented here — is achieving stable, zero-packet-loss operation from two sensors that share a bidirectional data line, without any additional hardware (no SPI multiplexers, no level shifters).

---

## Live Demo

https://github.com/alcad1us/stm32-dual-bno085-shared-spi/raw/master/docs/demo.mp4

Both IMUs streaming orientation data in real-time over a shared SPI bus. SAMPLES counters climbing in sync confirms zero packet loss.

---

## Key Features

| Feature | Detail |
|---|---|
| **Polling Rate** | 100 Hz per sensor (10 ms report interval) |
| **Orientation Output** | Quaternion (Q14 fixed-point) + Euler angles (Roll/Pitch/Yaw) |
| **Software Tare** | Auto-zeroing of yaw after 50 stable samples |
| **Boot Mechanism** | Sequential Polling Boot — sensors initialized one at a time |
| **Bus Recovery** | SPI OVR flag auto-clear on every transaction |
| **Debug Interface** | USART2 @ 115200 baud, ANSI terminal dashboard |
| **MCU** | STM32F407VG @ 168 MHz (HSI + PLL) |

---

## Hardware

### Board
- **STM32F407VG Discovery** (or compatible F407 target)
- System clock: 168 MHz

### Sensors
- 2× **BNO085** (Adafruit breakout or compatible)
- PS0 and PS1 pins tied to 3.3 V → SPI mode active

---

## Pin Mapping

| STM32 Pin | BNO085 Signal | Function | Notes |
|-----------|--------------|----------|-------|
| **PA5** | SCL | SPI1 SCK (AF5) | Shared clock |
| **PA6** | SDA | SPI1 MISO (AF5) | Shared data in |
| **PA7** | DI | SPI1 MOSI (AF5) | Shared data out |
| **PE0** | CS | IMU 1 Chip Select | Active LOW, Software NSS |
| **PE1** | CS | IMU 2 Chip Select | Active LOW, Software NSS |
| **PE4** | INT | IMU 1 Interrupt | Active LOW, Pull-up |
| **PE5** | INT | IMU 2 Interrupt | Active LOW, Pull-up |
| **PC0** | RST | Shared Reset | Active LOW, shared by both |
| **PA2** | — | USART2 TX | Debug output, 115200 baud |

> **SPI Config:** Mode 3 (CPOL=High, CPHA=2Edge), Prescaler 64 → **1.3 MHz**, 8-bit MSB, Software NSS

---

## The BNO085 MISO Bus-Contention Bug — Engineering Note

### Problem

The BNO085's SPI interface is partially bidirectional. On many breakout boards:
- The **DI** pin is not a true MOSI — it floats or reads zeros on some hardware revisions.
- The **SDA** pin acts as the primary data line for both TX and RX.

When two sensors share MISO on the same bus, the **inactive sensor** (CS HIGH) still drives the MISO line due to an internal tri-state timing issue, corrupting reads from the active sensor.

### Root Cause

The BNO085 does not release its MISO line immediately after CS goes HIGH. During the inter-transaction gap, both sensors weakly drive MISO simultaneously, causing bus contention and corrupted SHTP packets.

### Solution: Sequential Polling Boot

Instead of simultaneous initialization, sensors are booted **one at a time** with a strict ordering protocol:

```
1. Assert RST LOW (300 ms) → Release RST HIGH → Wait 1500 ms boot
2. Force CS2 HIGH (deselect IMU2)
3. Send SetFeature command to IMU1 via CS1, wait for INT1
4. Delay 200 ms   ← allows IMU1 MISO to fully tri-state
5. Send SetFeature command to IMU2 via CS2, wait for INT2
6. Enter polling loop: check INT1/INT2 independently each cycle
```

The additional `for(volatile int i=0; i<20; i++)` settling delay before each CS assertion ensures the bus is clean before the HAL transaction begins.

The `bno_transfer_select()` function explicitly deselects the **non-target** sensor before every transaction:

```c
// Ensure the other sensor is deselected before asserting our CS
HAL_GPIO_WritePin(GPIOE, (cs_pin == CS1_PIN ? CS2_PIN : CS1_PIN), GPIO_PIN_SET);
HAL_GPIO_WritePin(GPIOE, cs_pin, GPIO_PIN_RESET);
```

This guarantees only one sensor drives MISO at any point in time.

---

## Live Dashboard

Connect any serial terminal (PuTTY, minicom, screen) to USART2 at **115200 baud**. The firmware renders a live ANSI dashboard:

```
==============================================================
          RACLAB ROVER: DUAL IMU STABILIZED DASHBOARD
==============================================================
SENSOR | ROLL  | PITCH |  YAW  | SAMPLES | QUAT(W,X,Y,Z)
-------|-------|-------|-------|---------|-------------------------
IMU_01 |   2.4 |  -1.1 |   0.0 |    4821 | (16120, -312,  198,   47)
IMU_02 |  -0.8 |   3.2 |   0.0 |    4819 | (16305,  129, -203,  -61)
--------------------------------------------------------------
[STATUS] SPI: SHARED | MODE: POLLING | RATE: 100Hz
```

### SAMPLES Counter — Health Monitor

The `SAMPLES` column is not just a counter. It is the system's **health monitor**:

- Increments only when a valid Rotation Vector report (ID `0x05`) is parsed
- A steadily climbing counter confirms: bus is stable, SHTP packets are arriving intact, no data corruption
- If `SAMPLES` stalls on one sensor while the other climbs, it indicates a bus contention event or sensor hang
- Both counters climbing in sync at ~100 Hz is the proof of zero-packet-loss operation

### Startup Diagnostics

During boot, the terminal may show timeout messages such as:

```
FAILED! (Timeout)
```

**These are not errors.** They are the diagnostics layer working correctly. The `bno_transfer_select()` function polls the INT pin with a configurable timeout. If the sensor is still in its boot sequence, the function returns `-1` cleanly and the system retries — keeping the SPI bus idle and safe until the sensor signals readiness. This is intentional defensive design, not a failure condition.

---

## Project Structure

```
stm32-dual-bno085-shared-spi/
├── Core/
│   ├── Inc/
│   │   ├── main.h
│   │   └── stm32f4xx_hal_conf.h
│   ├── Src/
│   │   ├── main.c              ← All application logic
│   │   ├── stm32f4xx_hal_msp.c
│   │   └── stm32f4xx_it.c
│   └── Startup/
│       └── startup_stm32f407vgtx.s
├── Drivers/
│   ├── CMSIS/
│   └── STM32F4xx_HAL_Driver/
├── docs/
│   └── images/
├── imuspi.ioc                  ← STM32CubeMX config
├── STM32F407VGTX_FLASH.ld
└── README.md
```

---

## Build & Flash

### Requirements
- [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html) 1.15+
- ST-Link V2 (on-board on Discovery)

### Steps
1. Clone the repository
2. Open STM32CubeIDE → `File > Open Projects from File System` → select repo root
3. Build: `Project > Build All` (Ctrl+B)
4. Flash: `Run > Debug` or `Run > Run` with ST-Link connected
5. Open serial monitor on `/dev/ttyUSB0` (or COM port) at **115200 baud**

---

## Software Architecture

```
main()
  │
  ├── Sequential Boot
  │     ├── bno_transfer_select(CS1, INT1, SetFeature_cmd)  ← IMU1 init
  │     ├── HAL_Delay(200)
  │     └── bno_transfer_select(CS2, INT2, SetFeature_cmd)  ← IMU2 init
  │
  └── Polling Loop
        ├── if INT1 LOW → bno_transfer_select(CS1) → parse → Calculate_Euler(&imu1)
        ├── if INT2 LOW → bno_transfer_select(CS2) → parse → Calculate_Euler(&imu2)
        └── every 100ms → render dashboard via USART2

bno_transfer_select()
  ├── SPI OVR recovery
  ├── Deselect opposite sensor
  ├── Wait INT with timeout
  ├── Assert CS
  ├── Transfer header (4 bytes)
  ├── Transfer payload (byte-by-byte, TX+RX merged)
  └── Deassert CS → return

Calculate_Euler()
  ├── Q14 → float normalization (÷ 16384)
  ├── Roll:  atan2(2(wx+yz), 1−2(x²+y²))
  ├── Pitch: asin(2(wy−zx))
  └── Yaw:   atan2(2(wz+xy), 1−2(y²+z²)) − yaw_offset (tare after 50 samples)
```

---

## Roadmap

### Planned

- [ ] **micro-ROS Integration** — Expose IMU data as `sensor_msgs/Imu` topics over USB CDC transport, compatible with ROS 2 Humble
- [ ] **Dual-IMU Sensor Fusion (EKF)** — Extended Kalman Filter combining both sensors for increased orientation accuracy and fault tolerance
- [ ] **Rover Control Loop Integration** — Feed fused navigation data into the RACLAB Rover F407's autonomous path-following controller

### Known Limitations

- SPI clock limited to 1.3 MHz (Prescaler 64) for shared-bus stability; single-sensor configurations can run at 2.6 MHz (Prescaler 32)
- Shared RST line means both sensors reset simultaneously; staggered reset requires a second GPIO
- `Calculate_Euler()` uses `math.h` (`atan2f`, `asinf`) — requires linking with `-lm`; newlib-nano `float printf` is disabled, output uses Q14×10 integer scaling

---

## References

- [BNO085 Datasheet](https://www.hillcrestlabs.com/downloads) — SHTP protocol, SPI timing, register map
- [Adafruit BNO085 Breakout](https://learn.adafruit.com/adafruit-9-dof-orientation-imu-fusion-breakout-bno085) — Pinout and mode selection (PS0/PS1)
- [SHTP Reference Manual](https://www.ceva-dsp.com/wp-content/uploads/2019/10/BNO080_085-Sensor-Hub-Transport-Protocol.pdf) — Packet structure, channel definitions, SetFeature command
- [STM32F407 Reference Manual (RM0090)](https://www.st.com/resource/en/reference_manual/rm0090.pdf) — SPI peripheral, GPIO MODER register

---

## Author

**Muhammet Yusuf Ozkan**
Embedded Systems Engineer — RACLAB Rover Project

---

## License

MIT License — see [LICENSE](LICENSE) for details.
