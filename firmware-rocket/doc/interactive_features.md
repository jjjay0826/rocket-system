# Rocket Firmware Interactive Features & LED Indicators

This document describes the custom interactive features and the indicator light behaviors implemented using the previously unassigned GPIOB pins (`PB5`, `PB6`, `PB7`) and the status LEDs (`B2`, `B10`).

---

## 1. Pin Assignments (GPIOB Pin Reconfiguration)

The following pins, previously configured as outputs (`SIG_3/2/1`), are reconfigured in the `.ioc` as inputs with internal pull-up resistors (`GPIO_PULLUP`):

| Signal Name | MCU Pin | HW Connection Type | Function | Active State |
| :--- | :--- | :--- | :--- | :--- |
| **`SIG_1_Pin`** | **`PB7`** | Jumper Cap (to **GND**) | **Disable LoRa Telemetry** | `LOW` (Short to GND = Disabled) |
| **`SIG_2_Pin`** | **`PB6`** | Tactile Button (to **GND**) | **Manual Ignition Trigger** | `LOW` (Falling edge = Trigger) |
| **`SIG_3_Pin`** | **`PB5`** | Unused | Reserved / Unused | N/A |

> ⚠️ **重要接線說明（PULLUP 模式）**：
> - 互動引腳使用**內部上拉電阻**，懸空時預設為 HIGH（3.3V）。
> - 跳線帽/按鈕的一端接 **GND**，另一端接對應引腳。
> - 輸入 LOW (0V) = 功能啟動；HIGH (懸空) = 正常/停用。

---

## 2. Interactive Feature Logic

### A. LoRa Telemetry Disable Switch (PB7)
- **Operation**: Connect a jumper cap between **PB7** (`SIG_1`) and **GND** to pull the pin Low.
- **Logic**:
  - The firmware actively queries the PB7 state. If detected as Low:
    - Normal 500ms telemetry packets will not be transmitted via LoRa.
    - The initial startup diagnostic report will not be transmitted via LoRa.
    - Left indicator LED (`B10`) remains **solid OFF**.
  - When the jumper cap is removed (PB7 goes High), LoRa transmission automatically resumes.
- **Benefit**: Saves power during testing or launchpad waiting phases, and avoids radio interference.

### B. One-Shot Safe Manual Ignition Trigger (PB6)
- **Operation**: Connect a temporary push button between **PB6** (`SIG_2`) and **GND**. Press the button to trigger ignition.
- **Ignition Outputs**: Triggers both channel 1 (`FIRE_7V_1` / `PA0`) and channel 2 (`FIRE_7V_2` / `PA1`).
- **Safety & Anti-Stuck Guard (One-Shot)**:
  - **Edge-Triggered**: Ignition is initiated exclusively by a **falling edge** (from High to Low).
  - **1-Second Pulse**: Once triggered, the ignition outputs stay High for exactly **1.0 second (1000ms)** and are then automatically pulled Low.
  - **Safety Lockout**: If the button remains pressed or gets stuck (PB6 stays Low), the outputs will still shut off after 1 second. The button must be released (return to High) and pressed again to trigger a subsequent ignition pulse.
  - **Flight Lockout**: Manual ignition is disabled when the flight state is in the deployment phase (`FLIGHT_DEPLOYING`) to prevent interference with autonomous pyro systems.

---

## 3. Status LED Indicators
> **⚠️ 硬體與電位注意**:
> - **B10 LED**（左側）與 **B2 LED**（右側）均為 **Active High（高電位亮）**。
> - 韌體以 `GPIO_PIN_SET` (高電位) 點亮、`GPIO_PIN_RESET` (低電位) 熄滅。

The behavior of the left and right indicator LEDs has been redefined to present more meaningful system status:

### Left LED (B10) - LoRa TX Indicator
- Represents active wireless transmission.
- **Flicker (≈50ms - 250ms)**: LoRa is currently transmitting a telemetry packet (either periodic telemetry or startup logs).
- **Solid OFF**: LoRa is disabled (via PB5 jumper cap) or not transmitting.

### Right LED (B2) - System Health & GPS Status
Represents the collective status of the sensors, SD card logger, and GPS fix, using a priority-driven state machine:

| Priority | System Condition | B2 LED Pattern | Indication / Action Required |
| :---: | :--- | :--- | :--- |
| **1 (Highest)** | **Hardware Fault**<br>(BMP585, IMU, or SD card logger dead) | **Fast Blink (5 Hz)**<br>(100ms ON / 100ms OFF) | **DO NOT LAUNCH.** Check SPI buses, SD card presence, and wiring. |
| **2** | **GNSS Positioning**<br>(Hardware OK, but GPS has no 3D Fix) | **Slow Blink (1 Hz)**<br>(500ms ON / 500ms OFF) | Hardware OK. Wait for GPS satellite lock (outdoors). |
| **3 (Lowest)** | **All Systems Go**<br>(Hardware healthy, GPS has 3D Fix) | **Solid ON** | **Ready for Launch.** All subsystems nominal. |

---
