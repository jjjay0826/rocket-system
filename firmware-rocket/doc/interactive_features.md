# Rocket Firmware Interactive Features & LED Indicators

This document describes the custom interactive features and the indicator light behaviors implemented using the previously unassigned GPIOB pins (`PB5`, `PB6`, `PB7`) and the status LEDs (`B2`, `B10`).

---

## 1. Pin Assignments (GPIOB Pin Reconfiguration)

The following pins, previously configured as outputs (`SIG_OUT_3/2/1`), have been reconfigured as inputs with internal pull-up resistors (`GPIO_PULLUP`):

| Signal Name | MCU Pin | HW Connection Type | Function | Active State |
| :--- | :--- | :--- | :--- | :--- |
| **`SIG_OUT_3_Pin`** | **`PB5`** | Jumper Cap (to **3.3V**) | **Disable LoRa Telemetry** | `HIGH` (Short to 3.3V = Disabled) |
| **`SIG_OUT_2_Pin`** | **`PB6`** | Tactile Button (to **3.3V**) | **Manual Ignition Trigger** | `HIGH` (Rising edge = Trigger) |
| **`SIG_OUT_1_Pin`** | **`PB7`** | Reserved (to 3.3V) | Unused Input Pull-down | Reserved |

> ⚠️ **重要接線說明（PULLDOWN 模式）**：
> - 三支腳使用**內部下拉電阻**，浮空時電位為 LOW（0V），與原 OUTPUT LOW 相同，不影響感測器電路。
> - 跳線帽/按鈕的一端接 **3.3V**（不是 GND），另一端接對應引腳。
> - 輸入 HIGH (3.3V) = 功能啟動；LOW (浮空) = 正常/停用。

---

## 2. Interactive Feature Logic

### A. LoRa Telemetry Disable Switch (PB5)
- **Operation**: Connect a jumper cap between **PB5** and **GND** to pull the pin Low.
- **Logic**:
  - The firmware actively queries the pin state. If detected as Low:
    - Normal 500ms telemetry packets will not be transmitted via LoRa.
    - The initial startup diagnostic report will not be transmitted via LoRa.
    - Left indicator LED (`B10`) remains **solid OFF**.
  - When the jumper cap is removed (PB5 goes High), LoRa transmission automatically resumes.
- **Benefit**: Saves power during testing or launchpad waiting phases, and avoids radio interference.

### B. One-Shot Safe Manual Ignition Trigger (PB6)
- **Operation**: Connect a temporary push button between **PB6** and **GND**. Press the button to trigger ignition.
- **Ignition Outputs**: Triggers both channel 1 (`FIRE_7V_1` / `PA0`) and channel 2 (`FIRE_7V_2` / `PA1`).
- **Safety & Anti-Stuck Guard (One-Shot)**:
  - **Edge-Triggered**: Ignition is initiated exclusively by a **falling edge** (from High to Low).
  - **1-Second Pulse**: Once triggered, the ignition outputs stay High for exactly **1.0 second (1000ms)** and are then automatically pulled Low.
  - **Safety Lockout**: If the button remains pressed or gets stuck (PB6 stays Low), the outputs will still shut off after 1 second. The button must be released (return to High) and pressed again to trigger a subsequent ignition pulse.
  - **Flight Lockout**: Manual ignition is disabled when the flight state is in the deployment phase (`FLIGHT_DEPLOYING`) to prevent interference with autonomous pyro systems.

---

## 3. Status LED Indicators

> **⚠️ 硬體注意**: 兩個 LED (B2, B10) 均為 **Active Low（低電位亮）**。  
> 韌體以 `GPIO_PIN_RESET` (低電位) 點亮、`GPIO_PIN_SET` (高電位) 熄滅。

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
