# Programming
* [Programming Instructions](Programming/README.md)

# BROGX Documentation Overview
This BROGX instruction set is mostly adapted from BlueRetro's documentation. It covers pairing, button usage, system behavior, combo commands, web configuration, and more.
For detailed information on BlueRetro, visit DarthCloud's [BlueRetro wiki page](https://github.com/darthcloud/BlueRetro/wiki).

# 1 - Building Hardware HW1
HW1 is a simple, external build. It is suitable for beginners in electronics if the instructions are followed carefully.
* [Building HW1 Instructions](Hardware/HW1/README.md)

# 2 - Building Hardware HW2
HW2 requires significantly more connections and is not recommended for beginners. Its primary feature is active port connection detection.
* [Building HW2 Internal Instructions](Hardware/HW2/README.md)

You can also use BROGX HW2 as an external adapter:
* [Building HW2 External Instructions](Hardware/HW2/External/README.md)

Power management (Xbox power ON/OFF) is an optional feature supported by HW2 internal builds. The ESP32 must remain powered on.
* [Powering ESP32]

# 3 - Pairing Bluetooth Controller
In the default configuration, BlueRetro is always in inquiry mode (IO17 LED pulsing) if no controller is connected. This applies to HW2 internal/external builds when the system is ON with no controller connected.

- Pair controllers via inquiry first.
- Inquiry behavior can be changed to manual in the web config. In manual mode:
  - When the Xbox is ON with no Bluetooth controller connected, press the ESP32 BOOT button (HW1/HW2 external builds) or the Xbox DVD TRAY button (HW2 internal build) for 3 seconds to activate inquiry mode.

A paired Bluetooth controller can turn the Xbox ON (HW2 internal with powered ESP32) or OFF (HW2 internal).

[BlueRetro Controller List & Pairing Guide](https://github.com/darthcloud/BlueRetro/wiki/Controller-pairing-guide)

# 4 - Web Config
Power on the system and connect via Web Bluetooth at https://blueretro.io to configure the adapter.
**Config mode is only available if no controller is connected.** Ensure all controllers are disconnected from BROGX before accessing the Web configuration pages.
Web pages work only partially for BROGX as the adapter is not officially supported by DarthCloud.

**Supported only in Desktop or Android Chrome.**

Refer to [BlueRetro BLE Web Config User Manual](BlueRetro-BLE-Web-Config-User-Manual) for more details.

# 5 - Physical Button Usage

## 5.1 - ESP32 Button EN

### 5.1.1 - ESP32 Button EN (Reset) External Adapter
* Pressing the button resets the ESP32.

### 5.1.2 - ESP32 Button EN (Reset) Internal Adapter
* The ESP32 EN button is linked to the Xbox power button and resets the ESP32 along with the Xbox power signal, depending on its current state.

## 5.2 - BOOT (IO0) External Adapter
* Pressing the button for less than 3 seconds (all LEDs solid):
  - Stops pairing mode if active; otherwise, disconnects all Bluetooth devices.
* Pressing the button between 3 and 6 seconds (all LEDs blink slowly):
  - Starts pairing mode.
* Pressing between 6 and 10 seconds (all LEDs blink fast):
  - Factory resets the ESP32 to its original BlueRetro firmware and resets the configuration.

## 5.3 - BOOT (IO0) Internal Installation
The ESP32 BOOT button is connected to the Xbox DVD Tray button.

### 5.3.1 - System Behavior (ESP32 ON, Xbox ON)
* Short press (< 3 seconds): Opens/closes the DVD Tray.
* Longer presses:
  - Between 3 and 6 seconds (all LEDs blink slowly): Stops pairing mode or disconnects all Bluetooth devices.
  - Between 6 and 10 seconds (all LEDs blink fast): Starts pairing mode.
  - Over 10 seconds (all LEDs blink very fast): Factory resets the ESP32 and resets the configuration.
* Quick double press: Powers down the system via the power relay.

### 5.3.2 - System Behavior (ESP32 OFF, System OFF)
* Holding the DVD button while powering on the system put the ESP32 in boot (download) mode. 

### 5.3.3 - System Behavior (ESP32 ON, System OFF)
* Short press: Powers on the Xbox via the power pin.
* Long press and release: Powers on the Xbox and sends a DVD open signal.

### 5.3.4 - System Behavior (ESP32 OFF, System ON)
While the ESP32 is in boot mode or in deep sleep the system reset function is lost.
* While in boot mode or deep sleep, Open/Clos DVD Tray function is unavailable. The reset pin controls DVD Tray opening/closing instead of the original BlueRetro reset.

# 6 - Button Combination Functions
There are two types of button combos to activate macro functions:
* LT + RT + START + CommonFunction
* LT + RT + START + Y + RestrictedFunction

The default mappings for common functions:
* X: DVD Tray eject/close or BlueRetro System Reset
* A: System Shutdown/Controller disconnect
* B: Toggle Bluetooth pairing mode
* BACK (Select): Toggle wired output mode between GamePad and GamePadAlt (not currently used in BROGX).

The default mappings for restricted functions:
* D-pad Up: Factory Reset
* D-pad Down: Disable BlueRetro (Deep Sleep)

These mappings can be customized via the [Advanced Config Mapping Section](https://github.com/darthcloud/BlueRetro/wiki/BlueRetro-BLE-Web-Config-User-Manual#24---mapping-config).
Mappings are usually located at the end of the list. Change the source button to modify the base, common, or restricted function button mapping.

Refer to the [BlueRetro Mapping Reference](https://docs.google.com/spreadsheets/d/e/2PACX-1vT9rPK2__komCjELFpf0UYz0cMWwvhAXgAU7C9nnwtgEaivjsh0q0xeCEiZAMA-paMrneePV7IqdX48/pubhtml) to translate BlueRetro labels to specific system button names.

# 7 - LED Usage (IO17)

Refer to [5 - Physical Button Usage](#5---physical-button-usage) for LED behavior when the BOOT (IO0) button is pressed (DVD button in BROGX360).
* Solid: An error occurred. Try a power cycle or check serial logs for details.
* Pulsing: Bluetooth inquiry mode enabled (new pairing).
* Off: No error; Bluetooth inquiry mode disabled.

