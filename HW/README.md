I have after long timne decided to call this mixed fork as BROgx360 because its 2modul roots - Blueretro project from Darthcloud and Ogx360 by Ryzee119.Simple...
Maybe for myself only, we will see.


# All BROgx360 docs / taken from BlueRetro's documentation / 
Instant knwolege , without too much details (deatiled docs are at Darthclouds wiki docs page), system behaviour, shortcuts, etc. and mostly differences beetwen. 

# 1 - Building hardware HW1
HW1 stays as before no changes BROgx360.

# 2 - Building hardware HW2
Schematic is in https://github.com/konwektor/BlueRetro/blob/master/HW/HW2/BROgx360_11_24.pdf
HW2 specification require a lot more connection and as such is not recommended
for novice in electronic at all. HW2 main feature is active port connection detection.

Power & Reset management are optional feature supported by HW2 aswell.
Possible to use BROgx360 HW2 as external adapter , but its main purpose is 
sitting inside OGXbox and allowing powering ON console from couch!

# Currently ocupied port Rumble Feedback HW2
Because xbox gamepads doesn`t have player slot/place feedback (colours or diodes), exept xbox360 witch are not supported by BROgx360,
I have added small rumble feedback function - after succesfully connection, pushing XBOX button, or similar button in other controlller, will cause
controller to  rumble n-times, where n is the number of actually used port.
If orginal xbox gamepad is plugged in, to this slot, all connected controllers jumps over to first free and not occupied by wired gamepad slot. 
This also affect in rumble feedback n-times. / 


* [Internal install HW2 spec](BlueRetro-HW2-Internal-Install-Specification)
* [External adapter HW2 spec](BlueRetro-HW2-External-Specification)

[Nostalgic Indulgences](https://twitter.com/nosIndulgences) created multiple guides base on HW2 for internal install:: https://github.com/nostalgic-indulgences/BlueRetro_Internal_Installation\
[TharathielCB](https://github.com/TharathielCB) created BR4N64, an internal BlueRetro Flex-PCB for the Nintendo 64: https://github.com/TharathielCB/BR4N64

# 3 - Pairing Bluetooth controller

In default configuration BlueRetro is always in inquiry mode (LED pulsing) if no controller is connected\ or in hw2 internal when system is on, and no controller connected
Pair via inquiry first (SYNC or pairing mode), on subsequent connection you can simply page (button press or power on button).\
You may change this behavior by switching inquiry mode in the web config to manual.\ 
because after pairing - not-paired before controllers can not connect to BROgx360 and  switch xbox on
Pressing DVD TRAY button for 3 sec will activate inquiry mode.\
[Controller List & Pairing Guide](Controller-pairing-guide)

# 4 - Web config

Power on system and connect via Web Bluetooth at https://blueretro.io to configure adapter.\
**The config mode is only available if no controller is connected.** \ **Disconnect every controlller from BROgx360 to connect to Web configurations pages.**
web pages works only partly for BROgx360 - adapter isn officialy supported by Darthcloud
**Supported only in Desktop or Android Chrome**
See [BlueRetro BLE Web Config User Manual](BlueRetro-BLE-Web-Config-User-Manual) for more detail.

# 5 - Physical buttons usage

## 5.1 - ESP32 button EN (Reset)

## 5.2 - BOOT (IO0) External adapter
* Button press under < 3 sec (All LEDs solid):\
  If in pairing mode: Stop pairing mode otherwise all BT devices are disconnect.
* Button press between > 3 sec and < 6 sec (All LEDs blink slowly):\
  Start pairing mode.
* Button press between > 6 sec and < 10 sec (All LEDs blink fast):\
  Factory reset ESP32 to original BlueRetro firmware the device shipped with & reset configuration.

## 5.3 - BOOT (IO0) Internal install
  Connected/used as XBOX DVD Tray button.
  
### 5.3.1 - System behavior while ESP32 on and xbox on
* Short Button press DVD / or press under < 3 sec (All LEDs solid):\
  DVD Tray open/close
* DVD Button press between > 3 sec and < 6 sec (All LEDs blink slowly):\
  If in pairing mode: Stop pairing mode otherwise all BT devices are disconnect.
* Button press between > 6 sec and < 10 sec (All LEDs blink fast):\
  Start pairing mode.
* Button press over > 10 sec (All LEDs blink very fast):\
  Factory reset ESP32 to original BlueRetro firmware the device shipped with & reset configuration.
* Quick double press:\
  System is powered down via power relay / power pin.

### 5.3.2 - System behavior while ESP32 off & system off
* Holding DVD and then powering system put the ESP32 in sleep mode. Power on with already paired controller not possible.!!!

### 5.3.3 - System  behavior while ESP32 on & system off/ BROgx360 hw2 internal is exactly so made  
* Short press DVD: XBOX is powered on via power pin
* Long press DVD & release: Power on and also DVD open signal.

### 5.3.4 - System  behavior while ESP32 off and system on
* While the ESP32 is in boot mode or in deep sleep the system reset function is lost.
* We dont have anymore BlueRetros "system reset" - behaviour of reset pin is used for BROgx360 DVD Tray opening/closing, and also for system powerin .

# 6 - Button combinations functions
Their is two forms of button combo to activate macro functions:
*LT+RT+START+ CommonFunction
* LT+RT+START+Y+ RestrictedFunction

The default mapping for the common function last button:
* X (SNES Y, PS Square) -> System Reset
* A (SNES B, PS X) -> System Shutdown/Controller disconnect
* B (SNES A, PS Circle) -> Toggle BT pairing mode on/off
* BACK (Select) -> Toogle wired output mode between GamePad & GamePadAlt - Not used in BROgx360, not now....

The default mapping for the restricted function last button
* D-pad Up -> Factory Reset
* D-pad Down -> Disable BlueRetro (Deep sleep)

These default can be modified via the [advance config mapping section](https://github.com/darthcloud/BlueRetro/wiki/BlueRetro-BLE-Web-Config-User-Manual#24---mapping-config).\
Those mapping are generaly located at the end of the mapping list.\
Simply change the source button to alter the mapping of a base, common function or restricted function button.
gamepads mapping for specified adapter / output number.... x/ works also fo BROgx360

Refer to [BlueRetro mapping reference](https://docs.google.com/spreadsheets/d/e/2PACX-1vT9rPK2__komCjELFpf0UYz0cMWwvhAXgAU7C9nnwtgEaivjsh0q0xeCEiZAMA-paMrneePV7IqdX48/pubhtml) to translate BlueRetro label to specific system button names.

# 7 - LED usage (IO17)

* See [5 - Physical buttons usage](#5---physical-buttons-usage) for LED meaning while button BOOT (IO0) is pressed/ (DVD button in BROgx360)
* Solid: An error occured, try power cycle, check serial logs for detail.
* Pulsing: Bluetooth inquiry mode enable (new pairing).
* Off: No error and Bluetooth inquiry mode disabled.

    


## READ THIS FIRST
* [Project documentation](https://github.com/darthcloud/BlueRetro/wiki)

## Need help?
* [Open a GitHub discussion](https://github.com/darthcloud/BlueRetro/discussions)
