

This is a modified version of BlueRetro with support for sending commands to an OGX360 over I2C. 
See https://github.com/konwektor/ogx360 for a compatible build of ogx360.
<BR>

v24.04 Latest

Based on BlueRetro v24.04

    Fixed rumble with i2c repeated start - gamepads are not hanging anymore for 1 sec - tested with wiiu and ps4
    Added hw2 internal/external support
    Added port detection in hw2
    Added rubmle feedback for actually used port - push xbox button and gamepad will rumble shortly x-times , where x is port number used

This version need also updating arduino firmware in case of use hw2, use mine fork at https://github.com/konwektor/ogx360.
hw1 doesn`t need arduino firmware update if You have used already ogx360 fork for BlueRetro support.

Small detailed info in [!compiled](https://github.com/konwektor/BlueRetro/tree/master/!compiled).
<br>
This can be compiled the same as BlueRetro. <br>
Copy /configs/hw1/ogx360 to Your /Blueretro/sdkconfig start esp-idf (v5.1.2) and rund "esp.py build".

Or just go to [Releases](https://github.com/konwektor/BlueRetro/releases), and download already compiled files.
<BR>
# Need help? Found Bug? Any Ideas?
* [Open a GitHub discussion](https://github.com/konwektor/BlueRetro/discussions)

<BR>



- All Xbox one x/s series x/s controllers working with rumble, no delay no stuck.
- Wiiu pro rumble tested and working
- Ps4 rumble tested and working

- All other bluetooth gamepads supported by Blueretro should work - not tested - got any???? Give me feedback.

<BR>


<BR>
 
 - look to [Davidxgames](https://github.com/davidxgames) or [XGAMES VIDEOJUEGOS](https://www.youtube.com/@XGAMESVIDEOJUEGOS),  - video how to do mod, assembly instruction and compiled files ready to go  (based o BlueRetro 1.8.x I think) -channel in spanish.

    Respect and many BIG Thanks goes to Ervin: [Eolvera85](https://github.com/eolvera85):- author of PS5 support patch.



<BR>

 
    

# BlueRetro

<p align="center"><img src="/static/PNGs/BRE_Logo_Color_Outline.png" width="600"/></p>
<br>
<p align="justify">BlueRetro is a multiplayer Bluetooth controllers adapter for various retro game consoles & computers. Lost or broken controllers? Reproduction too expensive? Need those rare and obscure accessories? Just use the Bluetooth devices you already got! The project is open source hardware & software under the CERN-OHL-P-2.0 & Apache-2.0 licenses respectively. It's built for the popular ESP32 chip. Wii, Switch, PS3, PS4, PS5, Xbox One, Xbox Series X|S & generic HID Bluetooth (BR/EDR & LE) devices are supported. Parallel 1P (Computers, NeoGeo, Supergun, JAMMA, Handheld, etc), Parallel 2P (Atari 2600/7800, Master System, Computers, etc), NES, PCE / TG16, Mega Drive / Genesis, SNES, CD-i, 3DO, Jaguar, Saturn, PSX, PC-FX, JVS (Arcade), Virtual Boy, N64, Dreamcast, PS2, GameCube & Wii extension are supported with simultaneous 4+ players using a single adapter.</p>

## READ THIS FIRST
* [Project documentation](https://github.com/darthcloud/BlueRetro/wiki)

## Need help?
* [Open a GitHub discussion](https://github.com/darthcloud/BlueRetro/discussions)

## Makers sponsoring BlueRetro
Buying BlueRetro adapters from these makers helps support the continued development of the BlueRetro firmware!\
Thanks to all sponsors!

* [Laser Bear Industries](https://www.laserbear.net)
* [Humble Bazooka](https://www.humblebazooka.com)
* [RetroOnyx](https://www.retroonyx.com/)
* [RetroTime](https://8bitmods.com/retrotime)

## Community Contribution
* BlueRetro PS1/2 Receiver by [mi213](https://twitter.com/mi213ger): 3D printed case & PCB for building DIY PS1/2 dongle.\
  https://github.com/Micha213/BlueRetro-PS1-2-Receiver
* N64 BlueRetro Mount by [reventlow64](https://twitter.com/reventlow): 3d printed mount for ESP32-DevkitC for N64.\
  https://www.prusaprinters.org/prints/90275-nintendo-64-blueretro-bluetooth-receiver-mount
* BlueRetro Adapter Case by [Sigismond0](https://twitter.com/Sigismond0): 3d printed case for ESP32-DevkitC.\
  https://www.prusaprinters.org/prints/116729-blueretro-bluetooth-controller-adapter-case
* BlueRetro AIO by [pmgducati](https://github.com/pmgducati): BlueRetro Through-hole base and cable PCBs.\
  https://github.com/pmgducati/Blue-Retro-AIO-Units
* BlueRetro HW2 internal guides by [Nostalgic Indulgences](https://twitter.com/nosIndulgences): Internal install guides\
  https://github.com/nostalgic-indulgences/BlueRetro_Internal_Installation
* BlueRetro latency test by [GamingNJncos](https://twitter.com/GamingNJncos): Documentation on how to run BlueRetro latency test\
  https://github.com/GamingNJncos/BLE-3D-Saturn-Public/tree/main/BlueRetro_Latency_Testing
* BR4N64 by [TharathielCB](https://github.com/TharathielCB): Internal BlueRetro Flex-PCB for Nintendo 64\
  https://github.com/TharathielCB/BR4N64
* BlueMemCard by [ChrispyNugget](https://github.com/ChrispyNugget): Replacement PCB for PSX memory card that incorporates PicoMemcard and BlueRetro support\
  https://github.com/ChrispyNugget/BlueMemCard
* BlueRetro HW2 QSB for GameCube by [Arthrimus](https://github.com/Arthrimus): Internal Blueretro PCB with CurrentTrigger for GameCube\
  https://github.com/Arthrimus/BlueRetro-HW2-GameCube

<br><p align="center"><img src="https://cdn.hackaday.io/images/4560691598833898038.png" height="200"/></p>
