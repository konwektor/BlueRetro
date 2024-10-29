

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
<p align="justify">BlueRetro is a multiplayer Bluetooth controllers adapter for various retro game consoles. Lost or broken controllers? Reproduction too expensive? Need those rare and obscure accessories? Just use the Bluetooth devices you already got! The project is open source hardware & software under the CERN-OHL-P-2.0 & Apache-2.0 licenses respectively. It's built for the popular ESP32 chip. Wii, Switch, PS3, PS4, PS5, Xbox One, Xbox Series X|S & generic HID Bluetooth (BR/EDR & LE) devices are supported. Parallel 1P (NeoGeo, Supergun, JAMMA, Handheld, etc), Parallel 2P (Atari 2600/7800, Master System, etc), NES, PCE / TG16, Mega Drive / Genesis, SNES, CD-i, 3DO, Jaguar, Saturn, PSX, PC-FX, JVS (Arcade), Virtual Boy, N64, Dreamcast, PS2, GameCube & Wii extension are supported with simultaneous 4+ players using a single adapter.</p>

## READ THIS FIRST
* [Project documentation](https://github.com/darthcloud/BlueRetro/wiki)

## Need help?
* [Open a GitHub discussion](https://github.com/darthcloud/BlueRetro/discussions)

## Commercial solution sponsoring BlueRetro FW development
Buying these commercial adapters help the continued development of the BlueRetro firmware!!\
Thanks to all sponsors!!

[<img src="https://www.humblebazooka.com/images/HB_2021_white_rainbow_1200px.png" width="400"/>](https://www.humblebazooka.com)
* JagBT by [Humble Bazooka](https://twitter.com/humblebazooka): Atari Jaguar single port dongle.\
  https://www.humblebazooka.com/products/jag-bt-bluetooth-controller-adapter-for-the-atari-jaguar/
* Neo Geo BT by [Humble Bazooka](https://twitter.com/humblebazooka): SNK Neo Geo single port dongle.\
  https://www.humblebazooka.com/products/neo-bt-neo-geo-bluetooth-adapter/
* PCE BT by [Humble Bazooka](https://twitter.com/humblebazooka): NEC PC-Engine dongle.\
  https://www.humblebazooka.com/product/pce-bt-pc-engine-bluetooth-adapter/
* Turbo BT by [Humble Bazooka](https://twitter.com/humblebazooka): NEC TurboGrafx-16 dongle.\
  https://www.humblebazooka.com/product/turbo-bt-turbografx-16-bluetooth-adapter/
* 3DO BT by [Humble Bazooka](https://twitter.com/humblebazooka): 3DO dongle.\
  https://www.humblebazooka.com/products/3do-bt-3do-bluetooth-adapter
* Saturn BT by [Humble Bazooka](https://twitter.com/humblebazooka): Saturn dongle.\
  https://www.humblebazooka.com/products/saturn-bt-bluetooth-adapter-for-the-sega-saturn/

<BR>
  
[<img src="https://i.imgur.com/qsCLN9R.png" width="400"/>](https://www.tindie.com/stores/retrotime/)
* N64 Bluetooth Controller Receiver by [bixxewoscht](https://twitter.com/bixxewoscht): N64 single port dongle.\
  https://8bitmods.com/n64-blueretro-bt-controller-receiver-with-memory-pak-original-grey/

<BR>

[<img src="https://static.wixstatic.com/media/ce8a57_fe2ccaa817704811aefee7a0bbd74098~mv2.png/v1/fill/w_445,h_60,al_c,q_85,enc_auto/RetroOnyx_logo_primary_grey_RGB60px.png" width="400"/>](https://www.retroonyx.com/)
* Virtual Boy BlueRetro Adapter by [RetroOnyx](https://twitter.com/mellott124): Virtual Boy dongle.\
  https://www.retroonyx.com/product-page/virtual-boy-blueretro-adapter

<BR>
  
  [<img src="https://github.com/darthcloud/img/blob/main/%E5%BE%AE%E4%BF%A1%E5%9B%BE%E7%89%87_20220407151631.png" width="400"/>](https://www.aliexpress.com/item/1005004114458491.html)
* RetroScaler Mini Blueretro Wireless Game Controller Adapter for PS1 PS2 by [RetroScaler](https://twitter.com/RetroScaler): PS1 & PS2 single port dongle.\
  https://www.aliexpress.us/item/3256805870708776.html?gatewayAdapt=glo2usa4itemAdapt
* RetroScaler Blueretro Wireless Game Controller Adapter for NEO.GEO by [RetroScaler](https://twitter.com/RetroScaler): Neo-Geo single port dongle.\
  https://www.aliexpress.us/item/3256806140926261.html?gatewayAdapt=glo2usa4itemAdapt
* RetroScaler Blueretro Wireless Game Controller Adapter for SNES SFC by [RetroScaler](https://twitter.com/RetroScaler): SFC/SNES single port adapter.\
  https://www.aliexpress.us/item/3256805054285627.html?gatewayAdapt=glo2usa4itemAdapt

<BR>
  
[<img src="https://cdn.shopify.com/s/files/1/0550/1855/3556/files/mikegoble2ND_Logo_05_R1A_5f70d3e5-90bb-4e08-a4bc-1e91a60bd833.png" width="400"/>](https://www.laserbear.net/)
* GameCube Blue Retro Internal Adapter by [Laser Bear Industries](https://twitter.com/collingall): GameCube controller PCB replacement with integrated BlueRetro.\
  https://www.laserbear.net/products/gamecube-blue-retro-internal-adapter \
  https://8bitmods.com/internal-bluetooth-blueretro-adapter-for-gamecube/

<BR>
  
[<img src="https://raw.githubusercontent.com/GrechTech/RetroRosetta/main/Marketing/cover2.png" width="400"/>](http://www.grechtech.co.uk/)
* RetroRosetta by [GrechTech](https://twitter.com/GrechTech): BlueRetro universal core and cables.\
  http://www.grechtech.co.uk/

## Community Contribution
* BlueRetro PS1/2 Receiver by [mi213 ](https://twitter.com/mi213ger): 3D printed case & PCB for building DIY PS1/2 dongle.\
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

<br><p align="center"><img src="https://cdn.hackaday.io/images/4560691598833898038.png" height="200"/></p>
