"Ready to go files"  for people who got problems with compilators, IDF etc(I know how annoying this could be), under Windows.
Just go to releases and download "Blueretro+Ogx360 *.zip" file.
Use HW1 version as standard - no changes in wiring.
HW2 is for internal build.
## ESP32  part :
I am using esp32-wroom32d DevkitC V4. 
Dowload [flash download tool](https://www.espressif.com/sites/default/files/tools/flash_download_tool_3.9.5.zip) 
Run program and set :
Chip Type : ESP32
WorkMode : Develop

<img src="./Images/flash1.jpg" alt="flash1"/>  



Put all downloaded bin files into flash tool (3 dots), and set everything like on this screen.

<img src="./Images/flash2.jpg" alt="flash2"/>  

Change COM to Yours -  Determine what COM number Esp32 appears in the device manager.
Click START - and done.
## Arduino part :

* Download avrdude-6.3-mingw32.zip for Windows from  [avrdude](http://download.savannah.gnu.org/releases/avrdude/), and unzip it where you wish to.
*   Determine what COM number Arduino appears in the device manager.
* Go into command prompt in Windows, change folder to avrdudes folder
* write command:
avrdude -C avrdude.conf -F -p atmega32u4 -c avr109 -b 57600 -P COMx -Uflash:w:ogx360.hex:i
* !ALter COMx with Yours for arduino!
* Done.

 
## Connections

BE AVARE: ESP32 logic is 3,3Volt!.

Arduino pro micro (leonardo) needed for ogx360 is 16MHz version, 5Volt - that means his logic is also 5Volt!!.

Connecting those two, CAN DAMAGE YOUR ESP32!!!!

However it works - and the question is: for how much long??  It is not safe for esp32 3,3V logic.

For safety use Bidirectional Level shifter!!!

like this one :

<img src="./Images/Level Shifter.jpg" alt="flash2"/>  

Search for example "4 Channels IIC I2C Logic Level Converter Bi-Directional Module 3.3V to 5V Shifter for Arduino" in Your favourite shopping site.








Wiring


For Player 1: Connect right side pin6 (A2), and pin7 (A1) on the Arduino to ground(GND).

For Player 2: Connect right side pin7 (A1) on the Arduino to (GND).

For Player 3: Connect right side pin6 (A2) on the Arduino to (GND).

For Player 4: Don't connect any additional pins to ground.

<img src="./Images/Arduino pinouts.jpg" alt="flash2"/>  


Connect right side pin1 (RAW) from Arduino for Player1, with left side bottom (5V) pin on ESP32, and with Level shifter (HV).

Connect (GND) from Arduino for Player1 with Level shifter (GND) on HV side.

If You got more Arduino boards connect (GND) from all Arduino boards together.

Connect left side pin8 (D2) on all Arduinos together and with Level shifter (HV1)

Connect left side pin7 (D3) on all Arduinos together and with Level shifter (HV2)

Connect  ground pin (GND) from ESP32 with Level shifter (GND) on LV side. 

Connect right side pin22 (SCL) from Esp32 with Level shifter (LV1).

Connect right side pin21 (SDA) from Esp32 with Level shifter (LV2).


<img src="./Images/Esp32 pinouts.jpg" alt="flash2"/>  

From version 24.04 port detection and hotplug is working in HW2. HW1 wirings doesnt change.

HW2 need a lot of extra wirings -  not experienced in electronic users should stay with HW1.

For general info check darthcloud`s wiki about it: 

https://github.com/darthcloud/BlueRetro/wiki/BlueRetro-HW2-Internal-Install-Specification









