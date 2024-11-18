"Ready to go files"  for people who got problems with compilators, IDF etc(I know how annoying this could be), under Windows.
Just go to releases and download "BROgx360.zip" file.
Use HW1 version for external hardware build or HW2 for external/internal build.
## ESP32  part :
I am using esp32-wroom32d DevkitC V4. 
Dowload [flash download tool](https://www.espressif.com/sites/default/files/tools/flash_download_tool_3.9.5.zip) 
Run program and set :
Chip Type : ESP32
WorkMode : Develop

<img src="../Images/flash1.jpg" alt="flash1"/>  



Put all downloaded bin files into flash tool (3 dots), and set everything like on this screen.

<img src="../Images/flash2.jpg" alt="flash2"/>  

Change COM to Yours -  Determine what COM number Esp32 appears in the device manager.
Click START - and done.
## Arduino part :

* Download avrdude-6.3-mingw32.zip for Windows from  [avrdude](http://download.savannah.gnu.org/releases/avrdude/), and unzip it where you wish to.
*   Determine what COM number Arduino appears in the device manager.
*   Download firmware from https://github.com/konwektor/ogx360/releases/download/v1.00/Ogx360.zip
* Go into command prompt in Windows, change folder to avrdudes folder
* write command:
avrdude -C avrdude.conf -F -p atmega32u4 -c avr109 -b 57600 -P COMx -Uflash:w:firmware.hex:i
* !ALter COMx with Yours for arduino!
* Done.
