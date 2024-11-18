
BROgx HW1 build instruction


## Connections
[Detailed Schematic](BROgx_HW1.pdf)

BE AVARE: ESP32 logic is 3,3Volt!.

Arduino pro micro (leonardo) needed for ogx360 is 16MHz version, 5Volt - that means his logic is also 5Volt!!.

Connecting those two, CAN DAMAGE YOUR ESP32!!!!

However it works - and the question is: for how much long??  It is not safe for esp32 3,3V logic.

For safety use Bidirectional Level shifter!!!

like this one :

<img src="../../Images/Level Shifter.jpg" alt="flash2"/>  

Search "4 Channels IIC I2C Logic Level Converter Bi-Directional Module 3.3V to 5V Shifter for Arduino" in Your favourite shopping site.

For example [SparkFun Logic Level Converter - Bi-Directional](https://www.sparkfun.com/products/12009)




Wiring


For Player 1: Connect right side pin6 (A2), and pin7 (A1) on the Arduino to ground(GND).

For Player 2: Connect right side pin7 (A1) on the Arduino to (GND).

For Player 3: Connect right side pin6 (A2) on the Arduino to (GND).

For Player 4: Don't connect any additional pins to ground.

<img src="../../Images/Arduino pinouts.jpg" alt="flash2"/>  


Connect right side pin1 (RAW) from Arduino for Player1, with left side bottom (5V) pin on ESP32, and with Level shifter (HV).

Connect (GND) from Arduino for Player1 with Level shifter (GND) on HV side.

If You got more Arduino boards connect (GND) from all Arduino boards together.

Connect left side pin8 (D2/SDA) on all Arduinos together and with Level shifter (HV1)

Connect left side pin7 (D3/SCL) on all Arduinos together and with Level shifter (HV2)

Connect left side top (3V3) pin on ESP32 with Level shifter (LV)

Connect  ground pin (GND) from ESP32 with Level shifter (GND) on LV side. 

Connect right side GPIO21 (SDA) from Esp32 with Level shifter (LV1).

Connect right side GPIO22 (SCL) from Esp32 with Level shifter (LV2).
IN CODE sda 22 /scl21 - 
NEED TO BE SDA21/SCL22
arduino sda d2 /  scl d3






<img src="../../Images/Esp32 pinouts.jpg" alt="flash2"/>  

Connect every used arduino board to xbox with XBOX Micro USB to Controller Port Cable.

<img src="../../Images/Original-XBOX-Micro-USB-to-Controller-Port-Cable.jpg" alt="flash2"/>  

Enjoy.











