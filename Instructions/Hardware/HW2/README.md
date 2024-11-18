
BROgx HW2 build instruction

[Detailed Schematic](BROgx_HW2.pdf) 

Lets assume we are using also like in HW1 build :ESP32-DevKitC V4, and Arduino ProMicro/leonardo 
## Connections




Same story as in HW1 - for communication over i2c beetwen 2 devices with different SDA/SCL voltage level (arduino accepts 5V, Esp32 3,3V) we need to use Bidirectional Level shifter!!!

For example [SparkFun Logic Level Converter - Bi-Directional](https://www.sparkfun.com/products/12009)


Wiring


For Player 1: Connect right side pin6 (A2), and pin7 (A1) on the Arduino to ground(GND).

For Player 2: Connect right side pin7 (A1) on the Arduino to (GND).

For Player 3: Connect right side pin6 (A2) on the Arduino to (GND).

For Player 4: Don't connect any additional pins to ground.

<img src="../../Images/Arduino pinouts.jpg" alt="flash2"/>  


Connect right side pin4 (Vcc) from Arduino for Player1,  with Level shifter (HV).

Connect (GND) from Arduino for Player1 with Level shifter (GND) on HV side.

If You got more Arduino boards connect (GND) from all Arduino boards together.

Connect left side pin8 (D2/SDA) on all Arduinos together and with Level shifter (HV1)

Connect left side pin7 (D3/SCL) on all Arduinos together and with Level shifter (HV2)

Connect left side top (3V3) pin on ESP32 with Level shifter (LV)

Connect  ground pin (GND) from ESP32 with Level shifter (GND) on LV side. 

Connect right side GPIO21 (SDA) from Esp32 with Level shifter (LV1).

Connect right side GPIO22 (SCL) from Esp32 with Level shifter (LV2).
TODO:
FIXIN
IN CODE is mistake  sda at gpio22 /scl at gpio21 - 
NEED TO BE SDA21/SCL22
arduino sda is d2 /  scl is d3
picture below  to replace with correct one

<img src="../../Images/Esp32 pinouts.jpg" alt="flash2"/>  


## GAMEPORTS

Arduino player 1:
USB_D+ connect together with Green cable from gameport1 - Pin3  Mainboard plug on the left
Usb_D- connect with white cable from gameport1 - Pin2 Mainboard plug on the left
Black (GND)cable coming from gameport1 to Mainboard plug on the left at Pin5 - take it out/disconnect from plug -> connect Arduino GND to plug Pin5 
Join end of Black cable with ESP32 GPIO35 -> from GPIO35 Connect 10K resistor to esp32_3.3V3

Port detection explaination:
    Port detection pin for player1 - GPIO35 iS pulled HI thru resistor - other end of black cable is connected to  GND pin on gameport1 -> nothing happens.
    Standard xbox plugs are having internal connection between GND pin and plug shield.
    If wired gamepad is plugged in, our Black cable is connected to GND, thru GND pin -> gamepad plug GND pin -> gamepad plug shield - > xbox gameport shieldi.
    In this way GPIO35 input HI is pulled down - when port detect signal is LOW, adapter is disabling coresponding port, sending USB DETACH command to ARDUINO, and moves connected BT gamepads
    to next available port. When wired controller is plugged out, signal on GPIO35 goes back to HI, port is set to enabled, BT gamepad is jumbing back to his old place and so on.
    Tested on couple of ogx Dukes, and s controller.
    CONS - if GND and shield in plug is not connected internally, this will not work/ port shield and xbox shield "touch" point need to have good contact.
    Other way for port detection without "shield trick" is adding "current mirror" circuit designed by DarthCloud. look at [Schematic](BROgx_HW2.pdf).

Arduino player 2: 
    USB_D+ connect together with Green cable from gameport1 - Pin10  Mainboard plug on the left
    Usb_D- connect with white cable from gameport2 - Pin7 Mainboard plug on the left
    Black (GND)cable coming from gameport1 to Mainboard plug on the left at Pin11 - take it out/disconnect from plug -> connect Arduino GND to plug Pin11 
    Join end of Black cable with ESP32 GPIO36 -> from GPIO36 Connect 10K resistor to esp32_3.3V3

Arduino player 3:
    USB_D+ connect together with Green cable from gameport3 - Pin3  Mainboard plug on the right
    Usb_D- connect with white cable from gameport3 - Pin2 Mainboard plug on the right
    Black (GND)cable coming from gameport3 to Mainboard plug on the right at Pin5 - take it out/disconnect from plug -> connect Arduino GND to plug Pin5 
    Join end of Black cable with ESP32 GPIO32 -> from GPIO32 Connect 10K resistor to esp32_3.3V3

Arduino player 4:
USB_D+ connect together with Green cable from gameport4 - Pin10 Mainboard plug on the right
Usb_D- connect with white cable from gameport4 - Pin7 Mainboard plug on the right
Black (GND)cable coming from gameport4 to Mainboard plug on the right at Pin11 - take it out/disconnect from plug -> connect Arduino GND to plug Pin11 
Join end of Black cable with ESP32 GPIO33 -> from GPIO33 Connect 10K resistor to esp32_3.3V3

## FRONT BUTTON/LED`S panel

From plug coming from buttons/led panel connected to mainboard (Double_row_9_pins) Disconnect/pull out cables from pin2/SW2_POWER and from pin4/SW1_EJECT.
In empty places put:
- At pin2/ GPIO13 from esp32 together with ANODE from schotky diode Katode to SW2_POWER cable disconnected from this pin -> from GPIO13 10K pull_up resistor to esp32_3.3V 
- At pin4/GPIO14 -> from GPIO14 10K pull_up resistor to esp32_3.3V
- esp32 GPIO0/BOOT pin together with SW1_EJECT cable disconnected from pin4
- esp32 EN/CHIP_PU Connect thru 10K resistor to esp32_3.3V3 -> also connect EN/CHIP_PU pin at point where schotky diode KATODE is connected with  SW2_POWER cable 








thtt


