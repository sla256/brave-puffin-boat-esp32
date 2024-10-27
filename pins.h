/*

*** Analog temperature sensor TMP36
left pin to 3.3V (red inside the battery pack)
middle pin to A0 on ADS1115 (yellow inside the battery pack)
right pin to GND (common ground)

*** AD converter ADS1115, I2C address 0x48
I2C connection on 3.3V


*** NEO 6M GPS
Uses UART2 via custom defined pins
GPS's TX connects to UART2's RX on pin 2
GPS's RX connects to UART2's TX on pin 0


*** Status LED
built in on GPIO 25

*** SD card breakout board
CS to GPIO 5
DI (MOSI) to GPIO 23
DO (MISO) to GPIO 19
CLK to GPIO 18
3.3V and GND 

*** CMPS14 - compass and accelerometer, gyroscope not used; I2C address 0x60
Mounted such that its native direction is 90 degrees to the left of boat's long axis
I2C protocol (Wire) pins:
SDA (pin 2 on CMPS) to GPIO 21 (5V)
SCL (pin 3) to GPIO 22 (5V)
VCC on pin 1 (5V)
GND last pin 6


*** BME280 temperature/humidity/pressure sensors, I2C
VCC 3.3V (off ESP32)
SCL to pin 22 (3.3V)
SDA to pin 21 (3.3V)
Onboard sensor I2C address: 0x76 (default)


*** INA219 I2C current sensor, 2x, high side of ESC's for both motors
INA219_I2C_ADDRESS1: 0x45 - left motor
INA219_I2C_ADDRESS4: 0x44 - right motor
C (blue) - SCL (pin 22)
D (green) - SDA (pin 21)


*** INA226 current & voltage sensor for battery draw and incoming solar current
INA226_1
- main buttery draw current and bus voltage
- I2C address 0x40 (default)
- high of all loads. IN+ to battery's positive terminal, IN- to all loads
- Connect V+ (or Vbus) to IN+
INA226_2
- solar charging current and voltage from MPPT controller
- I2C address 0x4C (keep A0 pad G soldered to ground; clear solder on A1 G from ground; solder A1 L to ground)
-- see https://github.com/RobTillaart/INA226/tree/master?tab=readme-ov-file#address
- high relative to battery's charging IN connection


*** BLUE MX SR3100 DSM2 RF receiver (remote control mode)
GPIO 36: steering (marked STR on the receiver) (5V)
GPIO 37: throttle (marked AUX on the receiver) (5V)
5V

Iridium RockBLOCK 9603
Pinout: https://learn.adafruit.com/using-the-rockblock-iridium-modem?view=all#pinout-3058779
Also https://learn.adafruit.com//assets/89520
Connector: Molex PicoBlade

GND to pin 10 on RockBLOCK (left most pin on the modem i.e. closest to the edge)
5V to pin 8
GPIO 33 (TX3) to pin 6 (blue wire, labeled TXD on modem's pinout - input into 9603; safe to connect directly to 3.3V pins on ESP32 w/o level shifter)
GPIO 32 (RX3) to pin 1 (yellow wire, labeled RXD, output from 9603)
GPIP 26 to pin 7 (white wire, labeled SLP) - to be pulled to ground to switch the modem off

*** Mission control switch
PIN 14 - pulled up pin, expecting normally open
Wired to a reed switch
When pulled LOW / grounded (via reed switch), allows initial mission control operation such as launch
During testing controls simulation

*/

#define PIN_RX2 2
#define PIN_TX2 0

// 4 used for display

#define PIN_SD_CS 5

#define PIN_MOTOR_L 12
#define PIN_MOTOR_R 13

#define PIN_HEARTBEAT_SIGNAL 14

// 16 used for display
// 18, 19, 23 used for SPI (SD card)
// 21, 22 used for I2C

#define PIN_STATUS_LED 25

#define PIN_SAT_SLEEP  26

#define PIN_TOUCH_DISPLAY 27

#define PIN_SAT_RX 32
#define PIN_SAT_TX 33

#define PIN_DSM2_INPUT_STEERING 36
#define PIN_DSM2_INPUT_THROTTLE 37

#define PIN_TMP36_MAIN_BAT  38

// 34-39 input only