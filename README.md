# Power-board
Power Management and distribution board for Open Dynamic Robot Initiative Robots

##Main feature

- 40A 32V DC load switching
- Inrush current limiting
- Monitor voltage, Current and Energy
- Under voltage, Over current shutdown with settable thresholds
- Telemetry, load switching via SPI

##Wiring 

####J1 - Programming port
Serial Wire Debug (SWD)

| 1 | 2 | 3 | 4 |
|:---:|:---:|:---:|:---:|
| 3V3 |  SWDIO | SWCLK | GND |

####J2 - SPI
Communication port, power board is acting as a SPI Slave device. To be connected to a SPI Master device such as the MasterBoard.

| 1 | 2 | 3 | 4 | 5 |
|:---:|:---:|:---:|:---:|:---:|
| GND | MISO | CLK | /CS | MOSI |

#### J3 - DC Input
DC input port to be connected to a battery pack / PSU using female XT30 connector

#### J4-J12 - DC outputs
DC output ports to be connected to any load on the robot using XT30 male connector

#### J13 External On/Off pushbutton
| 1 | 2 | 3 |
|:---:|:---:|:---:|
| OFF | COM | ON |

Connect a

- "Power OFF" push button NORMALLY CLOSED between 1 and 2
- "Power ON" push button NORMALLY OPEN between 2 and 3

During programming, short pins 1-2-3 together for the MCU to stay powered at anytime.

If no external pushbutton are used, short pin 1 and 2 to only use onboard pushbuttons. If not, the board will not turn on.

Note: the onboard OFF button control to the MCU reset line.

## Communication protocol
####Sensor packet
todo
#### Command packet
Not implemented yet

## Flashing firmware
todo
