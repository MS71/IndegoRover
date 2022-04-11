# IndegoRover
new MainBoard for Indego Lawn Mower

Schematic:
https://github.com/MS71/IndegoRover/blob/main/hw/indego_adapter_sch.pdf

PCB:
https://github.com/MS71/IndegoRover/blob/main/hw/indego_adapter_brd.pdf

Parts:
* ESP32 (S2/S3) Module (https://github.com/MS71/ROS2Mower/blob/master/hw/esp32_module.pdf)
* ARV Mega 128 for low level IO, security and motor driver
* origonal Indego connectors (26SHC-B-1A => mouser)
* DC motor driver: 3 * VNH2SP30 module (41V, 30A H Bridge)
* GPS receiver (ZED-F9P) (https://www.ardusimple.com/product/simplertk2blite/)
* 5 pin header for low cost, non RTK GPS receiver
* 10 Pin RPI Connector (5V, 3.3V, GND, I2C, UART)
* 10 Pin I2C Bus Connector
* 6 Pin I2C Bus Connector
* 5V Step Down Converter
* Bat Charger Circuit (Compatible with Indego Charge Station, difficult)
* Bat Charger Circuit (Compatible with std. 36V solar panel)
* Coil Sensor Amp
* low power + standby
* support for all available sensors
* BNO055 IMU module headers
* support for all sensors (bumper, coil fensem ...)
* power supply for onboard computer
* beeper
* support for LCD and keyboard (difficult)
* ...

onboard ROS2 computer: (optional)
* RPI4 or RPI Zero 2W headers 
* Orange Pi Zero 2 headers (http://www.orangepi.org/Orange%20Pi%20Zero2/)
* ...

Other:
* made with free Eagle  (max. 80 qcm) or kicad6
