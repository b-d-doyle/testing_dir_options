# MKS925_serialRead
Arduino script to read pressures from an MKS 925 and display them to an OLED screen.

MKS925_serialRead v0.0.1: Controller Software for MKS 925 MicroPirani Pressure Transducer

Description:

It just reads via Serial and displays to OLED. No funnybusiness.
I may add some funnybusiness later though, who knows?

Devices and connections:
- MKS 925 via hardware serial (using a level shifter)
- OLED screen (128x32) via I2C

The level shifter should be something like a MAX3232. Here's a breakout board:
https://www.digikey.com/en/products/detail/adafruit-industries-llc/5987/24713837
