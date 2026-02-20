# Lesker275_logger
Arduino script to read pressure from a Lesker 275 convectron pressure gauge and store those pressures to an SD card

Lesker275_logger v0.0.2 11/21/2025

Purpose:

Store a time-marked log of pressure, as measured from the analog output of the convectron on GRIT, onto a micro SD card.
v0.0.2 updated for an OLED, and ADS1115. Must now be put on something like a mega for memory reasons. Sorry =)


Components:

- Arduino Mega or similar (I'm using too much memory for Uno)
- Lesker 275 convectron pressure gauge
- DS3231 RTC module
- CR2032 holder (usually comes with RTC module)
- uSD reader/writer module
- OLED display (128x64) 
- Status LEDS, 1 each of Red, Green, and Amber (Yellow)
- Resistive voltage divider. Should either be precision matched or tunable with a pot
- Various switches and basic components
