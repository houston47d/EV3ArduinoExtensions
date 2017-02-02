# EV3ArduinoExtensions
A cooperative project between Dan Miley, Steve Cogger, and myself (David Houston) to add capabilities to the Lego EV3 by connecting it to an Arduino.

The connection to the EV3 is via I2C because, well, that is the only connection you get.

The Arduino capabilities we added were to play music files from an SD card through a VS1053 (the Adafruit breakout board), play MIDI sounds (also through the VS1053), light up and animate Adafruit NeoPixels, and control servos.

There exists an EV3 extension that makes a set of blocks available to control the functions at a high level (not included here). Commands can also be sent using the low level I2C communications.

The final package included the SparkFun ProMicro and the Adafruit VS1053 + SD card Breakout board combined on a custom board with supporting circuitry for the I2C link, audio amplifier, power supply, and connectors for the servos and NeoPixels.

