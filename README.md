# MiniPill LoRa Hardware version 1.1 and 1.2

Please look at https://www.iot-lab.org/blog/370/ for general information on this
project. In this this file I will share some software specific information.


## Update 2020-10-08
Added code so you can also use ABP as activation mechanisme. Please look in the source code for instructions.

## PlatformIO
Remember that this code is used in combination with the PlatformIO toolset.
This can be found at https://platformIO.org. I use the toolset in combination
with the Atom IDE on MacOS.

## Adding a custom board
To work with the MiniPill LoRa in more than one project you should add this custom
board to you PlatformIO toolset.
On MacOS under the user's homedirectory a .platformio is a directory available for
the toolset. This is platformio homedirectory. Please check for your OS where this is located.

- copy the *boards* and *variants* directory from customboard directory to the .platformio directory
- change the absolute path in the boards/minipill_l051c8_lora.json file for the variants path
- restart platformio/IDE

Now you should be able to use the MiniPill LoRa board in a new project.
Not all functions are tested with this custom board configurations.

## debugging
I use the hardwareSerial for debugging and connected a serial TTL-USB converter. This will take
some power in Low Power mode.
