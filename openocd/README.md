For command-line junkies, once you've built once with STM32CubeIDE you can build and install at the CLI using ```make``` and ```openocd```.  Here's how.

How to:
* Install packages: ```sudo apt install openocd gcc-arm-none-eabi```
* Build: ```cd dfm17/Release``` and then run ```make all```
* Convert from elf to bin: ```arm-none-eabi-objcopy -O binary dfm17.elf dfm17.bin```
* Run openocd which talks to the STLinkv2 programmer: ```openocd -f ../../openocd/openocd.cfg```
* In another shell, run: ```telnet localhost 4444``` and run
```
reset halt
flash write_bank 0 dfm17.bin
reset run
```

Much of the commands here are borrowed from: https://nx3d.org/gnuk-st-link-v2/
