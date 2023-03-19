#!/usr/bin/env bash

set -u
set -e

killall openocd || true
#make all
arm-none-eabi-objcopy -O binary dfm17.elf dfm17.bin

openocd -f ../../openocd/openocd.cfg &
sleep 3

set +e
t="telnet localhost 4444"
echo reset halt | $t
sleep 0.5
echo "stm32f1x mass_erase 0" | $t

sleep 2
echo "flash write_bank 0 dfm17.bin" | $t
sleep 2
echo "reset run" | $t
sleep 0.5

kill %1
