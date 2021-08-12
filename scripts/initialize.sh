#!/bin/bash

echo Loading kernel modules...
cd /lib/modules/4.9.140-tegra/kernel/drivers/media/i2c/

echo ----------------------
ls
echo ---load max9295.ko-------------------
sudo insmod max9295.ko
echo ---load max9296.ko-------------------
sudo insmod max9296.ko
echo ---load isx021.ko--------------------
sudo insmod isx021.ko
echo Kernel module load completed.

echo Initializing trigger GPIO...
echo ---export GPIO 408-------------------
echo 408 > /sys/class/gpio/export
echo ---export GPIO 350-------------------
echo 350 > /sys/class/gpio/export
echo ---export GPIO 446-------------------
echo 446 > /sys/class/gpio/export
echo ---export GPIO 445-------------------
echo 445 > /sys/class/gpio/export
echo GPIO initilaization completed.
