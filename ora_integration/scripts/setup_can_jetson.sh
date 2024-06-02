#!/bin/bash

# Write to pinmux register settings to hardware registers
sudo busybox devmem 0x0c303018 w 0x458
sudo busybox devmem 0x0c303010 w 0xc400

# Load the CAN kernel drivers
sudo modprobe can
sudo modprobe can_raw
sudo modprobe mttcan

# Set the interface properties
sudo ip link set can0 up type can bitrate 500000 dbitrate 1000000 berr-reporting on fd on