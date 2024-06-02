#!/bin/bash

echo "Setting up CAN interface..."

# Write to pinmux register settings to hardware registers
echo $PW | sudo busybox devmem 0x0c303018 w 0xc458
echo $PW | sudo busybox devmem 0x0c303010 w 0xc400

# Load the CAN kernel drivers
echo $PW | sudo modprobe can
echo $PW | sudo modprobe can_raw
echo $PW | sudo modprobe mttcan

# Set the interface properties
echo $PW | sudo ip link set can0 up type can bitrate 500000 dbitrate 1000000 berr-reporting on fd on