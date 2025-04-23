#!/bin/bash

# This script is used to configure the DJI PSDK for the Jetson Orin boards

# Input argument to select the USB role
if [ $# -ne 1 ]; then
    echo "Usage: $0 <usb_role>"
    echo "usb_role: device or host"
    exit 1
fi

usb_role=$1

# Check if the input argument is valid
if [ "$usb_role" != "device" ] && [ "$usb_role" != "host" ]; then
    echo "Invalid usb_role: $usb_role"
    echo "Valid options are: device or host"
    exit 1
fi

# If the input argument is valid, proceed with the configuration
echo "Configuring PSDK for USB role: $usb_role"
# if input is device, set the USB role to device
if [ "$usb_role" == "device" ]; then
    echo device | sudo tee /sys/class/usb_role/usb2-0-role-switch/role

    sudo ifconfig usb0 192.168.1.1 netmask 255.255.255.0 up

    # check if the l4tbr0 interface is up
    echo "checking if the l4tbr0 interface is up:"
    ifconfig | grep l4tbr0
else
    echo host | sudo tee /sys/class/usb_role/usb2-0-role-switch/role

    # check if device is available
    echo "checking if the device is available:"
    ls /dev/ttyACM*
fi

