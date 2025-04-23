#!/bin/bash

# This script is used to configure the DJI PSDK for the Jetson Orin AGX used in the M300

# Sync date and time with the host
./utils/sync_date.bash rafa

# Set the USB role to host
./utils/config_psdk.bash host

# Set multicast on for the loopback interface
sudo ip l set lo multicast on