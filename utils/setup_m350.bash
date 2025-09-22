#!/bin/bash

# This script is used to configure the DJI PSDK for the Jetson Orin AGX used in the M300

# Sync date and time with the host
$HOME/project_insertion/utils/sync_date.bash guille

# Set the USB role to host
$HOME/project_insertion/utils/config_psdk.bash device

# Set multicast on for the loopback interface
sudo ip l set lo multicast on