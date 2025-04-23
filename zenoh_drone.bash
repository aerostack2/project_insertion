#!/bin/bash
IP="192.168.0.33"

echo "Debug: zenoh-bridge-ros2dds -e tcp/$IP:7447 -c config/zenoh_config_drone.json5"

zenoh-bridge-ros2dds -e tcp/$IP:7447 -c config/zenoh_config_drone.json5
