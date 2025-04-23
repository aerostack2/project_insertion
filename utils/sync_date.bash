#!/bin/bash

# This script is used to sync date and time between the Jetson Orin board and a host

# Input argument to select the host
if [ $# -ne 1 ]; then
    echo "Usage: $0 <host>"
    echo "host: rafa or others"
    exit 1
fi

host=$1

echo "Syncing date and time with host: $host"
sudo ntpdate $host