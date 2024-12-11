#!/bin/bash

ROOT=$(dirname $0)
sudo cp ${ROOT}/99-dji-psdk-custom.rules /etc/udev/rules.d/
cat /etc/udev/rules.d/99-dji-psdk-custom.rules
#
sudo udevadm control --reload-rules &&  sudo udevadm trigger 
