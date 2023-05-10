#!/bin/bash

#initialisation of CAN interface on BBAI
#assumes proper configuration on CAN1 (see https://stackoverflow.com/questions/62207737/beaglebone-ai-how-to-setup-can-bus)

echo "Enabling CAN1"
sudo ip link set can1 up type can bitrate 1000000
sudo ifconfig can1 up
#sudo ifconfig can1 txqueuelen 1000

#echo "can1 up. Dumping (ctrl+c to close):"
#candump -c -t z can1,080~111111 #Filter out 080 sync messages