#! /bin/bash
USB_CAN_DEVICE=$(dmesg | grep -o 'ttyACM[0-9]*'| tail -n 1)

# Check if the USB-CAN device was found
if [ -z "$USB_CAN_DEVICE" ]; then
    echo "USB-to-CAN adapter not found!"
    exit 1
fi

sudo modprobe can
sudo modprobe can_raw
#sudo modprobe slcan
sudo slcand -o -c -s8 "$USB_CAN_DEVICE" can0
sudo ifconfig can0 txqueuelen 1000
sudo ip link set can0 up


