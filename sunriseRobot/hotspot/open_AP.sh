#! /bin/bash

sleep 3
systemctl stop wpa_supplicant
ip addr flush dev wlan0
sleep 0.5
ifconfig wlan0 down
sleep 1
ifconfig wlan0 up

hostapd -B /etc/hostapd.conf
ifconfig wlan0 192.168.8.88 netmask 255.255.255.0
systemctl start isc-dhcp-server

