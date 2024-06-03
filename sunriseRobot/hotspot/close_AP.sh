#! /bin/bash

# 停止 hostapd
killall -q hostapd

# 关闭DHCP服务器
systemctl stop isc-dhcp-server

# 清除 wlan0 的地址
ip addr flush dev wlan0
sleep 0.5
ifconfig wlan0 down
sleep 1
ifconfig wlan0 up

# 重启 wpa_supplicant
systemctl restart wpa_supplicant

# 连接热点
# wifi_connect "WiFi-Test" "12345678"
