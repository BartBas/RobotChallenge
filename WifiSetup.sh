#!/bin/bash

# Check if running as root
if [ "$EUID" -ne 0 ]; then
  echo "Please run as root (sudo ./WifiSetup.sh)"
  exit
fi

echo "--- 1. Cleaning up Legacy Conflicts ---"
# Stop and mask old services that fight with NetworkManager
systemctl stop hostapd dnsmasq dhcpcd 2>/dev/null
systemctl disable hostapd dnsmasq dhcpcd 2>/dev/null
systemctl mask hostapd dnsmasq dhcpcd 2>/dev/null

echo "--- 2. Disabling Onboard Wi-Fi (Hardware Level) ---"
# Pi 5 firmware path is usually /boot/firmware/config.txt
CONFIG_PATH="/boot/firmware/config.txt"
if [ ! -f "$CONFIG_PATH" ]; then CONFIG_PATH="/boot/config.txt"; fi

if ! grep -q "dtoverlay=disable-wifi" "$CONFIG_PATH"; then
    echo "dtoverlay=disable-wifi" >> "$CONFIG_PATH"
    echo "Onboard Wi-Fi disabled in $CONFIG_PATH"
fi

echo "--- 3. Installing Driver Dependencies ---"
apt update
apt install linux-headers-$(uname -r) build-essential bc dkms git -y

echo "--- 4. Installing BrosTrend (RTL88x2BU) Drivers ---"
# Note: Keeping your driver logic as it was working
rm -rf /tmp/88x2bu
git clone https://github.com/morrownr/88x2bu-20210702.git /tmp/88x2bu
cd /tmp/88x2bu
./install-driver.sh <<EOF
y
n
EOF

echo "--- 5. Configuring NetworkManager Hotspot (wlan1) ---"
# Ensure NetworkManager is managing devices
sed -i 's/managed=false/managed=true/g' /etc/NetworkManager/NetworkManager.conf
systemctl restart NetworkManager

# Delete any existing RobotAP profile to start fresh
nmcli connection delete RobotAP 2>/dev/null

# Create the Hotspot on the USB adapter (wlan1)
# Using 'shared' method automatically handles DNS (dnsmasq) and DHCP
nmcli con add type wifi ifname wlan1 con-name RobotAP autoconnect yes ssid robotpigr1
nmcli con modify RobotAP 802-11-wireless.mode ap 802-11-wireless-security.key-mgmt wpa-psk 802-11-wireless-security.psk group1234
nmcli con modify RobotAP ipv4.method shared ipv4.addresses 192.168.4.1/24

echo "--- 6. Setting Local Hostname ---"
if ! grep -q "192.168.4.1 robot.pi" /etc/hosts; then
    echo "192.168.4.1 robot.pi" >> /etc/hosts
fi

echo "--------------------------------------------------------"
echo "SETUP COMPLETE."
echo "The Hotspot 'robotpigr1' will activate on wlan1 (USB)."
echo "Internal Wi-Fi (wlan0) has been disabled."
echo "--------------------------------------------------------"

sleep 5
reboot
