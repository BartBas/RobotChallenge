#!/bin/bash

# Define variables for easy editing
SSID="RobotPiGr1"
PASSWORD="group1234"
CONN_NAME="Pi-AP"

echo "Creating WiFi Hotspot: $SSID..."

# Create the hotspot
sudo nmcli device wifi hotspot ssid "$SSID" password "$PASSWORD" con-name "$CONN_NAME"

# Ensure it connects automatically on boot
echo "Setting autoconnect to yes..."
sudo nmcli connection modify "$CONN_NAME" connection.autoconnect yes

# Show active connections to verify
echo "--------------------------------"
echo "Current Active Connections:"
nmcli connection show --active
