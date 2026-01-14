#!/bin/bash
#
# Uninstall script for skymqttd
#

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
NC='\033[0m'

print_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

# Check if running as root
if [ "$EUID" -ne 0 ]; then 
    echo -e "${RED}[ERROR]${NC} Please run as root (sudo)"
    exit 1
fi

print_info "Uninstalling Skycharge MQTT Bridge..."

# Stop and disable service
if systemctl is-active --quiet skymqttd; then
    print_info "Stopping service..."
    systemctl stop skymqttd
fi

if systemctl is-enabled --quiet skymqttd 2>/dev/null; then
    print_info "Disabling service..."
    systemctl disable skymqttd
fi

# Remove systemd service
if [ -f /etc/systemd/system/skymqttd.service ]; then
    print_info "Removing systemd service..."
    rm /etc/systemd/system/skymqttd.service
    systemctl daemon-reload
fi

# Remove executable
if [ -f /usr/bin/skymqttd ]; then
    print_info "Removing executable..."
    rm /usr/bin/skymqttd
fi

# Remove Python module
if [ -d /usr/lib/python3/dist-packages/skycharge_mqtt ]; then
    print_info "Removing Python module..."
    rm -rf /usr/lib/python3/dist-packages/skycharge_mqtt
fi

# Remove documentation
if [ -d /usr/share/doc/skymqttd ]; then
    print_info "Removing documentation..."
    rm -rf /usr/share/doc/skymqttd
fi

# Ask about config and certificates
echo ""
read -p "Remove configuration and certificates? (y/N) " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    if [ -f /etc/skycharge/mqtt.json ]; then
        print_info "Removing configuration..."
        rm /etc/skycharge/mqtt.json
    fi
    if [ -d /etc/skycharge/certs ]; then
        print_info "Removing certificates..."
        rm -rf /etc/skycharge/certs
    fi
fi

echo ""
print_info "Uninstallation completed!"
