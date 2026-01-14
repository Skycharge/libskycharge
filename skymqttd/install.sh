#!/bin/bash
#
# Standalone installation script for skymqttd
# This installs skymqttd without requiring a full libskycharge build
#

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

print_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if running as root
if [ "$EUID" -ne 0 ]; then 
    print_error "Please run as root (sudo)"
    exit 1
fi

print_info "Skycharge MQTT Bridge - Standalone Installation"
echo ""

# Get the directory where the script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

# Check if we're in the skymqttd directory
if [ ! -f "skymqttd.py" ] || [ ! -d "skycharge_mqtt" ]; then
    print_error "Must be run from the skymqttd directory"
    exit 1
fi

# Check Python
print_info "Checking Python..."
if ! command -v python3 &> /dev/null; then
    print_error "Python3 not found. Install with: apt-get install python3"
    exit 1
fi
PYTHON_VERSION=$(python3 --version | awk '{print $2}')
print_info "Found Python ${PYTHON_VERSION}"

# Check dependencies
print_info "Checking dependencies..."
MISSING_DEPS=()

if ! python3 -c "import zmq" 2>/dev/null; then
    MISSING_DEPS+=("python3-zmq")
fi

if ! python3 -c "import docopt" 2>/dev/null; then
    MISSING_DEPS+=("python3-docopt")
fi

if ! python3 -c "import awscrt" 2>/dev/null; then
    print_warn "awsiotsdk not found (will install via pip)"
fi

if [ ${#MISSING_DEPS[@]} -ne 0 ]; then
    print_error "Missing dependencies: ${MISSING_DEPS[*]}"
    echo ""
    echo "Install with:"
    echo "  apt-get install ${MISSING_DEPS[*]}"
    exit 1
fi

# Install Python packages via pip if needed
print_info "Installing Python packages..."
if [ -f "requirements.txt" ]; then
    pip3 install -r requirements.txt
else
    pip3 install awsiotsdk
fi

# Install Python module
print_info "Installing skycharge_mqtt module..."
if [ -d "/usr/lib/python3/dist-packages/skycharge_mqtt" ]; then
    rm -rf /usr/lib/python3/dist-packages/skycharge_mqtt
fi
cp -r skycharge_mqtt /usr/lib/python3/dist-packages/
print_info "Module installed to /usr/lib/python3/dist-packages/"

# Install executable
print_info "Installing skymqttd executable..."
cp skymqttd.py /usr/bin/skymqttd
chmod +x /usr/bin/skymqttd
print_info "Executable installed to /usr/bin/skymqttd"

# Create config directory
print_info "Creating configuration directories..."
mkdir -p /etc/skycharge
mkdir -p /etc/skycharge/certs
chmod 700 /etc/skycharge/certs

# Install example config if not exists
if [ ! -f /etc/skycharge/mqtt.json ]; then
    if [ -f mqtt.json.example ]; then
        cp mqtt.json.example /etc/skycharge/mqtt.json
        chmod 600 /etc/skycharge/mqtt.json
        print_info "Example config installed to /etc/skycharge/mqtt.json"
    fi
fi

# Install systemd service
print_info "Installing systemd service..."
if [ -f skymqttd.service ]; then
    cp skymqttd.service /etc/systemd/system/skymqttd.service
    systemctl daemon-reload
    print_info "Service installed"
else
    print_warn "systemd service file not found, skipping"
fi

# Install documentation
print_info "Installing documentation..."
mkdir -p /usr/share/doc/skymqttd
cp README.md QUICKSTART.md AWS-IOT-POLICY.md INTEGRATION-SUMMARY.md /usr/share/doc/skymqttd/ 2>/dev/null || true
cp mqtt.json.example setup-aws-iot.sh /usr/share/doc/skymqttd/ 2>/dev/null || true

echo ""
print_info "Installation completed successfully!"
echo ""
echo "Next steps:"
echo "  1. Configure AWS IoT: ./setup-aws-iot.sh"
echo "     OR manually edit: /etc/skycharge/mqtt.json"
echo "  2. Start service: sudo systemctl start skymqttd"
echo "  3. Enable on boot: sudo systemctl enable skymqttd"
echo "  4. Check status: sudo systemctl status skymqttd"
echo "  5. View logs: sudo journalctl -u skymqttd -f"
echo ""
