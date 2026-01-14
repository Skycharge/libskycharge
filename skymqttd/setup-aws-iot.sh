#!/bin/bash
#
# AWS IoT Core setup script for Skycharge devices
# This script helps provision a new Skycharge device in AWS IoT Core
#

set -e

# Configuration
POLICY_NAME="SkychargeDevicePolicy"
CERT_DIR="/etc/skycharge/certs"
CONFIG_FILE="/etc/skycharge/mqtt.json"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

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

# Check if AWS CLI is installed
if ! command -v aws &> /dev/null; then
    print_error "AWS CLI not installed. Install with: pip3 install awscli"
    exit 1
fi

# Check AWS credentials
if ! aws sts get-caller-identity &> /dev/null; then
    print_error "AWS credentials not configured. Run: aws configure"
    exit 1
fi

print_info "AWS IoT Core Setup for Skycharge"
echo ""

# Get device UUID from skycharge.conf
if [ -f /etc/skycharge.conf ]; then
    DEVICE_UUID=$(grep -E "^device-uuid\s*=" /etc/skycharge.conf | awk -F= '{print $2}' | tr -d ' ')
    if [ -z "$DEVICE_UUID" ]; then
        print_warn "device-uuid not found in skycharge.conf, generating random UUID"
        DEVICE_UUID=$(uuidgen | tr '[:upper:]' '[:lower:]')
    fi
else
    print_warn "skycharge.conf not found, generating random UUID"
    DEVICE_UUID=$(uuidgen | tr '[:upper:]' '[:lower]')
fi

THING_NAME="skycharge-${DEVICE_UUID}"
print_info "Device: ${THING_NAME}"

# Get AWS region
AWS_REGION=$(aws configure get region)
if [ -z "$AWS_REGION" ]; then
    print_error "AWS region not configured"
    exit 1
fi
print_info "Region: ${AWS_REGION}"

# Get AWS account ID
AWS_ACCOUNT=$(aws sts get-caller-identity --query Account --output text)
print_info "Account: ${AWS_ACCOUNT}"

# Get IoT endpoint
IOT_ENDPOINT=$(aws iot describe-endpoint --endpoint-type iot:Data-ATS --query endpointAddress --output text)
print_info "Endpoint: ${IOT_ENDPOINT}"

echo ""
read -p "Continue with setup? (y/n) " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    exit 0
fi

# Create certificates directory
print_info "Creating certificates directory..."
mkdir -p ${CERT_DIR}
chmod 700 ${CERT_DIR}

# Create IoT Thing
print_info "Creating IoT Thing: ${THING_NAME}"
if aws iot describe-thing --thing-name ${THING_NAME} &> /dev/null; then
    print_warn "Thing already exists"
else
    aws iot create-thing --thing-name ${THING_NAME}
    print_info "Thing created"
fi

# Create certificate and keys
print_info "Creating certificate and keys..."
CERT_OUTPUT=$(aws iot create-keys-and-certificate \
    --set-as-active \
    --certificate-pem-outfile ${CERT_DIR}/device-cert.pem \
    --public-key-outfile ${CERT_DIR}/device-public.key \
    --private-key-outfile ${CERT_DIR}/device-key.pem \
    --output json)

CERT_ARN=$(echo $CERT_OUTPUT | python3 -c "import sys, json; print(json.load(sys.stdin)['certificateArn'])")
print_info "Certificate created: ${CERT_ARN}"

# Set proper permissions
chmod 600 ${CERT_DIR}/device-cert.pem
chmod 600 ${CERT_DIR}/device-key.pem
chmod 644 ${CERT_DIR}/device-public.key

# Download Amazon Root CA
print_info "Downloading Amazon Root CA..."
curl -s https://www.amazontrust.com/repository/AmazonRootCA1.pem \
    -o ${CERT_DIR}/AmazonRootCA1.pem
chmod 644 ${CERT_DIR}/AmazonRootCA1.pem

# Create policy if it doesn't exist
print_info "Creating/checking IoT policy..."
if aws iot get-policy --policy-name ${POLICY_NAME} &> /dev/null; then
    print_warn "Policy already exists"
else
    # Create policy
    cat > /tmp/iot-policy.json << EOF
{
  "Version": "2012-10-17",
  "Statement": [
    {
      "Effect": "Allow",
      "Action": ["iot:Connect"],
      "Resource": ["arn:aws:iot:${AWS_REGION}:${AWS_ACCOUNT}:client/skycharge-*"]
    },
    {
      "Effect": "Allow",
      "Action": ["iot:Subscribe"],
      "Resource": ["arn:aws:iot:${AWS_REGION}:${AWS_ACCOUNT}:topicfilter/skycharge/*/cmd/*"]
    },
    {
      "Effect": "Allow",
      "Action": ["iot:Receive"],
      "Resource": ["arn:aws:iot:${AWS_REGION}:${AWS_ACCOUNT}:topic/skycharge/*/cmd/*"]
    },
    {
      "Effect": "Allow",
      "Action": ["iot:Publish"],
      "Resource": [
        "arn:aws:iot:${AWS_REGION}:${AWS_ACCOUNT}:topic/skycharge/*/status",
        "arn:aws:iot:${AWS_REGION}:${AWS_ACCOUNT}:topic/skycharge/*/telemetry",
        "arn:aws:iot:${AWS_REGION}:${AWS_ACCOUNT}:topic/skycharge/*/events",
        "arn:aws:iot:${AWS_REGION}:${AWS_ACCOUNT}:topic/skycharge/*/heartbeat",
        "arn:aws:iot:${AWS_REGION}:${AWS_ACCOUNT}:topic/skycharge/*/cmd/*/response"
      ]
    }
  ]
}
EOF
    
    aws iot create-policy \
        --policy-name ${POLICY_NAME} \
        --policy-document file:///tmp/iot-policy.json
    
    rm /tmp/iot-policy.json
    print_info "Policy created"
fi

# Attach policy to certificate
print_info "Attaching policy to certificate..."
aws iot attach-policy \
    --policy-name ${POLICY_NAME} \
    --target ${CERT_ARN}

# Attach certificate to thing
print_info "Attaching certificate to thing..."
aws iot attach-thing-principal \
    --thing-name ${THING_NAME} \
    --principal ${CERT_ARN}

# Create MQTT configuration
print_info "Creating MQTT configuration..."
cat > ${CONFIG_FILE} << EOF
{
  "device_id": "${THING_NAME}",
  "endpoint": "${IOT_ENDPOINT}",
  "port": 8883,
  "cert_file": "${CERT_DIR}/device-cert.pem",
  "key_file": "${CERT_DIR}/device-key.pem",
  "ca_file": "${CERT_DIR}/AmazonRootCA1.pem",
  "skycharged_endpoint": "tcp://localhost:21103",
  "keep_alive": 60,
  "status_interval": 30,
  "telemetry_interval": 10
}
EOF

chmod 600 ${CONFIG_FILE}

echo ""
print_info "Setup completed successfully!"
echo ""
echo "Summary:"
echo "  Thing Name: ${THING_NAME}"
echo "  Certificate: ${CERT_DIR}/device-cert.pem"
echo "  Configuration: ${CONFIG_FILE}"
echo ""
echo "Next steps:"
echo "  1. Start the service: systemctl start skymqttd"
echo "  2. Enable on boot: systemctl enable skymqttd"
echo "  3. Check logs: journalctl -u skymqttd -f"
echo ""
