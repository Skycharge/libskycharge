# Skycharge MQTT Bridge (skymqttd)

Standalone Python daemon for connecting Skycharge devices to AWS IoT Core.

**Requirements**: Python 3.5+ (compatible with BeagleBone's Python 3.5)

## What It Does

- Remote monitoring of charging status and telemetry
- Remote control via MQTT commands  
- Event notifications
- Cloud integration with AWS IoT Core

## Architecture

```
┌─────────────────────────────────────┐
│          AWS IoT Core               │  (Cloud)
│          MQTT Broker                │
└───────────────┬─────────────────────┘
                │ MQTT/TLS (8883)
┌───────────────▼─────────────────────┐
│      BeagleBone Device              │
│                                     │
│  ┌──────────────────────────────┐  │
│  │   skymqttd (Python)          │  │  ← Install Here
│  │   - paho-mqtt                │  │
│  │   - Command routing          │  │
│  └──────────┬───────────────────┘  │
│             │ ZeroMQ                │
│             │ localhost:21103       │
│  ┌──────────▼───────────────────┐  │
│  │   skycharged (C)             │  │
│  │   - Local hardware control   │  │
│  └──────────┬───────────────────┘  │
│             │                       │
│      Charging Hardware              │
└─────────────────────────────────────┘
```

## Quick Start

### 1. Copy to BeagleBone

```bash
# From your desktop
scp -r skymqttd/ root@<beaglebone-ip>:/root/

# SSH to BeagleBone
ssh root@<beaglebone-ip>
cd /root/skymqttd
```

### 2. Install Dependencies

```bash
# Fix any broken packages first
apt --fix-broken install

# Install system packages
apt-get update
apt-get install python3 python3-zmq python3-docopt

# Install pip for Python 3.5
wget https://bootstrap.pypa.io/pip/3.5/get-pip.py
python3 get-pip.py
rm get-pip.py

# Install Python packages
pip3 install -r requirements.txt
```

**Note**: If you encounter dependency conflicts with debug packages, you can safely remove them:
```bash
apt-get remove skycharge-cli-dbgsym skycharged-dbgsym skyhttpd-dbgsym skyuartd-dbgsym
```

### 3. Run Installation

```bash
./install.sh
```

The script installs:
- Python package to `/usr/lib/python3/dist-packages/`
- Executable to `/usr/bin/skymqttd`
- systemd service
- Configuration directory `/etc/skycharge/`

### 4. Configure AWS IoT

**Option A: Automated (Recommended)**
```bash
./setup-aws-iot.sh
```

**Option B: Manual**
1. Create IoT Thing in AWS Console
2. Generate certificates
3. Download Root CA
4. Create policy (see AWS-IOT-POLICY.md)
5. Edit `/etc/skycharge/mqtt.json`

### 5. Start Service

```bash
systemctl start skymqttd
systemctl enable skymqttd
journalctl -u skymqttd -f
```

## Configuration

Edit `/etc/skycharge/mqtt.json`:

```json
{
  "device_id": "skycharge-{uuid}",
  "endpoint": "xxx.iot.us-east-1.amazonaws.com",
  "port": 8883,
  "cert_file": "/etc/skycharge/certs/device-cert.pem",
  "key_file": "/etc/skycharge/certs/device-key.pem",
  "ca_file": "/etc/skycharge/certs/AmazonRootCA1.pem",
  "skycharged_endpoint": "tcp://localhost:21103",
  "keep_alive": 60,
  "status_interval": 30,
  "telemetry_interval": 10
}
```

## MQTT Topics

### Commands (Subscribe)
- `skycharge/{device-id}/cmd/get-status`
- `skycharge/{device-id}/cmd/get-params`
- `skycharge/{device-id}/cmd/set-params`
- `skycharge/{device-id}/cmd/resume-scan`
- `skycharge/{device-id}/cmd/stop-scan`
- `skycharge/{device-id}/cmd/open-droneport`
- `skycharge/{device-id}/cmd/close-droneport`
- `skycharge/{device-id}/cmd/reset-device`

### Data (Publish)
- `skycharge/{device-id}/status` - Status updates (every 30s)
- `skycharge/{device-id}/telemetry` - Telemetry data (every 10s)
- `skycharge/{device-id}/events` - Event notifications
- `skycharge/{device-id}/cmd/{command}/response` - Command responses

## Message Format

**Command:**
```json
{
  "param1": "value1"
}
```

**Response:**
```json
{
  "status": "success",
  "command": "resume-scan",
  "result": {...},
  "timestamp": 1704123456,
  "device_id": "skycharge-abc123"
}
```

## Testing

### Test Local Connection
```bash
python3 -c "from skycharge_mqtt import SkychargeZMQClient; \
  print(SkychargeZMQClient().get_charging_state())"
```

### Test MQTT
```bash
# Subscribe
mosquitto_sub -h {endpoint} -p 8883 \
  --cafile /etc/skycharge/certs/AmazonRootCA1.pem \
  --cert /etc/skycharge/certs/device-cert.pem \
  --key /etc/skycharge/certs/device-key.pem \
  -t 'skycharge/+/status'

# Publish command
mosquitto_pub -h {endpoint} -p 8883 \
  --cafile /etc/skycharge/certs/AmazonRootCA1.pem \
  --cert /etc/skycharge/certs/device-cert.pem \
  --key /etc/skycharge/certs/device-key.pem \
  -t 'skycharge/{device-id}/cmd/get-status' -m '{}'
```

## Troubleshooting

**Check skycharged:**
```bash
systemctl status skycharged
netstat -ln | grep 21103
```

**Check certificates:**
```bash
openssl x509 -in /etc/skycharge/certs/device-cert.pem -text -noout
```

**Debug mode:**
```bash
skymqttd --config /etc/skycharge/mqtt.json --log-level DEBUG
```

**View logs:**
```bash
journalctl -u skymqttd -n 100
```

## Uninstall

```bash
./uninstall.sh
```

## Files

```
skymqttd/
├── skymqttd.py              # Main daemon
├── skycharge_mqtt/          # Python package
│   ├── bridge.py            # MQTT/ZeroMQ bridge
│   ├── zmq_client.py        # skycharged client
│   ├── aws_client.py        # AWS IoT client
│   └── handlers.py          # Command handlers
├── install.sh               # Installation
├── uninstall.sh             # Removal
├── setup-aws-iot.sh         # AWS provisioning
├── requirements.txt         # Dependencies
└── mqtt.json.example        # Config template
```

## License

Copyright (C) 2026 Skycharge GmbH
