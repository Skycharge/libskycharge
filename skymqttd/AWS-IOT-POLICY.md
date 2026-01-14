# AWS IoT Policy for Skycharge Devices

This is the recommended AWS IoT policy for Skycharge devices.

```json
{
  "Version": "2012-10-17",
  "Statement": [
    {
      "Effect": "Allow",
      "Action": [
        "iot:Connect"
      ],
      "Resource": [
        "arn:aws:iot:*:*:client/skycharge-*"
      ]
    },
    {
      "Effect": "Allow",
      "Action": [
        "iot:Subscribe"
      ],
      "Resource": [
        "arn:aws:iot:*:*:topicfilter/skycharge/*/cmd/*",
        "arn:aws:iot:*:*:topicfilter/$aws/things/skycharge-*/shadow/update/delta",
        "arn:aws:iot:*:*:topicfilter/$aws/things/skycharge-*/shadow/get/accepted",
        "arn:aws:iot:*:*:topicfilter/$aws/things/skycharge-*/shadow/get/rejected"
      ]
    },
    {
      "Effect": "Allow",
      "Action": [
        "iot:Receive"
      ],
      "Resource": [
        "arn:aws:iot:*:*:topic/skycharge/*/cmd/*",
        "arn:aws:iot:*:*:topic/$aws/things/skycharge-*/shadow/update/delta",
        "arn:aws:iot:*:*:topic/$aws/things/skycharge-*/shadow/get/accepted",
        "arn:aws:iot:*:*:topic/$aws/things/skycharge-*/shadow/get/rejected"
      ]
    },
    {
      "Effect": "Allow",
      "Action": [
        "iot:Publish"
      ],
      "Resource": [
        "arn:aws:iot:*:*:topic/skycharge/*/status",
        "arn:aws:iot:*:*:topic/skycharge/*/telemetry",
        "arn:aws:iot:*:*:topic/skycharge/*/events",
        "arn:aws:iot:*:*:topic/skycharge/*/cmd/*/response",
        "arn:aws:iot:*:*:topic/$aws/things/skycharge-*/shadow/update",
        "arn:aws:iot:*:*:topic/$aws/things/skycharge-*/shadow/get"
      ]
    }
  ]
}
```

## Creating the Policy

### Using AWS CLI

```bash
aws iot create-policy \
  --policy-name SkychargeDevicePolicy \
  --policy-document file://policy.json
```

### Using AWS Console

1. Go to AWS IoT Core Console
2. Navigate to Security → Policies
3. Click "Create policy"
4. Name: `SkychargeDevicePolicy`
5. Paste the JSON above
6. Click "Create"

## Attaching Policy to Certificate

```bash
# Get certificate ARN
CERT_ARN=$(aws iot list-certificates --query 'certificates[0].certificateArn' --output text)

# Attach policy
aws iot attach-policy \
  --policy-name SkychargeDevicePolicy \
  --target $CERT_ARN

# Attach certificate to thing
aws iot attach-thing-principal \
  --thing-name skycharge-{UUID} \
  --principal $CERT_ARN
```

## Testing Policy

Use AWS IoT MQTT test client to verify:

1. Connect with device certificate
2. Subscribe to `skycharge/+/cmd/#`
3. Publish to `skycharge/{device-id}/status`
4. Both operations should succeed

## Security Notes

- Policy uses wildcard (*) for region and account - adjust for production
- Client ID must start with `skycharge-`
- Topics are namespaced by device ID
- Device Shadow access is optional (for future features)
