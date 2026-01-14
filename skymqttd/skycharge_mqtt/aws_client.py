#!/usr/bin/env python3
"""
AWS IoT Core MQTT client wrapper using paho-mqtt
Compatible with Python 3.5+
"""

import logging
import json
import ssl
from typing import Dict, Any, Callable
import paho.mqtt.client as mqtt

logger = logging.getLogger(__name__)


class AWSIoTClient:
    """AWS IoT Core MQTT client wrapper using paho-mqtt"""
    
    def __init__(self, config):
        """
        Initialize AWS IoT client
        
        Args:
            config: Configuration dictionary with AWS IoT settings
        """
        self.config = config
        self.device_id = config['device_id']
        self.client = None
        self.connected = False
        self.on_resumed_callback = None
        
    def connect(self, on_interrupted = None, on_resumed = None):
        """
        Connect to AWS IoT Core
        
        Args:
            on_interrupted: Callback for connection interruption
            on_resumed: Callback for connection resumption
        """
        logger.info("Connecting to AWS IoT: {}".format(self.config['endpoint']))
        
        # Store callbacks
        self.on_resumed_callback = on_resumed
        
        # Create MQTT client
        self.client = mqtt.Client(client_id=self.device_id, clean_session=False)
        
        # Set callbacks
        self.client.on_connect = self._on_connect
        self.client.on_disconnect = self._on_disconnect
        
        # Configure TLS
        self.client.tls_set(
            ca_certs=self.config['ca_file'],
            certfile=self.config['cert_file'],
            keyfile=self.config['key_file'],
            tls_version=ssl.PROTOCOL_TLSv1_2
        )
        
        # Connect
        port = self.config.get('port', 8883)
        keep_alive = self.config.get('keep_alive', 60)
        
        try:
            self.client.connect(self.config['endpoint'], port, keep_alive)
            self.client.loop_start()
        except Exception as e:
            logger.error("Failed to connect: {}".format(e))
            raise
        
        # Wait for connection
        import time
        timeout = 30
        start = time.time()
        while not self.connected and (time.time() - start) < timeout:
            time.sleep(0.5)
        
        if not self.connected:
            logger.error("Connection timeout after {} seconds".format(timeout))
            raise Exception("Failed to connect to AWS IoT Core")
            
        logger.info("Connected to AWS IoT Core")
        
    def disconnect(self):
        """Disconnect from AWS IoT Core"""
        if self.client and self.connected:
            logger.info("Disconnecting from AWS IoT Core")
            self.client.loop_stop()
            self.client.disconnect()
            self.connected = False
            logger.info("Disconnected from AWS IoT Core")
    
    def subscribe(self, topic, callback, qos = 1):
        """
        Subscribe to MQTT topic
        
        Args:
            topic: MQTT topic pattern
            callback: Message callback function
            qos: Quality of Service level (0, 1, or 2)
        """
        logger.info("Subscribing to topic: {}".format(topic))
        
        def paho_callback(client, userdata, message):
            """Wrapper to convert paho message to our format"""
            callback(topic=message.topic, payload=message.payload.decode('utf-8'))
        
        self.client.message_callback_add(topic, paho_callback)
        self.client.subscribe(topic, qos)
        logger.info("Successfully subscribed to: {}".format(topic))
    
    def publish(self, topic, payload, qos = 1):
        """
        Publish message to MQTT topic
        
        Args:
            topic: MQTT topic
            payload: Message payload (dict will be converted to JSON)
            qos: Quality of Service level (0, 1, or 2)
        """
        if isinstance(payload, dict):
            payload = json.dumps(payload)
        
        try:
            result = self.client.publish(topic, payload, qos)
            if result.rc != mqtt.MQTT_ERR_SUCCESS:
                logger.error("Failed to publish to {}: {}".format(topic, result.rc))
            else:
                logger.debug("Published to {}: {}...".format(topic, payload[:100]))
        except Exception as e:
            logger.error("Failed to publish to {}: {}".format(topic, e))
    
    def _on_connect(self, client, userdata, flags, rc):
        """Handle connection event"""
        rc_messages = {
            0: "Connection successful",
            1: "Incorrect protocol version",
            2: "Invalid client identifier",
            3: "Server unavailable",
            4: "Bad username or password",
            5: "Not authorized"
        }
        
        if rc == 0:
            self.connected = True
            logger.info("MQTT connection established - {}".format(rc_messages.get(rc, "Unknown")))
            if self.on_resumed_callback:
                self.on_resumed_callback(None, rc, flags.get('session present', False))
        else:
            logger.error("MQTT connection failed with code {}: {}".format(rc, rc_messages.get(rc, "Unknown error")))
    
    def _on_disconnect(self, client, userdata, rc):
        """Handle disconnection event"""
        rc_messages = {
            0: "Clean disconnect",
            1: "Incorrect protocol version",
            2: "Invalid client identifier",  
            3: "Server unavailable",
            4: "Bad username or password",
            5: "Not authorized",
            7: "Connection lost"
        }
        
        self.connected = False
        if rc != 0:
            logger.warning("MQTT connection interrupted - code {}: {}".format(rc, rc_messages.get(rc, "Unknown error")))
        else:
            logger.info("MQTT disconnected cleanly")
    
    def _on_connection_resumed(self, connection, return_code, session_present, **kwargs):
        """Handle connection resumption"""
        self.connected = True
        logger.info("MQTT connection resumed: {}, session_present: {}".format(return_code, session_present))
