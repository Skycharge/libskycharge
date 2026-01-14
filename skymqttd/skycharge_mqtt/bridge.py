#!/usr/bin/env python3
"""
Main MQTT bridge implementation
"""

import json
import time
import logging
from typing import Dict, Any

from .zmq_client import SkychargeZMQClient
from .aws_client import AWSIoTClient
from .handlers import CommandHandlers

logger = logging.getLogger(__name__)

# Get process start time for uptime calculation
PROCESS_START_TIME = time.time()


class SkyMQTTBridge:
    """AWS IoT MQTT Bridge for Skycharge"""
    
    def __init__(self, config):
        """
        Initialize MQTT bridge
        
        Args:
            config: Configuration dictionary
        """
        self.config = config
        self.device_id = config['device_id']
        self.running = False
        
        # Initialize clients
        self.zmq_client = SkychargeZMQClient(
            endpoint=config.get('skycharged_endpoint', 'tcp://localhost:21103')
        )
        self.aws_client = AWSIoTClient(config)
        
        # Initialize command handlers
        self.handlers = CommandHandlers(self.zmq_client)
        
        # Topics
        self.cmd_topic = "skycharge/{}/cmd/+".format(self.device_id)
        self.status_topic = "skycharge/{}/status".format(self.device_id)
        self.telemetry_topic = "skycharge/{}/telemetry".format(self.device_id)
        self.events_topic = "skycharge/{}/events".format(self.device_id)
        self.heartbeat_topic = "skycharge/{}/heartbeat".format(self.device_id)
        
        # Configuration
        self.status_interval = config.get('status_interval', 30)
        self.telemetry_interval = config.get('telemetry_interval', 10)
        self.heartbeat_interval = config.get('heartbeat_interval', 1)  # Every 1 second
        
        self.last_status_time = 0
        self.last_telemetry_time = 0
        self.last_heartbeat_time = 0
        
    def _on_command_received(self, topic, payload, **kwargs):
        """
        Handle incoming MQTT command
        
        Args:
            topic: MQTT topic
            payload: Message payload
        """
        try:
            # Extract command from topic: skycharge/{device_id}/cmd/{command}
            cmd_name = topic.split('/')[-1]
            cmd_data = json.loads(payload) if payload else {}
            
            logger.info("Received command: {}".format(cmd_name))
            logger.debug("Command data: {}".format(cmd_data))
            
            # Execute command
            response = self.handlers.handle_command(cmd_name, cmd_data)
            
            # Add metadata
            response['timestamp'] = int(time.time())
            response['device_id'] = self.device_id
            
            # Publish response
            response_topic = "skycharge/{}/cmd/{}/response".format(self.device_id, cmd_name)
            self.aws_client.publish(
                topic=response_topic,
                payload=response,
                qos=1  # AT_LEAST_ONCE
            )
            
            logger.info("Command {} executed: {}".format(cmd_name, response['status']))
            
        except json.JSONDecodeError as e:
            logger.error("Invalid JSON in command payload: {}".format(e))
        except Exception as e:
            logger.error("Error handling command: {}".format(e), exc_info=True)
    
    def _on_connection_resumed(self, connection, return_code, session_present, **kwargs):
        """Handle connection resumption"""
        logger.info("MQTT connection resumed: {}".format(return_code))
        if return_code == 0 and not session_present:  # 0 = ACCEPTED
            logger.info("Session did not persist, resubscribing to topics")
            self._subscribe_to_commands()
    
    def _subscribe_to_commands(self):
        """Subscribe to command topics"""
        self.aws_client.subscribe(
            topic=self.cmd_topic,
            callback=self._on_command_received,
            qos=1  # AT_LEAST_ONCE
        )
    
    def _publish_status(self):
        """Publish periodic status update"""
        try:
            status = self.zmq_client.get_charging_state()
            status['timestamp'] = int(time.time())
            status['device_id'] = self.device_id
            
            self.aws_client.publish(
                topic=self.status_topic,
                payload=status,
                qos=0  # AT_MOST_ONCE
            )
            
            logger.debug("Status published")
            
        except Exception as e:
            logger.error("Error publishing status: {}".format(e))
    
    def _publish_telemetry(self):
        """Publish telemetry data"""
        try:
            telemetry = self.zmq_client.get_charging_state()
            telemetry['timestamp'] = int(time.time())
            telemetry['device_id'] = self.device_id
            
            self.aws_client.publish(
                topic=self.telemetry_topic,
                payload=telemetry,
                qos=0  # AT_MOST_ONCE
            )
            
            logger.debug("Telemetry published")
            
        except Exception as e:
            logger.error("Error publishing telemetry: {}".format(e))
    
    def _publish_heartbeat(self):
        """Publish heartbeat with uptime"""
        try:
            uptime_seconds = int(time.time() - PROCESS_START_TIME)
            heartbeat = {
                'timestamp': int(time.time()),
                'device_id': self.device_id,
                'uptime_seconds': uptime_seconds,
                'uptime_human': self._format_uptime(uptime_seconds),
                'status': 'alive'
            }
            
            self.aws_client.publish(
                topic=self.heartbeat_topic,
                payload=heartbeat,
                qos=0  # AT_MOST_ONCE for heartbeat
            )
            
            logger.debug("Heartbeat published (uptime: {})".format(heartbeat['uptime_human']))
            
        except Exception as e:
            logger.error("Error publishing heartbeat: {}".format(e))
    
    def _format_uptime(self, seconds):
        """Format uptime in human readable format"""
        days = seconds // 86400
        hours = (seconds % 86400) // 3600
        minutes = (seconds % 3600) // 60
        secs = seconds % 60
        
        if days > 0:
            return "{}d {}h {}m {}s".format(days, hours, minutes, secs)
        elif hours > 0:
            return "{}h {}m {}s".format(hours, minutes, secs)
        elif minutes > 0:
            return "{}m {}s".format(minutes, secs)
        else:
            return "{}s".format(secs)
    
    def publish_event(self, event_type, event_data):
        """
        Publish event
        
        Args:
            event_type: Type of event
            event_data: Event data
        """
        try:
            event = {
                'event_type': event_type,
                'timestamp': int(time.time()),
                'device_id': self.device_id,
                'data': event_data
            }
            
            self.aws_client.publish(
                topic=self.events_topic,
                payload=event,
                qos=1  # AT_LEAST_ONCE
            )
            
            logger.info("Event published: {}".format(event_type))
            
        except Exception as e:
            logger.error("Error publishing event: {}".format(e))
    
    def start(self):
        """Start the bridge"""
        logger.info("Starting Skycharge MQTT Bridge")
        logger.info("Device ID: {}".format(self.device_id))
        logger.info("AWS IoT Endpoint: {}".format(self.config['endpoint']))
        
        # Connect to AWS IoT
        self.aws_client.connect(on_resumed=self._on_connection_resumed)
        
        # Subscribe to commands
        self._subscribe_to_commands()
        
        # Publish initial status
        self._publish_status()
        self.publish_event('bridge_started', {'version': '1.0.0'})
        
        # Main loop
        self.running = True
        
        try:
            while self.running:
                current_time = time.time()
                
                # Publish heartbeat
                if current_time - self.last_heartbeat_time >= self.heartbeat_interval:
                    self._publish_heartbeat()
                    self.last_heartbeat_time = current_time
                
                # Publish status
                if current_time - self.last_status_time >= self.status_interval:
                    self._publish_status()
                    self.last_status_time = current_time
                
                # Publish telemetry
                if current_time - self.last_telemetry_time >= self.telemetry_interval:
                    self._publish_telemetry()
                    self.last_telemetry_time = current_time
                
                # Sleep to avoid busy loop
                time.sleep(1)
                
        except KeyboardInterrupt:
            logger.info("Received interrupt signal")
        except Exception as e:
            logger.error("Error in main loop: {}".format(e), exc_info=True)
        finally:
            self.stop()
    
    def stop(self):
        """Stop the bridge"""
        logger.info("Stopping Skycharge MQTT Bridge")
        self.running = False
        
        # Publish shutdown event
        try:
            self.publish_event('bridge_stopped', {})
        except:
            pass
        
        # Disconnect
        try:
            self.aws_client.disconnect()
        except:
            pass
        
        # Close Skycharge client
        try:
            self.zmq_client.close()
        except:
            pass
        
        logger.info("Stopped")
