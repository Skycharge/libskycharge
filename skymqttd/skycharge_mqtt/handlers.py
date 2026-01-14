#!/usr/bin/env python3
"""
Command handlers for MQTT bridge
"""

import logging
from typing import Dict, Any, Callable

logger = logging.getLogger(__name__)


class CommandHandlers:
    """Command handlers for Skycharge MQTT bridge"""
    
    def __init__(self, zmq_client):
        """
        Initialize command handlers
        
        Args:
            zmq_client: SkychargeZMQClient instance
        """
        self.zmq_client = zmq_client
        
        # Command routing table
        self.handlers = {
            'get-status': self.handle_get_status,
            'get-params': self.handle_get_params,
            'set-params': self.handle_set_params,
            'resume-scan': self.handle_resume_scan,
            'stop-scan': self.handle_stop_scan,
            'open-droneport': self.handle_open_droneport,
            'close-droneport': self.handle_close_droneport,
            'reset-device': self.handle_reset_device,
        }
    
    def handle_command(self, cmd_name, params):
        """
        Route and execute command
        
        Args:
            cmd_name: Command name
            params: Command parameters
            
        Returns:
            Command response dictionary
        """
        logger.info("Handling command: {} with params: {}".format(cmd_name, params))
        
        handler = self.handlers.get(cmd_name)
        if handler:
            try:
                result = handler(params)
                return {'status': 'success', 'command': cmd_name, 'result': result}
            except Exception as e:
                logger.error("Error executing command {}: {}".format(cmd_name, e), exc_info=True)
                return {'status': 'error', 'command': cmd_name, 'message': str(e)}
        else:
            logger.warning("Unknown command: {}".format(cmd_name))
            return {
                'status': 'error',
                'command': cmd_name,
                'message': 'Unknown command: {}'.format(cmd_name),
                'available_commands': list(self.handlers.keys())
            }
    
    def handle_get_status(self, params):
        """Get current charging status"""
        return self.zmq_client.get_charging_state()
    
    def handle_get_params(self, params):
        """Get device parameters"""
        return self.zmq_client.get_device_params()
    
    def handle_set_params(self, params):
        """Set device parameters"""
        if not params:
            return {'error': 'No parameters provided'}
        return self.zmq_client.set_device_params(params)
    
    def handle_resume_scan(self, params):
        """Resume scanning for drones"""
        return self.zmq_client.resume_scan()
    
    def handle_stop_scan(self, params):
        """Stop scanning"""
        return self.zmq_client.stop_scan()
    
    def handle_open_droneport(self, params):
        """Open droneport"""
        return self.zmq_client.open_droneport()
    
    def handle_close_droneport(self, params):
        """Close droneport"""
        return self.zmq_client.close_droneport()
    
    def handle_reset_device(self, params):
        """Reset device"""
        return self.zmq_client.reset_device()
