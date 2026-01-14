#!/usr/bin/env python3
"""
Client for communicating with skycharged via skycharge-cli
"""

import subprocess
import json
import logging
import re

logger = logging.getLogger(__name__)


class SkychargeZMQClient:
    """Client for communicating with skycharged via skycharge-cli"""
    
    def __init__(self, endpoint = "tcp://localhost:5555", timeout = 5000):
        """
        Initialize skycharge-cli client
        
        Args:
            endpoint: ZeroMQ endpoint (default: tcp://localhost:5555)
            timeout: Command timeout in milliseconds (default: 5000)
        """
        # Parse endpoint to get host and port
        match = re.match(r'tcp://([^:]+):(\d+)', endpoint)
        if match:
            self.host = match.group(1)
            self.port = match.group(2)
        else:
            self.host = 'localhost'
            self.port = '5555'
        
        self.timeout = timeout / 1000.0  # Convert to seconds
        logger.info("Initialized skycharge-cli client for {}:{}".format(self.host, self.port))
        
    def _run_cli(self, command, args=None, json_output=True):
        """
        Execute skycharge-cli command
        
        Args:
            command: CLI command name
            args: Additional arguments list
            json_output: Whether to request JSON output
            
        Returns:
            Response dictionary or None on error
        """
        try:
            cmd = ['skycharge-cli', command]
            if json_output:
                cmd.append('--json')
            if args:
                cmd.extend(args)
            cmd.extend([self.host, self.port])
            
            logger.debug("Running: {}".format(' '.join(cmd)))
            
            result = subprocess.run(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                universal_newlines=True,
                timeout=self.timeout
            )
            
            if result.returncode != 0:
                logger.error("skycharge-cli error: {}".format(result.stderr))
                return {'error': 'cli_error', 'message': result.stderr.strip()}
            
            if json_output and result.stdout.strip():
                return json.loads(result.stdout)
            elif result.stdout.strip():
                return {'output': result.stdout.strip()}
            else:
                return {'status': 'success'}
                
        except subprocess.TimeoutExpired:
            logger.error("skycharge-cli timeout")
            return {'error': 'timeout', 'message': 'Command timed out'}
        except json.JSONDecodeError as e:
            logger.error("JSON decode error: {}".format(e))
            return {'error': 'json_error', 'message': str(e), 'output': result.stdout}
        except Exception as e:
            logger.error("Error running skycharge-cli: {}".format(e))
            return {'error': 'communication', 'message': str(e)}
    
    def get_charging_state(self):
        """Get current charging state"""
        return self._run_cli('show-charging-state')
    
    def get_device_params(self):
        """Get device parameters"""
        return self._run_cli('show-dev-params')
    
    def set_device_params(self, params):
        """Set device parameters"""
        # Convert params dict to key=value string
        param_str = ' '.join(["{}={}".format(k, v) for k, v in params.items()])
        return self._run_cli('set-dev-params', args=[param_str], json_output=False)
    
    def resume_scan(self):
        """Start scanning for drones"""
        return self._run_cli('resume-scan', json_output=False)
    
    def stop_scan(self):
        """Stop scanning"""
        return self._run_cli('stop-scan', json_output=False)
    
    def open_droneport(self):
        """Open droneport"""
        return self._run_cli('open-droneport', json_output=False)
    
    def close_droneport(self):
        """Close droneport"""
        return self._run_cli('close-droneport', json_output=False)
    
    def reset_device(self):
        """Reset device"""
        return self._run_cli('reset', json_output=False)
    
    def close(self):
        """Close client (no-op for CLI-based client)"""
        logger.info("skycharge-cli client closed")
