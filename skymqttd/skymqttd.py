#!/usr/bin/env python3
"""
Skycharge MQTT Bridge Daemon

Connects Skycharge devices to AWS IoT Core for remote monitoring and control.

Copyright (C) 2026 Skycharge GmbH

Usage:
    skymqttd --config <config_file> [--daemon] [--pidfile <pidfile>] [--log-level <level>]
    skymqttd --version
    skymqttd --help

Options:
    -c --config <config_file>   Configuration file path [default: /etc/skycharge/mqtt.json]
    -d --daemon                 Run as daemon
    -p --pidfile <pidfile>      PID file path [default: /var/run/skymqttd.pid]
    -l --log-level <level>      Log level (DEBUG, INFO, WARNING, ERROR) [default: INFO]
    -v --version                Show version
    -h --help                   Show this help message
"""

import sys
import json
import logging
import signal
import os
from pathlib import Path
from docopt import docopt

# Add package to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from skycharge_mqtt import SkyMQTTBridge

__version__ = "1.0.0"

# Global bridge instance for signal handling
bridge_instance = None


def setup_logging(log_level= "INFO"):
    """Setup logging configuration"""
    numeric_level = getattr(logging, log_level.upper(), None)
    if not isinstance(numeric_level, int):
        numeric_level = logging.INFO
    
    logging.basicConfig(
        level=numeric_level,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S'
    )


def load_config(config_file: str) -> dict:
    """
    Load configuration from JSON file
    
    Args:
        config_file: Path to configuration file
        
    Returns:
        Configuration dictionary
    """
    try:
        with open(config_file, 'r') as f:
            config = json.load(f)
        
        # Validate required fields
        required_fields = ['device_id', 'endpoint', 'cert_file', 'key_file', 'ca_file']
        missing_fields = [field for field in required_fields if field not in config]
        
        if missing_fields:
            raise ValueError("Missing required configuration fields: {}".format(', '.join(missing_fields)))
        
        # Validate file paths
        for file_field in ['cert_file', 'key_file', 'ca_file']:
            file_path = config[file_field]
            if not os.path.exists(file_path):
                raise FileNotFoundError("Certificate file not found: {}".format(file_path))
        
        return config
        
    except json.JSONDecodeError as e:
        print("Error: Invalid JSON in configuration file: {}".format(e), file=sys.stderr)
        sys.exit(1)
    except Exception as e:
        print("Error loading configuration: {}".format(e), file=sys.stderr)
        sys.exit(1)


def daemonize(pidfile: str):
    """
    Daemonize the process
    
    Args:
        pidfile: Path to PID file
    """
    try:
        # First fork
        pid = os.fork()
        if pid > 0:
            sys.exit(0)
    except OSError as e:
        print("Fork failed: {}".format(e), file=sys.stderr)
        sys.exit(1)
    
    # Decouple from parent environment
    os.chdir('/')
    os.setsid()
    os.umask(0)
    
    # Second fork
    try:
        pid = os.fork()
        if pid > 0:
            sys.exit(0)
    except OSError as e:
        print("Fork failed: {}".format(e), file=sys.stderr)
        sys.exit(1)
    
    # Redirect standard file descriptors
    sys.stdout.flush()
    sys.stderr.flush()
    
    with open('/dev/null', 'r') as f:
        os.dup2(f.fileno(), sys.stdin.fileno())
    with open('/dev/null', 'a+') as f:
        os.dup2(f.fileno(), sys.stdout.fileno())
    with open('/dev/null', 'a+') as f:
        os.dup2(f.fileno(), sys.stderr.fileno())
    
    # Write PID file
    pid = str(os.getpid())
    with open(pidfile, 'w') as f:
        f.write(pid + '\n')


def signal_handler(signum, frame):
    """Handle termination signals"""
    global bridge_instance
    
    logger = logging.getLogger(__name__)
    logger.info("Received signal {}".format(signum))
    
    if bridge_instance:
        bridge_instance.stop()
    
    sys.exit(0)


def main():
    """Main entry point"""
    global bridge_instance
    
    # Parse command line arguments
    args = docopt(__doc__, version="skymqttd {}".format(__version__))
    
    # Setup logging
    setup_logging(args['--log-level'])
    logger = logging.getLogger(__name__)
    
    logger.info("Skycharge MQTT Bridge v{}".format(__version__))
    
    # Load configuration
    config_file = args['--config']
    logger.info("Loading configuration from: {}".format(config_file))
    config = load_config(config_file)
    
    # Daemonize if requested
    if args['--daemon']:
        pidfile = args['--pidfile']
        logger.info("Daemonizing with PID file: {}".format(pidfile))
        daemonize(pidfile)
        # Reconfigure logging after daemonization
        setup_logging(args['--log-level'])
        logger = logging.getLogger(__name__)
    
    # Setup signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # Create and start bridge
    try:
        bridge_instance = SkyMQTTBridge(config)
        bridge_instance.start()
    except Exception as e:
        logger.error("Fatal error: {}".format(e), exc_info=True)
        sys.exit(1)
    finally:
        # Cleanup PID file
        if args['--daemon']:
            try:
                os.remove(args['--pidfile'])
            except:
                pass


if __name__ == '__main__':
    main()
