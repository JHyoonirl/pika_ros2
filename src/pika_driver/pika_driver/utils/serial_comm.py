#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Serial communication module for Pika-series devices."""

import threading
import time
import json
import serial
import logging
import re # Regular expressions for JSON cleanup.
import struct # Pack command payloads into bytes.

# Configure logging.
logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(name)s - %(levelname)s - %(message)s")
logger = logging.getLogger("pika.serial_comm")

class SerialComm:
    """Serial communication class for device I/O.

    Args:
        port (str): Serial device path, defaults to '/dev/ttyUSB0'.
        baudrate (int): Baud rate, defaults to 460800.
        timeout (float): Read timeout in seconds, defaults to 1.0.
    """
    def __init__(self, port=r"/dev/ttyUSB0", baudrate=460800, timeout=1.0):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial = None
        self.is_connected = False
        self.reading_thread = None
        self.stop_thread = False
        self.buffer = ""
        self.callback = None
        self.data_lock = threading.Lock()
        self.latest_data = {}
    
    def connect(self):
        """Connect to the serial device.

        Returns:
            bool: True if connected successfully.
        """
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=self.timeout
            )
            self.is_connected = True
            logger.info(f"Connected to serial device: {self.port}")
            return True
        except serial.SerialException as e:
            logger.error(f"Failed to connect to serial device: {e}")
            self.is_connected = False
            return False
    
    def disconnect(self):
        """Disconnect the serial device."""
        self.stop_reading_thread()
        if self.serial and self.is_connected:
            self.serial.close()
            self.is_connected = False
            logger.info(f"Disconnected from serial device: {self.port}")
    
    def send_data(self, data):
        """Send raw bytes to the serial port.

        Args:
            data (bytes): Data to send.

        Returns:
            bool: True if send succeeds.
        """
        if not self.is_connected or not self.serial:
            logger.error("Serial port is not connected; cannot send data")
            return False
        
        try:
            self.serial.write(data)
            self.serial.flush()
            return True
        except serial.SerialException as e:
            logger.error(f"Failed to send data: {e}")
            return False
    
    def send_command(self, command_type, value=0, big_endian=False):
        """Send a binary command to the device.

        Args:
            command_type (int): Command type byte.
            value (float): Command value, defaults to 0.
            big_endian (bool): Use big-endian encoding if True.

        Returns:
            bool: True if send succeeds.
        """
        try:
            # Build the command payload.
            data = bytearray()
            data.append(command_type)  # Command type.
            
            if big_endian:
                value_bytes = bytearray(struct.pack('>i', value))  # Big-endian.
            else:
                value_bytes = bytearray(struct.pack('<f', value))  # Little-endian.
            
            data.extend(value_bytes)
            
            # Append line ending: \r\n
            data.extend(b'\r\n')
            
            return self.send_data(data)
        except Exception as e:
            logger.error(f"Failed to build command payload: {e}")
            return False
        
    def get_device_info_command(self):
        """Send a `GET_INFO\r\n` command to the device.

        Returns:
            bool: True if send succeeds.
        """
        try:
            # Build GET_INFO command bytes.
            command = 'GET_INFO\r\n'
            data = command.encode('utf-8')
            
            return self.send_data(data)
        except Exception as e:
            logger.error(f"Failed to send GET_INFO command: {e}")
            return False
        
    def read_data(self):
        """Read available bytes from the serial port.

        Returns:
            bytes: Bytes read from the device.
        """
        if not self.is_connected or not self.serial:
            logger.error("Serial port is not connected; cannot read data")
            return b''
        
        try:
            if self.serial.in_waiting > 0:
                return self.serial.read(self.serial.in_waiting)
            return b''
        except serial.SerialException as e:
            logger.error(f"Failed to read data: {e}")
            return b''
    
    def _reading_thread_func(self):
        """Read loop that continuously receives and parses serial data."""
        logger.info("Started serial reading thread")
        while not self.stop_thread:
            if not self.is_connected:
                time.sleep(0.1)
                continue
            
            try:
                # Read bytes from serial.
                data = self.read_data()
                if data:
                    # Append decoded bytes to the text buffer.
                    self.buffer += data.decode('utf-8', errors='ignore')
                    
                    # Try extracting a complete JSON object.
                    json_data = self._find_json()
                    if json_data:
                        # Invoke callback when available.
                        if self.callback:
                            self.callback(json_data)
                        
                        # Update the latest parsed data.
                        with self.data_lock:
                            self.latest_data = json_data
                    # Clear oversized buffer if no complete JSON was found.
                    else:
                        if len(self.buffer) > 2000:
                            self.buffer = ""
                
                # Short sleep to avoid high CPU usage.
                time.sleep(0.001)
            except Exception as e:
                logger.error(f"Reading thread error: {e}")
                time.sleep(0.1)
        
        logger.info("Serial reading thread stopped")
    
    def _find_json(self):
        """Find and parse one complete JSON object from the buffer.

        Returns:
            dict: Parsed JSON object, or None if not available.
        """
        try:
            # Locate the start position of a JSON object.
            start = self.buffer.find('{')
            if start == -1:
                self.buffer = ""
                return None
            
            # Use a stack to match braces.
            stack = []
            for i in range(start, len(self.buffer)):
                if self.buffer[i] == '{':
                    stack.append(i)
                elif self.buffer[i] == '}':
                    if stack:
                        stack.pop()
                        if not stack:  # Found a complete JSON object.
                            json_str = self.buffer[start:i+1]
                            self.buffer = self.buffer[i+1:]
                            
                            # Handle trailing commas in malformed JSON.
                            cleaned_json_str = re.sub(r',\s*}', '}', json_str)
                            cleaned_json_str = re.sub(r',\s*\]', ']', cleaned_json_str)
                            
                            return json.loads(cleaned_json_str)
            
            # Keep buffer content if object is incomplete.
            return None
        except json.JSONDecodeError as e:
            # logger.error(f"JSON parse error: {e}")
            self.buffer = ""  # Skip invalid content and resync.
            return None
        except Exception as e:
            logger.error(f"Serial JSON processing error: {e}")
            self.buffer = ""
            return None
    
    def start_reading_thread(self, callback=None):
        """Start the serial reading thread.

        Args:
            callback (callable): Called with parsed JSON data.
        """
        if self.reading_thread and self.reading_thread.is_alive():
            logger.warning("Reading thread is already running")
            return
        
        self.callback = callback
        self.stop_thread = False
        self.reading_thread = threading.Thread(target=self._reading_thread_func)
        self.reading_thread.daemon = True
        self.reading_thread.start()
    
    def stop_reading_thread(self):
        """Stop the serial reading thread."""
        self.stop_thread = True
        if self.reading_thread and self.reading_thread.is_alive():
            self.reading_thread.join(timeout=1.0)
            logger.info("Reading thread stopped")
    
    def get_latest_data(self):
        """Get the latest parsed data.

        Returns:
            dict: Latest parsed JSON data.
        """
        with self.data_lock:
            return self.latest_data.copy()
    
    def __del__(self):
        """Destructor to ensure resources are released."""
        self.disconnect()


