#!/usr/bin/env python3
"""
General Purpose TCP Server for receiving data over ethernet
Based on new_tcpserver.py but designed to be flexible for different data formats.
The server can handle raw data, structured data, or custom protocols.
"""

import socket
import struct
import sys
import threading
import time
import json
import argparse
from typing import Optional, Callable, Any, Dict
from enum import Enum

class DataMode(Enum):
    RAW = "raw"              # Raw bytes, no parsing
    STRUCTURED = "structured" # Expects header with data size/count
    JSON = "json"            # JSON messages
    CUSTOM = "custom"        # Custom parsing function

class GeneralTCPServer:
    def __init__(self, host: str = '0.0.0.0', port: int = 5001,
                 data_mode: DataMode = DataMode.RAW,
                 buffer_size: int = 4096,
                 timeout: float = 1.0,
                 header_format: str = '<I',
                 data_format: str = '<f',
                 custom_parser: Optional[Callable] = None,
                 verbose: bool = True):
        """
        Initialize the general TCP server

        Args:
            host: Server host to bind to
            port: Server port to listen on
            data_mode: How to interpret incoming data
            buffer_size: Default buffer size for raw data
            timeout: Socket timeout in seconds
            header_format: Struct format for header (structured mode)
            data_format: Struct format for data elements (structured mode)
            custom_parser: Custom parsing function for CUSTOM mode
            verbose: Enable verbose logging
        """
        self.host = host
        self.port = port
        self.data_mode = data_mode
        self.buffer_size = buffer_size
        self.timeout = timeout
        self.header_format = header_format
        self.data_format = data_format
        self.custom_parser = custom_parser
        self.verbose = verbose

        self.server_socket = None
        self.running = False
        self.client_handlers = []

    def log(self, message: str):
        """Log message if verbose mode is enabled"""
        if self.verbose:
            timestamp = time.strftime('%Y-%m-%d %H:%M:%S')
            print(f"[{timestamp}] {message}")

    def receive_data(self, client_socket: socket.socket, size: int) -> bytes:
        """Receive exactly 'size' bytes from the client socket"""
        data = b''
        while len(data) < size:
            chunk = client_socket.recv(size - len(data))
            if not chunk:
                raise ConnectionError("Connection lost while receiving data")
            data += chunk
        return data

    def parse_raw_data(self, client_socket: socket.socket) -> bytes:
        """Parse raw data - just receive buffer_size bytes"""
        return client_socket.recv(self.buffer_size)

    def parse_structured_data(self, client_socket: socket.socket) -> Dict[str, Any]:
        """Parse structured data with header indicating data size/count"""
        # Receive header
        header_size = struct.calcsize(self.header_format)
        header_data = self.receive_data(client_socket, header_size)
        header_values = struct.unpack(self.header_format, header_data)

        # Assume first header value is count/size
        data_count = header_values[0]

        if data_count == 0:
            return {"header": header_values, "data": [], "count": 0}

        # Calculate data size based on format and count
        data_element_size = struct.calcsize(self.data_format)
        total_data_size = data_element_size * data_count

        # Receive data
        data_bytes = self.receive_data(client_socket, total_data_size)

        # Unpack data
        format_str = '<' + self.data_format.lstrip('<>!@=') * data_count
        data_values = struct.unpack(format_str, data_bytes[:struct.calcsize(format_str)])

        return {
            "header": header_values,
            "data": data_values,
            "count": data_count,
            "raw_bytes": data_bytes
        }

    def parse_json_data(self, client_socket: socket.socket) -> Dict[str, Any]:
        """Parse JSON data - receive until complete JSON message"""
        data = b''
        while True:
            chunk = client_socket.recv(1024)
            if not chunk:
                break
            data += chunk

            # Try to parse as JSON
            try:
                message = data.decode('utf-8')
                return json.loads(message)
            except (json.JSONDecodeError, UnicodeDecodeError):
                continue

        raise ValueError("Failed to parse JSON data")

    def parse_data(self, client_socket: socket.socket) -> Any:
        """Parse data based on the configured mode"""
        if self.data_mode == DataMode.RAW:
            return self.parse_raw_data(client_socket)
        elif self.data_mode == DataMode.STRUCTURED:
            return self.parse_structured_data(client_socket)
        elif self.data_mode == DataMode.JSON:
            return self.parse_json_data(client_socket)
        elif self.data_mode == DataMode.CUSTOM and self.custom_parser:
            return self.custom_parser(client_socket)
        else:
            raise ValueError(f"Unsupported data mode: {self.data_mode}")

    def process_data(self, data: Any, frame_count: int, client_address: tuple) -> None:
        """Process received data - override this method for custom processing"""
        if self.data_mode == DataMode.RAW:
            self.log(f"Frame {frame_count}: Received {len(data)} raw bytes from {client_address}")
            if self.verbose and len(data) > 0:
                # Show first 32 bytes as hex
                hex_data = ' '.join(f'{b:02x}' for b in data[:32])
                self.log(f"  First 32 bytes: {hex_data}")

        elif self.data_mode == DataMode.STRUCTURED:
            self.log(f"Frame {frame_count}: Received structured data from {client_address}")
            self.log(f"  Header: {data['header']}")
            self.log(f"  Data count: {data['count']}")
            if data['count'] > 0:
                self.log(f"  First few data elements: {data['data'][:min(10, len(data['data']))]}")

        elif self.data_mode == DataMode.JSON:
            self.log(f"Frame {frame_count}: Received JSON data from {client_address}")
            self.log(f"  Data: {json.dumps(data, indent=2)}")

        else:
            self.log(f"Frame {frame_count}: Received custom data from {client_address}: {data}")

    def handle_client(self, client_socket: socket.socket, client_address: tuple):
        """Handle data from a connected client"""
        self.log(f"Client connected from {client_address}")

        try:
            frame_count = 0
            while self.running:
                try:
                    data = self.parse_data(client_socket)
                    self.process_data(data, frame_count, client_address)
                    frame_count += 1

                except socket.timeout:
                    continue
                except ConnectionError:
                    self.log(f"Client {client_address} disconnected")
                    break
                except Exception as e:
                    self.log(f"Error processing frame {frame_count}: {e}")
                    if self.verbose:
                        import traceback
                        traceback.print_exc()
                    break

        except Exception as e:
            self.log(f"Error handling client {client_address}: {e}")
        finally:
            self.log(f"Client {client_address} disconnected")
            client_socket.close()

    def start(self):
        """Start the TCP server"""
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind((self.host, self.port))
            self.server_socket.listen(5)
            self.running = True

            self.log(f"General TCP server listening on {self.host}:{self.port}")
            self.log(f"Data mode: {self.data_mode.value}")
            self.log("Waiting for connections...")
            self.log("Press Ctrl+C to stop")

            while self.running:
                try:
                    client_socket, client_address = self.server_socket.accept()
                    client_socket.settimeout(self.timeout)

                    # Handle each client in a separate thread
                    client_thread = threading.Thread(
                        target=self.handle_client,
                        args=(client_socket, client_address)
                    )
                    client_thread.daemon = True
                    client_thread.start()
                    self.client_handlers.append(client_thread)

                except socket.error as e:
                    if self.running:
                        self.log(f"Socket error: {e}")

        except KeyboardInterrupt:
            self.log("Shutting down server...")
        except Exception as e:
            self.log(f"Server error: {e}")
        finally:
            self.stop()

    def stop(self):
        """Stop the TCP server"""
        self.running = False
        if self.server_socket:
            self.server_socket.close()
            self.log("Server stopped")

def create_radar_parser():
    """Create a custom parser for radar data (like the original new_tcpserver.py)"""
    def radar_parser(client_socket):
        # Receive header: numObj (uint32_t) + dummy (uint16_t)
        header_data = server.receive_data(client_socket, 6)  # 4 + 2 bytes
        num_obj, dummy = struct.unpack('<IH', header_data)

        if num_obj == 0:
            return {"num_objects": 0, "objects": []}

        # Receive point cloud data
        pointcloud_size = 24 * num_obj  # 24 bytes per object
        pointcloud_data = server.receive_data(client_socket, pointcloud_size)

        # Parse point cloud data
        pointcloud_format = '<' + 'ffffii' * num_obj
        pointcloud = struct.unpack(pointcloud_format, pointcloud_data)

        objects = []
        for i in range(num_obj):
            obj = {
                'x': pointcloud[i*6 + 0],
                'y': pointcloud[i*6 + 1],
                'z': pointcloud[i*6 + 2],
                'velocity': pointcloud[i*6 + 3],
                'range_idx': pointcloud[i*6 + 4],
                'doppler_idx': pointcloud[i*6 + 5]
            }
            objects.append(obj)

        return {"num_objects": num_obj, "objects": objects}

    return radar_parser

def main():
    parser = argparse.ArgumentParser(description='General Purpose TCP Server')
    parser.add_argument('--host', default='0.0.0.0', help='Server host (default: 0.0.0.0)')
    parser.add_argument('--port', type=int, default=5001, help='Server port (default: 5001)'
)
    parser.add_argument('--mode', choices=['raw', 'structured', 'json', 'radar'],
                       default='raw', help='Data parsing mode (default: raw)')
    parser.add_argument('--buffer-size', type=int, default=4096,
                       help='Buffer size for raw mode (default: 4096)')
    parser.add_argument('--timeout', type=float, default=1.0,
                       help='Socket timeout in seconds (default: 1.0)')
    parser.add_argument('--header-format', default='<I',
                       help='Struct format for header in structured mode (default: <I)')
    parser.add_argument('--data-format', default='<f',
                       help='Struct format for data elements in structured mode (default: <f)')
    parser.add_argument('--quiet', action='store_true', help='Disable verbose logging')

    args = parser.parse_args()

    # Convert mode string to enum
    mode_map = {
        'raw': DataMode.RAW,
        'structured': DataMode.STRUCTURED,
        'json': DataMode.JSON,
        'radar': DataMode.CUSTOM
    }

    data_mode = mode_map[args.mode]
    custom_parser = None

    if args.mode == 'radar':
        custom_parser = create_radar_parser()

    global server
    server = GeneralTCPServer(
        host=args.host,
        port=args.port,
        data_mode=data_mode,
        buffer_size=args.buffer_size,
        timeout=args.timeout,
        header_format=args.header_format,
        data_format=args.data_format,
        custom_parser=custom_parser,
        verbose=not args.quiet
    )

    server.start()

if __name__ == "__main__":
    main()
