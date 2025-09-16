#!/usr/bin/env python3
"""
TCP Server to receive mmWave radar data with range and Doppler indices
The radar device connects to this server as a client.
"""

import socket
import struct
import sys
import threading

# Constants - adjust these based on your radar configuration
SERVER_HOST = '0.0.0.0'  # Listen on all interfaces
SERVER_PORT = 5001       # Port to listen on
DPIF_POINTCLOUD_CARTESIAN_SIZE = 24  # DPIF_PointCloudCartesian: 4 floats + 2 ints = 24 bytes

class RadarTCPServer:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.server_socket = None
        self.running = False
        
    def receive_data(self, client_socket, size):
        """Receive exactly 'size' bytes from the client socket"""
        data = b''
        while len(data) < size:
            chunk = client_socket.recv(size - len(data))
            if not chunk:
                raise ConnectionError("Connection lost while receiving data")
            data += chunk
        return data
    
    def handle_client(self, client_socket, client_address):
        """Handle data from a connected radar client"""
        print(f"Radar connected from {client_address}")
        
        try:
            frame_count = 0
            while self.running:
                try:
                    # Receive header: numObj (uint32_t) + dummy (uint16_t)
                    header_data = self.receive_data(client_socket, 6)  # 4 + 2 bytes
                    num_obj, dummy = struct.unpack('<IH', header_data)
                    
                    print(f"\n--- Frame {frame_count} ---")
                    print(f"Objects detected: {num_obj}")
                    
                    if num_obj == 0:
                        frame_count += 1
                        continue
                    
                    # Receive point cloud data - DPIF_PointCloudCartesian structure
                    # Structure: x(float), y(float), z(float), velocity(float), rangeIdx(int), dopplerIdx(int)
                    pointcloud_size = (4 * 4 + 2 * 4) * num_obj  # 4 floats + 2 ints per object = 24 bytes per object
                    pointcloud_data = self.receive_data(client_socket, pointcloud_size)
                    
                    # Parse DPIF_PointCloudCartesian data
                    # Format: x, y, z, velocity (floats), rangeIdx, dopplerIdx (ints)
                    pointcloud_format = '<' + 'ffffii' * num_obj  # 4 floats + 2 ints per object
                    if len(pointcloud_data) >= struct.calcsize(pointcloud_format):
                        pointcloud = struct.unpack(pointcloud_format, pointcloud_data[:struct.calcsize(pointcloud_format)])
                        
                        # Display the data
                        for i in range(num_obj):
                            x = pointcloud[i*6 + 0]
                            y = pointcloud[i*6 + 1] 
                            z = pointcloud[i*6 + 2]
                            velocity = pointcloud[i*6 + 3]
                            range_idx = pointcloud[i*6 + 4]
                            doppler_idx = pointcloud[i*6 + 5]
                            
                            print(f"  Object {i:2d}: x={x:8.3f}m, y={y:8.3f}m, z={z:8.3f}m, "
                                  f"vel={velocity:8.3f}m/s, rangeIdx={range_idx:4d}, dopplerIdx={doppler_idx:4d}")
                    else:
                        print(f"  Received {len(pointcloud_data)} bytes of point cloud data (expected {struct.calcsize(pointcloud_format)})")
                    
                    frame_count += 1
                            
                except socket.timeout:
                    continue
                except Exception as e:
                    print(f"Error receiving frame: {e}")
                    break
                    
        except Exception as e:
            print(f"Error handling client {client_address}: {e}")
        finally:
            print(f"Radar client {client_address} disconnected")
            client_socket.close()
    
    def start(self):
        """Start the TCP server"""
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind((self.host, self.port))
            self.server_socket.listen(5)  # Allow up to 5 pending connections
            self.running = True
            
            print(f"Radar TCP server listening on {self.host}:{self.port}")
            print("Waiting for radar connections...")
            print("Press Ctrl+C to stop")
            
            while self.running:
                try:
                    client_socket, client_address = self.server_socket.accept()
                    client_socket.settimeout(1.0)  # 1 second timeout for recv operations
                    
                    # Handle each client in a separate thread
                    client_thread = threading.Thread(
                        target=self.handle_client, 
                        args=(client_socket, client_address)
                    )
                    client_thread.daemon = True
                    client_thread.start()
                    
                except socket.error as e:
                    if self.running:
                        print(f"Socket error: {e}")
                        
        except KeyboardInterrupt:
            print("\nShutting down server...")
        except Exception as e:
            print(f"Server error: {e}")
        finally:
            self.stop()
    
    def stop(self):
        """Stop the TCP server"""
        self.running = False
        if self.server_socket:
            self.server_socket.close()
            print("Server stopped")

if __name__ == "__main__":
    if len(sys.argv) >= 2:
        SERVER_PORT = int(sys.argv[1])
    if len(sys.argv) >= 3:
        SERVER_HOST = sys.argv[2]
        
    print(f"Starting radar TCP server on {SERVER_HOST}:{SERVER_PORT}")
    
    server = RadarTCPServer(SERVER_HOST, SERVER_PORT)
    server.start()
