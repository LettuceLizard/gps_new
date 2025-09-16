# rtk_radar_capture.py - AWR2944PEVM Version
import traceback
import time
import datetime
import threading
import signal
import sys
import json
import serial
import pynmea2
import os
import socket
import csv
from pyproj import Transformer as ProjTransformer
from sensor_fusion import Transformer as RadarTransformer
from ahrs.common.frames import ecef2enu
from ctypes import *
import numpy as np
from enum import Enum

# --- ASCII Protocol Classes (from data_collector.py) ---
class ASCIIHEADER:
    def __init__(self, message, cpu_idle, time_ref, time_status, week_number,
                 milliseconds, reserved, version, leap_sec, output_delay):
        self.message = message
        self.cpu_idle = cpu_idle
        self.time_ref = time_ref
        self.time_status = time_status
        self.week_number = week_number
        self.milliseconds = milliseconds
        self.reserved = reserved
        self.version = version
        self.leap_sec = leap_sec
        self.output_delay = output_delay

    def __str__(self):
        return (f"HEADER:\n"
                f"  message: {self.message}\n"
                f"  cpu_idle: {self.cpu_idle}\n"
                f"  time_ref: {self.time_ref}\n"
                f"  time_status: {self.time_status}\n"
                f"  week_number: {self.week_number}\n"
                f"  milliseconds: {self.milliseconds}\n"
                f"  reserved: {self.reserved}\n"
                f"  version: {self.version}\n"
                f"  leap_sec: {self.leap_sec}\n"
                f"  output_delay: {self.output_delay}")

    def __repr__(self):
        return (f"ASCIIHEADER(message='{self.message}', cpu_idle='{self.cpu_idle}', "
                f"time_ref='{self.time_ref}', time_status='{self.time_status}', "
                f"week_number='{self.week_number}', milliseconds='{self.milliseconds}', "
                f"reserved='{self.reserved}', version='{self.version}', "
                f"leap_sec='{self.leap_sec}', output_delay='{self.output_delay}')")

class BESTNAVXYZA:
    def __init__(self, header: ASCIIHEADER, sol_status, pos_type, pos_x, pos_y, pos_z,
                 pos_x_stdev, pos_y_stdev, pos_z_stdev, vel_status, vel_type, vel_x, vel_y, vel_z,
                 vel_x_stdev, vel_y_stdev, vel_z_stdev, stn_id, diff_age, sol_age, sol_age2,
                 num_svs, num_soln_svs, num_gg_l1, num_soln_multi_svs, reserved, ext_sol_stat,
                 galileo_bds3_sig_mask, gps_glonass_bds2_sig_mask, crc):
        self.header = header
        self.sol_status = sol_status
        self.pos_type = pos_type
        self.pos_x = float(pos_x) if pos_x else 0.0
        self.pos_y = float(pos_y) if pos_y else 0.0
        self.pos_z = float(pos_z) if pos_z else 0.0
        self.pos_x_stdev = float(pos_x_stdev) if pos_x_stdev else 0.0
        self.pos_y_stdev = float(pos_y_stdev) if pos_y_stdev else 0.0
        self.pos_z_stdev = float(pos_z_stdev) if pos_z_stdev else 0.0
        self.vel_status = vel_status
        self.vel_type = vel_type
        self.vel_x = float(vel_x) if vel_x else 0.0
        self.vel_y = float(vel_y) if vel_y else 0.0
        self.vel_z = float(vel_z) if vel_z else 0.0
        self.vel_x_stdev = float(vel_x_stdev) if vel_x_stdev else 0.0
        self.vel_y_stdev = float(vel_y_stdev) if vel_y_stdev else 0.0
        self.vel_z_stdev = float(vel_z_stdev) if vel_z_stdev else 0.0
        self.stn_id = stn_id
        self.diff_age = float(diff_age) if diff_age else 0.0
        self.sol_age = float(sol_age) if sol_age else 0.0
        self.sol_age2 = float(sol_age2) if sol_age2 else 0.0
        self.num_svs = int(float(num_svs)) if num_svs else 0
        self.num_soln_svs = int(float(num_soln_svs)) if num_soln_svs else 0
        self.num_gg_l1 = int(float(num_gg_l1)) if num_gg_l1 else 0
        self.num_soln_multi_svs = int(float(num_soln_multi_svs)) if num_soln_multi_svs else 0
        self.reserved = reserved
        self.ext_sol_stat = ext_sol_stat
        self.galileo_bds3_sig_mask = galileo_bds3_sig_mask
        self.gps_glonass_bds2_sig_mask = gps_glonass_bds2_sig_mask
        self.crc = crc

class BESTNAVA:
    def __init__(self, header: ASCIIHEADER, sol_status, pos_type, lat, lon, hgt, undulation, datum_id,
                 lat_stdev, lon_stdev, hgt_stdev, stn_id, diff_age, sol_age, num_svs, num_soln_svs,
                 reserved1, reserved2, reserved3, ext_sol_stat, galileo_bds3_sig_mask, gps_glonass_bds2_sig_mask,
                 vsol_status, vel_type, latency, age, hor_spd, trk_gnd, vert_spd, vert_spd_stdev, hor_spd_stdev,
                 crc):

        self.header = header
        self.sol_status = sol_status
        self.pos_type = pos_type
        self.lat = lat
        self.lon = lon
        self.hgt = hgt
        self.undulation = undulation
        self.datum_id = datum_id
        self.lat_stdev = lat_stdev
        self.lon_stdev = lon_stdev
        self.hgt_stdev = hgt_stdev
        self.stn_id = stn_id
        self.diff_age = diff_age
        self.sol_age = sol_age
        self.num_svs = num_svs
        self.num_soln_svs = num_soln_svs
        self.reserved1 = reserved1
        self.reserved2 = reserved2
        self.reserved3 = reserved3
        self.ext_sol_stat = ext_sol_stat
        self.galileo_bds3_sig_mask = galileo_bds3_sig_mask
        self.gps_glonass_bds2_sig_mask = gps_glonass_bds2_sig_mask
        self.vsol_status = vsol_status
        self.vel_type = vel_type
        self.latency = latency
        self.age = age
        self.hor_spd = hor_spd
        self.trk_gnd = trk_gnd
        self.vert_spd = vert_spd
        self.vert_spd_stdev = vert_spd_stdev
        self.hor_spd_stdev = hor_spd_stdev
        self.crc = crc

class NMEACommandType(Enum):
    BESTNAVXYZA = "BESTNAVXYZA"
    BESTNAVA = "BESTNAVA"

class NMEACommandParser():

    @classmethod
    def _parse_header(cls, header_part):
        """Parse the common header format"""
        header_fields = header_part.split(',')
        if len(header_fields) >= 10:
            return ASCIIHEADER(
                message=header_fields[0].replace('#', ''),
                cpu_idle=header_fields[1],
                time_ref=header_fields[2],
                time_status=header_fields[3],
                week_number=header_fields[4],
                milliseconds=header_fields[5],
                reserved=header_fields[6],
                version=header_fields[7],
                leap_sec=header_fields[8],
                output_delay=header_fields[9]
            )

    @classmethod
    def parse_bestnava(cls, line: str) -> BESTNAVA:
        parts = line.split(";")
        if len(parts) != 2:
            return None

        header = cls._parse_header(parts[0])
        data_fields = parts[1].split(",")

        if len(data_fields) > 0 and '*' in data_fields[-1]:
            crc_part = data_fields[-1].split('*')
            data_fields[-1] = crc_part[0]
            data_fields.append(crc_part[1] if len(crc_part) > 1 else '')
        else:
            data_fields.append("")

        # Pad data_fields to ensure we have enough fields
        while len(data_fields) < 30:
            data_fields.append('')

        nav_msg = BESTNAVA(
            header,
            *data_fields
        )

        return nav_msg

    @classmethod
    def parse_bestnavxyza(cls, line: str) -> BESTNAVXYZA:
        """Parse BESTNAVXYZA message"""
        parts = line.split(';')
        if len(parts) != 2:
            return None

        # Parse header
        header = cls._parse_header(parts[0])

        data_fields = parts[1].split(',')

        # Remove CRC from last field if present
        if len(data_fields) > 0 and '*' in data_fields[-1]:
            crc_part = data_fields[-1].split('*')
            data_fields[-1] = crc_part[0]
            data_fields.append(crc_part[1] if len(crc_part) > 1 else '')
        else:
            data_fields.append('')

        # Pad data_fields to ensure we have enough fields
        while len(data_fields) < 29:
            data_fields.append('')

        # Create BESTNAVXYZA object
        nav_msg = BESTNAVXYZA(
            header,
            data_fields[0],   # sol_status
            data_fields[1],   # pos_type
            data_fields[2],   # pos_x
            data_fields[3],   # pos_y
            data_fields[4],   # pos_z
            data_fields[5],   # pos_x_stdev
            data_fields[6],   # pos_y_stdev
            data_fields[7],   # pos_z_stdev
            data_fields[8],   # vel_status
            data_fields[9],   # vel_type
            data_fields[10],  # vel_x
            data_fields[11],  # vel_y
            data_fields[12],  # vel_z
            data_fields[13],  # vel_x_stdev
            data_fields[14],  # vel_y_stdev
            data_fields[15],  # vel_z_stdev
            data_fields[16].replace('"', ''),  # stn_id (remove quotes)
            data_fields[17],  # diff_age
            data_fields[18],  # sol_age
            data_fields[19],  # sol_age2
            data_fields[20],  # num_svs
            data_fields[21],  # num_soln_svs
            data_fields[22],  # num_gg_l1
            data_fields[23],  # num_soln_multi_svs
            data_fields[24],  # reserved
            data_fields[25],  # ext_sol_stat
            data_fields[26] if len(data_fields) > 26 else '',  # galileo_bds3_sig_mask
            data_fields[27] if len(data_fields) > 27 else '',  # gps_glonass_bds2_sig_mask
            data_fields[28]   # crc
        )

        return nav_msg

class UM980Reader:

    class GPSData:
        def __init__(self):
            self.x_ecef = None
            self.y_ecef = None
            self.z_ecef = None
            self.lat = None
            self.lon = None
            self.height = None
            self.pos_type = None
            self.undulation = None
        def __str__(self):
            return f"GPSData(x_ecef={self.x_ecef}, y_ecef={self.y_ecef}, z_ecef={self.z_ecef}, lat={self.lat}, lon={self.lon}, height={self.height}, undulation={self.undulation}, pos_type={self.pos_type})"


    def __init__(self, port: str, baudrate: int):
        self.port = port
        self.baudrate = baudrate
        self.serial_connection = None

        self.running = False
        self.read_thread = None
        self.rover_data = self.GPSData()

    def start(self):
        try:
            self.serial_connection = serial.Serial(port=self.port,baudrate=self.baudrate, timeout=2)
        except Exception as e:
            print(f"Could not connect to serial port: {e}")
            raise e

        try:
            self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
            self.running = True
            self.read_thread.start()
        except Exception as e:
            print(f"Could not start thread: {e}")
            raise e

    def stop(self):
        if self.running:
            self.running = False
            self.read_thread.join()

    def _read_loop(self):
        while self.running:
            try:
                data = self.serial_connection.readline().decode().strip()
                if NMEACommandType.BESTNAVA.value in data:
                    bestnava = NMEACommandParser.parse_bestnava(data)
                    self.rover_data.lat = float(bestnava.lat)
                    self.rover_data.lon = float(bestnava.lon)
                    self.rover_data.height = float(bestnava.hgt)
                    self.rover_data.undulation = float(bestnava.undulation)
                elif NMEACommandType.BESTNAVXYZA.value in data:
                    bestnavxyz = NMEACommandParser.parse_bestnavxyza(data)
                    self.rover_data.x_ecef = bestnavxyz.pos_x
                    self.rover_data.y_ecef = bestnavxyz.pos_y
                    self.rover_data.z_ecef = bestnavxyz.pos_z
                    self.rover_data.pos_type = bestnavxyz.pos_type
                else:
                    continue

            except Exception as e:
                print(f"Read loop stopped: {e}")
                self.serial_connection.close()
                self.running = False
                continue

    def get_data(self) -> "UM980Reader.GPSData":
        return self.rover_data

# --- Global State ---
stop_capture = False
is_rtk_fixed = threading.Event()

FIX_QUALITY = {
    "0": "No Fix", "1": "Standard GPS", "2": "DGPS",
    "4": "RTK Fixed", "5": "RTK Float"
}


def get_base_station_coords(port, baudrate=115200):
    """
    Gets base station coordinates using ASCII protocol (BESTNAVA messages).
    Waits for a position with acceptable accuracy.
    """
    print(f"--- Reading Base Station coordinates from {port}... ---")
    base_reader = None
    try:
        base_reader = UM980Reader(port, baudrate)
        base_reader.start()
        print("✅ Base station reader started")

        # Wait for valid position data
        attempts = 0
        max_attempts = 30  # 30 seconds timeout

        while not stop_capture and attempts < max_attempts:
            try:
                base_data = base_reader.get_data()

                if base_data.lat is not None and base_data.pos_type is not None:
                    pos_type = base_data.pos_type
                    print(f"\r📍 Base station status: {pos_type}", end="")

                    # Accept any position type that has coordinates
                    if pos_type in ['NARROW_INT', 'WIDE_INT', 'L1_FLOAT', 'IONOFREE_FLOAT', 'SINGLE', 'FIXEDPOS']:
                        print(f"\n✅ Base station coordinates acquired: ({base_data.lat:.6f}, {base_data.lon:.6f}, {base_data.height:.2f})")
                        print(f"✅ Position type: {pos_type}")
                        base_reader.stop()
                        return base_data  # Return GPSData object directly

                time.sleep(0.5)  # Shorter sleep for better responsiveness
                attempts += 1

            except KeyboardInterrupt:
                print("\n🛑 Base station coordinate reading interrupted")
                if base_reader:
                    base_reader.stop()
                return None

        print(f"\n⚠️ Timeout waiting for base station coordinates after {max_attempts//2}s")
        if base_reader:
            base_reader.stop()
        return None

    except KeyboardInterrupt:
        print("\n🛑 Base station coordinate reading interrupted")
        if base_reader:
            base_reader.stop()
        return None
    except Exception as e:
        print(f"\n❌ ERROR: Could not read base station coordinates: {e}")
        if base_reader:
            base_reader.stop()
        return None

def create_enu_transformer(base_coords):
    return ProjTransformer.from_crs("EPSG:4979", "EPSG:4978", always_xy=True)

def convert_gps_to_enu_cm(base_coords, rover_data, transformer=None):
    """
    Converts the rover's GPS coordinates to a local East, North, Up (ENU)
    frame relative to the base station using ECEF coordinates and ahrs.ecef2enu.
    Accepts both GPSData objects and dict format for backward compatibility.
    """
    try:
        # Convert base and rover LLA to ECEF using pyproj
        ecef_transformer = ProjTransformer.from_crs("EPSG:4979", "EPSG:4978", always_xy=True)

        # Handle both GPSData objects and dict format
        if hasattr(base_coords, 'lat'):  # GPSData object
            base_lon, base_lat, base_alt = base_coords.lon, base_coords.lat, base_coords.height
        else:  # Dict format (backward compatibility)
            base_lon, base_lat, base_alt = base_coords['lon'], base_coords['lat'], base_coords['alt']

        if hasattr(rover_data, 'lat'):  # GPSData object
            rover_lon, rover_lat, rover_alt = rover_data.lon, rover_data.lat, rover_data.height
        else:  # Dict format (backward compatibility)
            rover_lon, rover_lat, rover_alt = rover_data['longitude'], rover_data['latitude'], rover_data['altitude']

        base_x, base_y, base_z = ecef_transformer.transform(base_lon, base_lat, base_alt)
        rover_x, rover_y, rover_z = ecef_transformer.transform(rover_lon, rover_lat, rover_alt)

        # Calculate ECEF difference
        diff_ecef = np.array([
            rover_x - base_x,
            rover_y - base_y,
            rover_z - base_z
        ])

        # Use ahrs.ecef2enu to get ENU transformation matrix
        enu_matrix = ecef2enu(lat=base_lat, lon=base_lon)

        # Apply transformation
        enu_coords = enu_matrix.dot(diff_ecef)

        return {"x_cm": enu_coords[0] * 100, "y_cm": enu_coords[1] * 100, "z_cm": enu_coords[2] * 100}

    except Exception as e:
        print(f"!!! ERROR during ENU conversion: {e}")
        return {"x_cm": 0.0, "y_cm": 0.0, "z_cm": 0.0}

def signal_handler(sig, frame):
    global stop_capture
    if not stop_capture:
        print('\n🛑 Capture stop requested. Shutting down gracefully...')
        stop_capture = True

def save_radar_vectors(radar_objects, frame_count, rover_position_cm, output_dir, csv_writer):
    file_timestamp = datetime.datetime.now().strftime('%H-%M-%S-%f')
    
    # Write to CSV for each detected object
    csv_writer.writerow(["Frame", frame_count, "Timestamp", file_timestamp, "RTK_Position_cm", rover_position_cm])
    csv_writer.writerow(["Objects_Detected", len(radar_objects)])
    csv_writer.writerow(["Object_ID", "X_m", "Y_m", "Z_m", "Velocity_m/s", "RTK_X_cm", "RTK_Y_cm", "RTK_Z_cm"])
    
    for i, obj in enumerate(radar_objects, 1):
        csv_writer.writerow([
            i, 
            obj.get('x', 0.0), 
            obj.get('y', 0.0), 
            obj.get('z', 0.0), 
            obj.get('v', 0.0),
            rover_position_cm['x_cm'],
            rover_position_cm['y_cm'], 
            rover_position_cm['z_cm']
        ])
    
    csv_writer.writerow([])  # Empty row to separate frames

def get_capture_mode():
    while True:
        print("\n--- Select Capture Mode ---")
        print("1: Capture with RTK position data")
        print("2: Capture without position data (radar only)")
        choice = input("Enter your choice (1 or 2): ")
        if choice == '1':
            return True
        elif choice == '2':
            return False
        else:
            print("Invalid choice. Please enter 1 or 2.")

class AWR2944RadarProcessor:
    """
    AWR2944PEVM radar processor for real-time position vector capture.
    Integrates with RTK GPS system for synchronized data collection.
    """
    def __init__(self, host='192.168.1.10', port=7, transformer=None):
        self.host = host
        self.port = port
        self.buffer = b''
        self.no_of_objects = 0
        self.object_parsed = 0
        self.results = []
        self.running = False
        self.transformer = transformer or RadarTransformer()
        
    def int_from_bytes(self, data):
        return int.from_bytes(data, "little")
    
    def convert_to_float_from_bytes(self, data):
        input_val = self.int_from_bytes(data)
        hex_val = hex(input_val)
        i = int(hex_val[2:], 16)
        cp = pointer(c_int(i))
        fp = cast(cp, POINTER(c_float))
        return float("{:.4f}".format(fp.contents.value))
    
    def process_data_packet(self, data):
        if self.no_of_objects == 0:
            if len(data) >= 4:
                self.no_of_objects = self.int_from_bytes(data[:4])
                if self.no_of_objects > 0:
                    print(f"Frame detected with {self.no_of_objects} objects")
        else:
            self.buffer += data
            
            while len(self.buffer) >= 16 and self.object_parsed < self.no_of_objects:
                position_vector = [
                    self.convert_to_float_from_bytes(self.buffer[0:4]),
                    self.convert_to_float_from_bytes(self.buffer[4:8]),
                    self.convert_to_float_from_bytes(self.buffer[8:12])
                ]
                try:
                    transformed_pos_vector = self.transformer.apply(position_vector)
                    result = {
                        'x': transformed_pos_vector[0],
                        'y': transformed_pos_vector[1],
                        'z': transformed_pos_vector[2],
                        'v': self.convert_to_float_from_bytes(self.buffer[12:16])
                    }
                    
                    self.buffer = self.buffer[16:]
                    self.object_parsed += 1
                    self.results.append(result)
                except Exception as e:
                    print(f"Error processing radar object: {e}")
                    
            if self.object_parsed == self.no_of_objects:
                return self.get_frame_results()
        return None
    
    def get_frame_results(self):
        results = self.results.copy()
        self.reset_frame_state()
        return results
    
    def reset_frame_state(self):
        self.no_of_objects = 0
        self.object_parsed = 0
        self.results = []
        self.buffer = b''

def main():
    global stop_capture
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTSTP, signal_handler)  # Handle Ctrl+Z as interrupt too

    use_rtk = get_capture_mode()

    BASE_STATION_PORT = '/dev/ttyUSB0'  # Port with GPS fix
    ROVER_PORT = '/dev/ttyUSB1'         # Rover port
    RADAR_HOST = '192.168.1.10'
    RADAR_PORT = 7
    BUFFER_SIZE = 1024

    rover_reader = None
    base_station_coords = None
    enu_transformer = None
    radar_processor = None
    radar_socket = None
    csv_file = None
    csv_writer = None

    try:
        if use_rtk:
            base_station_coords = get_base_station_coords(BASE_STATION_PORT, 57600)
            if not base_station_coords or stop_capture:
                print("Could not get Base Station coordinates. Exiting.")
                return

            print(f"\n--- Starting Rover GPS reader on {ROVER_PORT}... ---")
            rover_reader = UM980Reader(port=ROVER_PORT, baudrate=115200)  # Rover baudrate
            try:
                rover_reader.start()
                print("✅ Rover GPS reader started successfully")
            except Exception as e:
                print(f"❌ Failed to start rover GPS reader on {ROVER_PORT}: {e}")
                return
            
            print("\n--- Waiting for Rover to achieve RTK FIXED... ---")
            while not is_rtk_fixed.is_set() and not stop_capture:
                try:
                    rover_data = rover_reader.get_data()
                    if rover_data.pos_type:
                        print(f"\r📍 Rover status: {rover_data.pos_type} (waiting for RTK FIXED...)", end="", flush=True)

                        # Check if RTK is achieved (NARROW_INT or FIXEDPOS = RTK Fixed)
                        if rover_data.pos_type in ['NARROW_INT', 'FIXEDPOS']:
                            print(f"\n✅ Rover achieved RTK FIXED solution! ({rover_data.pos_type})")
                            is_rtk_fixed.set()
                            break
                    else:
                        print(f"\r📍 Rover status: No data (waiting for RTK FIXED...)", end="", flush=True)

                    # Shorter sleep with signal check
                    for _ in range(10):  # 10 x 0.1s = 1s total, but responsive to signals
                        if stop_capture:
                            break
                        time.sleep(0.1)

                except KeyboardInterrupt:
                    print("\n🛑 RTK setup interrupted by user")
                    stop_capture = True
                    break

            if stop_capture:
                print("\n🛑 RTK setup interrupted")
                return

            print("\n🚀 All systems ready! GPS is locked.")
            enu_transformer = create_enu_transformer(base_station_coords)

        else:
            print("\n✅ Skipping RTK setup. Capturing without position data.")

        session_timestamp = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        mode_tag = "rtk" if use_rtk else "no_pos"
        output_filename = f"capture_{mode_tag}_{session_timestamp}.csv"
        print(f"📂 Saving session data to: '{output_filename}'")
        
        # Initialize CSV output
        csv_file = open(output_filename, 'w', newline='')
        csv_writer = csv.writer(csv_file, delimiter=',', quoting=csv.QUOTE_MINIMAL)
        csv_writer.writerow(["AWR2944PEVM Radar Capture Session", session_timestamp])
        csv_writer.writerow(["Mode", "RTK Position Data" if use_rtk else "Radar Only"])
        csv_writer.writerow([])
        
        # Initialize AWR2944 radar processor
        radar_processor = AWR2944RadarProcessor(RADAR_HOST, RADAR_PORT)
        
        # Create TCP server socket to receive radar data with proper error handling
        try:
            radar_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            radar_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            radar_socket.bind((RADAR_HOST, RADAR_PORT))
            radar_socket.listen(1)
            radar_socket.settimeout(1.0)
        except OSError as e:
            print(f"❌ Cannot create radar socket on {RADAR_HOST}:{RADAR_PORT}: {e}")
            print("💡 This is normal if no radar hardware is connected")
            if use_rtk:
                print("🔄 Continuing with GPS-only data collection...")
                # Continue without radar
                radar_socket = None
            else:
                print("❌ No radar socket and no GPS - nothing to do")
                return
        
        input("\nPress ENTER to start AWR2944PEVM capture...")
        if stop_capture: return
        
        print(f"\n📡 AWR2944 radar server listening on {RADAR_HOST}:{RADAR_PORT}")
        print("Waiting for radar connection...")
        
        frame_count = 0
        start_time = time.time()
        conn = None
        
        # Wait for radar connection
        try:
            while not stop_capture and conn is None:
                try:
                    conn, addr = radar_socket.accept()
                    conn.settimeout(5.0)
                    print(f"✅ Radar connected from {addr}")
                except socket.timeout:
                    continue
                    
            if stop_capture:
                return
                
            print("\n📡 Starting data capture...")
            
            # Main data capture loop
            while not stop_capture:
                rover_pos_cm = {"x_cm": 0.0, "y_cm": 0.0, "z_cm": 0.0}

                if use_rtk:
                    if not is_rtk_fixed.is_set():
                        time.sleep(0.1)
                        continue
                    current_rover_data = rover_reader.get_data()
                    if current_rover_data and current_rover_data.lat is not None:
                        # Use GPSData objects directly - no conversion needed
                        rover_pos_cm = convert_gps_to_enu_cm(base_station_coords, current_rover_data, enu_transformer)

                # Receive radar data
                try:
                    data = conn.recv(BUFFER_SIZE)
                    if not data:
                        print("No data received, radar connection lost")
                        break
                    
                    # Process radar data packet
                    radar_objects = radar_processor.process_data_packet(data)
                    
                    if radar_objects is not None and len(radar_objects) > 0:
                        frame_count += 1
                        save_radar_vectors(radar_objects, frame_count, rover_pos_cm, output_filename, csv_writer)
                        csv_file.flush()

                        if frame_count % 10 == 0:
                            elapsed_time = time.time() - start_time
                            fps = frame_count / elapsed_time if elapsed_time > 0 else 0
                            status_line = f"--- Processed {frame_count} frames | FPS: {fps:.2f} "
                            if use_rtk and (rover_pos_cm['x_cm'] != 0 or rover_pos_cm['y_cm'] != 0):
                                pos_line = f"| RTK Pos (cm): x={rover_pos_cm['x_cm']:.1f}, y={rover_pos_cm['y_cm']:.1f}, z={rover_pos_cm['z_cm']:.1f} ---"
                            elif use_rtk:
                                pos_line = "| RTK Pos (cm): Acquiring... ---"
                            else:
                                pos_line = "| Mode: Radar Only ---"
                            obj_line = f" | Objects: {len(radar_objects)}"
                            print(status_line + pos_line + obj_line)
                        
                except socket.timeout:
                    continue
                except ConnectionResetError:
                    print("Radar connection reset")
                    break
                    
        except Exception as e:
            print(f"Error in radar connection: {e}")

    except KeyboardInterrupt:
        print("\nKeyboard interrupt detected in main.")
    except Exception as e:
        print(f"An error occurred in the main loop: {e}")
        traceback.print_exc()
    finally:
        print("\n--- Starting shutdown sequence ---")
        stop_capture = True
        
        if 'conn' in locals() and conn:
            conn.close()
            print("Radar connection closed.")
            
        if radar_socket:
            radar_socket.close()
            print("Radar server socket closed.")
            
        if csv_file:
            csv_file.close()
            print("CSV file saved.")
        
        if rover_reader:
            rover_reader.stop()
        
        print("--- Shutdown complete ---")

if __name__ == "__main__":
    main()

