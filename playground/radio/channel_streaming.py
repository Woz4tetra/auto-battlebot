#!/usr/bin/env python3
"""
OpenTX Channel Data Streaming Test Script

This script connects to the OpenTX radio via USB serial and parses the custom
channel data packets along with CRSF telemetry data.

Channel packet format:
- Sync: 0xA5
- Length: packet length excluding sync byte
- Data: channel values as 16-bit signed integers (little-endian)
- Checksum: XOR of all bytes excluding sync

Usage:
    python3 test_channel_streaming.py [--port /dev/ttyACM0] [--baudrate 115200]
"""

import serial
from serial.tools.list_ports import comports
import struct
import argparse
import time
import sys
from typing import List, Optional


def find_transmitter() -> serial.Serial:
    for port in comports():
        if port.pid == 22336 and port.vid == 1155:
            return serial.Serial(port.device, 115200)
    raise RuntimeError("Transmitter not found")


class ChannelStreamingParser:
    def __init__(self):
        self.sync_sequence = bytes([0xA3, 0xA4, 0xA5])
        self.max_channels = 32
        self.channels_per_packet = 16
        self.buffer = bytearray()
        self.stats = {
            'packets_received': 0,
            'packets_corrupted': 0,
            'bytes_received': 0,
            'crsf_packets': 0,
            'phase_0_packets': 0,
            'phase_1_packets': 0
        }
        self.last_channels = [0] * self.max_channels
        self.partial_channels = [None, None]  # Store partial channel sets
        
    def process_data(self, data: bytes) -> List[List[int]]:
        """Process incoming serial data and return list of channel arrays"""
        self.buffer.extend(data)
        self.stats['bytes_received'] += len(data)
        
        channel_updates = []
        
        while len(self.buffer) >= len(self.sync_sequence) + 2:  # Minimum packet size
            # Look for sync sequence (0xA3 0xA4 0xA5)
            sync_index = -1
            for i in range(len(self.buffer) - len(self.sync_sequence) + 1):
                if self.buffer[i:i+len(self.sync_sequence)] == self.sync_sequence:
                    sync_index = i
                    break
            
            if sync_index == -1:
                # No sync sequence found, check for CRSF and clear buffer
                self._check_crsf_data(bytes(self.buffer))
                self.buffer.clear()
                break
                
            # Remove data before sync sequence
            if sync_index > 0:
                discarded = self.buffer[:sync_index]
                self.buffer = self.buffer[sync_index:]
                self._check_crsf_data(discarded)
                
            # Check if we have enough data for phase and length bytes
            if len(self.buffer) < len(self.sync_sequence) + 2:
                break
                
            phase = self.buffer[len(self.sync_sequence)]
            packet_length = self.buffer[len(self.sync_sequence) + 1]
            
            # Check if we have the complete packet
            total_packet_size = len(self.sync_sequence) + 2 + packet_length
            if len(self.buffer) < total_packet_size:
                break
                
            # Extract packet data (phase + length + data + checksum)
            packet_data = self.buffer[len(self.sync_sequence):total_packet_size]
            
            # Remove processed packet from buffer
            self.buffer = self.buffer[total_packet_size:]
            
            # Parse the channel packet
            channels = self._parse_channel_packet_phase(packet_data)
            if channels is not None:
                # Check if we have a complete set of channels
                complete_channels = self._update_channels_from_phase(phase, channels)
                if complete_channels is not None:
                    channel_updates.append(complete_channels)
                self.stats['packets_received'] += 1
                if phase == 0:
                    self.stats['phase_0_packets'] += 1
                else:
                    self.stats['phase_1_packets'] += 1
            else:
                self.stats['packets_corrupted'] += 1
        
        return channel_updates
    
    def _parse_channel_packet_phase(self, packet_data: bytes) -> Optional[List[int]]:
        """Parse phase-based channel data packet"""
        if len(packet_data) < 4:  # phase + length + at least 2 bytes data
            return None
            
        phase = packet_data[0]
        length = packet_data[1]
        
        # Expected: 16 channels * 2 bytes + 1 checksum = 33 bytes
        expected_length = (self.channels_per_packet * 2) + 1
        if length != expected_length:
            print(f"Invalid packet length for phase {phase}: got {length}, expected {expected_length}")
            return None
            
        if len(packet_data) < 2 + length:
            return None
            
        # Calculate expected checksum (phase XOR length XOR all data bytes)
        expected_checksum = phase ^ length
        for i in range(2, len(packet_data) - 1):
            expected_checksum ^= packet_data[i]
            
        received_checksum = packet_data[-1]
        
        if expected_checksum != received_checksum:
            print(f"Checksum mismatch in phase {phase}: expected 0x{expected_checksum:02X}, got 0x{received_checksum:02X}")
            return None
            
        # Extract channel data (skip phase, length, and checksum)
        channel_data = packet_data[2:-1]
        
        if len(channel_data) != self.channels_per_packet * 2:
            print(f"Invalid channel data length in phase {phase}: {len(channel_data)}, expected {self.channels_per_packet * 2}")
            return None
            
        # Unpack 16-bit little-endian signed integers
        channels = []
        for i in range(0, len(channel_data), 2):
            value = struct.unpack('<h', channel_data[i:i+2])[0]
            channels.append(value)
            
        return channels
    
    def _update_channels_from_phase(self, phase: int, channels: List[int]) -> Optional[List[int]]:
        """Update channel array based on phase and return complete set if both phases received"""
        if phase == 0:
            self.partial_channels[0] = channels.copy()
        elif phase == 1:
            self.partial_channels[1] = channels.copy()
        else:
            print(f"Invalid phase: {phase}")
            return None
            
        # Check if we have both phases
        if self.partial_channels[0] is not None and self.partial_channels[1] is not None:
            complete_channels = self.partial_channels[0] + self.partial_channels[1]
            # Reset partial channels for next cycle
            self.partial_channels = [None, None]
            return complete_channels
            
        return None
    
    def _check_crsf_data(self, data: bytes):
        """Check for CRSF packets in the data"""
        # CRSF uses 0xC8 sync byte
        crsf_count = data.count(0xC8)
        if crsf_count > 0:
            self.stats['crsf_packets'] += crsf_count
    
    def get_stats(self) -> dict:
        return self.stats.copy()

def format_channel_value(value: int) -> str:
    """Format channel value for display"""
    # Convert from internal range (-1024 to +1024) to percentage
    percentage = (value / 1024.0) * 100
    return f"{value:5d} ({percentage:+6.1f}%)"

def display_channels(channels: List[int], show_all: bool = False):
    """Display channel values in a readable format"""
    print("\n" + "="*80)
    print(f"Channel Values (Time: {time.strftime('%H:%M:%S')})")
    print("="*80)
    
    if show_all:
        # Show all channels in a grid
        for row in range(8):
            line = ""
            for col in range(4):
                ch_num = row * 4 + col
                if ch_num < len(channels):
                    line += f"CH{ch_num+1:2d}: {format_channel_value(channels[ch_num])}  "
                else:
                    line += f"CH{ch_num+1:2d}: {format_channel_value(0)}  "
            print(line)
    else:
        # Show only first 8 channels
        for i in range(min(8, len(channels))):
            print(f"CH{i+1:2d}: {format_channel_value(channels[i])}")
    
    print("="*80)

def main():
    parser = argparse.ArgumentParser(description='Test OpenTX channel data streaming')
    parser.add_argument('--show-all', action='store_true', help='Show all 32 channels')
    parser.add_argument('--raw', action='store_true', help='Show raw packet data')
    parser.add_argument('--stats-interval', type=int, default=5, help='Statistics display interval in seconds')
    
    args = parser.parse_args()
    
    print(f"OpenTX Channel Data Streaming Test")
    
    try:
        ser = find_transmitter()
        print(f"Connected successfully!")
        print(f"Use Ctrl+C to exit")
    except serial.SerialException as e:
        print(f"Failed to connect: {e}")
        sys.exit(1)
    
    parser = ChannelStreamingParser()
    last_stats_time = time.time()
    ser.write(b"telemetry on\r\n")
    ser.write(b"channels on\r\n")
    
    try:
        while True:
            # Read available data
            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting)
                
                if args.raw:
                    print(f"Raw data ({len(data)} bytes): {data.hex()}")
                
                # Process data
                channel_updates = parser.process_data(data)
                
                # Display each channel update
                for channels in channel_updates:
                    display_channels(channels, args.show_all)
            
            # Display statistics periodically
            current_time = time.time()
            if current_time - last_stats_time >= args.stats_interval:
                stats = parser.get_stats()
                print(f"\nStats: {stats['packets_received']} packets "
                      f"(P0:{stats['phase_0_packets']} P1:{stats['phase_1_packets']}), "
                      f"{stats['packets_corrupted']} corrupted, "
                      f"{stats['bytes_received']} bytes, "
                      f"{stats['crsf_packets']} CRSF packets")
                last_stats_time = current_time
            
            time.sleep(0.01)  # Small delay to prevent CPU spinning
            
    except KeyboardInterrupt:
        print("\nExiting...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        ser.close()
        
        # Final statistics
        stats = parser.get_stats()
        print("\nFinal Statistics:")
        print(f"  Channel packets received: {stats['packets_received']}")
        print(f"    Phase 0 packets: {stats['phase_0_packets']}")
        print(f"    Phase 1 packets: {stats['phase_1_packets']}")
        print(f"  Corrupted packets: {stats['packets_corrupted']}")
        print(f"  Total bytes received: {stats['bytes_received']}")
        print(f"  CRSF packets detected: {stats['crsf_packets']}")
        
        if stats['packets_received'] > 0:
            corruption_rate = (stats['packets_corrupted'] / (stats['packets_received'] + stats['packets_corrupted'])) * 100
            print(f"  Packet corruption rate: {corruption_rate:.1f}%")
            
            phase_balance = abs(stats['phase_0_packets'] - stats['phase_1_packets'])
            print(f"  Phase balance difference: {phase_balance}")

if __name__ == "__main__":
    main()