#!/usr/bin/env python3
"""
ESP32-P4 HaLow H.264 RTP Stream Viewer

Receives RTP H.264 (RFC 6184) on UDP port 5600 and displays via OpenCV.
Handles Single NAL and FU-A fragmentation.

Usage:
    pip install opencv-python av
    python h264_viewer.py [--port 5600] [--bind 0.0.0.0]

Press 'q' to quit, 's' to save screenshot.
"""

import socket
import struct
import sys
import time
import argparse
import threading
from collections import deque

try:
    import cv2
    import av
except ImportError:
    print("Required: pip install opencv-python av")
    sys.exit(1)


class RTPParser:
    """Parse RTP packets and reassemble H.264 NAL units."""

    def __init__(self):
        self.fua_buf = bytearray()
        self.fua_nal_header = 0
        self.last_seq = -1
        self.seq_drops = 0
        self.pkt_count = 0

    def parse(self, data):
        """Parse one RTP packet. Yields complete NAL units."""
        if len(data) < RTP_HDR_SIZE:
            return

        # RTP header (12 bytes minimum)
        byte0, byte1, seq, ts, ssrc = struct.unpack('!BBHII', data[:12])
        pt = byte1 & 0x7F
        marker = bool(byte1 & 0x80)
        cc = byte0 & 0x0F
        payload_offset = 12 + cc * 4

        if len(data) <= payload_offset:
            return

        self.pkt_count += 1

        # Sequence continuity check
        if self.last_seq >= 0:
            expected = (self.last_seq + 1) & 0xFFFF
            if seq != expected:
                gap = (seq - self.last_seq) & 0xFFFF
                self.seq_drops += gap - 1
                # Discard any in-progress FU-A on sequence gap
                self.fua_buf.clear()
        self.last_seq = seq

        payload = data[payload_offset:]
        nal_type = payload[0] & 0x1F

        if 1 <= nal_type <= 23:
            # Single NAL unit packet
            yield bytes(payload), marker

        elif nal_type == 28:
            # FU-A fragmentation
            if len(payload) < 2:
                return
            fu_indicator = payload[0]
            fu_header = payload[1]
            start = bool(fu_header & 0x80)
            end = bool(fu_header & 0x40)
            frag_nal_type = fu_header & 0x1F

            if start:
                # Reconstruct NAL header: NRI from FU indicator + type from FU header
                self.fua_nal_header = (fu_indicator & 0xE0) | frag_nal_type
                self.fua_buf = bytearray([self.fua_nal_header])
                self.fua_buf.extend(payload[2:])
            elif self.fua_buf:
                self.fua_buf.extend(payload[2:])

            if end and self.fua_buf:
                yield bytes(self.fua_buf), marker
                self.fua_buf.clear()

        # Type 24-27 (STAP/MTAP) not used by our encoder


RTP_HDR_SIZE = 12
ANNEX_B_PREFIX = b'\x00\x00\x00\x01'


class H264Decoder:
    """Decode H.264 NAL units using PyAV."""

    def __init__(self):
        self.codec = av.CodecContext.create('h264', 'r')
        self.codec.thread_type = 'AUTO'
        self.codec.open()
        self.frame_count = 0
        self.last_frame = None
        self.last_frame_time = 0

    def decode(self, nalus):
        """Decode a list of NAL units (one frame's worth). Returns decoded frames."""
        if not nalus:
            return []

        # Build Annex-B bitstream
        bitstream = bytearray()
        for nalu in nalus:
            bitstream.extend(ANNEX_B_PREFIX)
            bitstream.extend(nalu)

        packet = av.Packet(bytes(bitstream))
        frames = []
        try:
            decoded = self.codec.decode(packet)
            for frame in decoded:
                img = frame.to_ndarray(format='bgr24')
                frames.append(img)
                self.frame_count += 1
                self.last_frame = img
                self.last_frame_time = time.time()
        except av.error.InvalidDataError:
            pass
        return frames


class StreamStats:
    """Track streaming statistics."""

    def __init__(self):
        self.start_time = time.time()
        self.bytes_recv = 0
        self.frames_decoded = 0
        self.fps_window = deque(maxlen=30)
        self.last_print = 0

    def update(self, nbytes, nframes):
        self.bytes_recv += nbytes
        self.frames_decoded += nframes
        now = time.time()
        for _ in range(nframes):
            self.fps_window.append(now)

    def get_fps(self):
        if len(self.fps_window) < 2:
            return 0.0
        dt = self.fps_window[-1] - self.fps_window[0]
        if dt <= 0:
            return 0.0
        return (len(self.fps_window) - 1) / dt

    def print_stats(self, rtp):
        now = time.time()
        if now - self.last_print < 3.0:
            return
        self.last_print = now
        elapsed = now - self.start_time
        kbps = (self.bytes_recv * 8 / 1000 / elapsed) if elapsed > 0 else 0
        fps = self.get_fps()
        print(f"[stats] {elapsed:.0f}s | {fps:.1f} fps | {kbps:.0f} kbps | "
              f"pkts={rtp.pkt_count} drops={rtp.seq_drops} | "
              f"frames={self.frames_decoded}")


def main():
    parser = argparse.ArgumentParser(description='ESP32-P4 H.264 RTP Viewer')
    parser.add_argument('--port', type=int, default=5600, help='UDP port (default: 5600)')
    parser.add_argument('--bind', default='0.0.0.0', help='Bind address (default: 0.0.0.0)')
    parser.add_argument('--save-dir', default='.', help='Screenshot save directory')
    args = parser.parse_args()

    # UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((args.bind, args.port))
    sock.settimeout(0.5)

    print(f"Listening for RTP H.264 on {args.bind}:{args.port}")
    print("Waiting for stream... (press 'q' in window to quit)")

    rtp = RTPParser()
    decoder = H264Decoder()
    stats = StreamStats()

    # Collect NALUs for the current frame (until RTP marker bit)
    frame_nalus = []
    window_name = 'ESP32-P4 HaLow Camera'
    screenshot_count = 0

    try:
        while True:
            try:
                data, addr = sock.recvfrom(2048)
            except socket.timeout:
                # Check for keypress even when no data
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                continue

            stats.update(len(data), 0)

            for nalu, marker in rtp.parse(data):
                frame_nalus.append(nalu)

                if marker:
                    # End of frame — decode
                    frames = decoder.decode(frame_nalus)
                    frame_nalus = []

                    stats.update(0, len(frames))
                    stats.print_stats(rtp)

                    for img in frames:
                        # Draw OSD
                        fps = stats.get_fps()
                        h, w = img.shape[:2]
                        osd = f"{w}x{h} | {fps:.1f} fps | pkts={rtp.pkt_count} drop={rtp.seq_drops}"
                        cv2.putText(img, osd, (10, 25),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                        cv2.imshow(window_name, img)

                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    raise KeyboardInterrupt
                elif key == ord('s') and decoder.last_frame is not None:
                    fname = f"screenshot_{screenshot_count:04d}.png"
                    cv2.imwrite(fname, decoder.last_frame)
                    print(f"Saved: {fname}")
                    screenshot_count += 1

    except KeyboardInterrupt:
        pass
    finally:
        elapsed = time.time() - stats.start_time
        print(f"\n--- Session Summary ---")
        print(f"Duration: {elapsed:.1f}s")
        print(f"Frames decoded: {stats.frames_decoded}")
        print(f"Average FPS: {stats.frames_decoded / elapsed:.1f}" if elapsed > 0 else "")
        print(f"Data received: {stats.bytes_recv / 1024:.1f} KB")
        print(f"RTP packets: {rtp.pkt_count} (drops: {rtp.seq_drops})")
        sock.close()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
