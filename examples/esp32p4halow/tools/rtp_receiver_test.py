#!/usr/bin/env python3
"""
RTP H.264 수신 테스트 스크립트

ESP32-P4에서 UDP RTP 포트 5600으로 보내는 H.264 스트림을 수신하여:
1. RTP 패킷 파싱 + FU-A 리어셈블리
2. NAL unit 추출 → h264 파일 저장
3. 실시간 통계 출력 (패킷 수, 프레임 수, 비트레이트, 손실률)

사용법:
  python3 rtp_receiver_test.py              # 기본 포트 5600
  python3 rtp_receiver_test.py --port 5600  # 포트 지정
  python3 rtp_receiver_test.py --save       # h264 파일 저장
  python3 rtp_receiver_test.py --vlc        # 수신 후 VLC로 파이프

VLC로 직접 보기 (이 스크립트 없이):
  vlc rtp://@:5600
  또는 SDP 파일 사용:
    echo "v=0\nm=video 5600 RTP/AVP 96\na=rtpmap:96 H264/90000\nc=IN IP4 0.0.0.0" > stream.sdp
    vlc stream.sdp
"""

import socket
import struct
import time
import argparse
import sys
import os
from collections import defaultdict


class RTPPacket:
    """Parse RTP header (RFC 3550)"""
    def __init__(self, data):
        if len(data) < 12:
            raise ValueError("RTP packet too short")
        b0, b1 = data[0], data[1]
        self.version = (b0 >> 6) & 0x3
        self.padding = bool(b0 & 0x20)
        self.extension = bool(b0 & 0x10)
        self.cc = b0 & 0x0F
        self.marker = bool(b1 & 0x80)
        self.payload_type = b1 & 0x7F
        self.seq = struct.unpack('>H', data[2:4])[0]
        self.timestamp = struct.unpack('>I', data[4:8])[0]
        self.ssrc = struct.unpack('>I', data[8:12])[0]

        header_len = 12 + self.cc * 4
        if self.extension and len(data) > header_len + 4:
            ext_len = struct.unpack('>H', data[header_len+2:header_len+4])[0]
            header_len += 4 + ext_len * 4

        self.payload = data[header_len:]


class H264NALReassembler:
    """Reassemble H.264 NAL units from RTP packets (RFC 6184)"""

    def __init__(self):
        self.fua_buffer = bytearray()
        self.fua_nal_header = 0
        self.in_fua = False

    def process_rtp(self, rtp: RTPPacket):
        """Process one RTP packet, yield complete NAL units"""
        if len(rtp.payload) == 0:
            return

        nal_type = rtp.payload[0] & 0x1F

        if nal_type <= 23:
            # Single NAL unit packet
            yield bytes(rtp.payload), nal_type

        elif nal_type == 24:
            # STAP-A (aggregation)
            offset = 1
            while offset + 2 <= len(rtp.payload):
                size = struct.unpack('>H', rtp.payload[offset:offset+2])[0]
                offset += 2
                if offset + size <= len(rtp.payload):
                    nal = rtp.payload[offset:offset+size]
                    yield bytes(nal), nal[0] & 0x1F
                offset += size

        elif nal_type == 28:
            # FU-A (fragmentation)
            if len(rtp.payload) < 2:
                return
            fu_indicator = rtp.payload[0]
            fu_header = rtp.payload[1]
            start = bool(fu_header & 0x80)
            end = bool(fu_header & 0x40)
            frag_nal_type = fu_header & 0x1F

            if start:
                # Reconstruct NAL header
                self.fua_nal_header = (fu_indicator & 0xE0) | frag_nal_type
                self.fua_buffer = bytearray([self.fua_nal_header])
                self.fua_buffer.extend(rtp.payload[2:])
                self.in_fua = True
            elif self.in_fua:
                self.fua_buffer.extend(rtp.payload[2:])

            if end and self.in_fua:
                self.in_fua = False
                yield bytes(self.fua_buffer), frag_nal_type


NAL_TYPE_NAMES = {
    1: "non-IDR",
    5: "IDR",
    6: "SEI",
    7: "SPS",
    8: "PPS",
    9: "AUD",
}


def main():
    parser = argparse.ArgumentParser(description="RTP H.264 수신 테스트")
    parser.add_argument("--port", type=int, default=5600, help="UDP 수신 포트 (default: 5600)")
    parser.add_argument("--save", action="store_true", help="h264 파일로 저장")
    parser.add_argument("--output", default="output.h264", help="저장 파일명 (default: output.h264)")
    parser.add_argument("--duration", type=int, default=0, help="수신 시간 (초, 0=무제한)")
    args = parser.parse_args()

    # UDP 소켓 생성
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('0.0.0.0', args.port))
    sock.settimeout(5.0)

    print(f"=== RTP H.264 수신 테스트 ===")
    print(f"포트 {args.port}에서 대기 중...")
    print(f"ESP32-P4에서 UDP RTP 브로드캐스트가 오면 자동 수신")
    print(f"Ctrl+C로 종료")
    print()

    reassembler = H264NALReassembler()

    # 통계
    pkt_count = 0
    nal_count = 0
    frame_count = 0
    total_bytes = 0
    lost_packets = 0
    last_seq = None
    last_ts = None
    start_time = time.time()
    last_stat_time = start_time
    stat_bytes = 0
    nal_type_counts = defaultdict(int)

    outfile = None
    if args.save:
        outfile = open(args.output, 'wb')
        print(f"저장: {args.output}")

    try:
        while True:
            try:
                data, addr = sock.recvfrom(65536)
            except socket.timeout:
                elapsed = time.time() - start_time
                if pkt_count == 0:
                    print(f"\r  대기 중... ({elapsed:.0f}초)", end="", flush=True)
                continue

            pkt_count += 1
            total_bytes += len(data)
            stat_bytes += len(data)

            try:
                rtp = RTPPacket(data)
            except ValueError:
                continue

            # 첫 패킷
            if last_seq is None:
                print(f"  수신 시작! (from {addr[0]}:{addr[1]}, SSRC=0x{rtp.ssrc:08X})")
                print()

            # 패킷 손실 감지
            if last_seq is not None:
                expected = (last_seq + 1) & 0xFFFF
                if rtp.seq != expected:
                    gap = (rtp.seq - expected) & 0xFFFF
                    if gap < 1000:  # 합리적인 범위
                        lost_packets += gap
            last_seq = rtp.seq

            # 프레임 경계 (marker bit = NAL 마지막 조각, timestamp 변경 = 새 프레임)
            if last_ts is not None and rtp.timestamp != last_ts:
                frame_count += 1
            last_ts = rtp.timestamp

            # NAL 리어셈블리
            for nal_data, nal_type in reassembler.process_rtp(rtp):
                nal_count += 1
                nal_type_counts[nal_type] += 1
                type_name = NAL_TYPE_NAMES.get(nal_type, f"type{nal_type}")

                # IDR/SPS/PPS는 항상 출력
                if nal_type in (5, 7, 8):
                    print(f"  NAL #{nal_count}: {type_name} ({len(nal_data)} bytes)")

                # 파일 저장 (Annex B format: 00 00 00 01 + NAL)
                if outfile:
                    outfile.write(b'\x00\x00\x00\x01')
                    outfile.write(nal_data)

            # 3초마다 통계 출력
            now = time.time()
            if now - last_stat_time >= 3.0:
                elapsed = now - start_time
                bitrate = stat_bytes * 8 / (now - last_stat_time) / 1000
                total_loss_pct = (lost_packets / max(1, pkt_count + lost_packets)) * 100

                types_str = " ".join(f"{NAL_TYPE_NAMES.get(t, f't{t}')}={c}"
                                    for t, c in sorted(nal_type_counts.items()))

                print(f"  [{elapsed:.0f}s] pkts={pkt_count} NALs={nal_count} "
                      f"frames={frame_count} bitrate={bitrate:.0f}kbps "
                      f"loss={total_loss_pct:.1f}% | {types_str}")

                stat_bytes = 0
                last_stat_time = now

            # 시간 제한
            if args.duration > 0 and (now - start_time) >= args.duration:
                break

    except KeyboardInterrupt:
        print("\n\n=== 종료 ===")
    finally:
        elapsed = time.time() - start_time
        if pkt_count > 0:
            avg_bitrate = total_bytes * 8 / max(1, elapsed) / 1000
            print(f"  총 시간: {elapsed:.1f}초")
            print(f"  패킷: {pkt_count} (손실: {lost_packets})")
            print(f"  NAL units: {nal_count}")
            print(f"  프레임: {frame_count}")
            print(f"  평균 비트레이트: {avg_bitrate:.0f} kbps")
            print(f"  총 수신: {total_bytes/1024:.1f} KB")

            if outfile:
                outfile.close()
                fsize = os.path.getsize(args.output)
                print(f"  저장: {args.output} ({fsize/1024:.1f} KB)")
                print(f"\n  재생: ffplay {args.output}")
                print(f"  또는: vlc {args.output}")
        else:
            print("  패킷 수신 없음")

        sock.close()


if __name__ == "__main__":
    main()
