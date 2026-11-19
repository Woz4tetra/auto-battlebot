from typing import Optional, get_args

from bw_shared.radio.crsf.crsf_packet import CrsfPacket

PacketResult = tuple[Optional[CrsfPacket], str]


class CrsfParser:
    def __init__(self) -> None:
        self.buffer = b""

    def _parse_single_packet(self, packet: bytes) -> PacketResult:
        crc = packet[-1]
        calc_crc = crc8(packet[2:-1])
        if crc != calc_crc:
            return None, f"CRC mismatch. Expected {calc_crc}, got {crc}"
        command = packet[2]
        payload = packet[3:-1]

        for packet_cls in get_args(CrsfPacket):
            if packet_cls.type == command:
                return packet_cls.from_bytes(payload), ""
        return None, f"Unsupported command {command}"

    def parse(self, buffer: bytes) -> list[PacketResult]:
        buffer = self.buffer + buffer
        results: list[PacketResult] = []
        while len(buffer) > 4:
            if buffer[0] not in CRSF_RC_START_BYTES:
                buffer = buffer[1:]
                continue
            frame_length = buffer[1]
            if not (2 <= frame_length <= 64):
                buffer = buffer[1:]
                continue
            frame_length += 2
            if len(buffer) < frame_length:
                break
            if result := self._parse_single_packet(buffer[:frame_length]):
                packet, error_msg = result
                if packet:
                    results.append((packet, error_msg))
            buffer = buffer[frame_length:]
        self.buffer = buffer
        return results


CRSF_RC_START_BYTES = (0x00, 0xEE, 0xEA)


CRSF_CRC8_POLY = 0xD5


def _crc8_table(poly: int) -> list[int]:
    table = []
    for byte in range(256):
        crc = byte
        for _ in range(8):
            crc = ((crc << 1) ^ poly) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
        table.append(crc)
    return table


crc8tab = _crc8_table(CRSF_CRC8_POLY)


def crc8(buffer):
    crc = 0
    for c in buffer:
        crc = crc8tab[crc ^ c]
    return crc
