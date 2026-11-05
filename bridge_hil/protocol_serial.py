"""
protocol_serial.py - Protocolo Binário Compacto USB/Serial PC <-> ESP32
EA-291 - Pilotos Automáticos de VANTs (ITA)

Estruturas binárias compactas (struct) com cabeçalhos de sincronismo e checksum.
Garante alta taxa de transmissão (>100 Hz) sem perda de pacotes e sem alocação dinâmica.
"""

import struct
from typing import NamedTuple, Optional, Tuple

DOWNLINK_HEADER = 0xAA55
UPLINK_HEADER   = 0x55AA

# < = little-endian, H = uint16, I = uint32, f = float (4B), B = uint8
# Header(2) + Seq(4) + 14*float(56) + 2*uint8(2) + Checksum(2) = 66 bytes
DOWNLINK_FORMAT = "<HI14f2BH"
DOWNLINK_SIZE = struct.calcsize(DOWNLINK_FORMAT)

# Header(2) + Seq(4) + 5*float(20) + Checksum(2) = 28 bytes
UPLINK_FORMAT = "<HI5fH"
UPLINK_SIZE = struct.calcsize(UPLINK_FORMAT)


class DownlinkData(NamedTuple):
    seq: int
    ref_h: float
    ref_VT: float
    ref_theta: float
    ref_phi: float
    u: float
    v: float
    w: float
    p: float
    q: float
    r: float
    phi: float
    theta: float
    psi: float
    h: float
    enable_ff: int  # Bitmask: bit0=Alt-Roll, bit1=Alt-Vel, bit2=Roll-Yaw (7=todos)
    flags: int = 0


class UplinkData(NamedTuple):
    seq: int
    delta_e: float
    delta_T: float
    delta_a: float
    delta_r: float
    dt_loop: float


def compute_checksum(data_bytes: bytes) -> int:
    """Calcula soma aritmética simples (uint16) sobre o payload."""
    return sum(data_bytes) & 0xFFFF


def pack_downlink(d: DownlinkData) -> bytes:
    """Empacota os estados e referências no formato binário de Downlink."""
    # Empacota primeiro sem o checksum
    body_fmt = "<HI14f2B"
    body = struct.pack(
        body_fmt,
        DOWNLINK_HEADER,
        d.seq,
        d.ref_h,
        d.ref_VT,
        d.ref_theta,
        d.ref_phi,
        d.u,
        d.v,
        d.w,
        d.p,
        d.q,
        d.r,
        d.phi,
        d.theta,
        d.psi,
        d.h,
        d.enable_ff,
        d.flags,
    )
    chk = compute_checksum(body)
    return body + struct.pack("<H", chk)


def unpack_uplink(raw_bytes: bytes) -> Optional[UplinkData]:
    """
    Desempacota os comandos recebidos do ESP32.
    Retorna UplinkData ou None se o pacote for inválido.
    """
    if len(raw_bytes) != UPLINK_SIZE:
        return None

    header, seq, de, dt, da, dr, dt_loop, chk = struct.unpack(UPLINK_FORMAT, raw_bytes)

    if header != UPLINK_HEADER:
        return None

    # Verifica checksum sobre os bytes excluindo os últimos 2 bytes (o próprio chk)
    expected_chk = compute_checksum(raw_bytes[:-2])
    if chk != expected_chk:
        return None

    return UplinkData(
        seq=seq,
        delta_e=de,
        delta_T=dt,
        delta_a=da,
        delta_r=dr,
        dt_loop=dt_loop,
    )
