"""
xpc.py - X-Plane Connect (XPC) Client Library for Python 3.x
Native Python 3.14 compatible implementation.
Ref: NASA XPlaneConnect (https://github.com/nasa/XPlaneConnect)
"""

import socket
import struct
from typing import List, Sequence, Union


class XPlaneConnect:
    """XPlaneConnect facilitates UDP communication to and from the XPlaneConnect plugin."""

    def __init__(self, xpHost: str = "127.0.0.1", xpPort: int = 49009, port: int = 0, timeout: int = 100):
        self.xpHost = socket.gethostbyname(xpHost)
        self.xpPort = xpPort
        self.xpDst = (self.xpHost, self.xpPort)

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.socket.bind(("0.0.0.0", port))
        self.socket.settimeout(timeout / 1000.0)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

    def close(self):
        if self.socket is not None:
            try:
                self.socket.close()
            except Exception:
                pass
            self.socket = None

    def sendUDP(self, buffer: bytes):
        if not buffer:
            raise ValueError("sendUDP: buffer is empty.")
        self.socket.sendto(buffer, self.xpDst)

    def readUDP(self) -> bytes:
        return self.socket.recv(16384)

    def sendCTRL(self, values: Sequence[float], ac: int = 0):
        """
        Sets control surface deflections on the specified aircraft.
        values: [elevator, aileron, rudder, throttle, gear, flaps]
        elevator, aileron, rudder: [-1.0, 1.0]
        throttle: [0.0, 1.0]
        """
        # Formato: b"CTRL\x00" + 4 floats (elev, ail, rud, thr) + 1 byte (gear) + 1 float (flaps) + 1 byte (ac)
        elev = float(values[0]) if len(values) > 0 else -998.0
        ail  = float(values[1]) if len(values) > 1 else -998.0
        rud  = float(values[2]) if len(values) > 2 else -998.0
        thr  = float(values[3]) if len(values) > 3 else -998.0
        gear = int(values[4]) if len(values) > 4 and values[4] != -998 else -1
        flaps= float(values[5]) if len(values) > 5 else -998.0

        buffer = struct.pack("<4sx", b"CTRL")
        buffer += struct.pack("<ffff", elev, ail, rud, thr)
        buffer += struct.pack("b", gear)
        buffer += struct.pack("<f", flaps)
        buffer += struct.pack("B", ac)
        self.sendUDP(buffer)

    def getDREF(self, dref: str) -> List[float]:
        return self.getDREFs([dref])[0]

    def getDREFs(self, drefs: List[str]) -> List[List[float]]:
        """Requests and reads dataref values from X-Plane via XPC."""
        buffer = struct.pack("<4sxB", b"GETD", len(drefs))
        for dref in drefs:
            b_dref = dref.encode("ascii")
            buffer += struct.pack(f"<B{len(b_dref)}s", len(b_dref), b_dref)
        self.sendUDP(buffer)

        try:
            resp = self.readUDP()
        except socket.timeout:
            return [[0.0]] * len(drefs)
        except Exception:
            return [[0.0]] * len(drefs)

        if len(resp) < 6 or resp[:4] != b"RESP":
            return [[0.0]] * len(drefs)

        result_count = struct.unpack_from("B", resp, 5)[0]
        offset = 6
        results = []
        for _ in range(result_count):
            if offset >= len(resp):
                break
            row_len = struct.unpack_from("B", resp, offset)[0]
            offset += 1
            row = list(struct.unpack_from(f"<{row_len}f", resp, offset))
            offset += row_len * 4
            results.append(row)

        return results

    def pauseSim(self, pause: bool):
        val = 1 if pause else 0
        buffer = struct.pack("<4sxB", b"SIMU", val)
        self.sendUDP(buffer)

    def sendPOSI(self, values: Sequence[float], ac: int = 0):
        """
        Sets position information on the specified aircraft.
        values: [lat, lon, alt (m MSL), pitch (deg), roll (deg), true_heading (deg), gear (0/1)]
        Uses 46-byte double-precision LLA format (XPC 1.3+) for X-Plane 9 compatibility.
        """
        vals = [float(values[i]) if i < len(values) else -998.0 for i in range(7)]
        buffer = struct.pack(
            "<4sxBdddffff",
            b"POSI",
            ac,
            vals[0],  # lat (double)
            vals[1],  # lon (double)
            vals[2],  # alt m MSL (double)
            vals[3],  # pitch deg (float)
            vals[4],  # roll deg (float)
            vals[5],  # heading deg (float)
            vals[6],  # gear (float)
        )
        self.sendUDP(buffer)

    def sendDREF(self, dref: str, value: Union[float, Sequence[float]]):
        """Sets the specified dataref to the specified value."""
        self.sendDREFs([dref], [value])

    def sendDREFs(self, drefs: List[str], values: List[Union[float, Sequence[float]]]):
        """Sets the specified datarefs to the specified values."""
        buffer = struct.pack("<4sx", b"DREF")
        for dref, val in zip(drefs, values):
            b_dref = dref.encode("ascii")
            if hasattr(val, "__len__"):
                fmt = f"<B{len(b_dref)}sB{len(val)}f"
                buffer += struct.pack(fmt, len(b_dref), b_dref, len(val), *[float(x) for x in val])
            else:
                fmt = f"<B{len(b_dref)}sBf"
                buffer += struct.pack(fmt, len(b_dref), b_dref, 1, float(val))
        self.sendUDP(buffer)

