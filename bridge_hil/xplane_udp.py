"""
xplane_udp.py - Driver de comunicação UDP para X-Plane 9 (Nativo e XPlaneConnect)
EA-291 - Pilotos Automáticos de VANTs (ITA)

Suporta dois modos de comunicação:
1) 'native': Protocolo UDP padrão do X-Plane 9 (RX porta 49010 / TX porta 49000)
2) 'xpc'   : Plugin XPlaneConnect (porta 49009 via DREFs e sendCTRL)
"""

import math
import socket
import struct
from typing import Dict, Optional

# Constantes de conversão
KNOTS_TO_MS = 0.5144444444
FT_TO_M = 0.3048
DEG_TO_RAD = 0.017453292519943295
RAD_TO_DEG = 57.29577951308232
XPLANE_IGNORE = -999.0

try:
    from xpc import XPlaneConnect
    HAS_XPC = True
except ImportError:
    HAS_XPC = False


class XPlaneUDP:
    def __init__(self, mode: str = "native",
                 rx_ip: str = "0.0.0.0", rx_port: int = 49010,
                 tx_ip: str = "127.0.0.1", tx_port: int = 49000,
                 timeout: float = 0.05):
        self.mode = mode.lower()
        self.rx_ip = rx_ip
        self.rx_port = rx_port
        self.tx_ip = tx_ip
        self.tx_port = tx_port
        self.timeout = timeout
        self.xpc_client = None

        # Estados em unidades SI
        self.states = {
            "VT": 15.0,
            "u": 15.0,
            "v": 0.0,
            "w": -1.8343,
            "p": 0.0,
            "q": 0.0,
            "r": 0.0,
            "phi": 0.0,
            "theta": -0.1217,
            "psi": 0.0,
            "h": 100.0,
            "lat": 0.0,
            "lon": 0.0,
        }

        if self.mode == "xpc" and HAS_XPC:
            try:
                self.xpc_client = XPlaneConnect(xpHost=self.tx_ip, xpPort=49009, timeout=int(timeout * 1000))
                print("[XPlaneUDP] Modo XPlaneConnect (XPC) ativado na porta 49009.")
            except Exception as e:
                print(f"[XPlaneUDP] Falha ao iniciar XPC: {e}. Alternando para modo nativo.")
                self.mode = "native"

        if self.mode == "native":
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.sock.bind((self.rx_ip, self.rx_port))
            self.sock.settimeout(self.timeout)
            print(f"[XPlaneUDP] Modo UDP Nativo escutando em {self.rx_ip}:{self.rx_port} e enviando para {self.tx_ip}:{self.tx_port}.")

    def check_connection(self, timeout: float = 1.0) -> bool:
        """Verifica se o X-Plane está ativo e respondendo aos pacotes UDP."""
        if self.mode == "xpc" and self.xpc_client is not None:
            try:
                old_to = self.xpc_client.socket.gettimeout()
                self.xpc_client.socket.settimeout(timeout)
                res = self.xpc_client.getDREF("sim/time/total_running_time_sec")
                self.xpc_client.socket.settimeout(old_to)
                return bool(res and len(res) > 0 and len(res[0]) > 0 and res[0][0] > 0.0)
            except Exception:
                return False
        elif self.mode == "native" and hasattr(self, "sock"):
            try:
                old_to = self.sock.gettimeout()
                self.sock.settimeout(timeout)
                data, _ = self.sock.recvfrom(2048)
                self.sock.settimeout(old_to)
                return len(data) >= 5 and data[:4] == b"DATA"
            except Exception:
                return False
        return False

    def receive_states(self) -> Optional[Dict[str, float]]:
        """Lê os estados atuais da aeronave."""
        if self.mode == "xpc" and self.xpc_client is not None:
            return self._receive_xpc()
        else:
            return self._receive_native()

    def _receive_xpc(self) -> Optional[Dict[str, float]]:
        drefs = [
            "sim/flightmodel/position/true_airspeed",  # m/s
            "sim/flightmodel/position/Prad",           # rad/s
            "sim/flightmodel/position/Qrad",           # rad/s
            "sim/flightmodel/position/Rrad",           # rad/s
            "sim/flightmodel/position/theta",          # deg
            "sim/flightmodel/position/phi",            # deg
            "sim/flightmodel/position/psi",            # deg
            "sim/flightmodel/position/y_agl",          # m AGL
        ]
        try:
            vals = self.xpc_client.getDREFs(drefs)
            if not vals or len(vals) < len(drefs):
                return None

            vt = vals[0][0] if len(vals[0]) > 0 else 15.0
            self.states["VT"] = vt
            self.states["u"] = vt
            self.states["p"] = vals[1][0] if len(vals[1]) > 0 else 0.0
            self.states["q"] = vals[2][0] if len(vals[2]) > 0 else 0.0
            self.states["r"] = vals[3][0] if len(vals[3]) > 0 else 0.0
            self.states["theta"] = (vals[4][0] if len(vals[4]) > 0 else 0.0) * DEG_TO_RAD
            self.states["phi"] = (vals[5][0] if len(vals[5]) > 0 else 0.0) * DEG_TO_RAD
            self.states["psi"] = (vals[6][0] if len(vals[6]) > 0 else 0.0) * DEG_TO_RAD
            self.states["h"] = vals[7][0] if len(vals[7]) > 0 else 100.0
            return self.states
        except Exception:
            return None

    def _receive_native(self) -> Optional[Dict[str, float]]:
        try:
            data, _ = self.sock.recvfrom(2048)
        except socket.timeout:
            return None
        except Exception:
            return None

        if len(data) < 5 or data[:4] != b"DATA":
            return None

        payload = data[5:]
        n_blocks = len(payload) // 36

        for i in range(n_blocks):
            block = payload[i * 36 : (i + 1) * 36]
            idx, *vals = struct.unpack("<i8f", block)

            if idx == 3:  # Speeds
                ktas = vals[2] if vals[2] > 0.0 else vals[0]
                vt = ktas * KNOTS_TO_MS
                self.states["VT"] = vt
                self.states["u"] = vt
            elif idx in (16, 17):  # Angular rates
                self.states["q"] = vals[0] * DEG_TO_RAD
                self.states["p"] = vals[1] * DEG_TO_RAD
                self.states["r"] = vals[2] * DEG_TO_RAD
            elif idx == 18:  # Attitude (pitch, roll, heading) em graus
                self.states["theta"] = vals[0] * DEG_TO_RAD
                self.states["phi"] = vals[1] * DEG_TO_RAD
                self.states["psi"] = vals[2] * DEG_TO_RAD
            elif idx == 20:  # Lat, lon, alt (ft)
                self.states["lat"] = vals[0]
                self.states["lon"] = vals[1]
                self.states["h"] = vals[2] * FT_TO_M

        return self.states

    def init_trim_flight(self, alt_m: float = 100.0, vt_ms: float = 15.0):
        """Inicializa a aeronave em voo estabilizado no ponto de equilíbrio (Trim)."""
        import time
        print(f"[XPlaneUDP] Posicionando aeronave em voo: Altitude={alt_m:.1f} m AGL, Velocidade={vt_ms:.1f} m/s...", flush=True)
        if self.mode == "xpc" and self.xpc_client is not None:
            try:
                # 1) Obter elevação do solo e posição atual
                pos_vals = self.xpc_client.getDREFs([
                    "sim/flightmodel/position/latitude",
                    "sim/flightmodel/position/longitude",
                    "sim/flightmodel/position/elevation",
                    "sim/flightmodel/position/y_agl",
                    "sim/flightmodel/position/psi"
                ])
                lat = pos_vals[0][0] if (pos_vals and len(pos_vals[0]) > 0 and pos_vals[0][0] != 0.0) else 47.25884
                lon = pos_vals[1][0] if (pos_vals and len(pos_vals[1]) > 0 and pos_vals[1][0] != 0.0) else 11.33107
                elev = pos_vals[2][0] if (pos_vals and len(pos_vals[2]) > 0) else 583.67
                y_agl = pos_vals[3][0] if (pos_vals and len(pos_vals[3]) > 0) else 0.0
                psi = pos_vals[4][0] if (pos_vals and len(pos_vals[4]) > 0 and pos_vals[4][0] != 0.0) else 81.0

                grnd_elev = max(0.0, elev - y_agl)
                target_alt_msl = grnd_elev + alt_m

                # 2) Posiciona no ar em altitude AGL com ângulo de arfagem de trim (-6.97 deg)
                self.xpc_client.sendPOSI([lat, lon, target_alt_msl, -6.97, 0.0, psi, 1.0])
                time.sleep(0.05)

                # 3) Desativa freio de estacionamento, ativa luz diurna e visão externa
                self.xpc_client.sendDREF("sim/cockpit2/controls/parking_brake_ratio", 0.0)
                self.xpc_client.sendDREF("sim/graphics/view/view_type", 1018)  # Chase view
                self.xpc_client.sendDREF("sim/time/zulu_time_sec", 43200.0)     # 12:00 Zulu

                # 4) Envia comandos de trim para superfícies e manete
                self.send_controls(0.0155, 0.4914, 0.0, 0.0)
                time.sleep(0.1)
            except Exception as e:
                print(f"[XPlaneUDP] Aviso ao inicializar voo: {e}", flush=True)
        elif self.mode == "native" and hasattr(self, "sock"):
            # Modo nativo: Grupo 20 (alt em pés) e Grupo 3 (velocidade em nós)
            alt_ft = alt_m / FT_TO_M
            vt_kts = vt_ms / KNOTS_TO_MS
            header = b"DATA\x00"
            block_20 = struct.pack(
                "<i8f", 20, XPLANE_IGNORE, XPLANE_IGNORE, alt_ft,
                XPLANE_IGNORE, XPLANE_IGNORE, XPLANE_IGNORE, XPLANE_IGNORE, XPLANE_IGNORE
            )
            block_3 = struct.pack(
                "<i8f", 3, vt_kts, vt_kts, vt_kts,
                XPLANE_IGNORE, XPLANE_IGNORE, XPLANE_IGNORE, XPLANE_IGNORE, XPLANE_IGNORE
            )
            self.sock.sendto(header + block_20 + block_3, (self.tx_ip, self.tx_port))
            self.send_controls(0.0155, 0.4914, 0.0, 0.0)

    def send_controls(self, elevator: float, throttle: float,
                      aileron: float, rudder: float):
        """Envia comandos para as superfícies e manete."""
        elev = max(-1.0, min(1.0, float(elevator)))
        ail  = max(-1.0, min(1.0, float(aileron)))
        rud  = max(-1.0, min(1.0, float(rudder)))
        thr  = max(0.0, min(1.0, float(throttle)))

        if self.mode == "xpc" and self.xpc_client is not None:
            # XPC sendCTRL: [elev, aileron, rudder, throttle, gear, flaps]
            self.xpc_client.sendCTRL([elev, ail, rud, thr, -998, -998])
            self.xpc_client.sendDREF("sim/flightmodel/engine/ENGN_thro", [thr] * 8)
        else:
            # Protocolo Nativo UDP: Grupo 11 e Grupo 25
            header = b"DATA\x00"
            block_11 = struct.pack(
                "<i8f", 11, elev, ail, rud,
                XPLANE_IGNORE, XPLANE_IGNORE, XPLANE_IGNORE, XPLANE_IGNORE, XPLANE_IGNORE
            )
            block_25 = struct.pack(
                "<i8f", 25, thr,
                XPLANE_IGNORE, XPLANE_IGNORE, XPLANE_IGNORE,
                XPLANE_IGNORE, XPLANE_IGNORE, XPLANE_IGNORE, XPLANE_IGNORE
            )
            self.sock.sendto(header + block_11 + block_25, (self.tx_ip, self.tx_port))

    def close(self):
        if self.mode == "native" and hasattr(self, "sock"):
            self.sock.close()
        if self.xpc_client is not None:
            self.xpc_client.close()
