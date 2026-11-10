"""
bridge_xplane_esp32.py - Ponte de Comunicação HIL (X-Plane 9 UDP <-> ESP32 Serial)
EA-291 - Pilotos Automáticos de VANTs (ITA)

Executa a ponte determinística em tempo real entre o simulador X-Plane 9
e o controlador embarcado no ESP32, gravando logs sincronizados para análise no MATLAB.
"""

import argparse
import csv
import os
import sys
import time
from typing import Optional

try:
    import serial
except ImportError:
    serial = None

from xplane_udp import XPlaneUDP
from xplane_launcher import ensure_xplane_ready
from protocol_serial import (
    DOWNLINK_SIZE,
    UPLINK_SIZE,
    DownlinkData,
    pack_downlink,
    unpack_uplink,
)
import scenarios


def run_bridge(
    port: str,
    baud: int,
    scenario: str,
    duration: float,
    feedforward: bool,
    output_path: str,
    rate_hz: float = 50.0,
    dry_run: bool = False,
    mode: str = "xpc",
    auto_launch: bool = True,
):
    dt_target = 1.0 / rate_hz
    enable_ff_flag = 7 if feedforward else 0  # 7 = todos os 3 compensadores ativos

    print("==================================================")
    print("   EA-291: Ponte HIL X-Plane 9 <-> ESP32 Serial   ")
    print("==================================================")
    print(f" Port Serial   : {port} @ {baud} bps")
    print(f" Cenario       : {scenario}")
    print(f" Duracao       : {duration:.1f} s | Taxa: {rate_hz:.0f} Hz")
    print(f" Feedforward   : {'ATIVADO (Adriano Bittar)' if feedforward else 'DESATIVADO'}")
    print(f" Arquivo Saida : {output_path}")
    print(f" Modo Dry-Run  : {'SIM (Simulacao sem hardware)' if dry_run else 'NAO (Hardware real)'}")
    print("==================================================")

    # 1) Inicializar Serial
    ser = None
    if not dry_run:
        if serial is None:
            print("ERRO: Pacote 'pyserial' nao encontrado.", flush=True)
            sys.exit(1)
        try:
            ser = serial.Serial()
            ser.port = port
            ser.baudrate = baud
            ser.timeout = 0.05
            ser.write_timeout = 0.5  # Protege contra travamento caso o buffer USB encha
            ser.dtr = True          # Necessário para ESP32-C3 USB CDC nativo
            ser.rts = False         # Evita entrar em bootloader
            ser.open()
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            print(f"[OK] Porta serial {port} aberta com sucesso.", flush=True)
            time.sleep(1.0)  # Aguarda estabilizacao do ESP32
        except serial.SerialException as se:
            if "PermissionError" in str(se) or "Acesso negado" in str(se):
                print(f"\n[ERRO CRITICO] Acesso negado a porta {port} (PermissionError)!", flush=True)
                print(">> CAUSA PROVAVEL: O Serial Monitor (Monitor Serial) do Arduino IDE está ABERTO!", flush=True)
                print(">> SOLUCAO: Feche a aba/janela do 'Serial Monitor' ou 'Serial Plotter' no Arduino IDE.", flush=True)
                print("   No Windows, a porta COM e exclusiva e nao pode ser compartilhada.", flush=True)
            else:
                print(f"[FALHA] Nao foi possivel abrir a porta serial {port}: {se}", flush=True)
            sys.exit(1)
        except Exception as e:
            print(f"[FALHA] Nao foi possivel abrir a porta serial {port}: {e}", flush=True)
            print("Dica: Use --dry-run para testar o fluxo sem o hardware conectado.", flush=True)
            sys.exit(1)

    # 2) Inicializar UDP com X-Plane 9 e verificar/garantir conexão ativa
    xp = None
    if not dry_run:
        try:
            xp = XPlaneUDP(mode=mode, rx_port=49010, tx_port=49000, timeout=0.05)
            if auto_launch:
                print("[INFO] Verificando conexao com X-Plane 9 (Auto-Launch ativo)...", flush=True)
                ready = ensure_xplane_ready(xp, max_wait_sec=35.0)
            else:
                print("[INFO] Verificando conexao com o simulador X-Plane 9...", flush=True)
                ready = xp.check_connection(timeout=1.5)

            if ready:
                print("[OK] Simulador X-Plane 9 conectado e pronto em tempo real!", flush=True)
                xp.init_trim_flight(alt_m=100.0, vt_ms=15.0)
                time.sleep(0.5)
            else:
                print("\n" + "=" * 68, flush=True)
                print(" [AVISO CRITICO] X-PLANE 9 NAO DETECTADO OU NAO RESPONDENDO!", flush=True)
                print("=" * 68, flush=True)
                print(" O simulador X-Plane 9 precisa estar ABERTO e executando.", flush=True)
                print(" Verifique se:", flush=True)
                print("   1) O executavel 'X-Plane.exe' foi aberto.", flush=True)
                print("   2) A aeronave (Piper_Sato_Rev00.acf) esta carregada.", flush=True)
                print("   3) O plugin XPlaneConnect esta ativo na porta 49009.", flush=True)
                print("\n Para simular apenas o fluxo de dados sem o simulador,", flush=True)
                print(" ajuste 'dry_run = true;' no script parameters.m.", flush=True)
                print("=" * 68 + "\n", flush=True)
                sys.exit(2)
        except Exception as e:
            print(f"[ERRO] Falha ao conectar ao X-Plane: {e}", flush=True)
            sys.exit(2)

    # 3) Preparar Arquivo de Log CSV
    os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
    log_file = open(output_path, "w", newline="", encoding="utf-8")
    writer = csv.writer(log_file)
    header = [
        "time",
        "ref_h", "ref_VT", "ref_theta", "ref_phi",
        "h", "VT", "u", "v", "w",
        "p", "q", "r",
        "phi", "theta", "psi",
        "delta_e", "delta_T", "delta_a", "delta_r",
        "ff_active", "dt_loop"
    ]
    writer.writerow(header)

    seq = 0
    t_start = time.perf_counter()
    next_tick = t_start

    # Estados simulados caso dry_run ou X-Plane ainda não tenha enviado pacotes
    sim_h = scenarios.H0
    sim_VT = scenarios.VT0
    sim_theta = scenarios.THETA0
    sim_phi = scenarios.PHI0
    sim_p = 0.0
    sim_q = 0.0
    sim_r = 0.0

    # Comandos atuais
    u_elev = 0.0155  # Trim elevator
    u_thr = 0.4914   # Trim throttle
    u_ail = 0.0
    u_rud = 0.0
    dt_esp = dt_target
    ground_recovered = False

    print("\nIniciando loop HIL... Pressione Ctrl+C para encerrar antecipadamente.\n")

    try:
        while True:
            t_now = time.perf_counter()
            t_sim = t_now - t_start
            if t_sim >= duration:
                break

            seq += 1

            # A) Obter referências temporais do cenário
            refs = scenarios.get_references(scenario, t_sim)

            # B) Obter estados da planta (X-Plane ou Dry-Run)
            if not dry_run and xp is not None:
                st = xp.receive_states()
                if st is not None:
                    sim_h = st["h"]
                    sim_VT = st["VT"]
                    sim_theta = st["theta"]
                    sim_phi = st["phi"]
                    sim_p = st["p"]
                    sim_q = st["q"]
                    sim_r = st["r"]

                    # Salvaguarda: se a aeronave for detectada no solo após o início, reposiciona no ar a 100m
                    if t_sim > 0.5 and sim_h < 3.0 and not ground_recovered:
                        print(f"\n[X-Plane] Aeronave no solo detectada (h={sim_h:.1f}m)! Reposicionando em voo trimado a 100m...", flush=True)
                        xp.init_trim_flight(alt_m=100.0, vt_ms=15.0)
                        ground_recovered = True
            else:
                # Simulação simplificada de resposta no dry-run
                sim_h += (refs.ref_h - sim_h) * 0.05
                sim_VT += (refs.ref_VT - sim_VT) * 0.04
                sim_phi += (refs.ref_phi - sim_phi) * 0.1
                sim_theta += (refs.ref_theta - sim_theta) * 0.1

            # C) Montar pacote Downlink para o ESP32
            downlink = DownlinkData(
                seq=seq,
                ref_h=refs.ref_h,
                ref_VT=refs.ref_VT,
                ref_theta=refs.ref_theta,
                ref_phi=refs.ref_phi,
                u=sim_VT,
                v=0.0,
                w=-1.8343,
                p=sim_p,
                q=sim_q,
                r=sim_r,
                phi=sim_phi,
                theta=sim_theta,
                psi=0.0,
                h=sim_h,
                enable_ff=enable_ff_flag,
                flags=0,
            )
            raw_down = pack_downlink(downlink)

            # D) Transmitir ao ESP32 via Serial e ler resposta Uplink
            if not dry_run and ser is not None:
                try:
                    ser.write(raw_down)
                    # Lê resposta de 28 bytes do ESP32
                    raw_up = ser.read(UPLINK_SIZE)
                    up_data = unpack_uplink(raw_up)
                    if up_data is None and len(raw_up) > 0:
                        # Tenta recuperar sincronismo se houver deslocamento de bytes
                        idx = raw_up.find(b"\xaa\x55")
                        if idx > 0:
                            rem = ser.read(idx)
                            raw_up = raw_up[idx:] + rem
                            up_data = unpack_uplink(raw_up)

                    if up_data is not None:
                        u_elev = up_data.delta_e
                        u_thr = up_data.delta_T
                        u_ail = up_data.delta_a
                        u_rud = up_data.delta_r
                        dt_esp = up_data.dt_loop
                    else:
                        # Se não recebeu resposta válida nesta iteração
                        if seq % int(rate_hz) == 0:
                            print(f"[AVISO] Aguardando resposta do ESP32... (bytes lidos: {len(raw_up)}/{UPLINK_SIZE})", flush=True)
                except serial.SerialTimeoutException:
                    if seq % int(rate_hz) == 0:
                        print("[AVISO] Timeout de escrita USB no ESP32! Certifique-se de compilar com 'USB CDC On Boot: Enabled'.", flush=True)
                except Exception as ex:
                    if seq % int(rate_hz) == 0:
                        print(f"[AVISO] Erro na comunicacao serial: {ex}", flush=True)
            else:
                # No dry-run, simula saídas coerentes
                u_elev = 0.0155 - 0.2 * (refs.ref_theta - sim_theta)
                u_thr = 0.4914 + 0.1 * (refs.ref_VT - sim_VT)
                u_ail = 0.1 * (refs.ref_phi - sim_phi)
                u_rud = -0.05 * sim_r

            # E) Enviar comandos de volta ao X-Plane 9 via UDP
            if not dry_run and xp is not None:
                xp.send_controls(u_elev, u_thr, u_ail, u_rud)


            # F) Registrar linha de telemetria no CSV
            writer.writerow([
                f"{t_sim:.4f}",
                f"{refs.ref_h:.4f}", f"{refs.ref_VT:.4f}", f"{refs.ref_theta:.4f}", f"{refs.ref_phi:.4f}",
                f"{sim_h:.4f}", f"{sim_VT:.4f}", f"{sim_VT:.4f}", "0.0000", "-1.8343",
                f"{sim_p:.4f}", f"{sim_q:.4f}", f"{sim_r:.4f}",
                f"{sim_phi:.4f}", f"{sim_theta:.4f}", "0.0000",
                f"{u_elev:.4f}", f"{u_thr:.4f}", f"{u_ail:.4f}", f"{u_rud:.4f}",
                enable_ff_flag, f"{dt_esp:.5f}"
            ])

            # Exibe status a cada 0.5 s
            if seq % int(rate_hz * 0.5) == 0:
                log_file.flush()
                print(
                    f"[{t_sim:5.1f}s / {duration:.0f}s] "
                    f"h={sim_h:5.1f}m (ref:{refs.ref_h:5.1f}) | "
                    f"VT={sim_VT:4.1f}m/s (ref:{refs.ref_VT:4.1f}) | "
                    f"phi={sim_phi*57.3:5.1f}deg | "
                    f"elev={u_elev:+.2f} thr={u_thr:.2f}",
                    flush=True
                )

            # Manter taxa de amostragem constante
            next_tick += dt_target
            sleep_time = next_tick - time.perf_counter()
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\n[INFO] Simulação interrompida pelo usuário.")

    finally:
        log_file.close()
        if ser is not None:
            ser.close()
        if xp is not None:
            xp.close()
        print(f"\n[CONCLUIDO] Telemetria salva com sucesso em: {output_path}")


def main():
    parser = argparse.ArgumentParser(description="Ponte HIL X-Plane 9 <-> ESP32 Serial")
    parser.add_argument("--port", type=str, default="COM3", help="Porta Serial USB do ESP32 (ex: COM3)")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate Serial (padrao: 115200)")
    parser.add_argument(
        "--scenario",
        type=str,
        default="curva_coordenada",
        choices=[
            "periodo_curto", "short_period",
            "fugoide", "phugoid",
            "modo_rolagem", "rolling_mode",
            "dutch_roll",
            "modo_espiral", "spiral_mode",
            "curva_coordenada", "comparativo_ff",
            "degrau_altitude", "degrau_velocidade", "degrau_rolamento", "trim_livre",
            "bateria_degraus", "degraus_sequenciais", "step_battery"
        ],
        help="Cenario de teste para injetar referencias",
    )
    parser.add_argument("--duration", type=float, default=30.0, help="Duracao total do ensaio em segundos")
    parser.add_argument("--feedforward", type=str, default="on", choices=["on", "off"], help="Ativar FeedForward (on/off)")
    parser.add_argument("--output", type=str, default="logs/ultimo_ensaio.csv", help="Caminho do arquivo CSV de log")
    parser.add_argument("--rate", type=float, default=50.0, help="Taxa do loop HIL em Hz (padrao: 50 Hz)")
    parser.add_argument("--mode", type=str, default="xpc", choices=["xpc", "native"], help="Modo de comunicacao (xpc ou native)")
    parser.add_argument("--dry-run", action="store_true", help="Executar modo de teste sem hardware conectado")
    parser.add_argument("--auto-launch", action="store_true", default=True, help="Iniciar o X-Plane automaticamente se nao estiver rodando")
    parser.add_argument("--no-auto-launch", dest="auto_launch", action="store_false", help="Nao iniciar o X-Plane automaticamente")

    args = parser.parse_args()
    ff_bool = args.feedforward.lower() == "on"

    run_bridge(
        port=args.port,
        baud=args.baud,
        scenario=args.scenario,
        duration=args.duration,
        feedforward=ff_bool,
        output_path=args.output,
        rate_hz=args.rate,
        dry_run=args.dry_run,
        mode=args.mode,
        auto_launch=args.auto_launch,
    )


if __name__ == "__main__":
    main()
