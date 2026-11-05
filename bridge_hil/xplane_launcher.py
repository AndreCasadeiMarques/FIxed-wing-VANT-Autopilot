"""
xplane_launcher.py - Gerenciador de Inicialização Automática do X-Plane 9
EA-291 - Pilotos Automáticos de VANTs (ITA)

Responsável por:
1) Detectar se o processo X-Plane.exe está em execução;
2) Iniciar o executável com o diretório de trabalho correto (CWD);
3) Lidar em segundo plano com as janelas modais de aviso iniciais (ex: "Understood" / demo mode);
4) Aguardar a prontidão da porta UDP do plugin XPlaneConnect (49009).
"""

import os
import sys
import time
import subprocess
import threading
from typing import Optional

def find_default_xplane_exe() -> Optional[str]:
    """Tenta localizar o executável do X-Plane 9 na estrutura do projeto."""
    base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    candidates = [
        os.path.join(base_dir, "XPlane", "X-Plane 9", "X-Plane.exe"),
        os.path.join(base_dir, "X-Plane 9", "X-Plane.exe"),
        r"C:\X-Plane 9\X-Plane.exe",
    ]
    for c in candidates:
        if os.path.isfile(c):
            return c
    return None


def is_xplane_running() -> bool:
    """Verifica se o processo X-Plane.exe já está em execução no Windows."""
    try:
        out = subprocess.check_output(
            'tasklist /FI "IMAGENAME eq X-Plane.exe" /NH',
            shell=True,
            text=True,
            stderr=subprocess.DEVNULL
        )
        return "X-Plane.exe" in out
    except Exception:
        return False


def _auto_dismiss_dialog_thread(timeout_sec: float = 30.0):
    """
    Thread de segundo plano para fechar a caixa de diálogo modal inicial
    ('BE ADVISED: No X-Plane-9 DISC-1 found... [Understood]').
    """
    if sys.platform != "win32":
        return

    import ctypes
    user32 = ctypes.windll.user32
    start_time = time.time()

    # Aguarda o surgimento da janela do X-Plane
    while time.time() - start_time < timeout_sec:
        hwnd = user32.FindWindowW(None, "X-System")
        if hwnd:
            # Janela encontrada: aguarda 2 segundos para o diálogo modal desenhar
            time.sleep(2.0)
            try:
                user32.SetForegroundWindow(hwnd)
                time.sleep(0.2)
                # Envia tecla Enter para fechar o diálogo "Understood"
                user32.keybd_event(0x0D, 0, 0, 0)
                time.sleep(0.05)
                user32.keybd_event(0x0D, 0, 0x0002, 0)
                time.sleep(0.5)
                # Tenta mais uma vez caso haja segundo aviso
                user32.keybd_event(0x0D, 0, 0, 0)
                time.sleep(0.05)
                user32.keybd_event(0x0D, 0, 0x0002, 0)
            except Exception:
                pass
            break
        time.sleep(0.5)


def launch_xplane(exe_path: Optional[str] = None) -> bool:
    """
    Inicia o X-Plane 9 com o working directory apropriado.
    """
    if exe_path is None:
        exe_path = find_default_xplane_exe()

    if not exe_path or not os.path.isfile(exe_path):
        print(f"[X-Plane] Executavel nao encontrado em: {exe_path}", flush=True)
        return False

    cwd = os.path.dirname(exe_path)
    print(f"[X-Plane] Inicializando simulador: '{exe_path}'...", flush=True)
    try:
        # Inicia processo desacoplado sem travar o chamador
        subprocess.Popen(
            [exe_path],
            cwd=cwd,
            creationflags=subprocess.CREATE_NEW_PROCESS_GROUP if sys.platform == "win32" else 0
        )
        # Dispara thread para lidar com o popup de Understood
        t = threading.Thread(target=_auto_dismiss_dialog_thread, args=(30.0,), daemon=True)
        t.start()
        return True
    except Exception as e:
        print(f"[X-Plane] Falha ao iniciar processo: {e}", flush=True)
        return False


def ensure_xplane_ready(xp_driver, exe_path: Optional[str] = None, max_wait_sec: float = 30.0) -> bool:
    """
    Garante que o X-Plane esteja em execução e respondendo pela interface UDP.
    Se não estiver em execução, inicia automaticamente e aguarda a prontidão.
    """
    # 1) Testa se já está respondendo
    if xp_driver.check_connection(timeout=0.8):
        print("[OK] X-Plane 9 ja em execucao e respondendo via UDP.", flush=True)
        return True

    # 2) Se não estiver respondendo mas o processo existir, aguarda carregar
    if not is_xplane_running():
        launched = launch_xplane(exe_path)
        if not launched:
            return False
    else:
        print("[X-Plane] Processo detectado, aguardando carregamento da aeronave e plugins...", flush=True)

    # 3) Loop de espera ativa com feedback visual
    print(f"[X-Plane] Aguardando prontidao do XPlaneConnect (porta 49009) [max: {max_wait_sec:.0f}s]...", flush=True)
    start_time = time.time()
    dots = 0
    while time.time() - start_time < max_wait_sec:
        time.sleep(1.0)
        dots += 1
        print(f"  -> Conectando ao X-Plane ({dots}s)...", flush=True)
        if xp_driver.check_connection(timeout=0.8):
            print(f"[OK] X-Plane 9 pronto e respondendo com sucesso apos {dots}s!", flush=True)
            return True

    return False
