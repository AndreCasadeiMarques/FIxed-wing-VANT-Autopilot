"""
scenarios.py - Gerador de Perfis de Referência e Manobras de Teste
EA-291 - Pilotos Automáticos de VANTs (ITA)
Instituto Tecnológico de Aeronáutica - PG-EAM

Implementa os cenários para os modos fundamentais de voo:
1) MODOS LONGITUDINAIS (Nelson, 1998):
   - 'periodo_curto' (Short Period): Dinâmica rápida de arfagem e ângulo de ataque (delta_u ~ 0).
   - 'fugoide' (Phugoid / Long Period): Dinâmica lenta de troca entre energia cinética e potencial (delta_alpha ~ 0).

2) MODOS LÁTERO-DIRECIONAIS (Nelson, 1998):
   - 'modo_rolagem' (Rolling Mode): Modo altamente convergente da taxa de rolamento (p).
   - 'dutch_roll' (Dutch Roll): Modo oscilatório acoplado de guinada e rolamento (r e phi).
   - 'modo_espiral' (Spiral Mode): Modo lento de evolução de proa e rolamento.

3) CENÁRIOS OPERACIONAIS E FEEDFORWARD:
   - 'curva_coordenada': Manobra composta de curva bancada com subida de altitude.
   - 'comparativo_ff': Bateria dupla para avaliar compensadores de Adriano Bittar.
   - 'degrau_altitude': Resposta a degrau de altitude (+20m).
   - 'degrau_velocidade': Resposta a degrau de velocidade (+3m/s).
"""

import math
from typing import NamedTuple

DEG_TO_RAD = 0.017453292519943295

# Ponto de Trimagem padrão (Sato 2021 / Marcelo 2018)
H0 = 100.0         # m (Altitude AGL de equilíbrio)
VT0 = 15.0         # m/s (Velocidade aerodinâmica de trim)
THETA0 = -0.1217   # rad (~ -6.97 deg, arfagem de trim)
PHI0 = 0.0         # rad (rolamento de trim)


class ReferenceSignals(NamedTuple):
    ref_h: float
    ref_VT: float
    ref_theta: float
    ref_phi: float


def get_references(scenario: str, t: float) -> ReferenceSignals:
    """
    Retorna as referências de comando para o instante de tempo t [segundos].
    """
    scen = scenario.lower()

    # =========================================================================
    # 1) MODOS LONGITUDINAIS (NELSON, 1998)
    # =========================================================================

    # 1.1) MODO DE PERÍODO CURTO (Short Period Mode)
    # Movimento rápido de arfagem e ângulo de ataque com velocidade constante (delta_u ~ 0)
    # Excitação: Doublet de arfagem (+-4 graus) em torno do trim
    if scen in ("periodo_curto", "short_period"):
        ref_h = H0
        ref_VT = VT0
        ref_phi = 0.0
        if 4.0 <= t < 5.5:
            ref_theta = THETA0 + (4.0 * DEG_TO_RAD)
        elif 5.5 <= t < 7.0:
            ref_theta = THETA0 - (4.0 * DEG_TO_RAD)
        else:
            ref_theta = THETA0
        return ReferenceSignals(ref_h, ref_VT, ref_theta, ref_phi)

    # 1.2) MODO FUGOIDE (Phugoid / Long Period Mode)
    # Movimento de período longo (>= 30s) caracterizado pela troca entre
    # energia cinética (VT) e energia potencial (h), com delta_alpha ~ 0.
    # Excitação: Degrau de altitude (+20m aos 5s) observado por tempo estendido
    elif scen in ("fugoide", "phugoid"):
        ref_VT = VT0
        ref_phi = 0.0
        ref_theta = THETA0
        if t < 5.0:
            ref_h = H0
        else:
            ref_h = H0 + 20.0
        return ReferenceSignals(ref_h, ref_VT, ref_theta, ref_phi)

    # =========================================================================
    # 2) MODOS LÁTERO-DIRECIONAIS (NELSON, 1998)
    # =========================================================================

    # 2.1) MODO DE ROLAGEM (Rolling Mode)
    # Modo altamente convergente da taxa de rolamento p (polo real rápido)
    # Excitação: Degrau puro de comando de rolamento (+20 deg)
    elif scen in ("modo_rolagem", "rolling_mode", "degrau_rolamento", "roll_step"):
        ref_h = H0
        ref_VT = VT0
        ref_theta = THETA0
        if 4.0 <= t < 16.0:
            ref_phi = 20.0 * DEG_TO_RAD
        else:
            ref_phi = 0.0
        return ReferenceSignals(ref_h, ref_VT, ref_theta, ref_phi)

    # 2.2) MODO DUTCH ROLL (Rolamento Holandês)
    # Movimento oscilatório acoplado de guinada (r) e rolamento (phi)
    # Excitação: Doublet látero-direcional (+-15 deg em phi) para excitar a oscilação acoplada
    elif scen in ("dutch_roll", "rolamento_holandes"):
        ref_h = H0
        ref_VT = VT0
        ref_theta = THETA0
        if 4.0 <= t < 5.5:
            ref_phi = 15.0 * DEG_TO_RAD
        elif 5.5 <= t < 7.0:
            ref_phi = -15.0 * DEG_TO_RAD
        else:
            ref_phi = 0.0
        return ReferenceSignals(ref_h, ref_VT, ref_theta, ref_phi)

    # 2.3) MODO ESPIRAL (Spiral Mode)
    # Movimento lento convergente ou divergente com polo real próximo da origem
    # Excitação: Pequena perturbação inicial de rolamento sustentada para observar a deriva lenta da proa psi
    elif scen in ("modo_espiral", "spiral_mode"):
        ref_h = H0
        ref_VT = VT0
        ref_theta = THETA0
        if 4.0 <= t < 8.0:
            ref_phi = 8.0 * DEG_TO_RAD
        elif t >= 8.0:
            ref_phi = 2.0 * DEG_TO_RAD  # Inclinação branda de equilíbrio para observar espiral
        else:
            ref_phi = 0.0
        return ReferenceSignals(ref_h, ref_VT, ref_theta, ref_phi)

    # =========================================================================
    # 3) CENÁRIOS OPERACIONAIS E FEEDFORWARD
    # =========================================================================

    # 3.1) Degrau de Altitude
    elif scen in ("degrau_altitude", "alt_step"):
        ref_VT = VT0
        ref_phi = 0.0
        ref_theta = THETA0
        if t < 5.0:
            ref_h = H0
        else:
            ref_h = H0 + 20.0
        return ReferenceSignals(ref_h, ref_VT, ref_theta, ref_phi)

    # 3.2) Degrau de Velocidade
    elif scen in ("degrau_velocidade", "speed_step"):
        ref_h = H0
        ref_phi = 0.0
        ref_theta = THETA0
        if t < 5.0:
            ref_VT = VT0
        else:
            ref_VT = VT0 + 3.0  # 18 m/s
        return ReferenceSignals(ref_h, ref_VT, ref_theta, ref_phi)

    # 3.3) Curva Coordenada Composta (Rolamento + Subida)
    elif scen in ("curva_coordenada", "comparativo_ff"):
        ref_VT = VT0
        if 5.0 <= t < 20.0:
            ref_phi = 25.0 * DEG_TO_RAD
        else:
            ref_phi = 0.0

        if t >= 12.0:
            ref_h = H0 + 15.0
        else:
            ref_h = H0
        ref_theta = THETA0
        return ReferenceSignals(ref_h, ref_VT, ref_theta, ref_phi)

    # 3.4) Equilíbrio Estacionário (Default)
    else:
        return ReferenceSignals(H0, VT0, THETA0, PHI0)
