/**
 * feedforward.cpp - Implementação dos Compensadores Feedforward de Adriano Bittar
 * EA-291 - Pilotos Automáticos de VANTs (ITA)
 */

#include "feedforward.h"

FeedForwardCompensators::FeedForwardCompensators(float kpe_r, float kpthr, float kpr_r)
    : Kpe_r(kpe_r), Kpthr(kpthr), Kpr_r(kpr_r) {}

// 1) Compensação de Profundor na curva:
// Quando a aeronave inclina (phi != 0), perde sustentação vertical proporcional a (1 - cos(phi)).
// O feedforward adiciona um comando positivo de profundor para manter altitude.
float FeedForwardCompensators::compute_alt_roll_compensation(float phi, float ref_phi) {
    float cos_phi = cosf(phi);
    if (cos_phi < 0.2f) cos_phi = 0.2f; // Proteção contra singularidades em 90 graus
    float loss_factor = 1.0f - cos_phi;
    return Kpe_r * loss_factor;
}

// 2) Compensação de Manete durante variação de altitude:
// Durante comandos de subida (error_h > 0), o profundor aumenta o pitch, freando a aeronave.
// O feedforward antecipa potência no motor para evitar a queda de velocidade.
float FeedForwardCompensators::compute_alt_speed_compensation(float error_h) {
    return Kpthr * error_h;
}

// 3) Compensação de Leme para curva coordenada:
// A deflexão de aileron gera arrasto induzido diferencial (adverse yaw).
// O leme é acionado proporcionalmente à referência de rolamento para manter a curva coordenada.
float FeedForwardCompensators::compute_roll_yaw_compensation(float ref_phi) {
    return Kpr_r * ref_phi;
}
