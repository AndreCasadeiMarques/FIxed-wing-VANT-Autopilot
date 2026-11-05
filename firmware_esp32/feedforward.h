/**
 * feedforward.h - Estratégias de Feedforward de Desacoplamento (Adriano Bittar)
 * EA-291 - Pilotos Automáticos de VANTs (ITA)
 */

#ifndef FEEDFORWARD_H
#define FEEDFORWARD_H

#include <math.h>

class FeedForwardCompensators {
public:
    // Ganhos de FeedForward (Bittar 2014)
    float Kpe_r;   // Ganho Altitude <-> Rolamento (Compensação no Profundor)
    float Kpthr;   // Ganho Altitude <-> Velocidade (Compensação na Manete)
    float Kpr_r;   // Ganho Rolamento <-> Guinada (Coordenação de Curva no Leme)

    FeedForwardCompensators(float kpe_r = 0.15f, float kpthr = 0.005f, float kpr_r = 0.25f);

    // 1) Compensação de Profundor durante curva
    float compute_alt_roll_compensation(float phi, float ref_phi);

    // 2) Compensação de Manete durante subida/descida
    float compute_alt_speed_compensation(float error_h);

    // 3) Compensação de Leme para curva coordenada (anti-guinada adversa)
    float compute_roll_yaw_compensation(float ref_phi);
};

#endif // FEEDFORWARD_H
