/**
 * controllers.cpp - Implementação dos Controladores PID Discretos
 * EA-291 - Pilotos Automáticos de VANTs (ITA)
 */

#include "controllers.h"

DiscretePID::DiscretePID(float kp, float ki, float kd, float krate, float min_val, float max_val)
    : Kp(kp), Ki(ki), Kd(kd), Krate(krate), out_min(min_val), out_max(max_val),
      integrator(0.0f), prev_error(0.0f) {}

void DiscretePID::reset() {
    integrator = 0.0f;
    prev_error = 0.0f;
}

float DiscretePID::update(float setpoint, float measurement, float rate, float dt) {
    if (dt <= 0.0f) dt = 0.01f;

    float error = setpoint - measurement;

    // Termo Proporcional
    float P = Kp * error;

    // Termo Integral com Discretização Trapezoidal (Tustin)
    float delta_I = 0.5f * Ki * dt * (error + prev_error);
    integrator += delta_I;

    // Anti-windup: clampa o integrador aos limites da saída
    integrator = clamp(integrator, out_min, out_max);

    // Termo Derivativo (direto na taxa medida para evitar "derivative kick")
    float D = 0.0f;
    if (Kd > 0.0f) {
        D = Kd * (error - prev_error) / dt;
    }

    // Termo de amortecimento SAS (Stability Augmentation System)
    float RateDamping = -Krate * rate;

    // Saída total
    float output = P + integrator + D + RateDamping;

    // Saturação final da superfície / atuador
    output = clamp(output, out_min, out_max);

    prev_error = error;
    return output;
}
