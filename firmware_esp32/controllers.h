/**
 * controllers.h - Implementação das Malhas de Controle PID Discretas (Marcelo Santos)
 * EA-291 - Pilotos Automáticos de VANTs (ITA)
 */

#ifndef CONTROLLERS_H
#define CONTROLLERS_H

#include <math.h>

class DiscretePID {
public:
    float Kp;
    float Ki;
    float Kd;
    float Krate;      // Ganho de amortecimento na taxa angular (q, p, r)
    float out_min;    // Limite inferior de saturação
    float out_max;    // Limite superior de saturação

    float integrator;
    float prev_error;

    DiscretePID(float kp, float ki, float kd, float krate, float min_val, float max_val);

    void reset();
    float update(float setpoint, float measurement, float rate, float dt);
};

// Funções utilitárias de saturação
inline float clamp(float val, float min_v, float max_v) {
    if (val < min_v) return min_v;
    if (val > max_v) return max_v;
    return val;
}

#endif // CONTROLLERS_H
