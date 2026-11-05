/**
 * protocol.h - Estruturas Binárias Compactas para Comunicação HIL Serial
 * EA-291 - Pilotos Automáticos de VANTs (ITA)
 */

#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

#define DOWNLINK_HEADER 0xAA55
#define UPLINK_HEADER   0x55AA

#pragma pack(push, 1)

// Pacote recebido do PC (66 bytes totais)
struct DownlinkPacket {
    uint16_t header;       // 0xAA55
    uint32_t seq;          // Contador sequencial
    float ref_h;           // Altitude de referência [m]
    float ref_VT;          // Velocidade de referência [m/s]
    float ref_theta;       // Arfagem de referência [rad]
    float ref_phi;         // Rolamento de referência [rad]
    float u;               // Velocidade frontal corpo [m/s]
    float v;               // Velocidade lateral corpo [m/s]
    float w;               // Velocidade vertical corpo [m/s]
    float p;               // Taxa de roll [rad/s]
    float q;               // Taxa de pitch [rad/s]
    float r;               // Taxa de yaw [rad/s]
    float phi;             // Ângulo de rolamento [rad]
    float theta;           // Ângulo de arfagem [rad]
    float psi;             // Ângulo de guinada [rad]
    float h;               // Altitude em relação ao nível do mar [m]
    uint8_t enable_ff;     // Bitmask: bit0=Alt-Roll, bit1=Alt-Vel, bit2=Roll-Yaw (7 = todos)
    uint8_t flags;         // Flags auxiliares
    uint16_t checksum;     // Soma aritmética dos bytes anteriores
};

// Pacote enviado para o PC (28 bytes totais)
struct UplinkPacket {
    uint16_t header;       // 0x55AA
    uint32_t seq;          // Eco do contador sequencial
    float delta_e;         // Comando de profundor normalizado [-1.0, +1.0]
    float delta_T;         // Comando de manete [0.0, +1.0]
    float delta_a;         // Comando de aileron normalizado [-1.0, +1.0]
    float delta_r;         // Comando de leme normalizado [-1.0, +1.0]
    float dt_loop;         // Tempo de execução medido [s]
    uint16_t checksum;     // Soma aritmética dos bytes anteriores
};

#pragma pack(pop)

// Função de cálculo de checksum sobre buffer de bytes
inline uint16_t computeChecksum(const uint8_t *data, uint16_t len) {
    uint16_t sum = 0;
    for (uint16_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return sum;
}

#endif // PROTOCOL_H
