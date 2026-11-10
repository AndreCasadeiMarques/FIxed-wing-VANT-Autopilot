/**
 * firmware_esp32.ino - Firmware do Piloto Automático HIL no ESP32
 * EA-291 - Pilotos Automáticos de VANTs (ITA)
 * 
 * Implementa:
 * 1) Malhas de controle longitudinais e látero-direcionais (Marcelo Santos)
 * 2) Compensadores Feedforward de acoplamento (Adriano Bittar)
 * 3) Protocolo serial binário determinístico com o PC / X-Plane 9
 * 4) Suporte universal a USB CDC nativo do ESP32-C3 (HWCDC / USB-Serial-JTAG)
 */

#include "protocol.h"
#include "controllers.h"
#include "feedforward.h"

// Configuração de canal serial / USB CDC
#if !ARDUINO_USB_CDC_ON_BOOT && SOC_USB_SERIAL_JTAG_SUPPORTED
#include "HWCDC.h"
HWCDC HILSerial;
#define HIL_PORT HILSerial
#else
#define HIL_PORT Serial
#endif

// Ponto de Equilíbrio / Trim do Piper 1/6 (Sato 2021)
const float TRIM_THROTTLE = 0.4914f;  // [0 a 1]
const float TRIM_ELEVATOR = 0.0155f;  // [rad ou normalizado]
const float TRIM_THETA    = -0.1217f; // [rad] (~ -6.97 deg)

// Controladores Longitudinais (Marcelo Santos)
// Velocidade: erro VT -> comando delta_T
DiscretePID pid_speed(0.00644f, 0.0548f, 0.0f, 0.0f, -0.4f, 0.5f);

// Altitude: erro h -> comando theta_ref
DiscretePID pid_alt(0.025f, 0.0015f, 0.0f, 0.0f, -0.26f, 0.26f);

// Arfagem: erro theta -> comando delta_e com amortecimento Kq*q
DiscretePID pid_pitch(0.632f, 0.967f, 0.0f, 0.25f, -0.5f, 0.5f);

// Controladores Látero-Direcionais (Marcelo Santos)
// Rolamento: erro phi -> comando delta_a com amortecimento Kp*p
DiscretePID pid_roll(1.20f, 0.40f, 0.0f, 0.15f, -0.6f, 0.6f);

// Amortecedor de Guinada (Yaw Damper): taxa r -> comando delta_r
const float GAIN_KR = 0.15f;

// Compensadores Feedforward (Adriano Bittar)
FeedForwardCompensators ff(0.20f, 0.005f, 0.25f);

// Buffers de comunicação
DownlinkPacket rx_pkt;
UplinkPacket   tx_pkt;

unsigned long last_time_micros = 0;
// No ESP32-C3 SuperMini o LED onboard é no GPIO 8; em outros DevKits pode ser 2
#if defined(LED_BUILTIN)
const int LED_PIN = LED_BUILTIN;
#else
const int LED_PIN = 8;
#endif

void execute_control_step();

void setup() {
    HIL_PORT.begin(115200);
#if SOC_USB_SERIAL_JTAG_SUPPORTED
    HIL_PORT.setRxBufferSize(512);
    HIL_PORT.setTxTimeoutMs(10);
#endif

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

    last_time_micros = micros();
}

void loop() {
    // Processamento de pacotes recebidos pela porta Serial / USB CDC
    while (HIL_PORT.available() > 0) {
        // Busca de sincronismo pelo header 0xAA55 (primeiro byte no little-endian: 0x55)
        if (HIL_PORT.peek() != (DOWNLINK_HEADER & 0xFF)) {
            HIL_PORT.read(); // Descarta byte fora de sincronismo
            continue;
        }

        // Aguarda a chegada do pacote completo de 66 bytes
        if (HIL_PORT.available() < (int)sizeof(DownlinkPacket)) {
            break;
        }

        // Lê estrutura binária completa
        int bytes_read = HIL_PORT.readBytes((char*)&rx_pkt, sizeof(DownlinkPacket));

        if (bytes_read == sizeof(DownlinkPacket)) {
            // Validação de cabeçalho
            if (rx_pkt.header == DOWNLINK_HEADER) {
                // Validação de Checksum
                uint16_t expected_chk = computeChecksum((uint8_t*)&rx_pkt, sizeof(DownlinkPacket) - 2);
                if (rx_pkt.checksum == expected_chk) {
                    // Pacote válido: executar passo de controle
                    execute_control_step();
                    // Alterna estado do LED
                    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
                }
            }
        }
    }
}

void execute_control_step() {
    unsigned long now_micros = micros();
    float dt = (now_micros - last_time_micros) * 1e-6f;
    if (dt <= 0.0f || dt > 0.1f) dt = 0.02f;
    last_time_micros = now_micros;

    // 1) Malha Longitudinal de Velocidade
    float d_thr = pid_speed.update(rx_pkt.ref_VT, rx_pkt.u, 0.0f, dt);
    float cmd_throttle = TRIM_THROTTLE + d_thr;

    // 2) Malha Longitudinal de Altitude (cascata: h -> theta_ref)
    float d_theta_ref = pid_alt.update(rx_pkt.ref_h, rx_pkt.h, 0.0f, dt);
    
    // Se houver comando explícito de arfagem injetado (ex: doublet no modo de período curto),
    // segue diretamente a referência de theta; caso contrário, segue a malha cascata de altitude
    float target_theta;
    if (fabs(rx_pkt.ref_theta - TRIM_THETA) > 0.005f) {
        target_theta = rx_pkt.ref_theta;
        pid_alt.reset(); // Previne windup do integrador de altitude durante excitação direta de pitch
    } else {
        target_theta = TRIM_THETA + d_theta_ref;
    }

    // 3) Malha Longitudinal de Arfagem (theta -> profundor com amortecedor Kq)
    float d_elev = pid_pitch.update(target_theta, rx_pkt.theta, rx_pkt.q, dt);
    float cmd_elevator = TRIM_ELEVATOR + d_elev;

    // 4) Malha Látero-Direcional de Rolamento (phi -> aileron com amortecedor Kp)
    float cmd_aileron = pid_roll.update(rx_pkt.ref_phi, rx_pkt.phi, rx_pkt.p, dt);

    // 5) Amortecedor de Guinada (Yaw Damper no leme)
    float cmd_rudder = -GAIN_KR * rx_pkt.r;

    // -------------------------------------------------------------
    // 6) COMPENSADORES FEEDFORWARD (ADRIANO BITTAR)
    // -------------------------------------------------------------
    // Bit 0: Feedforward Altitude <-> Rolamento (Profundor em curva)
    if (rx_pkt.enable_ff & 0x01) {
        cmd_elevator += ff.compute_alt_roll_compensation(rx_pkt.phi, rx_pkt.ref_phi);
    }

    // Bit 1: Feedforward Altitude <-> Velocidade (Manete em subida)
    if (rx_pkt.enable_ff & 0x02) {
        cmd_throttle += ff.compute_alt_speed_compensation(rx_pkt.ref_h - rx_pkt.h);
    }

    // Bit 2: Feedforward Rolamento <-> Guinada (Curva coordenada / anti-adverse yaw)
    if (rx_pkt.enable_ff & 0x04) {
        cmd_rudder += ff.compute_roll_yaw_compensation(rx_pkt.ref_phi);
    }

    // Saturações físicas finais das superfícies e motor
    cmd_throttle = clamp(cmd_throttle, 0.0f, 1.0f);
    cmd_elevator = clamp(cmd_elevator, -1.0f, 1.0f);
    cmd_aileron  = clamp(cmd_aileron,  -1.0f, 1.0f);
    cmd_rudder   = clamp(cmd_rudder,   -1.0f, 1.0f);

    // Montar pacote Uplink de resposta
    tx_pkt.header   = UPLINK_HEADER;
    tx_pkt.seq      = rx_pkt.seq;
    tx_pkt.delta_e  = cmd_elevator;
    tx_pkt.delta_T  = cmd_throttle;
    tx_pkt.delta_a  = cmd_aileron;
    tx_pkt.delta_r  = cmd_rudder;
    tx_pkt.dt_loop  = dt;
    tx_pkt.checksum = computeChecksum((uint8_t*)&tx_pkt, sizeof(UplinkPacket) - 2);

    // Transmitir via USB CDC / Serial ao PC
    HIL_PORT.write((uint8_t*)&tx_pkt, sizeof(UplinkPacket));
}
