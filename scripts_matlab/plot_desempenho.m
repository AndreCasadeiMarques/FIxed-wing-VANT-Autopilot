function plot_desempenho(csv_path_on, csv_path_off, pasta_figuras)
% PLOT_DESEMPENHO - Gera gráficos de análise de desempenho do Piloto Automático HIL
% EA-291 - Pilotos Automáticos de VANTs (ITA)
% Instituto Tecnológico de Aeronáutica - PG-EAM
%
% Padronização Estrita:
%   - Sinal de Referência: SEMPRE em preto pontilhado ('k:', LineWidth 2.0)
%   - Saída COM Feedforward / Ensaio Principal: Azul contínuo ('b-', LineWidth 1.6)
%   - Saída SEM Feedforward: Vermelho tracejado ('r--', LineWidth 1.6)
%   - Rumo psi: Verde escuro traço-ponto para distinguir da referência

if nargin < 2
    csv_path_off = '';
end
if nargin < 3 || isempty(pasta_figuras)
    pasta_figuras = fullfile(fileparts(mfilename('fullpath')), '..', 'figures');
end
if ~exist(pasta_figuras, 'dir')
    mkdir(pasta_figuras);
end

% 1) Carregar dados do ensaio principal
if ~exist(csv_path_on, 'file')
    error('Arquivo CSV nao encontrado: %s', csv_path_on);
end
data_on = readtable(csv_path_on);

RAD2DEG = 57.29577951308232;

% Configuração visual global para artigos/relatórios ITA
set(0, 'defaultAxesFontSize', 10);
set(0, 'defaultLineLineWidth', 1.5);
set(0, 'defaultAxesGridAlpha', 0.3);

% Padrão de cores e estilos
SPEC_REF = 'k:';
WIDTH_REF = 2.0;

%% =========================================================================
%% FIGURA 1: Desempenho do Modo Longitudinal
%% =========================================================================
fig1 = figure('Name', 'Modo Longitudinal HIL', 'Color', 'w', 'Position', [100, 100, 950, 750]);

% Subplot 1: Altitude
subplot(4, 1, 1);
plot(data_on.time, data_on.ref_h, SPEC_REF, 'LineWidth', WIDTH_REF, 'DisplayName', 'h_{ref} (Referencia)');
hold on;
plot(data_on.time, data_on.h, 'b-', 'LineWidth', 1.6, 'DisplayName', 'h (X-Plane)');
grid on;
ylabel('Altitude [m]');
title('Desempenho Longitudinal - Rastreamento e Estabilidade');
legend('Location', 'best');

% Subplot 2: Velocidade Aerodinâmica
subplot(4, 1, 2);
plot(data_on.time, data_on.ref_VT, SPEC_REF, 'LineWidth', WIDTH_REF, 'DisplayName', 'V_{Tr} (Referencia)');
hold on;
plot(data_on.time, data_on.VT, 'b-', 'LineWidth', 1.6, 'DisplayName', 'V_T (X-Plane)');
grid on;
ylabel('Velocidade [m/s]');
legend('Location', 'best');

% Subplot 3: Atitude de Arfagem (theta) e Taxa q
subplot(4, 1, 3);
plot(data_on.time, data_on.ref_theta * RAD2DEG, SPEC_REF, 'LineWidth', WIDTH_REF, 'DisplayName', '\theta_{ref} (Referencia)');
hold on;
plot(data_on.time, data_on.theta * RAD2DEG, 'b-', 'LineWidth', 1.6, 'DisplayName', '\theta (X-Plane)');
plot(data_on.time, data_on.q * RAD2DEG, 'm--', 'LineWidth', 1.2, 'DisplayName', 'q [deg/s]');
grid on;
ylabel('Arfagem [deg]');
legend('Location', 'best');

% Subplot 4: Comandos de Atuação (Profundor e Manete)
subplot(4, 1, 4);
plot(data_on.time, data_on.delta_e, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Profundor \delta_e');
hold on;
plot(data_on.time, data_on.delta_T, 'Color', [0.85 0.33 0.1], 'LineWidth', 1.5, 'DisplayName', 'Manete \delta_T');
yline(1.0, ':', 'Color', [0.6 0.6 0.6], 'LineWidth', 1.0, 'HandleVisibility', 'off');
yline(-1.0, ':', 'Color', [0.6 0.6 0.6], 'LineWidth', 1.0, 'HandleVisibility', 'off');
yline(0.0, ':', 'Color', [0.6 0.6 0.6], 'LineWidth', 1.0, 'HandleVisibility', 'off');
grid on;
xlabel('Tempo [s]');
ylabel('Comando [-]');
ylim([-1.1, 1.1]);
legend('Location', 'best');

% Salvar Figura 1
fig1_path = fullfile(pasta_figuras, 'fig1_desempenho_longitudinal.png');
saveas(fig1, fig1_path);
fprintf('[OK] Grafico longitudinal salvo em: %s\n', fig1_path);

%% =========================================================================
%% FIGURA 2: Desempenho do Modo Látero-Direcional
%% =========================================================================
fig2 = figure('Name', 'Modo Latero-Direcional HIL', 'Color', 'w', 'Position', [150, 150, 950, 650]);

% Subplot 1: Ângulo de Rolamento (phi) e Rumo (psi)
subplot(3, 1, 1);
plot(data_on.time, data_on.ref_phi * RAD2DEG, SPEC_REF, 'LineWidth', WIDTH_REF, 'DisplayName', '\phi_{ref} (Referencia)');
hold on;
plot(data_on.time, data_on.phi * RAD2DEG, 'b-', 'LineWidth', 1.6, 'DisplayName', '\phi (X-Plane)');
plot(data_on.time, data_on.psi * RAD2DEG, 'Color', [0.2 0.6 0.2], 'LineStyle', '-.', 'LineWidth', 1.4, 'DisplayName', '\psi (Rumo)');
grid on;
ylabel('Angulos [deg]');
title('Desempenho Latero-Direcional - Rolamento e Guinada');
legend('Location', 'best');

% Subplot 2: Taxas Angulares (p e r)
subplot(3, 1, 2);
plot(data_on.time, data_on.p * RAD2DEG, 'b-', 'LineWidth', 1.5, 'DisplayName', 'p (roll rate) [deg/s]');
hold on;
plot(data_on.time, data_on.r * RAD2DEG, 'r-', 'LineWidth', 1.5, 'DisplayName', 'r (yaw rate) [deg/s]');
grid on;
ylabel('Taxas [deg/s]');
legend('Location', 'best');

% Subplot 3: Comandos de Atuação (Aileron e Leme)
subplot(3, 1, 3);
plot(data_on.time, data_on.delta_a, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Aileron \delta_a');
hold on;
plot(data_on.time, data_on.delta_r, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Leme \delta_r');
yline(1.0, ':', 'Color', [0.6 0.6 0.6], 'LineWidth', 1.0, 'HandleVisibility', 'off');
yline(-1.0, ':', 'Color', [0.6 0.6 0.6], 'LineWidth', 1.0, 'HandleVisibility', 'off');
yline(0.0, ':', 'Color', [0.6 0.6 0.6], 'LineWidth', 1.0, 'HandleVisibility', 'off');
grid on;
xlabel('Tempo [s]');
ylabel('Comando [-]');
ylim([-1.1, 1.1]);
legend('Location', 'best');

% Salvar Figura 2
fig2_path = fullfile(pasta_figuras, 'fig2_desempenho_laterodirecional.png');
saveas(fig2, fig2_path);
fprintf('[OK] Grafico latero-direcional salvo em: %s\n', fig2_path);

%% =========================================================================
%% FIGURA 3: Comparativo Feedforward (Adriano Bittar) - Se fornecido
%% =========================================================================
if ~isempty(csv_path_off) && exist(csv_path_off, 'file')
    data_off = readtable(csv_path_off);

    fig3 = figure('Name', 'Comparativo Feedforward Bittar', 'Color', 'w', 'Position', [200, 200, 950, 750]);

    % Subplot 1: Desacoplamento Altitude <-> Rolamento
    subplot(3, 1, 1);
    plot(data_on.time, data_on.ref_h, SPEC_REF, 'LineWidth', WIDTH_REF, 'DisplayName', 'Referencia');
    hold on;
    plot(data_on.time, data_on.h, 'b-', 'LineWidth', 1.6, 'DisplayName', 'COM Feedforward (Bittar)');
    plot(data_off.time, data_off.h, 'r--', 'LineWidth', 1.6, 'DisplayName', 'SEM Feedforward');
    grid on;
    ylabel('Altitude [m]');
    title('Efeito dos Compensadores Feedforward de Adriano Bittar');
    legend('Location', 'best');

    % Subplot 2: Desacoplamento Altitude <-> Velocidade
    subplot(3, 1, 2);
    plot(data_on.time, data_on.ref_VT, SPEC_REF, 'LineWidth', WIDTH_REF, 'DisplayName', 'Referencia');
    hold on;
    plot(data_on.time, data_on.VT, 'b-', 'LineWidth', 1.6, 'DisplayName', 'COM Feedforward (Bittar)');
    plot(data_off.time, data_off.VT, 'r--', 'LineWidth', 1.6, 'DisplayName', 'SEM Feedforward');
    grid on;
    ylabel('Velocidade [m/s]');
    legend('Location', 'best');

    % Subplot 3: Coordenação de Curva e Guinada Adversa (Leme)
    subplot(3, 1, 3);
    plot(data_on.time, data_on.delta_r, 'b-', 'LineWidth', 1.6, 'DisplayName', '\delta_r COM Feedforward');
    hold on;
    plot(data_off.time, data_off.delta_r, 'r--', 'LineWidth', 1.6, 'DisplayName', '\delta_r SEM Feedforward');
    yline(0.0, ':', 'Color', [0.6 0.6 0.6], 'LineWidth', 1.0, 'HandleVisibility', 'off');
    grid on;
    xlabel('Tempo [s]');
    ylabel('Leme \delta_r [-]');
    legend('Location', 'best');

    % Salvar Figura 3
    fig3_path = fullfile(pasta_figuras, 'fig3_comparativo_feedforward_bittar.png');
    saveas(fig3, fig3_path);
    fprintf('[OK] Grafico comparativo de Feedforward salvo em: %s\n', fig3_path);
end

end
