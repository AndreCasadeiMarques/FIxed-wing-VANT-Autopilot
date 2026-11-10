%% parameters.m - Parâmetros Centrais de Simulação e Configuração HIL
% EA-291 - Pilotos Automáticos de VANTs (ITA)
% Instituto Tecnológico de Aeronáutica - PG-EAM
%
% Este script centraliza todos os parâmetros de ensaio, condições de trim,
% ganhos do piloto automático e especificações de estilo visual.
% Permite simulações paramétricas simples alterando as variáveis aqui.

%% =========================================================================
%% 1) CONFIGURAÇÃO DA COMUNICAÇÃO E AMBIENTE HIL
%% =========================================================================
% Porta serial conectada ao microcontrolador ESP32-C3
if ~exist('porta_serial', 'var') || isempty(porta_serial)
    porta_serial = 'COM5';
end

% Taxa de transmissão (baudrate)
if ~exist('baud_rate', 'var') || isempty(baud_rate)
    baud_rate = 115200;
end

% =========================================================================
% CENÁRIO DE ENSAIO SELECIONADO:
%
% --- MODOS LONGITUDINAIS (Nelson, 1998) ---
%   'periodo_curto'    -> Modo Período Curto: Dinâmica rápida em arfagem (delta_u ~ 0) (duracao: 15s)
%   'fugoide'          -> Modo Fugoide: Período longo, troca cinética/potencial (delta_alpha ~ 0) (duracao: 45s)
%
% --- MODOS LÁTERO-DIRECIONAIS (Nelson, 1998) ---
%   'modo_rolagem'     -> Modo de Rolagem: Altamente convergente da taxa p (duracao: 20s)
%   'dutch_roll'       -> Modo Dutch Roll: Oscilação acoplada de guinada e rolamento (duracao: 25s)
%   'modo_espiral'     -> Modo Espiral: Movimento lento de evolução de proa e rolamento (duracao: 35s)
%
% --- CENÁRIOS OPERACIONAIS E FEEDFORWARD ---
%   'bateria_degraus'  -> Bateria Multi-DoF de Degraus Sequenciais (Alt -> Vel -> Roll -> Retorno) (duracao: 65s)
%   'comparativo_ff'   -> Bateria dupla: Feedforward ON vs. OFF de Adriano Bittar (duracao: 25s)
%   'curva_coordenada' -> Curva coordenada única com Feedforward ativo (duracao: 25s)
%   'degrau_altitude'  -> Resposta a degrau de altitude (+20m aos 5s) (duracao: 25s)
%   'degrau_velocidade'-> Resposta a degrau de velocidade (+3m/s aos 5s) (duracao: 25s)
% =========================================================================
if ~exist('cenario', 'var') || isempty(cenario)
    cenario = 'comparativo_ff';
end

% Duração padrão nominal de cada cenário
switch cenario
    case {'bateria_degraus', 'degraus_sequenciais', 'step_battery'}
        duracao_padrao = 65.0;
    case {'periodo_curto', 'short_period'}
        duracao_padrao = 15.0;
    case {'fugoide', 'phugoid'}
        duracao_padrao = 45.0;
    case {'modo_espiral', 'spiral_mode'}
        duracao_padrao = 35.0;
    case {'modo_rolagem', 'rolling_mode'}
        duracao_padrao = 20.0;
    otherwise
        duracao_padrao = 25.0;
end

% Se a duração não foi definida OU pertencia ao cenário anterior, sincroniza com o cenário atual
if ~exist('duracao', 'var') || isempty(duracao) || (exist('cenario_anterior', 'var') && ~strcmp(cenario_anterior, cenario))
    duracao = duracao_padrao;
end
cenario_anterior = cenario;

% Taxa de amostragem determinística da malha HIL [Hz]
if ~exist('taxa_hz', 'var') || isempty(taxa_hz)
    taxa_hz = 50.0;
end

% Protocolo de comunicação com o simulador:
%   'xpc'    -> Plugin XPlaneConnect (UDP 49009 via DREFs)
%   'native' -> Protocolo UDP nativo do X-Plane (RX 49010 / TX 49000)
if ~exist('modo_xp', 'var') || isempty(modo_xp)
    modo_xp = 'xpc';
end

% Modo Dry-Run:
%   false -> Conexão HIL real em tempo real com ESP32 e X-Plane 9
%   true  -> Simulação em malha aberta no computador (sem hardware externo)
if ~exist('dry_run', 'var') || isempty(dry_run)
    dry_run = false;
end

% =========================================================================
% INICIALIZAÇÃO AUTOMÁTICA DO X-PLANE 9:
% =========================================================================
% Se true, o MATLAB e a ponte HIL inicializam automaticamente o X-Plane 9
% caso ele ainda não esteja aberto no Windows.
if ~exist('auto_iniciar_xplane', 'var') || isempty(auto_iniciar_xplane)
    auto_iniciar_xplane = true;
end

% Caminho do executável do X-Plane 9 (localizado automaticamente)
if ~exist('caminho_xplane', 'var') || isempty(caminho_xplane)
    caminho_xplane = fullfile(fileparts(mfilename('fullpath')), '..', 'XPlane', 'X-Plane 9', 'X-Plane.exe');
end

%% =========================================================================
%% 2) CONDIÇÕES NOMINAIS DE TRIMAGEM (Marcelo Santos 2018 / Sato 2021)
%% =========================================================================
trim.H0        = 100.0;     % Altitude de equilíbrio AGL [m]
trim.VT0       = 15.0;      % Velocidade aerodinâmica de trim [m/s]
trim.THETA0    = -0.1217;   % Ângulo de arfagem nominal [rad] (~ -6.97 deg)
trim.PHI0      = 0.0;       % Ângulo de rolamento nominal [rad]
trim.u0        = 15.0;      % Componente de velocidade u [m/s]
trim.v0        = 0.0;       % Componente de velocidade v [m/s]
trim.w0        = -1.8343;   % Componente de velocidade w [m/s]
trim.delta_e0  = 0.0155;    % Comando de profundor em trim [-]
trim.delta_T0  = 0.4914;    % Comando de manete em trim [-] (49.14%)

%% =========================================================================
%% 3) GANHOS DO PILOTO AUTOMÁTICO EMBARCADO (ESP32)
%% =========================================================================
% Malha Longitudinal (PID Altitude + Velocidade + Amortecedor de Arfagem Kq)
gains.long.K_theta   = -0.920;   % Ganho proporcional de arfagem
gains.long.K_q       = -0.160;   % Amortecedor de arfagem (Pitch Damping)
gains.long.Kp_h      =  0.025;   % Proporcional da malha de altitude
gains.long.Ki_h      =  0.003;   % Integrativo da malha de altitude
gains.long.Kd_h      =  0.035;   % Derivativo da malha de altitude
gains.long.Kp_v      =  0.080;   % Proporcional da malha de velocidade
gains.long.Ki_v      =  0.020;   % Integrativo da malha de velocidade
gains.long.Kd_v      =  0.010;   % Derivativo da malha de velocidade

% Malha Látero-Direcional (SAS + PID Rolamento)
gains.lat.K_p        = -0.045;   % Amortecedor de rolamento (Roll Damping)
gains.lat.K_r        = -0.120;   % Amortecedor de guinada (Yaw Damper)
gains.lat.Kp_phi     =  0.350;   % Proporcional de rolamento
gains.lat.Ki_phi     =  0.040;   % Integrativo de rolamento

% Compensadores Feedforward (Dissertação de Adriano Bittar, ITA)
gains.ff.K_phi2theta =  0.085;   % Compensador de sustentação (Rolamento -> Profundor)
gains.ff.K_phi2thr   =  0.120;   % Compensador de arrasto induzido (Rolamento -> Manete)
gains.ff.K_phi2rud   =  0.168;   % Compensador de guinada adversa (Rolamento -> Leme)

%% =========================================================================
%% 4) PADRONIZAÇÃO VISUAL DOS GRÁFICOS
%% =========================================================================
estilo.ref_color       = 'k';       % Cor preta para referências
estilo.ref_style       = ':';       % Linha pontilhada estrita
estilo.ref_width       = 2.0;       % Espessura destacada
estilo.ref_spec        = 'k:';      % Especificador resumido ('k:')

estilo.on_color        = 'b';       % Azul para ensaio principal / COM Feedforward
estilo.on_style        = '-';       % Linha contínua
estilo.on_width        = 1.6;

estilo.off_color       = 'r';       % Vermelho para ensaio comparativo SEM Feedforward
estilo.off_style       = '--';      % Linha tracejada
estilo.off_width       = 1.6;

estilo.font_size       = 10;
estilo.grid_alpha      = 0.3;
