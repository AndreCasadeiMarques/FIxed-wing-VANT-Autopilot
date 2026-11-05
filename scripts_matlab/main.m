%% main.m - Execução Principal do Ensaio HIL e Geração de Resultados
% EA-291 - Pilotos Automáticos de VANTs (ITA)
% Instituto Tecnológico de Aeronáutica - PG-EAM
%
% Este script gerencia:
% 1) Carregamento dos parâmetros definidos em parameters.m
% 2) Verificação prévia da conexão e processo do X-Plane 9
% 3) Execução automatizada da ponte HIL em tempo real (ESP32 <-> X-Plane 9)
% 4) Leitura dos logs de telemetria e cálculo dos índices de mérito (tr, ts, Mp, ess)
% 5) Geração automática dos gráficos padronizados em figures/

close all; clc;

fprintf('=================================================================\n');
fprintf('   EA-291: Orquestrador HIL Autopilot ESP32 & X-Plane 9 (ITA)    \n');
fprintf('=================================================================\n');

% 1) Carregamento dos parâmetros de simulação
if ~exist('parameters.m', 'file') && ~exist('parameters', 'file')
    error('Arquivo parameters.m nao encontrado! Execute a partir da pasta scripts_matlab.');
end
parameters;
fprintf('[CONFIG] Cenario: %s | Duracao: %.1fs | Taxa: %.0fHz | Porta: %s\n', ...
    cenario, duracao, taxa_hz, porta_serial);

% Diretórios do projeto
root_dir = fullfile(fileparts(mfilename('fullpath')), '..');
bridge_script = fullfile(root_dir, 'bridge_hil', 'bridge_xplane_esp32.py');
logs_dir = fullfile(root_dir, 'logs');
figures_dir = fullfile(root_dir, 'figures');

if ~exist(logs_dir, 'dir'), mkdir(logs_dir); end
if ~exist(figures_dir, 'dir'), mkdir(figures_dir); end

%% 2) Verificação e Inicialização Prévia do Simulador X-Plane 9
if ~dry_run
    if ispc
        [~, task_out] = system('tasklist /FI "IMAGENAME eq X-Plane.exe" /NH');
        if ~contains(task_out, 'X-Plane.exe')
            if auto_iniciar_xplane && exist(caminho_xplane, 'file')
                fprintf('[X-Plane] Simulador fechado. Inicializando automaticamente X-Plane 9...\n');
                [pasta_xp, ~, ~] = fileparts(caminho_xplane);
                cmd_start = sprintf('start "" /D "%s" "%s"', pasta_xp, caminho_xplane);
                system(cmd_start);
                fprintf('[X-Plane] Processo disparado. A ponte HIL sincronizara assim que a porta UDP estiver pronta.\n');
            else
                fprintf('\n*****************************************************************\n');
                fprintf(' [AVISO PREVIO] O processo X-Plane.exe NAO esta em execucao!\n');
                fprintf(' Certifique-se de que o simulador esta aberto antes do ensaio.\n');
                fprintf(' (Para rodar sem o X-Plane aberto, defina dry_run = true em parameters.m)\n');
                fprintf('*****************************************************************\n\n');
            end
        else
            fprintf('[OK] Processo X-Plane.exe detectado em execucao no Windows.\n');
        end
    end
else
    fprintf('[INFO] Modo Dry-Run ativado em parameters.m (Simulacao sem simulador/hardware).\n');
end

%% 3) Execução dos Ensaios HIL
auto_launch_flag = tern(auto_iniciar_xplane, '--auto-launch', '--no-auto-launch');

if strcmp(cenario, 'comparativo_ff')
    fprintf('\n[MODO COMPARATIVO] Executando bateria com FeedForward ON e OFF...\n');
    
    % Ensaio 1: FeedForward ATIVADO
    log_on = fullfile(logs_dir, 'ensaio_curva_ff_ON.csv');
    cmd_on = sprintf('python -u "%s" --port %s --baud %d --scenario curva_coordenada --duration %.1f --feedforward on --output "%s" --rate %.1f --mode %s %s %s', ...
        bridge_script, porta_serial, baud_rate, duracao, log_on, taxa_hz, modo_xp, tern(dry_run, '--dry-run', ''), auto_launch_flag);
    
    fprintf('\n-> Executando Ensaio 1/2 (FeedForward ON)...\n');
    status1 = system(cmd_on);
    if status1 ~= 0
        error(['Falha na execucao do Ensaio 1. Verifique o aviso acima ' ...
               '(ex: X-Plane fechado ou porta Serial COM ocupada/desconectada).']);
    end
    
    % Ensaio 2: FeedForward DESATIVADO
    log_off = fullfile(logs_dir, 'ensaio_curva_ff_OFF.csv');
    cmd_off = sprintf('python -u "%s" --port %s --baud %d --scenario curva_coordenada --duration %.1f --feedforward off --output "%s" --rate %.1f --mode %s %s %s', ...
        bridge_script, porta_serial, baud_rate, duracao, log_off, taxa_hz, modo_xp, tern(dry_run, '--dry-run', ''), auto_launch_flag);
    
    fprintf('\n-> Executando Ensaio 2/2 (FeedForward OFF)...\n');
    status2 = system(cmd_off);
    if status2 ~= 0
        error('Falha na execucao do Ensaio 2. Verifique o erro acima.');
    end
    
    log_principal = log_on;
    log_secundario = log_off;
else
    log_principal = fullfile(logs_dir, sprintf('ensaio_%s.csv', cenario));
    log_secundario = '';
    
    cmd = sprintf('python -u "%s" --port %s --baud %d --scenario %s --duration %.1f --feedforward on --output "%s" --rate %.1f --mode %s %s %s', ...
        bridge_script, porta_serial, baud_rate, cenario, duracao, log_principal, taxa_hz, modo_xp, tern(dry_run, '--dry-run', ''), auto_launch_flag);
    
    fprintf('\n-> Disparando Ponte HIL: %s...\n', cenario);
    status = system(cmd);
    if status ~= 0
        error(['Falha na execucao da ponte HIL. Verifique o aviso acima ' ...
               '(ex: X-Plane fechado ou porta Serial COM ocupada/desconectada).']);
    end
end

%% 4) Carregamento dos Dados e Análise de Mérito
if ~exist(log_principal, 'file')
    error('Arquivo de log nao foi gerado. Verifique a comunicacao da ponte.');
end

fprintf('\n=================================================================\n');
fprintf('   Processando Telemetria e Calculando Índices de Mérito...     \n');
fprintf('=================================================================\n');

dados = readtable(log_principal);

% Cálculo automático de métricas conforme o cenário
switch cenario
    case {'periodo_curto', 'short_period'}
        m_theta = calcula_metricas(dados.time, dados.theta * 57.3, dados.ref_theta * 57.3, 'Arfagem \theta (Periodo Curto)', 'deg');
    case {'fugoide', 'phugoid'}
        m_h = calcula_metricas(dados.time, dados.h, dados.ref_h, 'Altitude h (Fugoide)', 'm');
        m_vt = calcula_metricas(dados.time, dados.VT, dados.ref_VT, 'Velocidade VT (Fugoide)', 'm/s');
    case {'modo_rolagem', 'rolling_mode', 'degrau_rolamento', 'roll_step'}
        m_phi = calcula_metricas(dados.time, dados.phi * 57.3, dados.ref_phi * 57.3, 'Rolamento \phi (Modo de Rolagem)', 'deg');
    case {'dutch_roll', 'rolamento_holandes'}
        m_phi = calcula_metricas(dados.time, dados.phi * 57.3, dados.ref_phi * 57.3, 'Rolamento \phi (Dutch Roll)', 'deg');
    case {'modo_espiral', 'spiral_mode'}
        m_phi = calcula_metricas(dados.time, dados.phi * 57.3, dados.ref_phi * 57.3, 'Rolamento \phi (Modo Espiral)', 'deg');
    case {'degrau_altitude', 'alt_step'}
        m_h = calcula_metricas(dados.time, dados.h, dados.ref_h, 'Altitude', 'm');
    case {'degrau_velocidade', 'speed_step'}
        m_vt = calcula_metricas(dados.time, dados.VT, dados.ref_VT, 'Velocidade VT', 'm/s');
    case {'curva_coordenada', 'comparativo_ff'}
        m_h = calcula_metricas(dados.time, dados.h, dados.ref_h, 'Altitude em Curva', 'm');
        m_vt = calcula_metricas(dados.time, dados.VT, dados.ref_VT, 'Velocidade em Curva', 'm/s');
end

%% 5) Geração Automática dos Gráficos
fprintf('\n-> Gerando graficos de desempenho em figures/...\n');
plot_desempenho(log_principal, log_secundario, figures_dir);

fprintf('\n=================================================================\n');
fprintf('   Ensaio HIL Concluído com Sucesso!                           \n');
fprintf('   Logs salvos em    : %s\n', logs_dir);
fprintf('   Figuras salvas em : %s\n', figures_dir);
fprintf('=================================================================\n');

% Função auxiliar inline
function res = tern(cond, v_true, v_false)
    if cond, res = v_true; else, res = v_false; end
end
