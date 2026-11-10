function metricas = calcula_metricas(t, y, y_ref, nome_variavel, unidade)
% CALCULA_METRICAS - Calcula índices de desempenho de resposta ao degrau
% EA-291 - Pilotos Automáticos de VANTs (ITA)
%
% Entradas:
%   t            - Vetor de tempo [s]
%   y            - Resposta da planta
%   y_ref        - Sinal de referência (degrau)
%   nome_variavel- String com o nome da variável
%   unidade      - String com a unidade (ex: 'm', 'm/s', 'graus')

if nargin < 4, nome_variavel = 'Variavel'; end
if nargin < 5, unidade = ''; end

% Proteção contra vetor vazio ou insuficiente
if isempty(t) || length(t) < 5
    metricas.tr = NaN;
    metricas.ts = NaN;
    metricas.Mp = NaN;
    metricas.ess = NaN;
    fprintf('[AVISO] Dados insuficientes na janela temporal para calcular metricas de: %s\n', nome_variavel);
    return;
end

% Detectar o instante do degrau
dy_ref = diff(y_ref);
idx_step = find(abs(dy_ref) > 0.05 * max(abs(dy_ref)), 1, 'first');
if isempty(idx_step)
    idx_step = 1;
end
t_step = t(idx_step);

y0 = y_ref(max(1, idx_step-1));
y_final = y_ref(end);
delta_ref = y_final - y0;

if abs(delta_ref) < 1e-4
    metricas.tr = NaN;
    metricas.ts = NaN;
    metricas.Mp = 0;
    metricas.ess = abs(y_ref(end) - y(end));
    return;
end

% Janela após o degrau
t_post = t(idx_step:end) - t_step;
y_post = y(idx_step:end);

% Erro de regime permanente (últimos 5% dos dados)
n_tail = max(5, round(0.05 * length(y_post)));
y_ss = mean(y_post(end-n_tail:end));
ess = abs(y_final - y_ss);

% Sobressinal (Overshoot)
if delta_ref > 0
    y_peak = max(y_post);
    Mp = max(0, (y_peak - y_final) / abs(delta_ref) * 100);
else
    y_peak = min(y_post);
    Mp = max(0, (y_final - y_peak) / abs(delta_ref) * 100);
end

% Tempo de Subida (10% a 90% da variação)
y10 = y0 + 0.10 * delta_ref;
y90 = y0 + 0.90 * delta_ref;

if delta_ref > 0
    i10 = find(y_post >= y10, 1, 'first');
    i90 = find(y_post >= y90, 1, 'first');
else
    i10 = find(y_post <= y10, 1, 'first');
    i90 = find(y_post <= y90, 1, 'first');
end

if ~isempty(i10) && ~isempty(i90) && i90 >= i10
    tr = t_post(i90) - t_post(i10);
else
    tr = NaN;
end

% Tempo de Acomodação (faixa de 2%)
faixa = 0.02 * abs(delta_ref);
fora_faixa = find(abs(y_post - y_final) > faixa);
if isempty(fora_faixa)
    ts = 0;
elseif fora_faixa(end) < length(t_post)
    ts = t_post(fora_faixa(end) + 1);
else
    ts = NaN; % Não acomodou dentro do tempo
end

metricas.tr = tr;
metricas.ts = ts;
metricas.Mp = Mp;
metricas.ess = ess;
metricas.y_ss = y_ss;

% Exibe tabela formatada
fprintf('\n--- Metricas de Desempenho: %s [%s] ---\n', nome_variavel, unidade);
fprintf('  Valor Inicial (y0)     : %8.3f %s\n', y0, unidade);
fprintf('  Referencia Final (y_ref): %8.3f %s\n', y_final, unidade);
fprintf('  Valor Regime (y_ss)    : %8.3f %s\n', y_ss, unidade);
fprintf('  Erro Permanente (ess)  : %8.3f %s\n', ess, unidade);
fprintf('  Sobressinal (Mp)       : %8.2f %%\n', Mp);
if ~isnan(tr)
    fprintf('  Tempo de Subida (tr)   : %8.3f s\n', tr);
else
    fprintf('  Tempo de Subida (tr)   :      N/A\n');
end
if ~isnan(ts)
    fprintf('  Tempo Acomodacao (ts)  : %8.3f s (faixa 2%%)\n', ts);
else
    fprintf('  Tempo Acomodacao (ts)  : Nao acomodou no intervalo\n');
end
fprintf('---------------------------------------------------\n');

end
