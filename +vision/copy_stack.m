function [plan, view, R] = copy_stack(sim_interface, R)
% COPY_STACK  Modo "copiar pilha": com UMA SÓ fotografia reconhece um MODELO
%             deitado na mesa - uma fila de blocos encaixados uns nos outros
%             (uma pilha na horizontal) - e planeia reconstruí-lo na VERTICAL
%             no alvo verde, com outras peças das mesmas cores.
%
%   [~, ~, R] = copy_stack()                     % configuração por defeito
%   [plan, view, R] = copy_stack(sim_interface, R)  % fotografia + plano
%
%   R   : configuração (campos que se podem mudar depois de criar):
%         colors  - cores dos blocos (por defeito {'yellow','pink','orange'})
%         tableZ  - altura da mesa [m]            blockH - altura de um bloco [m]
%         base    - [x y] da base do robô no MUNDO [m]
%         target  - [x y z] do alvo verde no MUNDO [m] (onde se constrói a pilha)
%         gap     - [min max] distância entre centros de dois blocos seguidos
%                   do modelo [m]. Numa pilha deitada os blocos estão encaixados
%                   uns nos outros, por isso os centros ficam a ~1 altura de
%                   bloco (BLOCK_H = 1,92 cm): por defeito [0.012 0.030]
%         alignTol- quanto a fila se pode desviar de uma reta [rad] (20 graus)
%         minLen  - nº mínimo de blocos do modelo (2)
%         bottom  - 'near': a base da pilha é o bloco do modelo mais PERTO do
%                   robô; 'far': o mais longe
%         reachFn - função @(p) true se o robô alcança o centro de preensão p
%   Saídas (preenchidas também em R):
%   plan : struct array, um passo por bloco do modelo, de baixo para cima:
%          color, pick (centro da peça a apanhar), yaw, place (superfície onde
%          a pousar), level, blob (índice da peça em view)
%          [] se não há modelo ou se falta alguma peça (ver R.why)
%   view : o que a câmara viu, para show_vision (img, det, names, colors,
%          pos, pix, found, yaw, cam) + model (índices das peças do modelo)
%   R.recipe : sequência de cores do modelo (de baixo para cima)
%   R.why    : motivo, se plan = []
%
% Como funciona (só com a câmara, serve para o braço real):
%   1. procura entre todos os blocos a fila mais comprida de blocos seguidos
%      (centros a uma distância dentro de gap) e alinhados numa reta = MODELO;
%   2. ordena-a a começar pelo lado escolhido em bottom;
%   3. para cada cor do modelo, por ordem, escolhe a peça dessa cor mais perto
%      da base que o robô alcança (nunca as do modelo, cada peça só uma vez);
%   4. a fotografia NÃO se repete: as posições ficam guardadas no plano.
% Chamar com o braço fora da vista da câmara (ex.: VISION_POSE) e o modelo
% já montado.

    %% ---- configuração por defeito ----
    plan = [];  view = [];
    if nargin < 2 || isempty(R)
        R = struct('colors', {{'yellow', 'pink', 'orange'}}, 'tableZ', 0, 'blockH', 0.0192, ...
                   'base', [0 0], 'target', [0 0 0], ...
                   'gap', [0.012 0.030], 'alignTol', deg2rad(20), 'minLen', 2, ...
                   'bottom', 'near', 'reachFn', [], 'recipe', {{}}, 'why', '');
        if nargin < 1 || isempty(sim_interface), return; end
    end

    %% ---- a fotografia (só uma) ----
    % (também o verde: a mesma fotografia corrige a posição do alvo, que na
    %  fotografia inicial podia estar meio tapado por peças retiradas depois)
    [det, img, cam] = vision.vision_colors(sim_interface, [R.colors(:)' {'green'}], true);
    if ~isempty(det)
        isG = strcmpi({det.color}, 'green');
        if ~any(strcmpi(R.colors, 'green')) && any(isG)
            G = det(isG);
            [~, g] = max([G.area]);                  % a maior mancha verde = o alvo
            c = G(g).cam;  w = G(g).world;
            p = c + (R.target(3) - c(3)) / (w(3) - c(3)) * (w - c);   % à altura do alvo
            if hypot(p(1) - R.target(1), p(2) - R.target(2)) < 0.03
                fprintf('copy_stack: alvo corrigido de [%.3f %.3f] para [%.3f %.3f]\n', ...
                        R.target(1:2), p(1:2));
                R.target(1:2) = p(1:2);
            end
            det = det(~isG);
        end
    end
    n = numel(det);
    P = zeros(n, 2);  Y = nan(n, 1);  X = zeros(n, 2);  C = cell(n, 1);
    for j = 1:n
        c = det(j).cam;  w = det(j).world;
        p = c + (R.tableZ + R.blockH - c(3)) / (w(3) - c(3)) * (w - c);   % bloco na mesa
        P(j,:) = p(1:2);  Y(j) = det(j).yaw;  X(j,:) = det(j).pixel;  C{j} = det(j).color;
    end
    label = repmat({'peça'}, n, 1);

    %% ---- 1-2. o modelo ----
    chain = find_chain(P, (1:n)', R.gap, R.alignTol);
    if numel(chain) < R.minLen
        chain = [];
        R.recipe = {};
        R.why = 'não encontrei nenhum modelo (fila de blocos encaixados)';
    else
        dA = hypot(P(chain(1),1) - R.base(1),   P(chain(1),2) - R.base(2));
        dB = hypot(P(chain(end),1) - R.base(1), P(chain(end),2) - R.base(2));
        if xor(dA > dB, strcmp(R.bottom, 'far')), chain = flip(chain); end
        R.recipe = C(chain)';
        for m = 1:numel(chain)
            label{chain(m)} = sprintf('MODELO %d', m);
        end
        fprintf('copy_stack: MODELO (de baixo para cima): %s\n', strjoin(R.recipe, ' -> '));
    end

    %% ---- 3. uma peça para cada bloco do modelo ----
    if ~isempty(chain)
        used = false(n, 1);  used(chain) = true;     % as do modelo não se usam
        reach = nan(n, 1);                           % (alcance: calculado só se preciso)
        steps = struct('color', {}, 'pick', {}, 'yaw', {}, 'place', {}, 'level', {}, 'blob', {});
        missing = {};
        for s = 1:numel(R.recipe)
            want = R.recipe{s};
            cand = find(~used & strcmpi(C, want));
            [~, ord] = sort(hypot(P(cand,1) - R.base(1), P(cand,2) - R.base(2)));
            k = [];
            for kk = cand(ord)'
                if isnan(reach(kk))
                    reach(kk) = isempty(R.reachFn) || R.reachFn([P(kk,:), R.tableZ + R.blockH/2]);
                end
                if reach(kk), k = kk;  break; end
                label{kk} = 'fora do alcance';
            end
            if isempty(k)
                missing{end+1} = sprintf('%d (%s)', s, want);   %#ok<AGROW>
                continue;
            end
            used(k) = true;
            steps(end+1) = struct('color', want, 'pick', [P(k,:), R.tableZ + R.blockH/2], ...
                                  'yaw', Y(k), 'place', [R.target(1:2), R.target(3) + (s-1)*R.blockH], ...
                                  'level', s, 'blob', k);                     %#ok<AGROW>
            label{k} = sprintf('peça %d da pilha', s);
        end
        if isempty(missing)
            plan = steps;
            R.why = '';
        else
            R.why = sprintf('faltam peças ao alcance para o(s) bloco(s) %s do modelo', strjoin(missing, ', '));
        end
    end
    if ~isempty(R.why), fprintf('copy_stack: %s\n', R.why); end

    names = cell(1, n);
    for j = 1:n, names{j} = sprintf('%s: %s', C{j}, label{j}); end
    view = struct('img', img, 'det', {det}, 'names', {names}, 'colors', {C'}, ...
                  'pos', [P, (R.tableZ + R.blockH/2)*ones(n, 1)], 'pix', X, ...
                  'found', true(n, 1), 'yaw', Y, 'cam', cam, 'model', chain);
end

% -------------------------------------------------------------------------
function best = find_chain(P, idx, gap, tol)
% fila mais comprida de blocos (índices de idx) seguidos e alinhados:
% começa em cada par de vizinhos e estende-se para os dois lados ao longo
% da mesma direção
    best = [];
    for a = idx'
        for b = idx'
            if a == b, continue; end
            d = norm(P(b,:) - P(a,:));
            if d < gap(1) || d > gap(2), continue; end
            chain = [a b];
            chain = extend(P, idx, chain, gap, tol);             % para a frente
            chain = flip(extend(P, idx, flip(chain), gap, tol)); % para trás
            if numel(chain) > numel(best), best = chain; end
        end
    end
end

function chain = extend(P, idx, chain, gap, tol)
% acrescentar blocos no fim da fila enquanto houver um vizinho alinhado
    u = P(chain(2),:) - P(chain(1),:);  u = u / norm(u);   % direção da fila
    while true
        last = chain(end);  nxt = [];  bestAng = tol;
        for k = idx'
            if any(chain == k), continue; end
            v = P(k,:) - P(last,:);  d = norm(v);
            if d < gap(1) || d > gap(2), continue; end
            ang = acos(max(-1, min(1, dot(v/d, u))));
            if ang < bestAng, bestAng = ang;  nxt = k; end
        end
        if isempty(nxt), break; end
        chain(end+1) = nxt;                                 %#ok<AGROW>
    end
end