function [act, W, view] = watch_pairs(sim_interface, W, done, ok)
% WATCH_PAIRS  Modos "vigiar" pela câmara. Quando ENTRA na vista um bloco com
%              a mesma cor de um que já lá está (um par):
%   W.mode = 'request' (PEDIDO DE COR): o bloco novo é um pedido - indica a cor
%            da próxima peça da pilha do alvo verde. O robô vai buscar uma peça
%            dessa cor que alcance e empilha-a no alvo. O bloco do pedido pode
%            estar fora do alcance (é só um sinal).
%   W.mode = 'pairs' (PILHA DE PARES): o robô apanha o bloco solto do par mais
%            perto da base e empilha-o em cima do outro bloco/pilha dessa cor
%            (os dois têm de estar ao alcance).
%
%   [~, W] = watch_pairs()                          % estado inicial (valores por defeito)
%   [act, W, view] = watch_pairs(sim_interface, W)  % uma fotografia; act = [] se não há nada a fazer
%   [~, W] = watch_pairs([], W, act, ok)            % depois de executar act (ok = correu bem)
%
%   W   : estado + configuração (campos que se podem mudar depois de criar):
%         colors  - cores a vigiar (por defeito {'yellow','pink','orange'})
%         tableZ  - altura da mesa [m]            blockH - altura de um bloco [m]
%         base    - [x y] da base do robô no MUNDO [m] (para escolher a peça mais perto)
%         target  - [x y z] do alvo verde no MUNDO [m] (onde se faz a pilha)
%         targetR - raio à volta do alvo onde as manchas são "a pilha" [m] (0.03)
%         match   - distância [m] para dizer que uma mancha é o mesmo bloco (0.02)
%         still   - quanto um bloco novo se pode mexer entre duas fotografias
%                   para se considerar parado [m] (0.005)
%         reachFn - função @(p) true se o robô alcança o centro de preensão p
%                   [x y z] (MUNDO), ex.: @(p) can_reach(cfg, p). [] = não testar
%         nStack  - nº de blocos já na pilha do alvo (começa em 0; modo 'request')
%         mode    - 'request' (por defeito) ou 'pairs'
%         placeExtra - (modo 'pairs') quanto acima da superfície fica o centro
%                   do bloco ao pousar, além de meia altura [m] (folga + queda)
%   act : o que fazer (struct) ou []:
%         color - cor pedida            pick  - centro da peça a apanhar [x y z] (MUNDO)
%         yaw   - orientação [rad] (NaN = desconhecida)
%         place - superfície onde a pousar [x y z] (topo da pilha)
%         level - nível da peça na pilha ('request': 1 = em cima do alvo;
%                 'pairs': 2 = em cima de um bloco)
%         req   - [x y] do bloco novo que fez o pedido / formou o par
%   view: o que a câmara viu nesta fotografia, para show_vision
%
% Como funciona (só com a câmara, serve para o braço real):
%   - na primeira fotografia, todos os blocos vistos ficam "conhecidos";
%   - numa fotografia seguinte, uma mancha longe de todos os conhecidos dessa
%     cor é um bloco NOVO. Só conta quando aparece parado no mesmo sítio em
%     duas fotografias seguidas (ainda pode estar a ser mexido);
%   - se já havia um bloco conhecido dessa cor (forma-se um PAR), o bloco novo
%     é um PEDIDO: pode estar fora do alcance do braço (é só um sinal).
%     O robô apanha a peça dessa cor, solta na mesa e ao seu alcance, MAIS
%     PERTO da base (pode ser a do pedido, se a alcança) e põe-na na pilha
%     do alvo verde. O bloco do pedido passa a conhecido (pedido atendido);
%   - se não há nenhuma peça dessa cor ao alcance, o pedido fica à espera;
%   - as manchas em cima do alvo são a pilha: não contam para os pares.
% Chamar SÓ com o braço fora da vista da câmara (ex.: VISION_POSE).

    %% ---- estado inicial ----
    view = [];
    if nargin < 2 || isempty(W)
        W = struct('colors', {{'yellow', 'pink', 'orange'}}, 'tableZ', 0, ...
                   'blockH', 0.0192, 'base', [0 0], 'target', [0 0 0], 'targetR', 0.03, ...
                   'match', 0.02, 'still', 0.005, 'reachFn', [], 'nStack', 0, ...
                   'mode', 'request', 'placeExtra', 0.02, 'stacks', zeros(0, 4), ...
                   'init', false, 'known', {{}}, 'cand', {{}}, 'ignore', {{}}, 'lastMsg', {{}});
        act = [];
        if nargin < 1 || isempty(sim_interface), return; end
    end
    nc = numel(W.colors);
    if numel(W.known) < nc                     % (se as cores foram mudadas depois de criar)
        W.known   = repmat({zeros(0,2)}, 1, nc);
        W.cand    = repmat({zeros(0,2)}, 1, nc);
        W.ignore  = repmat({zeros(0,2)}, 1, nc);
        W.lastMsg = repmat({''}, 1, nc);
    end

    %% ---- depois de executar uma ação: atualizar o estado ----
    if nargin >= 4
        act = [];
        ci = done.ci;
        pk = done.pick(1:2);
        if ok && strcmp(W.mode, 'pairs')
            % o bloco apanhado saiu dali e está agora na pilha de destino
            pl = done.place(1:2);
            W.known{ci} = W.known{ci}(~near(W.known{ci}, pk, W.match), :);
            W.cand{ci}  = W.cand{ci}(~near(W.cand{ci},  pk, W.match), :);
            W.cand{ci}  = W.cand{ci}(~near(W.cand{ci},  pl, W.match), :);
            if ~any(near(W.known{ci}, pl, W.match))
                W.known{ci}(end+1, :) = pl;
            end
            W.stacks(end+1, :) = [pl, done.level, ci];   % pilhas feitas: [x y nível cor]
        elseif ok
            W.nStack = W.nStack + 1;           % mais um bloco na pilha do alvo
            % a peça apanhada saiu do sítio onde estava
            W.known{ci} = W.known{ci}(~near(W.known{ci}, pk, W.match), :);
            W.cand{ci}  = W.cand{ci}(~near(W.cand{ci},  pk, W.match), :);
            % pedido atendido: o bloco do pedido passa a conhecido
            if any(near(W.cand{ci}, done.req, W.match))
                W.cand{ci} = W.cand{ci}(~near(W.cand{ci}, done.req, W.match), :);
                W.known{ci}(end+1, :) = done.req;
            end
        else
            % não deu: ignorar aquela peça para não tentar sempre a mesma
            W.ignore{ci}(end+1, :) = pk;
        end
        return;
    end

    %% ---- uma fotografia ----
    act = [];
    [det, img] = vision_colors(sim_interface, W.colors, true);
    vNames = {};  vColors = {};  vPos = zeros(0, 3);  vPix = zeros(0, 2);  vYaw = zeros(0, 1);
    tz = W.target(3);                          % superfície do alvo
    for ci = 1:nc
        if isempty(det)
            D = det;
        else
            D = det(strcmpi({det.color}, W.colors{ci}));
        end
        n = numel(D);
        P = zeros(n, 2);  Y = nan(n, 1);  X = zeros(n, 2);  onStack = false(n, 1);  Zc = zeros(n, 1);
        L = ones(n, 1);                        % nº de blocos em cada mancha (pilhas de pares)
        for j = 1:n
            c = D(j).cam;  w = D(j).world;
            % ponto da face de cima de um bloco pousado na mesa
            p = c + (W.tableZ + W.blockH - c(3)) / (w(3) - c(3)) * (w - c);
            % em cima do alvo? então é a pilha (face de cima mais alta)
            onStack(j) = W.nStack > 0 && hypot(p(1) - W.target(1), p(2) - W.target(2)) < W.targetR;
            if onStack(j)
                zTop = tz + W.nStack*W.blockH;
                p = c + (zTop - c(3)) / (w(3) - c(3)) * (w - c);
                Zc(j) = tz + W.nStack*W.blockH/2;
            else
                L(j) = stack_level(W, p(1:2), ci);  % pilha de um par feita antes?
                if L(j) > 1
                    zTop = W.tableZ + L(j)*W.blockH;
                    p = c + (zTop - c(3)) / (w(3) - c(3)) * (w - c);
                end
                Zc(j) = W.tableZ + L(j)*W.blockH/2;
            end
            P(j, :) = p(1:2);
            Y(j) = D(j).yaw;
            X(j, :) = D(j).pixel;
        end
        label = repmat({'conhecido'}, n, 1);
        label(onStack) = {sprintf('pilha verde (%d)', W.nStack)};
        % blocos ignorados (tentativas falhadas) ficam de fora dos pares
        free = ~onStack & ~near(P, W.ignore{ci}, W.match);
        label(~onStack & ~free) = {'ignorado'};
        F = find(free);                        % índices das peças soltas que contam

        if ~W.init                             % primeira fotografia: tudo é conhecido
            W.known{ci} = P(F, :);
        else
            isKnown = false(n, 1);
            isKnown(F) = near(P(F,:), W.known{ci}, W.match);
            W.known{ci} = P(isKnown, :);       % (segue blocos empurrados; esquece os retirados)
            newIdx = F(~isKnown(F));
            stable = newIdx(near(P(newIdx, :), W.cand{ci}, W.still));
            W.cand{ci} = P(newIdx, :);
            label(newIdx) = {'NOVO (a mexer?)'};
            label(stable) = {'PEDIDO'};

            if ~isempty(stable) && ~any(isKnown)
                % primeiro bloco desta cor: passa a conhecido e espera pelo par
                k = stable(1);
                W.known{ci}(end+1, :) = P(k, :);
                label{k} = 'à espera do par';
                fprintf('watch_pairs: bloco %s em [%.3f %.3f], à espera do par\n', W.colors{ci}, P(k,:));
            elseif ~isempty(stable) && isempty(act) && strcmp(W.mode, 'pairs')
                % PAR: blocos que contam = conhecidos + novos parados
                req = stable(1);
                group = [find(isKnown); stable];
                % alcance: apanhar (bloco solto na mesa) e pousar (em cima do bloco/pilha)
                canPick = false(n, 1);  canPlace = false(n, 1);
                for j = group'
                    zPick  = W.tableZ + W.blockH/2;
                    zPlace = W.tableZ + L(j)*W.blockH + W.blockH/2 + W.placeExtra;
                    if isempty(W.reachFn)
                        canPick(j) = L(j) == 1;  canPlace(j) = true;
                    else
                        canPick(j)  = (L(j) == 1) && W.reachFn([P(j,:), zPick]);
                        canPlace(j) = W.reachFn([P(j,:), zPlace]);
                    end
                    if ~canPick(j) && ~canPlace(j), label{j} = 'fora do alcance'; end
                end
                % apanhar o bloco MAIS PERTO da base e pô-lo em cima do destino
                % alcançável mais perto dele
                pickList = group(canPick(group));
                [~, ord] = sort(hypot(P(pickList,1) - W.base(1), P(pickList,2) - W.base(2)));
                pickList = pickList(ord);
                k = [];  t = [];
                for kk = pickList'
                    dests = group(canPlace(group) & group ~= kk);
                    if ~isempty(dests)
                        [~, m] = min(hypot(P(dests,1) - P(kk,1), P(dests,2) - P(kk,2)));
                        k = kk;  t = dests(m);
                        break;
                    end
                end
                if isempty(k)
                    msg = sprintf('watch_pairs: par %s fora do alcance do robô, à espera', W.colors{ci});
                    if ~strcmp(W.lastMsg{ci}, msg), disp(msg); end   % escrever só uma vez
                    W.lastMsg{ci} = msg;
                else
                    W.lastMsg{ci} = '';
                    act = struct('color', W.colors{ci}, 'ci', ci, ...
                                 'pick',  [P(k,:), W.tableZ + W.blockH/2], 'yaw', Y(k), ...
                                 'place', [P(t,:), W.tableZ + L(t)*W.blockH], ...
                                 'level', L(t) + 1, 'req', P(req,:));
                    label{k} = 'A APANHAR';
                    label{t} = 'DESTINO';
                end
            elseif ~isempty(stable) && isempty(act)
                % PEDIDO desta cor: a peça a levar para a pilha é a peça solta
                % desta cor ao alcance MAIS PERTO da base (conhecida ou a do pedido)
                req = stable(1);
                [~, ord] = sort(hypot(P(F,1) - W.base(1), P(F,2) - W.base(2)));
                cands = F(ord);
                cands = cands(L(cands) == 1);      % só blocos soltos na mesa
                k = [];
                for kk = cands'
                    if isempty(W.reachFn) || W.reachFn([P(kk,:), W.tableZ + W.blockH/2])
                        k = kk;  break;
                    end
                    label{kk} = 'fora do alcance';
                end
                if isempty(k)
                    msg = sprintf('watch_pairs: pedido %s, mas nenhuma peça %s ao alcance', ...
                                  W.colors{ci}, W.colors{ci});
                    if ~strcmp(W.lastMsg{ci}, msg), disp(msg); end   % escrever só uma vez
                    W.lastMsg{ci} = msg;
                    label{req} = 'PEDIDO (sem peça ao alcance)';
                else
                    W.lastMsg{ci} = '';
                    act = struct('color', W.colors{ci}, 'ci', ci, ...
                                 'pick',  [P(k,:), W.tableZ + W.blockH/2], 'yaw', Y(k), ...
                                 'place', [W.target(1:2), tz + W.nStack*W.blockH], ...
                                 'level', W.nStack + 1, 'req', P(req,:));
                    if k ~= req, label{req} = 'PEDIDO'; end
                    label{k} = 'A APANHAR -> pilha verde';
                end
            end
        end

        % para a visualização
        for j = 1:n
            if L(j) > 1 && ~any(strcmp(label{j}, {'DESTINO', 'fora do alcance'}))
                label{j} = sprintf('pilha de %d', L(j));
            end
            vNames{end+1}  = sprintf('%s: %s', W.colors{ci}, label{j}); %#ok<AGROW>
            vColors{end+1} = W.colors{ci};                               %#ok<AGROW>
            vPos(end+1, :) = [P(j,:), Zc(j)];                            %#ok<AGROW>
            vPix(end+1, :) = X(j, :);                                    %#ok<AGROW>
            vYaw(end+1, 1) = Y(j);                                       %#ok<AGROW>
        end
    end
    W.init = true;
    view = struct('img', img, 'det', {det}, 'names', {vNames}, 'colors', {vColors}, ...
                  'pos', vPos, 'pix', vPix, 'found', true(numel(vNames), 1), 'yaw', vYaw);
end

% -------------------------------------------------------------------------
function L = stack_level(W, xy, ci)
% nível (nº de blocos) da pilha de pares desta cor naquele sítio; 1 se não há
    L = 1;
    S = W.stacks(W.stacks(:,4) == ci, :);
    if isempty(S), return; end
    hit = hypot(S(:,1) - xy(1), S(:,2) - xy(2)) < W.match;
    if any(hit), L = max(S(hit, 3)); end
end

function tf = near(P, Q, d)
% true para cada linha de P que está a menos de d de alguma linha de Q
    tf = false(size(P, 1), 1);
    for i = 1:size(Q, 1)
        tf = tf | hypot(P(:,1) - Q(i,1), P(:,2) - Q(i,2)) < d;
    end
end