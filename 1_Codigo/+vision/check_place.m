function [placeW, info] = check_place(sim_interface, placeW, tipW, R)
% CHECK_PLACE  Antes de pousar: confirmar com a câmara onde está a pilha
%              (ou o alvo) e corrigir o ponto de pouso se tiver sido mexida.
%
%   [placeW, info] = vision.check_place(sim_interface, placeW, tipW, R)
%
%   placeW : [x y z] ponto de pouso planeado (superfície da pilha/alvo, MUNDO) [m]
%   tipW   : [x y z] onde está agora a garra (com a peça), MUNDO [m]: as manchas
%            aí são a própria peça que o robô leva e são ignoradas
%   R      : (opcional) configuração:
%            search  - raio de procura à volta do ponto planeado [m] (0.04)
%            minMove - deslocamento abaixo do qual não se corrige [m] (0.006):
%                      a visão tem alguns mm de erro, e "corrigir" esse ruído
%                      desalinha a pilha em vez de a alinhar
%            minMoveWeak - o mesmo, quando a mancha usada não é de confiança
%                      (cor lavada ou tamanho a mais de 10 % do de um bloco)
%                      [m] (0.015): aí só um empurrão a sério conta
%            what    - nome nas mensagens: 'pilha' (por defeito) ou 'peça'
%            blockSize - lado de um bloco [m] (0.031; [] = não filtrar por tamanho,
%                      ex.: cubos e cilindros): manchas de blocos com
%                      tamanho fora de 75-130 % disto são pedaços (luz, sombra,
%                      tapadas pelo braço) e o centro delas não é de confiança
%            hide    - raio à volta da garra cujas manchas se ignoram [m] (0.03)
%            colors  - cores procuradas: a do topo da pilha ('green' se for o
%                      alvo vazio). Por defeito todas (o verde só com o alvo vazio)
%   placeW : ponto de pouso corrigido (só x, y mudam)
%   info   : 'ok' (no sítio), 'moved' (corrigido), 'lost' (não encontrada:
%            fica o ponto planeado)
%
% Como funciona (só com a câmara, serve para o braço real):
%   1. uma fotografia com o braço onde está (por cima da peça que acabou de
%      apanhar): não vai à VISION_POSE;
%   2. ignora as manchas à volta da garra (a peça que leva na mão);
%   3. cada mancha passa para a mesa à altura da superfície de pouso
%      (vision.at_height) e escolhe-se a mais perto do ponto planeado, até
%      R.search. Os blocos têm prioridade sobre o verde (o alvo por baixo da
%      pilha também se vê, mas a uma altura diferente);
%   4. se a diferença passar de R.minMove, o ponto de pouso passa a ser esse.

    if nargin < 4 || isempty(R), R = struct(); end
    if ~isfield(R, 'search'),  R.search  = 0.04;  end
    if ~isfield(R, 'minMove'), R.minMove = 0.006; end
    if ~isfield(R, 'blockSize'), R.blockSize = 0.031; end
    if ~isfield(R, 'what'), R.what = 'pilha'; end
    if ~isfield(R, 'minMoveWeak'), R.minMoveWeak = 0.015; end
    if ~isfield(R, 'hide'),    R.hide    = 0.03;  end
    if ~isfield(R, 'colors')
        % sem cor indicada: todas; mas o verde (o alvo, à altura da mesa) só
        % serve quando se pousa no alvo vazio - numa pilha, passá-lo para a
        % altura do topo da pilha dava um ponto errado em vários mm
        R.colors = {'red', 'orange', 'yellow', 'blue', 'pink'};
        if placeW(3) < 0.01, R.colors{end+1} = 'green'; end
    end
    R.colors = cellstr(R.colors);
    info = 'lost';

    % 1. fotografia (o braço fica onde está)
    [det, ~, cam] = vision.vision_colors(sim_interface, R.colors);
    if isempty(det)
        fprintf('check_place: a câmara não viu nenhuma mancha de %s\n', strjoin(R.colors, '/'));
        return;
    end

    % 2. a peça que a garra leva: pixel da garra e raio equivalente a R.hide
    uvTip = vision.cam_map(cam, tipW(:)');
    uvEdge = vision.cam_map(cam, tipW(:)' + [R.hide 0 0]);
    rHide = norm(uvEdge - uvTip);

    % 3. candidatos à altura da superfície de pouso
    best = [];  bestD = inf;  bestCol = '';  bestWeak = false;
    for pass = 1:2                                % 1.º blocos, 2.º o verde (alvo)
        for j = 1:numel(det)
            d = det(j);
            if (pass == 1) == strcmpi(d.color, 'green'), continue; end
            if ~any(isnan(uvTip)) && norm(d.pixel - uvTip) < rHide, continue; end   % a peça na garra
            weak = false;
            if pass == 1 && ~isnan(d.size) && ~isempty(R.blockSize)
                % tamanho à altura da superfície (mais perto da câmara = parece maior)
                s = d.size * (d.cam(3) - placeW(3)) / (d.cam(3) - d.world(3));
                if s < 0.75*R.blockSize || s > 1.3*R.blockSize, continue; end   % pedaço
                % pouco de confiança: tamanho longe do de um bloco ou cor lavada
                % (o centro de uma face meio lavada fica puxado para o lado claro)
                weak = abs(s - R.blockSize) > 0.10*R.blockSize || ...
                       (isfield(d, 'pale') && d.pale > 0.5);
            end
            p = vision.at_height(d, placeW(3));
            dist = hypot(p(1) - placeW(1), p(2) - placeW(2));
            if dist < R.search && dist < bestD
                best = p;  bestD = dist;  bestCol = d.color;  bestWeak = weak;
            end
        end
        if ~isempty(best), break; end
    end

    % 4. corrigir (ou não)
    if isempty(best)
        fprintf('check_place: não encontrei a %s a menos de %.0f cm do sítio planeado\n', R.what, 100*R.search);
        return;
    end
    lim = R.minMove;  tag = '';
    if bestWeak, lim = R.minMoveWeak;  tag = ', mancha pouco fiável'; end
    if bestD < lim
        info = 'ok';
        fprintf('check_place: %s no sítio (%s, %.1f mm%s: não corrijo abaixo de %.0f mm)\n', ...
                R.what, bestCol, 1000*bestD, tag, 1000*lim);
    else
        info = 'moved';
        fprintf('check_place: %s deslocada %.1f mm (%s%s) -> [%.3f %.3f] em vez de [%.3f %.3f]\n', ...
                R.what, 1000*bestD, bestCol, tag, best(1:2), placeW(1:2));
        placeW(1:2) = best(1:2);
    end
end