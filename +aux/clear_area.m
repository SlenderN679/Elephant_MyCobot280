function [nMoved, ok, why, parked] = clear_area(robot_arm, sim_interface, cfg, objIds, center, radius, spots, isCube, useYaw, halfHeights, posFn)
% CLEAR_AREA  Desimpede uma zona (ex.: o alvo): leva para pontos de
%             estacionamento todos os objetos que estejam dentro dela.
%
%   nMoved = clear_area(robot_arm, sim_interface, cfg, objIds, center, radius, spots)
%   nMoved = clear_area(robot_arm, sim_interface, cfg, objIds, center, radius, spots, isCube, useYaw, halfHeights)
%   [nMoved, ok, why, parked] = clear_area(..., halfHeights, posFn)
%
%   objIds : índices dos objetos que podem estar na zona (os de get_target_position)
%   center : [x y z] centro da zona, referencial do MUNDO [m] (ex.: posição do alvo)
%   radius : raio da zona [m], medido na horizontal
%   spots  : Nx2 pontos de estacionamento [x y] no MUNDO [m], alcançáveis e
%            afastados uns dos outros pelo menos cfg.CLEAR_SPACING
%   isCube : (opcional) função @(id) true para cubos (alinhar os dedos)
%   useYaw : (opcional) true para ler a orientação dos cubos do simulador, ou
%            função @(id) que devolve o yaw [rad] do objeto id (ex.: da visão)
%   halfHeights : (opcional) metade da altura de cada objeto de objIds [m],
%            medida com todos pousados na mesa. Necessário para pousar bem
%            um objeto tirado do topo de uma pilha.
%   nMoved : quantos objetos foram retirados
%   posFn  : (opcional) função @(id) que devolve [x y z] do objeto id no MUNDO.
%            Por defeito sim_interface.get_target_position; com a câmara,
%            uma função que lê as posições guardadas (ver main_program)
%   ok     : false se a zona não ficou livre (sem estacionamento ou falhou)
%   why    : motivo, se ok = false
%   parked : M x 4, uma linha [id x y z] por objeto retirado, com a posição
%            (centro, MUNDO) onde ficou - para atualizar posições guardadas
%
% Repete até a zona ficar livre: lê as posições de todos os objetos, escolhe
% o mais ALTO dentro da zona (o do topo de uma pilha sai primeiro) e leva-o
% com pick_and_place para o primeiro ponto de estacionamento livre (sem
% nenhum objeto a menos de cfg.CLEAR_SPACING).
%
% Visualização no CoppeliaSim (cfg.SHOW_CLEAR = 1, por defeito):
%   - zona a desimpedir: disco laranja semitransparente;
%   - pontos de estacionamento: círculos azul-claros;
%   - objeto a retirar: X vermelho, com uma seta amarela até ao ponto escolhido;
%   - no fim: anel VERDE se a zona ficou livre, VERMELHO se não.

    if nargin < 8 || isempty(isCube), isCube = @(id) true; end
    if nargin < 9 || isempty(useYaw), useYaw = false;      end
    if nargin < 10, halfHeights = []; end
    if nargin < 11 || isempty(posFn), posFn = @(id) sim_interface.get_target_position(id); end
    parked = zeros(0, 4);
    spacing = 0.05;
    if isfield(cfg, 'CLEAR_SPACING'), spacing = cfg.CLEAR_SPACING; end

    nMoved = 0;  ok = true;  why = '';

    % desenhar a zona e os pontos de estacionamento
    show = ~isfield(cfg, 'SHOW_CLEAR') || cfg.SHOW_CLEAR == 1;
    hZone = [];
    if show
        hZone = [drawing.draw_disc(sim_interface, center, radius, [1 0.5 0]), ...
                 drawing.draw_ring(sim_interface, center, radius, [1 0.5 0], 4)];
        for s = 1:size(spots, 1)
            hZone(end+1) = drawing.draw_ring(sim_interface, [spots(s,:) 0], 0.015, [0.3 0.8 1], 2); %#ok<AGROW>
        end
    end

    for k = 1:numel(objIds)                  % no máximo um movimento por objeto
        % posições atuais de todos os objetos
        P = zeros(numel(objIds), 3);
        for i = 1:numel(objIds)
            P(i,:) = posFn(objIds(i));
        end
        % objetos já retirados nesta chamada: usar a posição onde ficaram
        % (posFn pode devolver posições guardadas que ainda não sabem disso)
        for m = 1:size(parked, 1)
            P(objIds == parked(m,1), :) = parked(m, 2:4);
        end

        % objetos dentro da zona (distância horizontal ao centro)
        inside = find(hypot(P(:,1) - center(1), P(:,2) - center(2)) < radius);
        if isempty(inside)
            break;                           % zona livre
        end
        if k == 1
            fprintf('clear_area: %d objeto(s) dentro da zona a desimpedir\n', numel(inside));
        end
        [~, j] = max(P(inside, 3));          % o mais alto primeiro
        i = inside(j);

        % primeiro ponto de estacionamento livre
        spot = [];
        for s = 1:size(spots, 1)
            if all(hypot(P(:,1) - spots(s,1), P(:,2) - spots(s,2)) > spacing)
                spot = spots(s,:);
                break;
            end
        end
        if isempty(spot)
            ok = false;  why = 'não há pontos de estacionamento livres';
            fprintf('clear_area: %s\n', why);
            break;
        end

        objYaw = [];
        if isa(useYaw, 'function_handle')    % orientação dada (ex.: visão)
            if isCube(objIds(i)), objYaw = useYaw(objIds(i)); end
            if any(isnan(objYaw)), objYaw = []; end
        elseif useYaw && isCube(objIds(i))
            o = sim_interface.get_target_orientation(objIds(i));
            objYaw = o(3);
        end

        fprintf('clear_area: a retirar o objeto %d para [%.3f %.3f]\n', objIds(i), spot);
        hMove = [];
        if show                              % X no objeto e seta até ao estacionamento
            hMove = [drawing.draw_cross(sim_interface, P(i,:), 0.02, [1 0 0]), ...
                     drawing.draw_arrow(sim_interface, P(i,:), [spot 0], [1 0.9 0])];
        end
        half = [];
        if ~isempty(halfHeights), half = halfHeights(i); end
        [okP, whyP] = aux.pick_and_place(robot_arm, sim_interface, cfg, P(i,:), [spot cfg.TABLE_Z], objYaw, half);
        drawing.erase(sim_interface, hMove);
        if ~okP
            ok = false;  why = sprintf('falhou ao retirar o objeto %d (%s)', objIds(i), whyP);
            fprintf('clear_area: %s\n', why);
            break;
        end
        nMoved = nMoved + 1;
        if isempty(half), zc = P(i,3); else, zc = cfg.TABLE_Z + half; end   % centro, pousado na mesa
        parked(end+1, :) = [objIds(i), spot, zc];           %#ok<AGROW>
    end

    % resultado: apagar a zona laranja e deixar um anel verde
    % (livre) ou vermelho (não ficou livre)
    if ok
        fprintf('clear_area: zona livre (%d objeto(s) retirado(s))\n', nMoved);
    end
    if show
        drawing.erase(sim_interface, hZone);
        if ok, c = [0 0.9 0]; else, c = [1 0 0]; end
        drawing.draw_ring(sim_interface, center, radius, c, 4);
    end
end









