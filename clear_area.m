function [nMoved, ok, why] = clear_area(robot_arm, sim_interface, cfg, objIds, center, radius, spots, isCube, useYaw, halfHeights)
% CLEAR_AREA  Desimpede uma zona (ex.: o alvo): leva para pontos de
%             estacionamento todos os objetos que estejam dentro dela.
%
%   nMoved = clear_area(robot_arm, sim_interface, cfg, objIds, center, radius, spots)
%   nMoved = clear_area(robot_arm, sim_interface, cfg, objIds, center, radius, spots, isCube, useYaw, halfHeights)
%
%   objIds : índices dos objetos que podem estar na zona (os de get_target_position)
%   center : [x y z] centro da zona, referencial do MUNDO [m] (ex.: posição do alvo)
%   radius : raio da zona [m], medido na horizontal
%   spots  : Nx2 pontos de estacionamento [x y] no MUNDO [m], alcançáveis e
%            afastados uns dos outros pelo menos cfg.CLEAR_SPACING
%   isCube : (opcional) função @(id) true para cubos (alinhar os dedos)
%   useYaw : (opcional) true para ler a orientação dos cubos
%   halfHeights : (opcional) metade da altura de cada objeto de objIds [m],
%            medida com todos pousados na mesa. Necessário para pousar bem
%            um objeto tirado do topo de uma pilha.
%   nMoved : quantos objetos foram retirados
%   ok     : false se a zona não ficou livre (sem estacionamento ou falhou)
%   why    : motivo, se ok = false
%
% Repete até a zona ficar livre: lê as posições de todos os objetos, escolhe
% o mais ALTO dentro da zona (o do topo de uma pilha sai primeiro) e leva-o
% com pick_and_place para o primeiro ponto de estacionamento livre (sem
% nenhum objeto a menos de cfg.CLEAR_SPACING).

    if nargin < 8 || isempty(isCube), isCube = @(id) true; end
    if nargin < 9 || isempty(useYaw), useYaw = false;      end
    if nargin < 10, halfHeights = []; end
    spacing = 0.05;
    if isfield(cfg, 'CLEAR_SPACING'), spacing = cfg.CLEAR_SPACING; end

    nMoved = 0;  ok = true;  why = '';
    for k = 1:numel(objIds)                  % no máximo um movimento por objeto
        % posições atuais de todos os objetos
        P = zeros(numel(objIds), 3);
        for i = 1:numel(objIds)
            P(i,:) = sim_interface.get_target_position(objIds(i));
        end

        % objetos dentro da zona (distância horizontal ao centro)
        inside = find(hypot(P(:,1) - center(1), P(:,2) - center(2)) < radius);
        if isempty(inside)
            break;                           % zona livre
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
            return;
        end

        objYaw = [];
        if useYaw && isCube(objIds(i))
            o = sim_interface.get_target_orientation(objIds(i));
            objYaw = o(3);
        end

        fprintf('clear_area: a retirar o objeto %d para [%.3f %.3f]\n', objIds(i), spot);
        half = [];
        if ~isempty(halfHeights), half = halfHeights(i); end
        [okP, whyP] = pick_and_place(robot_arm, sim_interface, cfg, P(i,:), [spot cfg.TABLE_Z], objYaw, half);
        if ~okP
            ok = false;  why = sprintf('falhou ao retirar o objeto %d (%s)', objIds(i), whyP);
            fprintf('clear_area: %s\n', why);
            return;
        end
        nMoved = nMoved + 1;
    end
end