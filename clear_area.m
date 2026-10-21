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
% [ACRESCENTADO] Visualização no CoppeliaSim (cfg.SHOW_CLEAR = 1, por defeito):
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

    % [ACRESCENTADO] desenhar a zona e os pontos de estacionamento
    show = ~isfield(cfg, 'SHOW_CLEAR') || cfg.SHOW_CLEAR == 1;
    hZone = [];
    if show
        hZone = [draw_disc(sim_interface, center, radius, [1 0.5 0]), ...
                 draw_ring(sim_interface, center, radius, [1 0.5 0], 4)];
        for s = 1:size(spots, 1)
            hZone(end+1) = draw_ring(sim_interface, [spots(s,:) 0], 0.015, [0.3 0.8 1], 2); %#ok<AGROW>
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
        if isa(useYaw, 'function_handle')    % [ACRESCENTADO] orientação dada (ex.: visão)
            if isCube(objIds(i)), objYaw = useYaw(objIds(i)); end
            if any(isnan(objYaw)), objYaw = []; end
        elseif useYaw && isCube(objIds(i))
            o = sim_interface.get_target_orientation(objIds(i));
            objYaw = o(3);
        end

        fprintf('clear_area: a retirar o objeto %d para [%.3f %.3f]\n', objIds(i), spot);
        hMove = [];
        if show                              % X no objeto e seta até ao estacionamento
            hMove = [draw_cross(sim_interface, P(i,:), 0.02, [1 0 0]), ...
                     draw_arrow(sim_interface, P(i,:), [spot 0], [1 0.9 0])];
        end
        half = [];
        if ~isempty(halfHeights), half = halfHeights(i); end
        [okP, whyP] = pick_and_place(robot_arm, sim_interface, cfg, P(i,:), [spot cfg.TABLE_Z], objYaw, half);
        erase(sim_interface, hMove);
        if ~okP
            ok = false;  why = sprintf('falhou ao retirar o objeto %d (%s)', objIds(i), whyP);
            fprintf('clear_area: %s\n', why);
            break;
        end
        nMoved = nMoved + 1;
        if isempty(half), zc = P(i,3); else, zc = cfg.TABLE_Z + half; end   % centro, pousado na mesa
        parked(end+1, :) = [objIds(i), spot, zc];           %#ok<AGROW>
    end

    % [ACRESCENTADO] resultado: apagar a zona laranja e deixar um anel verde
    % (livre) ou vermelho (não ficou livre)
    if ok
        fprintf('clear_area: zona livre (%d objeto(s) retirado(s))\n', nMoved);
    end
    if show
        erase(sim_interface, hZone);
        if ok, c = [0 0.9 0]; else, c = [1 0 0]; end
        draw_ring(sim_interface, center, radius, c, 4);
    end
end

% =========================================================================
% [ACRESCENTADO] desenhos no CoppeliaSim (devolvem o identificador; 0 se falhar)
% =========================================================================
function sim = get_sim(sim_interface)
    sim = sim_interface.get_connection().require('sim');
end

function h = draw_disc(sim_interface, c, r, color)
% disco semitransparente de raio r centrado em c (na mesa)
    h = 0;
    try
        sim = get_sim(sim_interface);
        z = 0.006;  a = linspace(0, 2*pi, 41);
        h = double(sim.addDrawingObject(bitor(sim.drawing_triangles, sim.drawing_50percenttransparency), ...
                                        1, 0, -1, numel(a), color));
        for i = 1:numel(a) - 1
            sim.addDrawingObjectItem(h, [c(1) c(2) z, c(1) + r*cos(a(i)) c(2) + r*sin(a(i)) z, ...
                                         c(1) + r*cos(a(i+1)) c(2) + r*sin(a(i+1)) z]);
        end
    catch
    end
end

function h = draw_ring(sim_interface, c, r, color, width)
% circunferência de raio r centrada em c (na mesa)
    h = 0;
    try
        sim = get_sim(sim_interface);
        z = 0.007;  a = linspace(0, 2*pi, 41);
        h = double(sim.addDrawingObject(sim.drawing_lines, width, 0, -1, numel(a), color));
        for i = 1:numel(a) - 1
            sim.addDrawingObjectItem(h, [c(1) + r*cos(a(i))   c(2) + r*sin(a(i))   z, ...
                                         c(1) + r*cos(a(i+1)) c(2) + r*sin(a(i+1)) z]);
        end
    catch
    end
end

function h = draw_cross(sim_interface, p, r, color)
% X por cima de um objeto (p = centro do objeto)
    h = 0;
    try
        sim = get_sim(sim_interface);
        z = max(2*p(3), 0) + 0.004;          % um pouco acima da face de cima
        h = double(sim.addDrawingObject(sim.drawing_lines, 5, 0, -1, 2, color));
        sim.addDrawingObjectItem(h, [p(1) - r p(2) - r z, p(1) + r p(2) + r z]);
        sim.addDrawingObjectItem(h, [p(1) - r p(2) + r z, p(1) + r p(2) - r z]);
    catch
    end
end

function h = draw_arrow(sim_interface, p0, p1, color)
% seta na mesa de p0 até p1 (com ponta)
    h = 0;
    try
        sim = get_sim(sim_interface);
        z = 0.008;
        d = p1(1:2) - p0(1:2);  L = norm(d);
        if L < 1e-6, return; end
        u = d / L;  nrm = [-u(2) u(1)];  tip = 0.02;
        b = p1(1:2) - tip*u;
        h = double(sim.addDrawingObject(sim.drawing_lines, 4, 0, -1, 3, color));
        sim.addDrawingObjectItem(h, [p0(1:2) z, p1(1:2) z]);
        sim.addDrawingObjectItem(h, [p1(1:2) z, b + 0.6*tip*nrm z]);
        sim.addDrawingObjectItem(h, [p1(1:2) z, b - 0.6*tip*nrm z]);
    catch
    end
end

function erase(sim_interface, handles)
% apagar desenhos
    try
        sim = get_sim(sim_interface);
        for h = handles(handles > 0)
            sim.removeDrawingObject(h);
        end
    catch
    end
end