function hDraw = show_vision(sim_interface, img, det, names, colors, pos, pix, found, truth, yaw, truthYaw, oldDraw, titleTxt, extras)
% SHOW_VISION  Mostra o resultado da visão e o PLANO do modo atual: na imagem
%              da câmara (figura do MATLAB) e na mesa do CoppeliaSim.
%
%   show_vision(sim_interface, img, det, names, colors, pos, pix, found)
%   hDraw = show_vision(..., truth, yaw, truthYaw, oldDraw, titleTxt, extras)
%
%   img    : imagem da câmara ([] = só desenhar no CoppeliaSim)
%   det    : todas as manchas detetadas (manchas não usadas ficam com um X cinzento)
%   names  : cell com a etiqueta de cada objeto ('Block1', 'yellow: PEDIDO', ...)
%   colors : cell com a cor de cada objeto ('yellow', ...)
%   pos    : N x 3 centro de cada objeto no MUNDO [m]
%   pix    : N x 2 pixel do centro de cada objeto na imagem
%   found  : N x 1 true se o objeto foi encontrado
%   truth  : (opcional, só simulador) N x 3 posições reais, para mostrar o erro
%   yaw    : (opcional) N x 1 orientação detetada [rad] (NaN = sem orientação)
%   truthYaw : (opcional, só simulador) N x 1 orientação real [rad]
%   oldDraw  : (opcional) desenhos da chamada anterior, apagados antes de desenhar
%   titleTxt : (opcional) título da figura
%   extras   : (opcional) o plano do modo, struct com os campos:
%              cam    - modelo da câmara (vision_colors), para pôr as setas na imagem
%              arrows - struct array, uma seta por movimento planeado:
%                       from, to - [x y z] no MUNDO;  color - nome ou [r g b]
%                       label - texto;  state - 'next' (a seguir: grossa),
%                       'todo' (planeada: tracejada), 'done' (feita: cinzenta),
%                       'link' (só uma ligação, ex.: pedido -> peça: pontilhada),
%                       'model' (o modelo do modo 4: base -> topo)
%              info   - cell com linhas de texto para o painel da figura
%   hDraw    : desenhos feitos no CoppeliaSim nesta chamada
%
% Figura: cada objeto com um círculo da sua cor e uma etiqueta; as setas do
% plano por cima da imagem; um painel com o estado do modo. CoppeliaSim: um
% anel e uma cruz (alinhada com o yaw) em cada objeto e as setas do plano
% na mesa (as 'done' não se desenham).

    if nargin < 9, truth = []; end
    if nargin < 10 || isempty(yaw), yaw = nan(numel(names), 1); end
    if nargin < 11, truthYaw = []; end
    if nargin < 12, oldDraw = []; end
    if nargin < 13, titleTxt = ''; end
    if nargin < 14 || isempty(extras), extras = struct(); end
    if ~isfield(extras, 'arrows'), extras.arrows = []; end
    if ~isfield(extras, 'info'),   extras.info = {};   end
    if ~isfield(extras, 'cam'),    extras.cam = [];    end
    hDraw = [];
    n = numel(names);

    %% ---- 1. figura do MATLAB: imagem da câmara com as deteções e o plano ----
    if ~isempty(img)
        fig = findobj('Type', 'figure', 'Name', 'Visão');     % reutilizar a janela
        if isempty(fig), fig = figure('Name', 'Visão'); end
        figure(fig(1));  clf;
        image(img);  axis image;  axis off;  hold on;

        % manchas que não são de nenhum objeto (ruído, peças repetidas, ...)
        for i = 1:numel(det)
            if ~any(found & hypot(pix(:,1) - det(i).pixel(1), pix(:,2) - det(i).pixel(2)) < 0.5)
                plot(det(i).pixel(1) + 0.5, det(i).pixel(2) + 0.5, 'x', 'Color', [0.6 0.6 0.6], ...
                     'MarkerSize', 10, 'LineWidth', 2);
            end
        end

        labs = struct('h', {}, 'u', {}, 'v', {}, 'c', {});   % etiquetas, arrumadas no fim
        fixed = zeros(0, 4);                     % caixas que as etiquetas não podem tapar
        for k = 1:n
            if ~found(k) || any(isnan(pix(k,:))), continue; end
            c = to_rgb(colors{k});
            u = pix(k,1) + 0.5;  v = pix(k,2) + 0.5;
            plot(u, v, 'o', 'MarkerSize', 16, 'LineWidth', 2, 'MarkerEdgeColor', c);
            plot(u, v, 'k+', 'MarkerSize', 8, 'LineWidth', 1.5);
            txt = sprintf('%s\n(%.0f, %.0f) mm', names{k}, 1000*pos(k,1), 1000*pos(k,2));
            if ~isempty(truth)
                e = 1000*hypot(pos(k,1) - truth(k,1), pos(k,2) - truth(k,2));
                txt = sprintf('%s\nerro %.1f mm', txt, e);
            end
            if ~isnan(yaw(k))                        % orientação detetada
                txt = sprintf('%s, yaw %.0f°', txt, rad2deg(yaw(k)));
                if ~isempty(truthYaw)                % erro, a menos de 90 graus (quadrado)
                    ey = mod(yaw(k) - truthYaw(k) + pi/4, pi/2) - pi/4;
                    txt = sprintf('%s (erro %.0f°)', txt, rad2deg(ey));
                end
            end
            h = text(u + 12, v, txt, 'Color', 'w', 'BackgroundColor', [0 0 0], ...
                     'EdgeColor', c, 'LineWidth', 1, 'FontSize', 7, 'Margin', 1, ...
                     'VerticalAlignment', 'middle', 'HorizontalAlignment', 'left');
            labs(end+1) = struct('h', h, 'u', u, 'v', v, 'c', c);   %#ok<AGROW>
            fixed(end+1, :) = [u - 10, u + 10, v - 10, v + 10];     %#ok<AGROW> (o próprio círculo)
        end

        % setas do plano (por cima das etiquetas, para se verem bem)
        for a = 1:numel(extras.arrows)
            A = extras.arrows(a);
            uv = aux.world2pix(extras.cam, [A.from; A.to]) + 0.5;
            if any(isnan(uv(:))), continue; end
            [c, lw, ls] = arrow_style(A);
            plot(uv(:,1), uv(:,2), ls, 'Color', c, 'LineWidth', lw);
            if ~strcmp(A.state, 'link')
                head_px(uv(1,:), uv(2,:), c, lw);
            end
            if ~isempty(A.label)
                % a etiqueta da seta também é arrumada (não tapa os blocos):
                % fica presa ao meio da seta, com uma linha se tiver de sair dali
                m = mean(uv, 1);
                h = text(m(1), m(2), A.label, 'Color', 'k', 'BackgroundColor', c, ...
                         'FontSize', 8, 'FontWeight', 'bold', 'Margin', 1, ...
                         'HorizontalAlignment', 'left', 'VerticalAlignment', 'middle');
                labs(end+1) = struct('h', h, 'u', m(1), 'v', m(2), 'c', c);   %#ok<AGROW>
            end
        end

        % painel com o estado do modo (canto inferior esquerdo da imagem)
        if ~isempty(extras.info)
            text(6, size(img, 1) - 6, strjoin(extras.info, '\n'), 'Color', 'w', ...
                 'BackgroundColor', [0.1 0.1 0.1], 'EdgeColor', [1 1 1], 'FontSize', 9, ...
                 'FontName', 'FixedWidth', 'VerticalAlignment', 'bottom', 'Margin', 4, ...
                 'Interpreter', 'none');
            hp = findobj(gca, 'Type', 'text', '-and', 'FontName', 'FixedWidth');
            if ~isempty(hp), fixed(end+1, :) = text_box(hp(1)); end
        end

        % [ACRESCENTADO] arrumar as etiquetas dos objetos para não se taparem
        place_labels(labs, fixed, size(img, 2), size(img, 1));

        missing = names(~found);
        if ~isempty(titleTxt)
            title(titleTxt, 'Interpreter', 'none');
        elseif isempty(missing)
            title(sprintf('Visão: %d de %d objetos encontrados', nnz(found), n));
        else
            title({sprintf('Visão: %d de %d objetos encontrados', nnz(found), n), ...
                   ['NÃO ENCONTRADOS: ' strjoin(missing, ', ')]}, 'Color', [0.8 0 0]);
        end
        hold off;  drawnow;
    end

    %% ---- 2. CoppeliaSim: marcas nos objetos e setas do plano na mesa ----
    try
        sim = sim_interface.get_connection().require('sim');
        for i = 1:numel(oldDraw)                 % apagar as marcas da chamada anterior
            try, sim.removeDrawingObject(oldDraw(i)); catch, end
        end
        R = 0.022;                               % raio do anel [m]
        a = linspace(0, 2*pi, 25);
        for k = 1:n
            if ~found(k), continue; end
            c = to_rgb(colors{k});
            z = max(2*pos(k,3) + 0.002, 0.004);  % um pouco acima da face de cima (mesa em z = 0)
            x = pos(k,1);  y = pos(k,2);
            h = sim.addDrawingObject(sim.drawing_lines, 4, 0, -1, numel(a) + 2, c);
            hDraw(end+1) = double(h);            %#ok<AGROW>
            for i = 1:numel(a) - 1
                sim.addDrawingObjectItem(h, [x + R*cos(a(i)) y + R*sin(a(i)) z, ...
                                             x + R*cos(a(i+1)) y + R*sin(a(i+1)) z]);
            end
            % cruz no centro, alinhada com as faces detetadas (yaw)
            th = 0;  if ~isnan(yaw(k)), th = yaw(k); end
            e1 = R*[cos(th) sin(th)];  e2 = R*[-sin(th) cos(th)];
            sim.addDrawingObjectItem(h, [x - e1(1) y - e1(2) z, x + e1(1) y + e1(2) z]);
            sim.addDrawingObjectItem(h, [x - e2(1) y - e2(2) z, x + e2(1) y + e2(2) z]);
        end
        % setas do plano (acima dos objetos, para se verem)
        for i = 1:numel(extras.arrows)
            A = extras.arrows(i);
            if strcmp(A.state, 'done'), continue; end
            [c, lw] = arrow_style(A);
            z = max([2*A.from(3), 2*A.to(3), 0.03]) + 0.01;
            p0 = A.from(1:2);  p1 = A.to(1:2);
            d = p1 - p0;  L = norm(d);
            if L < 1e-6, continue; end
            u = d / L;  nr = [-u(2) u(1)];
            h = sim.addDrawingObject(sim.drawing_lines, max(2, round(lw*1.5)), 0, -1, 60, c);
            hDraw(end+1) = double(h);            %#ok<AGROW>
            if any(strcmp(A.state, {'todo', 'link'}))   % tracejada
                nSeg = max(2, floor(L / 0.012));
                for s = 0:2:nSeg-1
                    q0 = p0 + d*s/nSeg;  q1 = p0 + d*min(s+1, nSeg)/nSeg;
                    sim.addDrawingObjectItem(h, [q0 z, q1 z]);
                end
            else
                sim.addDrawingObjectItem(h, [p0 z, p1 z]);
            end
            if ~strcmp(A.state, 'link')          % ponta da seta
                b = p1 - 0.02*u;
                sim.addDrawingObjectItem(h, [p1 z, b + 0.012*nr z]);
                sim.addDrawingObjectItem(h, [p1 z, b - 0.012*nr z]);
            end
        end
    catch err
        fprintf('show_vision: não foi possível desenhar no CoppeliaSim (%s)\n', err.message);
    end
end

% -------------------------------------------------------------------------
function c = to_rgb(name)
% cor RGB para desenhar (nome ou [r g b]); branco se o nome não for conhecido
    if isnumeric(name), c = name; return; end
    RGB = struct('red', [1 0 0], 'orange', [1 0.5 0], 'yellow', [1 0.9 0], ...
                 'green', [0 0.8 0], 'blue', [0 0.3 1], 'pink', [1 0.4 0.8], ...
                 'white', [1 1 1], 'cyan', [0.3 0.9 1]);
    if isfield(RGB, lower(name)), c = RGB.(lower(name)); else, c = [1 1 1]; end
end

function [c, lw, ls] = arrow_style(A)
% aspeto de cada seta conforme o estado
    c = to_rgb(A.color);
    switch A.state
        case 'next', lw = 3.5;  ls = '-';
        case 'todo', lw = 1.5;  ls = '--';
        case 'link', lw = 1.5;  ls = ':';
        case 'model', lw = 2.5; ls = '-';
        otherwise,   lw = 1.0;  ls = '-';  c = [0.6 0.6 0.6];   % 'done'
    end
end

function b = text_box(h)
% caixa [x1 x2 y1 y2] (pixeis da imagem) ocupada por um texto
    e = get(h, 'Extent');                    % [x y largura altura] em unidades dos dados
    e(3:4) = abs(e(3:4));
    p = get(h, 'Position');
    switch get(h, 'HorizontalAlignment')
        case 'center', x1 = p(1) - e(3)/2;
        case 'right',  x1 = p(1) - e(3);
        otherwise,     x1 = p(1);
    end
    switch get(h, 'VerticalAlignment')
        case 'middle', y1 = p(2) - e(4)/2;
        case 'bottom', y1 = p(2) - e(4);     % (eixo y da imagem para baixo)
        otherwise,     y1 = p(2);
    end
    b = [x1, x1 + e(3), y1, y1 + e(4)];
end

function place_labels(labs, fixed, W, H)
% [ACRESCENTADO] coloca cada etiqueta junto do seu objeto sem tapar as outras:
% experimenta à direita e à esquerda, cada vez mais acima/abaixo, e escolhe
% a primeira posição livre dentro da imagem. Se a etiqueta saiu do lado do
% objeto, desenha uma linha fina a ligá-la ao objeto.
    if isempty(labs), return; end
    [~, ord] = sort([labs.v]);               % de cima para baixo
    placed = fixed;
    gap = 2;                                 % folga entre etiquetas [pixeis]
    for k = ord
        L = labs(k);
        b0 = text_box(L.h);
        w = b0(2) - b0(1);  hgt = b0(4) - b0(3);
        best = [];
        for step = 0:12
            for sgn = unique([1 -1] * step)
                dy = sgn * (hgt + gap);
                for side = [1 -1]            % 1 = à direita do objeto, -1 = à esquerda
                    if side == 1, x1 = L.u + 12; else, x1 = L.u - 12 - w; end
                    y1 = L.v + dy - hgt/2;
                    b = [x1, x1 + w, y1, y1 + hgt];
                    if b(1) < 0 || b(2) > W || b(3) < 0 || b(4) > H, continue; end
                    if ~any(overlap(b, placed, gap))
                        best = [x1, L.v + dy];  break;
                    end
                end
                if ~isempty(best), break; end
            end
            if ~isempty(best), break; end
        end
        if isempty(best), best = [L.u + 12, L.v]; end   % sem lugar livre: fica onde estava
        set(L.h, 'Position', [best 0]);
        bb = text_box(L.h);
        placed(end+1, :) = bb;                                       %#ok<AGROW>
        if abs(best(2) - L.v) > 1 || best(1) < L.u                   % saiu do lado: ligar
            xe = bb(1);  if best(1) < L.u, xe = bb(2); end
            plot([L.u xe], [L.v best(2)], '-', 'Color', L.c, 'LineWidth', 1);
        end
    end
end

function tf = overlap(b, B, gap)
% true para cada caixa de B que se sobrepõe à caixa b
    if isempty(B), tf = false; return; end
    tf = b(1) < B(:,2) + gap & b(2) > B(:,1) - gap & b(3) < B(:,4) + gap & b(4) > B(:,3) - gap;
end

function head_px(p0, p1, c, lw)
% ponta de seta na imagem (pixeis)
    d = p1 - p0;  L = norm(d);
    if L < 1, return; end
    u = d / L;  nr = [-u(2) u(1)];  s = 6 + 2*lw;
    b = p1 - s*u;
    fill([p1(1), b(1) + 0.6*s*nr(1), b(1) - 0.6*s*nr(1)], ...
         [p1(2), b(2) + 0.6*s*nr(2), b(2) - 0.6*s*nr(2)], c, 'EdgeColor', 'k');
end