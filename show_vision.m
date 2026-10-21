function hDraw = show_vision(sim_interface, img, det, names, colors, pos, pix, found, truth, yaw, truthYaw, oldDraw, titleTxt)
% SHOW_VISION  Mostra o resultado da visão: na imagem da câmara (figura do
%              MATLAB) e na mesa do CoppeliaSim (marcas desenhadas).
%
%   show_vision(sim_interface, img, det, names, colors, pos, pix, found)
%   show_vision(sim_interface, img, det, names, colors, pos, pix, found, truth)
%
%   img    : imagem da câmara (de vision_snapshot)
%   det    : todas as manchas detetadas (de vision_snapshot)
%   names  : cell com o nome de cada objeto ('Block1', ...)
%   colors : cell com a cor de cada objeto ('yellow', ...)
%   pos    : N x 3 centro de cada objeto no MUNDO [m] (de vision_snapshot)
%   pix    : N x 2 pixel do centro de cada objeto na imagem
%   found  : N x 1 true se o objeto foi encontrado
%   truth  : (opcional, só simulador) N x 3 posições reais, para mostrar o erro
%   yaw    : (opcional) N x 1 orientação detetada [rad] (NaN = sem orientação)
%   truthYaw : (opcional, só simulador) N x 1 orientação real [rad]
%   oldDraw  : (opcional) desenhos devolvidos pela chamada anterior; são
%              apagados antes de desenhar os novos (para mostrar fotografias
%              periódicas sem acumular marcas na mesa)
%   titleTxt : (opcional) título da figura em vez do título por defeito
%   hDraw    : desenhos feitos no CoppeliaSim nesta chamada
%
% Figura: cada objeto encontrado fica marcado com um círculo da sua cor e
% uma etiqueta (nome, posição em mm e, se houver truth, o erro). Manchas que
% não foram atribuídas a nenhum objeto aparecem com um X cinzento. Os
% objetos não encontrados aparecem a vermelho no título.
% CoppeliaSim: um anel e uma cruz da cor do objeto no ponto onde a visão o
% viu (em cima da peça). As marcas mostram a FOTOGRAFIA: não seguem as
% peças quando o robô as muda de sítio.

    if nargin < 9, truth = []; end
    if nargin < 10 || isempty(yaw), yaw = nan(numel(names), 1); end
    if nargin < 11, truthYaw = []; end
    if nargin < 12, oldDraw = []; end
    if nargin < 13, titleTxt = ''; end
    hDraw = [];
    n = numel(names);
    RGB = struct('red', [1 0 0], 'orange', [1 0.5 0], 'yellow', [1 0.9 0], ...
                 'green', [0 0.8 0], 'blue', [0 0.3 1], 'pink', [1 0.4 0.8]);

    %% ---- 1. figura do MATLAB: imagem da câmara com as deteções ----
    fig = findobj('Type', 'figure', 'Name', 'Visão');     % reutilizar a janela
    if isempty(fig), fig = figure('Name', 'Visão'); end
    figure(fig(1));  clf;
    if ~isempty(img), image(img); end
    axis image;  hold on;

    % manchas que não são de nenhum objeto (ruído, peças repetidas, ...)
    for i = 1:numel(det)
        if ~any(found & hypot(pix(:,1) - det(i).pixel(1), pix(:,2) - det(i).pixel(2)) < 0.5)
            plot(det(i).pixel(1) + 0.5, det(i).pixel(2) + 0.5, 'x', 'Color', [0.6 0.6 0.6], ...
                 'MarkerSize', 10, 'LineWidth', 2);
        end
    end

    for k = 1:n
        if ~found(k), continue; end
        c = pick_color(RGB, colors{k});
        u = pix(k,1) + 0.5;  v = pix(k,2) + 0.5;
        plot(u, v, 'o', 'MarkerSize', 16, 'LineWidth', 2, 'MarkerEdgeColor', c);
        plot(u, v, 'k+', 'MarkerSize', 8, 'LineWidth', 1.5);
        txt = sprintf('%s\n(%.0f, %.0f) mm', names{k}, 1000*pos(k,1), 1000*pos(k,2));
        if ~isempty(truth)
            e = 1000*hypot(pos(k,1) - truth(k,1), pos(k,2) - truth(k,2));
            txt = sprintf('%s\nerro %.1f mm', txt, e);
        end
        if ~isnan(yaw(k))                        % orientação detetada
            txt = sprintf('%s\nyaw %.0f°', txt, rad2deg(yaw(k)));
            if ~isempty(truthYaw)                % erro, a menos de 90 graus (quadrado)
                ey = mod(yaw(k) - truthYaw(k) + pi/4, pi/2) - pi/4;
                txt = sprintf('%s (erro %.0f°)', txt, rad2deg(ey));
            end
        end
        text(u + 12, v, txt, 'Color', 'w', 'BackgroundColor', [0 0 0], ...
             'EdgeColor', c, 'LineWidth', 1.5, 'FontSize', 8, 'Margin', 1);
    end

    missing = names(~found);
    if ~isempty(titleTxt)
        title(titleTxt);
    elseif isempty(missing)
        title(sprintf('Visão: %d de %d objetos encontrados', nnz(found), n));
    else
        title({sprintf('Visão: %d de %d objetos encontrados', nnz(found), n), ...
               ['NÃO ENCONTRADOS: ' strjoin(missing, ', ')]}, 'Color', [0.8 0 0]);
    end
    hold off;  drawnow;

    %% ---- 2. CoppeliaSim: marcas na mesa onde a visão viu cada objeto ----
    try
        sim = sim_interface.get_connection().require('sim');
        for i = 1:numel(oldDraw)                 % apagar as marcas da fotografia anterior
            try, sim.removeDrawingObject(oldDraw(i)); catch, end
        end
        R = 0.022;                               % raio do anel [m]
        a = linspace(0, 2*pi, 25);
        for k = 1:n
            if ~found(k), continue; end
            c = pick_color(RGB, colors{k});
            z = max(2*pos(k,3) + 0.002, 0.004);      % um pouco acima da face de cima (mesa em z = 0)
            x = pos(k,1);  y = pos(k,2);
            h = sim.addDrawingObject(sim.drawing_lines, 4, 0, -1, numel(a) + 2, c);
            hDraw(end+1) = double(h);            %#ok<AGROW>
            for i = 1:numel(a) - 1
                sim.addDrawingObjectItem(h, [x + R*cos(a(i)) y + R*sin(a(i)) z, ...
                                             x + R*cos(a(i+1)) y + R*sin(a(i+1)) z]);
            end
            % cruz no centro, alinhada com as faces detetadas (yaw); sem yaw
            % fica alinhada com os eixos do mundo
            th = 0;  if ~isnan(yaw(k)), th = yaw(k); end
            e1 = R*[cos(th) sin(th)];  e2 = R*[-sin(th) cos(th)];
            sim.addDrawingObjectItem(h, [x - e1(1) y - e1(2) z, x + e1(1) y + e1(2) z]);
            sim.addDrawingObjectItem(h, [x - e2(1) y - e2(2) z, x + e2(1) y + e2(2) z]);
        end
    catch err
        fprintf('show_vision: não foi possível desenhar no CoppeliaSim (%s)\n', err.message);
    end
end

% -------------------------------------------------------------------------
function c = pick_color(RGB, name)
% cor RGB para desenhar; branco se o nome não for conhecido
    if isfield(RGB, lower(name)), c = RGB.(lower(name)); else, c = [1 1 1]; end
end