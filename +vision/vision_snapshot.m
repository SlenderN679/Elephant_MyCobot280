function [pos, found, pix, img, det, yaw, cam] = vision_snapshot(sim_interface, objIds, colors, ranks, halfH, tableZ)
% VISION_SNAPSHOT  Posição de cada objeto obtida SÓ pela câmara (vision_colors).
%
%   [pos, found] = vision_snapshot(sim_interface, objIds, colors, ranks, halfH, tableZ)
%   [pos, found, pix, img, det, yaw, cam] = vision_snapshot(...)   % + dados para show_vision
%
%   objIds : índices dos objetos (os mesmos de get_target_position)
%   colors : cell com a cor de cada objeto de objIds ('blue', 'yellow', ...)
%   ranks  : qual das manchas dessa cor é o objeto, por área decrescente
%            (1 = a maior). Ex.: cubo azul (3x3 cm) = 1, cilindro azul (d 3 cm) = 2
%   halfH  : metade da altura de cada objeto [m] (o alvo plano ~0,001)
%   tableZ : altura da mesa [m]
%   pos    : N x 3 posição do CENTRO de cada objeto no MUNDO [m]
%            (NaN se não foi encontrado)
%   found  : N x 1 true se o objeto foi encontrado na imagem
%   pix    : N x 2 pixel [coluna linha] do centro de cada objeto na imagem (NaN se não)
%   img    : imagem da câmara (H x W x 3) e det: todas as manchas detetadas
%   yaw    : N x 1 orientação de cada objeto no MUNDO [rad], entre -45 e 45
%            graus (NaN se não foi encontrado / mancha pequena)
%
% Uma só fotografia para todas as cores (vision_colors em modo silencioso).
% A visão calcula o ponto da face de cima de cada peça no plano Z_PLANE de
% vision_colors (topo dos blocos); como as peças têm alturas diferentes, o
% ponto é recalculado ao longo do mesmo raio da câmara na altura certa da
% face de cima de cada objeto (tableZ + 2*halfH) e depois baixado para o
% centro (menos halfH).
%
% Atenção: a câmara só vê o que não está tapado. Chamar com o braço fora
% do caminho (ex.: na vertical) e antes de empilhar peças em cima de outras.

    n = numel(objIds);
    pos = nan(n, 3);  found = false(n, 1);  pix = nan(n, 2);  yaw = nan(n, 1);
    [det, img, cam] = vision.vision_colors(sim_interface, unique(colors), true);
    if isempty(det), return; end

    for k = 1:n
        cand = det(strcmpi({det.color}, colors{k}));
        if numel(cand) < ranks(k), continue; end
        [~, order] = sort([cand.area], 'descend');
        d = cand(order(ranks(k)));

        zTop = tableZ + 2*halfH(k);                  % face de cima deste objeto
        c = d.cam;  w = d.world;                     % câmara e ponto em Z_PLANE
        top = c + (zTop - c(3)) / (w(3) - c(3)) * (w - c);
        pos(k, :) = [top(1:2), zTop - halfH(k)];     % centro do objeto
        found(k) = true;
        pix(k, :) = d.pixel;
        yaw(k) = d.yaw;
    end
end