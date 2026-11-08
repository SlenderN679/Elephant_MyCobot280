function [pos, found, pix, img, det, yaw, cam] = vision_snapshot(sim_interface, objIds, colors, ranks, halfH, tableZ, shapes, sizes)
% VISION_SNAPSHOT  Posição de cada objeto obtida SÓ pela câmara (vision_colors).
%
%   [pos, found] = vision_snapshot(sim_interface, objIds, colors, ranks, halfH, tableZ)
%   [pos, found] = vision_snapshot(..., shapes, sizes)
%   [pos, found, pix, img, det, yaw, cam] = vision_snapshot(...)   % + dados para show_vision
%
%   objIds : índices dos objetos (os mesmos de get_target_position)
%   colors : cell com a cor de cada objeto de objIds ('blue', 'yellow', ...)
%   ranks  : qual das manchas dessa cor é o objeto, por área decrescente
%            (1 = a maior). Só se usa se não houver shapes/sizes para essa cor.
%   halfH  : metade da altura de cada objeto [m] (o alvo plano ~0,001)
%   tableZ : altura da mesa [m]
%   shapes : (opcional) cell com a forma da face de cima de cada objeto:
%            'square' (cubo, bloco), 'round' (cilindro) ou '' (não usar)
%   sizes  : (opcional) largura da face de cima de cada objeto [m]
%            (lado do cubo/bloco, diâmetro do cilindro; 0 = não usar).
%            Os valores medidos aparecem em vision.vision_check().
%   pos    : N x 3 posição do CENTRO de cada objeto no MUNDO [m]
%            (NaN se não foi encontrado)
%   found  : N x 1 true se o objeto foi encontrado na imagem
%   pix    : N x 2 pixel [coluna linha] do centro de cada objeto na imagem (NaN se não)
%   img    : imagem da câmara (H x W x 3) e det: todas as manchas detetadas
%   yaw    : N x 1 orientação de cada objeto no MUNDO [rad], entre -45 e 45
%            graus (NaN se não foi encontrado / mancha pequena)
%
% Uma só fotografia para todas as cores. A visão calcula o ponto da face de
% cima de cada peça à altura de um bloco; como as peças têm alturas
% diferentes, o ponto é recalculado ao longo do mesmo raio da câmara à altura
% certa da face de cima de cada objeto (tableZ + 2*halfH) e depois baixado
% para o centro (menos halfH).

    ROUND_FILL = 0.88;   % fill abaixo disto = face redonda (círculo ~0,79, quadrado ~1)
    SIZE_TOL   = 0.20;   % diferença de tamanho aceite (20 %)
    n = numel(objIds);
    if nargin < 7 || isempty(shapes), shapes = repmat({''}, 1, n); end
    if nargin < 8 || isempty(sizes),  sizes  = zeros(1, n); end
    sizes(isnan(sizes)) = 0;

    % Passo 1: uma fotografia com todas as cores pedidas
    pos = nan(n, 3);  found = false(n, 1);  pix = nan(n, 2);  yaw = nan(n, 1);
    [det, img, cam] = vision.vision_colors(sim_interface, unique(colors));
    if isempty(det), return; end

    % Passo 2: que mancha é que objeto (cor, forma, tamanho, área)
    choice = assign_blobs(det, colors, ranks, halfH, tableZ, shapes, sizes, ROUND_FILL, SIZE_TOL);

    % Passo 3: corrigir a altura ao longo do raio da câmara e baixar para o centro
    for k = 1:n
        if choice(k) == 0, continue; end
        d = det(choice(k));
        zTop = tableZ + 2*halfH(k);                  % face de cima deste objeto
        top = vision.at_height(d, zTop);
        pos(k, :) = [top(1:2), zTop - halfH(k)];     % centro do objeto
        found(k) = true;
        pix(k, :) = d.pixel;
        yaw(k) = d.yaw;
    end
end

% -------------------------------------------------------------------------
function choice = assign_blobs(det, colors, ranks, halfH, tableZ, shapes, sizes, ROUND_FILL, SIZE_TOL)
% choice(k) = índice em det da mancha do objeto k (0 = não encontrado).
% Para cada cor:
%   - se nenhum objeto dessa cor tem forma/tamanho: a ranks(k)-ésima maior
%     (por área); cada mancha só serve a um objeto
%   - senão: custo de cada par (objeto, mancha) pelo tamanho (a ordem de área
%     só desempata); pares impossíveis (forma errada, tamanho muito diferente)
%     nunca são escolhidos; escolhem-se primeiro os pares mais baratos.
%     Um objeto sem par fica "não encontrado" em vez de trocado.
    n = numel(colors);  choice = zeros(1, n);
    cols = unique(lower(colors));
    for ci = 1:numel(cols)
        objs  = find(strcmpi(colors, cols{ci}));     objs  = objs(:)';
        blobs = find(strcmpi({det.color}, cols{ci})); blobs = blobs(:)';
        if isempty(blobs), continue; end
        [~, ord] = sort([det(blobs).area], 'descend');
        rankOf = zeros(1, numel(blobs));  rankOf(ord) = 1:numel(blobs);

        useInfo = any(~cellfun(@isempty, shapes(objs))) || any(sizes(objs) > 0);
        C = inf(numel(objs), numel(blobs));
        for i = 1:numel(objs)
            k = objs(i);
            for j = 1:numel(blobs)
                if ~useInfo                                   % só a ordem de área
                    C(i, j) = abs(rankOf(j) - ranks(k));
                    continue;
                end
                d = det(blobs(j));  c = 0;
                if ~isempty(shapes{k}) && ~isnan(d.fill)
                    isRound = d.fill < ROUND_FILL;
                    if isRound ~= strcmpi(shapes{k}, 'round'), continue; end   % forma errada
                end
                if sizes(k) > 0 && ~isnan(d.size)
                    % o tamanho foi medido à altura de um bloco: passar para a
                    % altura da face de cima deste objeto (mais alto = mais perto
                    % da câmara = parece maior)
                    zTop = tableZ + 2*halfH(k);
                    s = d.size * (d.cam(3) - zTop) / (d.cam(3) - d.world(3));
                    e = abs(s - sizes(k)) / sizes(k);
                    if e > SIZE_TOL, continue; end                          % tamanho errado
                    c = c + e;
                end
                C(i, j) = c + 1e-3 * abs(rankOf(j) - ranks(k));             % desempate: área
            end
        end
        % pares mais baratos primeiro, cada objeto e cada mancha só uma vez
        while true
            [m, idx] = min(C(:));
            if isinf(m), break; end
            [i, j] = ind2sub(size(C), idx);
            choice(objs(i)) = blobs(j);
            C(i, :) = inf;  C(:, j) = inf;
        end
    end
end