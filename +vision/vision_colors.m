function [det, img, cam] = vision_colors(sim_interface, wanted)
% VISION_COLORS  Localiza peças por COR com o sensor de visão (/Vision_sensor).
%
%   det = vision_colors(sim_interface)                    % todas as cores
%   det = vision_colors(sim_interface, 'green')           % só uma cor
%   det = vision_colors(sim_interface, {'red','blue'})    % várias cores
%   [det, img, cam] = vision_colors(...)                  % + imagem e modelo da câmara
%
%   Para testar a visão sozinha (figura + erro em relação ao simulador):
%   vision.vision_check()
%
%   Cores disponíveis: 'red', 'orange', 'yellow', 'green', 'blue', 'pink'
%   (editar COLORS abaixo para ajustar os limiares de matiz/saturação).
%
%   det : struct array, um elemento por peça encontrada, com os campos
%         color - nome da cor
%         pixel - [coluna linha] do centro da mancha na imagem
%         area  - nº de pixeis da mancha
%         world - [x y z] estimado no referencial do MUNDO [m], à altura Z_PLANE
%         cam   - [x y z] posição da câmara no MUNDO [m] (para recalcular o
%                 ponto noutra altura: vision.at_height)
%         yaw   - orientação da face de cima no MUNDO [rad], entre -45 e 45
%                 graus (um quadrado repete-se a cada 90 graus); NaN se a
%                 mancha é pequena demais
%         size  - largura média da face de cima, medida à altura Z_PLANE [m]
%                 (lado de um quadrado, diâmetro de um círculo)
%         fill  - quanto a face de cima enche o seu retângulo mínimo:
%                 ~1 quadrado, ~0,79 círculo (pi/4)
%   img : imagem RGB (H x W x 3, 0..1) usada na deteção (para show_vision.m)
%   cam : modelo da câmara (W, H, f, Rc, pc, sgn) para vision.cam_map
% Como funciona
%   1. Lê a imagem do sensor e converte para HSV.
%   2. Para cada cor, cria uma máscara e separa em manchas ligadas (uma por peça).
%   3. Fica só com a face de cima de cada mancha (os pixeis mais claros).
%   4. O centro da face de cima define um raio a partir da câmara; o ponto da
%      peça é onde esse raio corta a altura Z_PLANE.
%   5. Os pixeis da face de cima, passados para a mesa, dão a orientação,
%      o tamanho e a forma (retângulo de área mínima).

    %% ===================== CONFIGURAÇÃO =====================
    Z_PLANE   = 0.0192;   % altura [m] da face de cima das peças (mundo). Os blocos
                          % têm BLOCK_H = 0.0192 (sem os pinos). Ajustar se necessário.
    MIN_AREA  = 40;       % manchas com menos pixeis são ignoradas (ruído)
    SIGN      = [-1 -1];  % sinais [x y] do modelo da câmara (vision_check sugere
                          % a convenção certa se estes estiverem errados)
    FOV_DEG   = [];       % [] = ler do simulador; ou pôr à mão o ângulo de visão [graus]

    % limiares HSV: {nome, [hmin hmax], smin, vmin}  (H em 0..1; vermelho dá a volta)
    COLORS = { ...
        'red',    [0.95 0.04], 0.45, 0.15; ...
        'orange', [0.05 0.125],0.40, 0.30; ...
        'yellow', [0.13 0.21], 0.40, 0.25; ...
        'green',  [0.22 0.47], 0.40, 0.15; ...
        'blue',   [0.55 0.72], 0.45, 0.15; ...
        'pink',   [0.72 0.92], 0.12, 0.50 };
    TOP_FRAC  = 0.85;     % só conta a face de cima: pixeis com brilho >= TOP_FRAC * brilho
                          % típico da mancha (as faces laterais são mais escuras e
                          % deslocavam o centro). 0 = usar a mancha toda.
    %% =========================================================
    if nargin < 2 || isempty(wanted), wanted = COLORS(:,1)'; end
    if ischar(wanted) || isstring(wanted), wanted = cellstr(wanted); end
    sim = sim_interface.get_connection().require('sim');
    hSensor = sim.getObject('/Vision_sensor');

    % o sensor só tem imagem depois de a simulação ter dado um passo
    for k = 1:3
        sim_interface.step_simulation();
    end

    %% ---- 1. imagem ----
    [imageData, res] = sim.getVisionSensorImg(hSensor);
    if iscell(res), W = res{1}; H = res{2}; else, W = res(1); H = res(2); end
    % o sensor devolve os pixeis em linha (R,G,B,R,G,B,...) a começar por baixo:
    % reorganizar numa imagem H x W x 3 com a linha 1 em cima e valores 0..1;
    % depois converter para HSV (matiz, saturação, brilho), mais fácil para cores
    img = flip(permute(reshape(double(imageData(:)), 3, W, H), [3 2 1]), 1) / 255;   % H x W x 3, linha 1 = topo
    hsv = rgb2hsv(img);

    %% ---- 2. modelo da câmara (lido do simulador) ----
    % posição e orientação do sensor no mundo + distância focal f (em pixeis),
    % a partir do ângulo de visão: é o que permite passar de pixel a ponto da mesa
    M  = to_vec(sim.getObjectMatrix(hSensor, -1));      % 12 valores, matriz 3x4 por linhas
    M  = reshape(M, 4, 3)';
    if isempty(FOV_DEG)
        fov = read_fov(sim, hSensor);
    else
        fov = deg2rad(FOV_DEG);
    end
    f = (max(W, H)/2) / tan(fov/2);                     % distância focal [pixeis]
    cam = struct('W', W, 'H', H, 'f', f, 'Rc', M(:, 1:3), 'pc', M(:, 4), 'sgn', SIGN);

    %% ---- 3. deteção por cor ----
    det = struct('color', {}, 'pixel', {}, 'area', {}, 'world', {}, ...
                 'cam', {}, 'yaw', {}, 'size', {}, 'fill', {});
    Hh = hsv(:,:,1);  Ss = hsv(:,:,2);  Vv = hsv(:,:,3);
    for ic = 1:size(COLORS, 1)
        name = COLORS{ic, 1};
        if ~any(strcmpi(name, wanted)), continue; end

        % 3a. máscara desta cor: pixeis com matiz no intervalo e saturação/brilho
        %     suficientes
        hr = COLORS{ic, 2};
        if hr(1) <= hr(2), inH = Hh >= hr(1) & Hh <= hr(2);
        else,              inH = Hh >= hr(1) | Hh <= hr(2);   % vermelho: dá a volta
        end
        mask = inH & Ss >= COLORS{ic, 3} & Vv >= COLORS{ic, 4};

        % 3b. separar a máscara em manchas ligadas (uma por peça); ignorar as pequenas
        [labels, n] = cc_label(mask);
        for b = 1:n
            [rows, cols] = find(labels == b);
            if numel(rows) < MIN_AREA, continue; end

            % 3c. ficar só com a face de cima (os pixeis mais claros da mancha)
            rr = rows;  cc = cols;
            if TOP_FRAC > 0
                vals = Vv(sub2ind([H W], rows, cols));
                sv   = sort(vals);
                ref  = sv(max(1, round(0.95*numel(sv))));
                top  = vals >= TOP_FRAC*ref;
                if nnz(top) >= 10, rr = rows(top);  cc = cols(top); end
            end

            % 3d. centro da face de cima -> ponto da mesa (raio da câmara cortado
            %     no plano Z_PLANE)
            u = mean(cc) - 0.5;  v = mean(rr) - 0.5;          % centro da mancha (pixeis)
            P = vision.cam_map(cam, [u v], Z_PLANE);
            if any(isnan(P)), continue; end

            % 3e. orientação, tamanho e forma: os pixeis da face de cima passam
            %     para a mesa e procura-se o retângulo mais pequeno que os contém
            yaw = NaN;  sz = NaN;  fill = NaN;
            if numel(rr) >= 20
                Pw = vision.cam_map(cam, [cc - 0.5 rr - 0.5], Z_PLANE);
                [yaw, sides, rectA] = min_rect(Pw(:,1), Pw(:,2));
                sz   = mean(sides);
                fill = hull_area(Pw(:,1), Pw(:,2)) / max(rectA, eps);
            end

            % 3f. guardar a deteção
            det(end+1) = struct('color', name, 'pixel', [u v], 'area', numel(rows), ...
                                'world', P, 'cam', cam.pc(:)', ...
                                'yaw', yaw, 'size', sz, 'fill', fill);   %#ok<AGROW>
        end
    end
end

% =========================================================================
function [yaw, sides, rectA] = min_rect(x, y)
% retângulo de área mínima que contém os pontos (x, y):
%   yaw   - o seu ângulo [rad], entre -45 e 45 graus (um quadrado repete-se a
%           cada 90 graus, por isso basta procurar entre 0 e 90 graus)
%   sides - [comprimento largura]      rectA - a sua área
    % rodar os pontos de 0 a 89,5 graus (passos de 0,5) e medir a área do
    % retângulo alinhado que os contém; o ângulo de área mínima é o da peça
    x = x(:)' - mean(x);  y = y(:)' - mean(y);
    ang = deg2rad(0:0.5:89.5);
    area = zeros(size(ang));
    for i = 1:numel(ang)
        xr =  cos(ang(i))*x + sin(ang(i))*y;
        yr = -sin(ang(i))*x + cos(ang(i))*y;
        area(i) = (max(xr) - min(xr)) * (max(yr) - min(yr));
    end
    [rectA, i] = min(area);
    yaw = mod(ang(i) + pi/4, pi/2) - pi/4;
    xr =  cos(ang(i))*x + sin(ang(i))*y;
    yr = -sin(ang(i))*x + cos(ang(i))*y;
    sides = [max(xr) - min(xr), max(yr) - min(yr)];
end

function A = hull_area(x, y)
% área do contorno convexo dos pontos (x, y): ignora buracos e sombras dentro
% da face de cima (ex.: os pinos dos blocos)
    try
        k = convhull(x(:), y(:));
        A = polyarea(x(k), y(k));
    catch
        A = NaN;                                 % pontos alinhados / poucos pontos
    end
end

function fov = read_fov(sim, hSensor)
% ângulo de visão (perspetiva) do sensor [rad]; aplica-se ao lado maior da imagem
    try,   id = sim.visionfloatparam_perspective_angle;
    catch, id = 1004;
    end
    try
        [~, v] = sim.getObjectFloatParam(hSensor, id);
    catch
        v = sim.getObjectFloatParam(hSensor, id);
    end
    v = to_vec(v);  v = v(end);
    if ~(v > 0.01 && v < pi)
        error('vision_colors: não consegui ler o ângulo de visão (%g). Definir FOV_DEG à mão.', v);
    end
    fov = v;
end

function v = to_vec(c)
% célula (ou vetor) de números -> vetor linha
    if iscell(c), v = cell2mat(c(:)'); else, v = double(c(:)'); end
end

function [labels, n] = cc_label(mask)
% componentes ligadas (8 vizinhos) sem precisar da Image Processing Toolbox
    [H, W] = size(mask);
    labels = zeros(H, W);
    idx = find(mask)';
    n = 0;
    stack = zeros(max(numel(idx), 1), 1);
    % para cada pixel ainda sem etiqueta: nova mancha; "pintar" com o mesmo
    % número todos os pixeis ligados a ele (pilha de pixeis por visitar, 8 vizinhos)
    for p = idx
        if labels(p) ~= 0, continue; end
        n = n + 1;  labels(p) = n;
        sp = 1;  stack(1) = p;
        while sp > 0
            q = stack(sp);  sp = sp - 1;
            r = mod(q - 1, H) + 1;  c = (q - r)/H + 1;
            for dr = -1:1
                for dc = -1:1
                    rr = r + dr;  cc = c + dc;
                    if rr >= 1 && rr <= H && cc >= 1 && cc <= W
                        qq = rr + (cc - 1)*H;
                        if mask(qq) && labels(qq) == 0
                            labels(qq) = n;  sp = sp + 1;  stack(sp) = qq;
                        end
                    end
                end
            end
        end
    end
end