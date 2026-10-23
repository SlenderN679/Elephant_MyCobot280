function [det, img, cam] = vision_colors(sim_interface, wanted, quiet)
% VISION_COLORS  Localiza peças por COR com o sensor de visão (/Vision_sensor).
%
%   det = vision_colors()                    % corre sozinho (liga ao CoppeliaSim,
%                                            % arranca a simulação, mede e pára)
%   det = vision_colors(sim_interface)       % usa a ligação já aberta (main_program)
%   det = vision_colors(sim_interface, 'green')           % só uma cor
%   det = vision_colors([], {'red','blue'})                % várias cores
%   det = vision_colors(sim_interface, 'green', true)     % modo silencioso: sem texto nem figura
%
%   Antes de correr: abrir mycobot280_vision.ttt no CoppeliaSim (sem correr a
%   simulação). Cores disponíveis: 'red', 'orange', 'yellow', 'green', 'blue', 'pink'
%   (editar COLORS abaixo para ajustar os limiares de matiz/saturação).
%
%   det : struct array, um elemento por peça encontrada, com os campos
%         color           - nome da cor
%         pixel           - [coluna linha] do centro da mancha na imagem
%         area            - nº de pixeis da mancha
%         world           - [x y z] estimado no referencial do MUNDO [m]
%         width_px        - largura detetada na imagem [pixeis]
%         height_px       - altura detetada na imagem [pixeis]
%         width_cm        - largura convertida [cm]
%         height_cm       - altura convertida [cm]
%         scale_px_per_cm - escala de conversão utilizada
%         cam             - [x y z] posição da câmara no MUNDO [m]  [ACRESCENTADO: para
%                           recalcular o ponto noutra altura, ver vision_snapshot.m]
%         yaw             - [ACRESCENTADO] orientação da face de cima no MUNDO [rad],
%                           entre -45 e 45 graus (um quadrado repete-se a cada 90
%                           graus); NaN se a mancha é pequena demais
%   img : [ACRESCENTADO] imagem RGB (H x W x 3, 0..1) usada na deteção (para show_vision.m)
%   cam : [ACRESCENTADO] modelo da câmara (W, H, f, Rc, pc, sgn) para passar pontos
%         do mundo para pixeis da imagem (ver world2pix.m)
%
% Como funciona
%   1. Lê a imagem do sensor e converte para HSV.
%   2. Para cada cor, cria uma máscara e separa em manchas ligadas (uma por peça).
%   3. O centro de cada mancha é convertido num raio a partir da câmara...
    
    %% ===================== CONFIGURAÇÃO =====================
    Z_PLANE   = 0.0192;   % altura [m] da face de cima das peças (mundo). Os blocos
                          % têm BLOCK_H = 0.0192 (sem os pinos). Ajustar se necessário.
    MIN_AREA  = 40;       % manchas com menos pixeis são ignoradas (ruído)
    TOL_MM    = 10;       % erro [mm] acima do qual se escreve FALHOU
    SIGN      = [-1 -1];  % sinais [x y] do modelo da câmara (ver nota no fim);
                          % o script sugere a convenção certa se estes estiverem errados
    FOV_DEG   = [];       % [] = ler do simulador; ou pôr à mão o ângulo de visão [graus]
    SHOW      = true;     % mostrar a imagem com as deteções
    
    % NOTA IMPORTANTE SOBRE A ESCALA: 
    % O valor de 500 píxeis por centímetro é a escala de referência atual. Para obter
    % dimensões físicas reais de forma fiável, esta escala tem de ser calibrada de acordo
    % com a resolução real da câmara, o campo de visão (FOV) e a distância aos objetos.
    % Se a distância dos objetos à câmara variar, a escala fixa deixará de estar correta.
    SCALE_PX_PER_CM = 500; 
    
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
    TRUTH_NAMES = {'BlueCube','BlueCylinder','RedCube','TargetGreen','Block1','Block2','Block3'};
    
    %% =========================================================
    if nargin < 3 || isempty(quiet), quiet = false; end
    if quiet, SHOW = false; end
    if nargin < 2 || isempty(wanted), wanted = COLORS(:,1)'; end
    if ischar(wanted) || isstring(wanted), wanted = cellstr(wanted); end
    
    % ---- ligação ao simulador (se não foi dada) ----
    ownSim = (nargin < 1 || isempty(sim_interface));
    if ownSim
        [sim_interface, err] = simulator_interface(0);
        if err == 1, det = []; img = []; cam = []; return; end
        cleanup = onCleanup(@() sim_interface.terminate()); %#ok<NASGU>
        sim_interface.start_simulation();
    end
    sim = sim_interface.get_connection().require('sim');
    hSensor = sim.getObject('/Vision_sensor');
    
    % o sensor só tem imagem depois de a simulação ter dado um passo
    for k = 1:3
        sim_interface.step_simulation();
    end
    
    %% ---- 1. imagem ----
    [imageData, res] = sim.getVisionSensorImg(hSensor);
    if iscell(res), W = res{1}; H = res{2}; else, W = res(1); H = res(2); end
    img = flip(permute(reshape(double(imageData(:)), 3, W, H), [3 2 1]), 1) / 255;   % H x W x 3, linha 1 = topo
    hsv = rgb2hsv(img);
    
    %% ---- 2. modelo da câmara (lido do simulador) ----
    M  = to_vec(sim.getObjectMatrix(hSensor, -1));      % 12 valores, matriz 3x4 por linhas
    M  = reshape(M, 4, 3)';
    Rc = M(:, 1:3);                                     % orientação do sensor no mundo
    pc = M(:, 4);                                       % posição do sensor no mundo
    if isempty(FOV_DEG)
        fov = read_fov(sim, hSensor);
    else
        fov = deg2rad(FOV_DEG);
    end
    f = (max(W, H)/2) / tan(fov/2);                     % distância focal [pixeis]
    cam = struct('W', W, 'H', H, 'f', f, 'Rc', Rc, 'pc', pc, 'sgn', SIGN);   % [ACRESCENTADO]
    if ~quiet
        fprintf('Sensor: %dx%d px, FOV %.1f graus, posição [%.3f %.3f %.3f]\n', ...
                W, H, rad2deg(fov), pc);
    end
    
    %% ---- 3. deteção por cor ----
    det = struct('color', {}, 'pixel', {}, 'area', {}, 'world', {}, ...
                 'width_px', {}, 'height_px', {}, 'width_cm', {}, 'height_cm', {}, 'scale_px_per_cm', {}, ...
                 'cam', {}, 'yaw', {});
             
    Hh = hsv(:,:,1);  Ss = hsv(:,:,2);  Vv = hsv(:,:,3);
    for ic = 1:size(COLORS, 1)
        name = COLORS{ic, 1};
        if ~any(strcmpi(name, wanted)), continue; end
        
        hr = COLORS{ic, 2};
        if hr(1) <= hr(2), inH = Hh >= hr(1) & Hh <= hr(2);
        else,              inH = Hh >= hr(1) | Hh <= hr(2);   % vermelho: dá a volta
        end
        mask = inH & Ss >= COLORS{ic, 3} & Vv >= COLORS{ic, 4};
        
        [labels, n] = cc_label(mask);
        for b = 1:n
            [rows, cols] = find(labels == b);
            if numel(rows) < MIN_AREA, continue; end
            
            rr = rows;  cc = cols;
            if TOP_FRAC > 0                                    % ficar só com a face de cima
                vals = Vv(sub2ind([H W], rows, cols));
                sv   = sort(vals);
                ref  = sv(max(1, round(0.95*numel(sv))));
                top  = vals >= TOP_FRAC*ref;
                if nnz(top) >= 10, rr = rows(top);  cc = cols(top); end
            end
            
            u = mean(cc) - 0.5;  v = mean(rr) - 0.5;          % centro da mancha (pixeis)

            % [ACRESCENTADO] orientação: os pixeis da face de cima passam para
            % o plano da mesa (mundo) e procura-se o retângulo mais pequeno
            % que os contém; o ângulo desse retângulo é o yaw da peça
            yaw = NaN;
            if numel(rr) >= 20
                Pw  = pix2world_many(cc - 0.5, rr - 0.5, W, H, f, Rc, pc, Z_PLANE, SIGN);
                yaw = min_rect_yaw(Pw(1,:), Pw(2,:));
            end
            P = pix2world(u, v, W, H, f, Rc, pc, Z_PLANE, SIGN);
            if isempty(P), continue; end
            
            % Cálculos de Dimensões em Píxeis e Centímetros
            width_px = max(cc) - min(cc) + 1;
            height_px = max(rr) - min(rr) + 1;
            width_cm = width_px / SCALE_PX_PER_CM;
            height_cm = height_px / SCALE_PX_PER_CM;
            
            det(end+1) = struct('color', name, 'pixel', [u v], ...
                                'area', numel(rows), 'world', P', ...
                                'width_px', width_px, 'height_px', height_px, ...
                                'width_cm', width_cm, 'height_cm', height_cm, ...
                                'scale_px_per_cm', SCALE_PX_PER_CM, 'cam', pc', ...
                                'yaw', yaw); %#ok<AGROW>
        end
    end
    
    %% ---- 4. resultados ----
    if quiet
        % modo silencioso: só devolve det
    elseif isempty(det)
        fprintf('Nenhuma peça encontrada (ver limiares de cor / MIN_AREA).\n');
    else
        % Apresentação de resultados de dimensão e área
        fprintf('\nDimensões calculadas por objeto:\n');
        fprintf('%-8s %-10s %-18s %-15s\n', 'Cor', 'Área (px)', 'Dimensões (px)', 'Dimensões (cm)');
        for i = 1:numel(det)
            fprintf('%-8s %-10d %4d x %-4d px     %4.2f x %-4.2f cm\n', ...
                    det(i).color, det(i).area, det(i).width_px, det(i).height_px, det(i).width_cm, det(i).height_cm);
        end
        
        % Avaliação de mundo vs simulador
        T = zeros(numel(TRUTH_NAMES), 3);
        for k = 1:numel(TRUTH_NAMES)
            T(k, :) = sim_interface.get_target_position(k);
        end
        
        fprintf('\nPosições REAIS no simulador [mm] (mundo):\n');
        for k = 1:numel(TRUTH_NAMES)
            fprintf('  %-13s x = %7.1f   y = %7.1f   z = %6.1f\n', TRUTH_NAMES{k}, 1000*T(k,:));
        end
        
        fprintf('\nDeteções (estimado vs real) [mm]:\n');
        fprintf('%-7s %-9s %-19s %-13s %-19s %-15s %s\n', 'cor', 'area', 'estimado x y', ...
                'objeto', 'real x y', 'dx dy', 'erro');
        
        used = false(numel(TRUTH_NAMES), 1);
        for i = 1:numel(det)
            d = det(i);
            dist = hypot(T(:,1) - d.world(1), T(:,2) - d.world(2));
            [dmin, j] = min(dist);
            used(j) = true;
            if dmin*1000 <= TOL_MM, verdict = 'OK'; else, verdict = 'FALHOU'; end
            fprintf('%-7s %-9d [%7.1f %7.1f]   %-13s [%7.1f %7.1f]   [%5.1f %5.1f]   %5.1f mm  %s\n', ...
                    d.color, d.area, 1000*d.world(1:2), TRUTH_NAMES{j}, 1000*T(j,1:2), ...
                    1000*(d.world(1:2) - T(j,1:2)), dmin*1000, verdict);
        end
        for j = find(~used)'
            fprintf('NÃO DETETADO: %s (real x = %.1f, y = %.1f mm)\n', TRUTH_NAMES{j}, 1000*T(j,1:2));
        end
        
        % diagnóstico dos sinais do modelo da câmara (só simulação)
        combos = [-1 -1; 1 -1; -1 1; 1 1];
        score = zeros(4, 1);
        for c = 1:4
            s = 0;
            for i = 1:numel(det)
                Pc = pix2world(det(i).pixel(1), det(i).pixel(2), W, H, f, Rc, pc, Z_PLANE, combos(c,:));
                if isempty(Pc), s = s + 1; continue; end
                s = s + min(hypot(T(:,1) - Pc(1), T(:,2) - Pc(2)));
            end
            score(c) = s / numel(det);
        end
        [~, best] = min(score);
        if ~isequal(combos(best,:), SIGN) && score(best) < 0.5*score(all(combos == SIGN, 2))
            fprintf('\nNOTA: com SIGN = [%d %d] os erros ficam muito menores (%.1f mm em vez de %.1f mm). Experimenta mudar SIGN.\n', ...
                    combos(best,:), 1000*score(best), 1000*score(all(combos == SIGN, 2)));
        end
    end
    
    %% ---- 5. visualização ----
    if SHOW
        figure('Name', 'vision_colors'); image(img); axis image; hold on;
        for i = 1:numel(det)
            plot(det(i).pixel(1) + 0.5, det(i).pixel(2) + 0.5, 'k+', 'MarkerSize', 12, 'LineWidth', 2);
            % Acrescentadas as dimensões convertidas para visualização
            text(det(i).pixel(1) + 8, det(i).pixel(2), ...
                 sprintf('%s [%.1fx%.1f cm]\n(%.0f, %.0f) mm', ...
                 det(i).color, det(i).width_cm, det(i).height_cm, 1000*det(i).world(1), 1000*det(i).world(2)), ...
                 'Color', 'w', 'BackgroundColor', 'k', 'FontSize', 8);
        end
        title('Peças localizadas por cor'); drawnow;
    end
end

% =========================================================================
function P = pix2world(u, v, W, H, f, Rc, pc, zPlane, sgn)
% Pixel (u,v) -> ponto do mundo em z = zPlane.
% Câmara do CoppeliaSim: olha segundo +z do sensor, +y do sensor aponta para o
% topo da imagem e +x do sensor aponta para a ESQUERDA da imagem (sgn = [-1 -1]).
    ray = [sgn(1)*(u - W/2)/f; sgn(2)*(v - H/2)/f; 1];
    d   = Rc * ray;                          % direção do raio no mundo
    if abs(d(3)) < 1e-9, P = []; return; end
    t = (zPlane - pc(3)) / d(3);
    if t <= 0, P = []; return; end           % o plano está atrás da câmara
    P = pc + t*d;
end

function P = pix2world_many(u, v, W, H, f, Rc, pc, zPlane, sgn)
% [ACRESCENTADO] como pix2world, mas para muitos pixeis de uma vez (u, v em coluna).
% P : 3 x N pontos do mundo em z = zPlane
    u = u(:)';  v = v(:)';
    ray = [sgn(1)*(u - W/2)/f; sgn(2)*(v - H/2)/f; ones(1, numel(u))];
    d   = Rc * ray;                          % direções dos raios no mundo
    t   = (zPlane - pc(3)) ./ d(3,:);
    P   = pc + t .* d;
end

function yaw = min_rect_yaw(x, y)
% [ACRESCENTADO] ângulo [rad] do retângulo de área mínima que contém os pontos
% (x, y). Basta procurar entre 0 e 90 graus (um quadrado repete-se a cada
% 90 graus). Resultado entre -45 e 45 graus.
    x = x - mean(x);  y = y - mean(y);
    ang = deg2rad(0:0.5:89.5);
    area = zeros(size(ang));
    for i = 1:numel(ang)
        xr =  cos(ang(i))*x + sin(ang(i))*y;
        yr = -sin(ang(i))*x + cos(ang(i))*y;
        area(i) = (max(xr) - min(xr)) * (max(yr) - min(yr));
    end
    [~, i] = min(area);
    yaw = mod(ang(i) + pi/4, pi/2) - pi/4;
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