function draw_zones(sim_interface, basePos, baseYaw, tableZ, j1Lim, Links)
% DRAW_ZONES  Desenha na mesa as zonas de alcance do myCobot 280 (só visual).
%
%   draw_zones(sim_interface, basePos, baseYaw, tableZ, j1Lim, Links)
%
%   basePos : [x y z] da base do robô no mundo (BASE_POSITION)
%   baseYaw : rotação da base em torno de z [rad] (BASE_YAW)
%   tableZ  : altura da mesa [m] (TABLE_Z)
%   j1Lim   : limite de J1 [rad] (MaxPositionJoint(1), 165 graus)
%   Links   : comprimentos dos elos (Links(5) = desvio lateral do braço)
%
% Desenha (centrado no eixo de J1):
%   - vermelho cheio : zona junto à base onde o braço não chega sem chocar
%                      com a placa da base. NÃO é um círculo: o alcance mínimo
%                      vai de 8,5 cm (em frente aos lados curtos da placa) a
%                      13 cm (nas diagonais, por causa dos cantos)
%   - círculo azul   : 23 cm, distância de referência (fora da cunha
%                      vermelha o braço chega a tudo entre a zona
%                      vermelha cheia e 35 cm)
%   - círculo verde  : alcance máximo (35 cm; a partir de 30 cm a garra
%                      já vai inclinada para fora)
%   - cunha vermelha : zona morta de J1 (+-165 graus): aí o braço (cotovelo
%                      para cima) não chega
% Os raios vêm do mapa de alcance calculado com plan_vertical para blocos
% pousados na mesa (LIFT = 0,10 m, FINGER_EXTRA = 0,015 m).
%
% São "drawing objects" do CoppeliaSim: não têm física e a garra e os
% sensores não os veem. Como são criados pela API remota, NÃO desaparecem
% quando a simulação para; por isso esta função apaga primeiro os desenhos
% da corrida anterior. Chamar depois de sim_interface.start_simulation().

    R_MIN  = 0.085;   R_SAFE = 0.23;   R_MAX = 0.35;     % raios [m]

    % alcance mínimo em cada direção (de 5 em 5 graus, no referencial da
    % BASE, a partir de +x), calculado com plan_vertical + arm_collision
    % para blocos pousados na mesa (LIFT = 0,10 m, FINGER_EXTRA = 0,015 m).
    % Se mudar o modelo de colisão ou estes valores, é preciso recalcular.
    R_MIN_DIR = [
        0.085 0.085 0.085 0.088 0.093 0.095 0.103 0.108 0.115 0.128 0.130 0.128   %   0.. 55 graus
        0.120 0.115 0.110 0.108 0.105 0.105 0.105 0.105 0.105 0.108 0.110 0.115   %  60..115 graus
        0.123 0.130 0.130 0.118 0.110 0.103 0.098 0.093 0.090 0.088 0.085 0.085   % 120..175 graus
        0.085 0.085 0.085 0.088 0.093 0.095 0.103 0.108 0.115 0.130 0.130 0.130   % 180..235 graus
        0.130 0.130 0.110 0.108 0.105 0.105 0.105 0.105 0.105 0.108 0.110 0.115   % 240..295 graus
        0.123 0.130 0.130 0.118 0.110 0.103 0.098 0.093 0.090 0.088 0.085 0.085   % 300..355 graus
    ];
    R_MIN_DIR = reshape(R_MIN_DIR.', 1, []);           % linha com os 72 valores, por ordem
    sim = sim_interface.get_connection().require('sim');

    % apagar os desenhos de corridas anteriores (senão acumulam-se uns em
    % cima dos outros e cintilam)
    try
        sim.removeDrawingObject(sim.handle_all);
    catch
    end
    cx = basePos(1);  cy = basePos(2);
    z1 = tableZ + 0.003;       % camadas acima da mesa, separadas umas das
    z2 = tableZ + 0.005;       % outras e do chão (para não cintilarem)
    z3 = tableZ + 0.004;

    trisType = sim.drawing_triangles;
    try
        trisType = bitor(trisType, sim.drawing_50percenttransparency);   % semitransparente
    catch
    end

    %% 1. zona junto à base onde o braço não chega (forma real, vermelho cheio)
    ang = deg2rad(0:5:355) + baseYaw;                   % ângulos no mundo
    px  = cx + R_MIN_DIR .* cos(ang);
    py  = cy + R_MIN_DIR .* sin(ang);
    n   = numel(ang);
    h = sim.addDrawingObject(trisType, 1, 0, -1, n + 2, [1 0 0]);
    for i = 1:n
        j = mod(i, n) + 1;
        sim.addDrawingObjectItem(h, [cx cy z1, px(i) py(i) z1, px(j) py(j) z1]);
    end
    h = sim.addDrawingObject(sim.drawing_lines, 3, 0, -1, n + 2, [0.7 0 0]);
    for i = 1:n
        j = mod(i, n) + 1;
        sim.addDrawingObjectItem(h, [px(i) py(i) z2, px(j) py(j) z2]);
    end

    %% 2. círculos de alcance (garantido e máximo)
    radii  = [R_SAFE R_MAX];
    colors = {[0 0.4 1], [0 0.8 0]};
    a = linspace(0, 2*pi, 73);
    for k = 1:numel(radii)
        h = sim.addDrawingObject(sim.drawing_lines, 3, 0, -1, 100, colors{k});
        for i = 1:72
            sim.addDrawingObjectItem(h, [cx + radii(k)*cos(a(i))   cy + radii(k)*sin(a(i))   z2, ...
                                         cx + radii(k)*cos(a(i+1)) cy + radii(k)*sin(a(i+1)) z2]);
        end
    end

    %% 3. zona morta de J1 (+-165 graus)
    % O limite de J1 é no ângulo do PLANO do braço, que está desviado
    % Links(5) do eixo: um objeto a distância r fica a psi = atan2(h, off)
    % do plano, com h = sqrt(r^2 - off^2). Por isso a cunha curva: longe da
    % base fica centrada atrás do robô (180 +- 15 graus), perto da base roda
    % para o lado (180 +- 15 + psi).
    off = Links(5);
    % pose cotovelo para cima: toda a cunha é zona morta (vermelho)
    wedge(sim, trisType, cx, cy, baseYaw, j1Lim, off, +1, R_MIN:0.01:R_MAX, z3, [0.9 0.1 0.1], [0.5 0 0]);
end

function wedge(sim, trisType, cx, cy, baseYaw, j1Lim, off, side, rs, z, fillColor, lineColor)
% desenha uma cunha da zona morta de J1 entre os raios rs(1) e rs(end)
%   side = +1 pose normal, -1 pose dobrada; trisType = [] -> só contorno
    w = zeros(numel(rs), 2, 2);                  % pontos das duas fronteiras (mundo)
    for i = 1:numel(rs)
        psi = atan2(sqrt(rs(i)^2 - off^2), off);
        for s = 1:2
            aBase = pi + (2*s - 3)*(pi - j1Lim) + side*psi;  % J1 = +-165 graus -> ângulo do objeto (base)
            aW    = aBase + baseYaw;                         % ângulo no mundo
            w(i, s, :) = [cx + rs(i)*cos(aW), cy + rs(i)*sin(aW)];
        end
    end
    if ~isempty(trisType)
        h = sim.addDrawingObject(trisType, 1, 0, -1, 2*numel(rs), fillColor);
        for i = 1:numel(rs) - 1
            p1 = squeeze(w(i,1,:))';    p2 = squeeze(w(i,2,:))';
            p3 = squeeze(w(i+1,1,:))';  p4 = squeeze(w(i+1,2,:))';
            sim.addDrawingObjectItem(h, [p1 z p2 z p3 z]);
            sim.addDrawingObjectItem(h, [p2 z p4 z p3 z]);
        end
    end
    h = sim.addDrawingObject(sim.drawing_lines, 3, 0, -1, 2*numel(rs) + 2, lineColor);
    for i = 1:numel(rs) - 1
        for s = 1:2
            sim.addDrawingObjectItem(h, [squeeze(w(i,s,:))' z squeeze(w(i+1,s,:))' z]);
        end
    end
    % fechar a cunha nas duas pontas
    sim.addDrawingObjectItem(h, [squeeze(w(1,1,:))' z squeeze(w(1,2,:))' z]);
    sim.addDrawingObjectItem(h, [squeeze(w(end,1,:))' z squeeze(w(end,2,:))' z]);
end