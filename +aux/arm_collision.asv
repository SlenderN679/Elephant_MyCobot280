function [hit, what] = arm_collision(q, Links)
% ARM_COLLISION  Verifica se uma pose do braço choca consigo próprio
%                (coluna da base, placa da base ou braço superior).
%
%   [hit, what] = arm_collision(q, Links)
%
%   q    : [J1 .. J6] (ou [J1 .. J4], com J5 = J6 = 0) [rad]
%   hit  : true se houver colisão
%   what : texto com a peça que choca (ex.: 'placa (garra)')
%
% Modelo simples (medidas lidas das malhas de mycobot280_vision.ttt): cada
% peça é um segmento com um raio (cilindro com pontas redondas):
%   antebraço J3->J4 (r 2,5 cm), pulso J4->J5->J6 (r 2,2 cm),
%   garra J6->dedos (r 3 cm) e dedos até ao centro de preensão (r 2 cm).
% Obstáculos (referencial da base):
%   - coluna da base: cilindro vertical de raio 4,8 cm até z = 0,13 m
%   - placa da base: 11 x 15 cm (x: +-5,5 cm, y: +-7,5 cm no referencial da
%     base), até z = 0,002 m (3,2 cm acima da mesa)
%   - braço superior J2->J3 (r 2,6 cm), só contra o pulso e a garra
% Cada ponto tem ainda MARGIN de folga. É uma aproximação: serve para
% rejeitar poses claramente más, não substitui ver a simulação.

    COL_R     = 0.048;   COL_TOP   = 0.13;     % coluna da base [m]
    PLATE_HX  = 0.055;   PLATE_HY  = 0.075;    % meias larguras da placa (x, y da base) [m]
    PLATE_TOP = 0.002;                         % topo da placa [m]
    UPPER_R   = 0.026;                         % raio do braço superior [m]
    MARGIN    = 0.008;                         % folga [m]

    L = Links;  q = q(:)';
    if numel(q) == 4, q = [q 0 0]; end

    % posições das juntas (mesmas transformações que direct_kin)
    T = rotz(q(1)) * trans(0,0,L(1));                 J2 = T(1:3,4);
    T = T * rotx(q(2)) * trans(0,0,L(2));             J3 = T(1:3,4);
    T = T * rotx(q(3)) * trans(L(5),0,L(3));          J4 = T(1:3,4);
    T = T * rotx(q(4)) * trans(0,0,L(4));             J5 = T(1:3,4);
    T = T * rotz(q(5)) * trans(0,L(6),0);             J6 = T(1:3,4);
    T = T * roty(q(6)) * trans(0,L(7),0);             G  = T(1:3,4);
    F = J6 + 0.75*(G - J6);                           % início dos dedos

    parts = { J3, J4, 0.025, 'antebraço';
              J4, J5, 0.022, 'pulso';
              J5, J6, 0.022, 'pulso';
              J6, F,  0.030, 'garra';
              F,  G,  0.020, 'dedos' };

    hit = false;  what = '';
    for k = 1:size(parts, 1)
        [a, b, r, nm] = parts{k, :};
        for t = linspace(0, 1, 8)
            p = a + t*(b - a);
            % coluna da base (os dedos chegam ao objeto, não à coluna)
            if k < 5 && hypot(p(1), p(2)) < COL_R + r + MARGIN && p(3) < COL_TOP + r
                hit = true;  what = ['coluna da base (' nm ')'];  return;
            end
            % placa da base
            if abs(p(1)) < PLATE_HX + r + MARGIN && abs(p(2)) < PLATE_HY + r + MARGIN && p(3) < PLATE_TOP + r
                hit = true;  what = ['placa da base (' nm ')'];  return;
            end
            % braço superior (só pulso a seguir a J5 e garra)
            if k >= 3 && k <= 4 && seg_dist(p, J2, J3) < UPPER_R + r + MARGIN
                hit = true;  what = ['braço superior (' nm ')'];  return;
            end
        end
    end
end

function d = seg_dist(p, a, b)
% distância do ponto p ao segmento a<->b
    ab = b - a;
    t  = max(0, min(1, dot(p - a, ab) / dot(ab, ab)));
    d  = norm(p - (a + t*ab));
end

function M = trans(x,y,z) % move em xyz
    M = eye(4);  M(1:3,4) = [x;y;z];
end
function M = rotx(t)      % roda em x
    M = [1 0 0 0; 
        0 cos(t) -sin(t) 0; 
        0 sin(t) cos(t) 0; 
        0 0 0 1];
end
function M = roty(t)      % roda em y
    M = [cos(t) 0 sin(t) 0; 
        0 1 0 0; 
        -sin(t) 0 cos(t) 0; 
        0 0 0 1];
end
function M = rotz(t)      % roda em z
    M = [cos(t) -sin(t) 0 0; 
        sin(t) cos(t) 0 0;
        0 0 1 0; 
        0 0 0 1];
end