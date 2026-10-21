function [armJoints, isReachable, pitchUsed] = inverse_kin(x, y, z, Links, qMin, qMax, objYaw, prefPitch, family, fixedPitch)
% INVERSE_KIN  Cinemática inversa analítica do myCobot 280
%
%   [armJoints, isReachable, pitchUsed] = inverse_kin(x, y, z, Links)
%   [armJoints, isReachable, pitchUsed] = inverse_kin(x, y, z, Links, qMin, qMax)
%   [armJoints, isReachable, pitchUsed] = inverse_kin(x, y, z, Links, qMin, qMax, objYaw)
%   [armJoints, isReachable, pitchUsed] = inverse_kin(x, y, z, Links, qMin, qMax, objYaw, prefPitch)
%   [armJoints, isReachable, pitchUsed] = inverse_kin(x, y, z, Links, qMin, qMax, objYaw, prefPitch, family, fixedPitch)
%
%   (x,y,z)   : posição desejada do centro de preensão, referencial da BASE [m]
%               (converter posições do mundo com world_to_base)
%   Links     : conexões entre juntas (ver direct_kin.m)
%   qMin/qMax : limites das juntas [rad] (6 valores, de get_RobotCharacteristics)
%   objYaw    : (opcional) rotação de um cubo em torno de z [rad]. Se for dada,
%               J6 roda a garra para os dedos fecharem em duas faces opostas.
%               [] ou omitido -> J6 = 0
%   prefPitch : (opcional) inclinação preferida da garra [rad], + = a apontar
%               para baixo. Por defeito pi/2 (vertical para baixo, o melhor
%               para apanhar objetos). Acima de pi/2 a garra aponta para baixo
%               e um pouco para a base.
%
%   family    : (opcional) família de soluções:
%               'auto'  (por defeito) - igual a 'front'
%               'front' - só "à frente" com o cotovelo para cima (J3 <= 0)
%               'back'  - só "atrás" com o cotovelo invertido (J3 >= 0): braço
%                         inclinado para trás e antebraço a dobrar por cima
%   fixedPitch: (opcional) true -> usar SÓ a inclinação prefPitch (sem procurar
%               outras). Usado por plan_vertical / move_linear para que uma
%               descida inteira use a mesma solução, sem saltos.
%
%   armJoints   = [J1 J2 J3 J4 J5 J6] em rad, pronto para robot_arm.set_joints
%   isReachable = true se existir solução dentro dos limites das juntas
%   pitchUsed   = inclinação da garra realmente usada [rad]
%
% Como funciona (J5 = 0)
%   Com J5 = 0 a garra fica no plano vertical do braço, por isso o braço é
%   uma cadeia planar de 3 elos (J2, J3, J4) mais a rotação da base J1:
%   1. J1: o plano do braço está desviado lateralmente Links(5) do eixo de
%      J1, por isso J1 = atan2(y,x) - atan2(h, Links(5)), h = distância em frente.
%   2. No plano do braço, para uma inclinação escolhida: ponto do pulso (J4) =
%      alvo menos o elo J4->J5 e o desvio J5->centro de preensão.
%   3. J2, J3 pela lei dos cossenos (braço de 2 elos), J4 = inclinação - J2 - J3.
%   As inclinações são testadas a partir da preferida (vertical), afastando
%   cada vez mais para os dois lados: inclinada para a base (> 90 graus, boa
%   para objetos perto da base) ou para fora (< 90 graus, objetos longe).
%   Para cada uma escolhe-se a solução "cotovelo para cima" e rejeitam-se as
%   soluções com o cotovelo dobrado mais de ELBOW_MAX (quase encostado ao
%   braço, onde a gravidade impede de chegar ao alvo) ou com o cotovelo ou o
%   pulso abaixo da base. As soluções "à frente" têm sempre prioridade sobre
%   as soluções "atrás" (braço inclinado para trás por cima da base).

    if nargin < 5 || isempty(qMin)
        qMin = deg2rad([-165 -165 -165 -165 -165 -175]); % valores de defeito para limites min
    end
    if nargin < 6 || isempty(qMax)
        qMax = deg2rad([ 165  165  165  165  165  175]); % valores de defeito para limites max
    end
    if nargin < 7
        objYaw = [];                                     % valor de defeito para a rotação do objeto
    end
    if nargin < 8 || isempty(prefPitch) 
        prefPitch = pi/2;                                % valor de defeito para a inclinação do objeto
    end
    if nargin < 9 || isempty(family)
        family = 'auto';                                 % família de soluções por defeito
    end
    if nargin < 10 || isempty(fixedPitch)
        fixedPitch = false;                              % por defeito procura a inclinação
    end
    
    % inicializar valores para os cálculos
    L  = Links;
    D  = L(6) + L(7);                  % eixo de J5 -> centro de preensão (ao longo da garra)
    armJoints   = zeros(1, 6);
    isReachable = false;
    pitchUsed   = NaN;

    r = hypot(x, y);
    if r <= L(5)
        return;                         % dentro do desvio lateral: inalcançável
    end
    hFwd = sqrt(r^2 - L(5)^2);          % distância em frente, no plano do braço
    v    = z - L(1);                    % altura relativa a J2

    ELBOW_MAX = deg2rad(140);           % dobra máxima do cotovelo (|J3|), com folga até ao limite de 165
    PITCH_MAX = deg2rad(135);           % garra no máximo 45 graus inclinada para a base
    PITCH_MIN = deg2rad(30);            % garra sempre pelo menos 30 graus abaixo da horizontal
                                        % (a apontar para cima não serve para apanhar nada)
    STEP      = deg2rad(5);

    % passagens: {lados, família do cotovelo (0 = qualquer, 1 = J3<=0, -1 = J3>=0)}
    switch family
        case 'front', passes = {{1, 1}};
        case 'back',  passes = {{-1, -1}};
        otherwise,    passes = {{1, 1}};               % por defeito: só "à frente", cotovelo para cima
    end
    if fixedPitch
        dList = 0;                       % só a inclinação pedida
    else
        dList = 0:STEP:(PITCH_MAX - PITCH_MIN);
    end
    for ip = 1:numel(passes)
        sides = passes{ip}(1);  fam = passes{ip}{2};
        for d = dList
            % candidatos com desvio d da inclinação preferida: primeiro para a
            % base (+d), depois para fora (-d)
            if d == 0
                cands = prefPitch;
            else
                cands = [prefPitch + d, prefPitch - d];
            end
            for psi = cands
                if ~fixedPitch && (psi > PITCH_MAX || psi < PITCH_MIN)
                    continue;
                end
                best = solve_pitch(psi, x, y, hFwd, v, L, D, qMin, qMax, ELBOW_MAX, sides{1}, fam);
                if isempty(best)
                    continue;
                end

                % Os candidatos vão de 5 em 5 graus. Procura-se por bisseção a
                % inclinação mais próxima da preferida que ainda funciona, para a
                % garra inclinar de forma contínua em vez de saltar 5 graus.
                if d > 0
                    good = psi;  bad = psi - sign(psi - prefPitch)*STEP;
                    for it = 1:15
                        mid = (good + bad)/2;
                        qm  = solve_pitch(mid, x, y, hFwd, v, L, D, qMin, qMax, ELBOW_MAX, sides{1}, fam);
                        if isempty(qm), bad = mid; else, good = mid; best = qm; end
                    end
                    psi = good;
                end

                q6 = 0;
                if ~isempty(objYaw) && psi > deg2rad(45)
                    % os dedos fecham segundo a direção J1 - J6 (módulo 180 graus); um
                    % cubo é simétrico a cada 90 graus -> escolhe-se a menor rotação de J6
                    q6 = mod(best(1) - objYaw + pi/4, pi/2) - pi/4;
                end
                armJoints   = [best 0 q6];
                isReachable = true;
                pitchUsed   = psi;
                return;
            end
        end
    end
end

function best = solve_pitch(psi, x, y, hFwd, v, L, D, qMin, qMax, elbowMax, sides, fam)
% melhor solução "cotovelo para cima" [J1..J4] para a inclinação psi, ou [] se não houver
    wrap = @(t) atan2(sin(t), cos(t));
    best = [];  bestElbow = -inf;
    % sides = 1 -> só a solução "à frente"; [1 -1] -> também a solução "atrás"
    % (J1 rodado ~180 graus, braço inclinado para trás)
    for side = sides
        h = side*hFwd;
        if side == 1    % ângulo da garra no plano
            phi = -psi; 
        else
            phi = pi + psi; 
        end   
        q1 = atan2(y, x) - atan2(h, L(5));

        % posição do pulso (J4) no plano do braço
        Wh = h + L(4)*sin(phi) - D*cos(phi);
        Wv = v - L(4)*cos(phi) - D*sin(phi);
        c  = (Wh^2 + Wv^2 - L(2)^2 - L(3)^2) / (2*L(2)*L(3));
        if abs(c) > 1
            continue;               % fora de alcance para esta inclinação
        end
        for g = [acos(c), -acos(c)]
            a1 = atan2(Wv, Wh) - atan2(L(3)*sin(g), L(2) + L(3)*cos(g)); % ângulo do braço superior
            q  = wrap([q1, a1 - pi/2, g, phi - (a1 - pi/2) - g]);
            if any(q < qMin(1:4)) || any(q > qMax(1:4)) || abs(q(3)) > elbowMax
                continue;           % fora dos limites, ou cotovelo dobrado demais
            end
            if (fam == 1 && q(3) > 0) || (fam == -1 && q(3) < 0)
                continue;           % cotovelo do lado errado para a família pedida
            end
            % alturas do cotovelo (J3) e do pulso (J4, J5): têm de ficar acima da base
            zElbow = L(1) + L(2)*cos(q(2));
            zJ4    = zElbow + L(3)*cos(q(2)+q(3));
            zJ5    = zJ4 + L(4)*cos(q(2)+q(3)+q(4));
            if min([zElbow zJ4 zJ5]) <= 0
                continue;
            end
            if zElbow > bestElbow   % preferir "cotovelo para cima"
                best = q;  bestElbow = zElbow;
            end
        end
    end
end