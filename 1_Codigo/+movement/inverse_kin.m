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
%   family    : (opcional) solução do braço de 2 elos (solution_type das aulas):
%               'up'   (por defeito) - cotovelo para cima  (J3 <= 0)
%               'down' - cotovelo para baixo (J3 >= 0)
%   fixedPitch: (opcional) true -> usar SÓ a inclinação prefPitch (sem procurar
%               outras). Usado por plan_vertical / move_linear para que uma
%               descida inteira use a mesma solução, sem saltos.
%
%   armJoints   = [J1 J2 J3 J4 J5 J6] em rad, pronto para robot_arm.set_joints
%   isReachable = true se existir solução dentro dos limites das juntas
%   pitchUsed   = inclinação da garra realmente usada [rad]
%
% Como funciona (J5 = 0), com a lógica das aulas (braço planar de 3 elos
% com o ângulo do pulso imposto + rotação da base):
%   1. plano horizontal - J1 (base): theta1 = atan2(y, x). O plano do braço
%      está desviado L5 do eixo de J1 e com J1 = 0 o braço aponta para +y, por
%      isso J1 = atan2(y,x) - atan2(d, L5), d = distância em frente no plano.
%   2. inclinação da garra: com fixedPitch usa-se prefPitch; senão, a função
%      solve_pitch escolhe a inclinação mais próxima da preferida que tenha
%      solução (3 incógnitas, 2 equações no plano -> fixa-se a inclinação).
%   3. plano lateral - impõe-se o ângulo da garra alpha3: o ponto do pulso (J4)
%      = alvo menos o elo J4->J5 e o desvio J5->centro de preensão
%      (x3d = xd - Lc*cos(alpha3), ...). J2, J3: braço planar de 2 elos até ao
%      pulso (ik_2dof: alpha1, beta1, beta2, cotovelo para cima).
%      J4 = alpha3 - theta1 - theta2.
%   4. rejeitam-se as soluções fora dos limites das juntas, com o cotovelo
%      dobrado mais de ELBOW_MAX (quase encostado ao braço, onde a gravidade
%      impede de chegar ao alvo) ou com o cotovelo ou o pulso abaixo da base.
%   5. J6 alinha os dedos com as faces do cubo.

    % Passo 0: valores por defeito dos argumentos opcionais
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
        family = 'up';                                   % por defeito: cotovelo para cima
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
    wrap = @(t) atan2(sin(t), cos(t));
    ELBOW_MAX = deg2rad(140);           % dobra máxima do cotovelo (|J3|), com folga até ao limite de 165

    if ~strcmp(family, 'down')
        family = 'up';                   % qualquer outro valor: cotovelo para cima
    end

    % Passo 1: plano horizontal. Distância horizontal ao eixo de J1; o braço
    % está desviado L5 para o lado, por isso a distância "em frente" no plano
    % do braço é dFwd. J1 aponta o plano do braço para o alvo
    r = hypot(x, y);
    if r <= L(5)
        return;                         % dentro do desvio lateral: inalcançável
    end
    dFwd = sqrt(r^2 - L(5)^2);          % d: distância em frente, no plano do braço
    hJ2  = z - L(1);                    % h: altura relativa a J2 (ombro)
    q1   = atan2(y, x) - atan2(dFwd, L(5));

    % Passo 2: inclinação da garra. Com fixedPitch usa-se só a pedida; senão,
    % solve_pitch procura a inclinação mais próxima da preferida com solução
    if fixedPitch
        psi = prefPitch;
    else
        [psi, ok] = movement.solve_pitch(x, y, z, Links, qMin, qMax, prefPitch, family);
        if ~ok
            return;                     % nenhuma inclinação tem solução
        end
    end

    % Passo 3: plano lateral, para a inclinação psi. alpha3 = ângulo da garra
    % no plano do braço (a partir da horizontal; -90 graus = vertical para
    % baixo). O ponto do pulso (J4) obtém-se recuando ao longo da garra (D) e
    % do elo J4->J5 (L4), que é perpendicular à garra
    alpha3 = -psi;
    x3d = dFwd - D*cos(alpha3) - L(4)*cos(alpha3 + pi/2);
    y3d = hJ2  - D*sin(alpha3) - L(4)*sin(alpha3 + pi/2);

    % braço planar de 2 elos (J2 -> J3 -> pulso) até (x3d, y3d)
    [theta1, theta2, ok] = movement.ik_2dof(x3d, y3d, L(2), L(3), family);
    if ~ok
        return;                         % fora de alcance para esta inclinação
    end
    % theta3: o elo J4->J5 faz alpha3 + 90 graus com a horizontal
    theta3 = (alpha3 + pi/2) - theta1 - theta2;

    % passar para as juntas do robô. Na pose zero o braço está vertical (o 1.º
    % elo faz 90 graus com a horizontal), por isso J2 = theta1 - 90
    q = wrap([q1, theta1 - pi/2, theta2, theta3]);

    % Passo 4: validar: limites das juntas, dobra do cotovelo, e o cotovelo
    % (J3) e o pulso (J4, J5) acima da base
    zElbow = L(1) + L(2)*cos(q(2));
    zJ4    = zElbow + L(3)*cos(q(2)+q(3));
    zJ5    = zJ4 + L(4)*cos(q(2)+q(3)+q(4));
    if any(q < qMin(1:4)) || any(q > qMax(1:4)) || abs(q(3)) > ELBOW_MAX ...
            || min([zElbow zJ4 zJ5]) <= 0
        return;
    end

    % Passo 5: J6 alinha os dedos com as faces do cubo (só se a garra estiver
    % quase vertical); J5 fica sempre a 0
    q6 = 0;
    if ~isempty(objYaw) && psi > deg2rad(45)
        % os dedos fecham segundo a direção J1 - J6 (módulo 180 graus); um
        % cubo é simétrico a cada 90 graus -> escolhe-se a menor rotação de J6
        q6 = mod(q(1) - objYaw + pi/4, pi/2) - pi/4;
    end
    armJoints   = [q 0 q6];
    isReachable = true;
    pitchUsed   = psi;
end