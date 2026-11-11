% --------------------------------------------------------------------------
% Resolve J1..J4 para UMA inclinação psi da garra (braço planar das aulas)
function q = solve_pitch(psi, x, y, dFwd, hJ2, L, D, qMin, qMax, elbowMax, family)
% solução [J1..J4] para a inclinação psi, ou [] se não houver
    wrap = @(t) atan2(sin(t), cos(t));
    q = [];

    % Passo 1: J1 aponta o plano do braço para o alvo (theta1 = atan2(y,x)),
    % descontando o desvio lateral L5 e a frente do braço ser +y com J1 = 0
    q1 = atan2(y, x) - atan2(dFwd, L(5));

    % Passo 2: ângulo do pulso imposto. alpha3 = ângulo da garra no plano do
    % braço (a partir da horizontal; -90 graus = vertical para baixo). O ponto
    % do pulso (J4) obtém-se recuando ao longo da garra (D) e do elo J4->J5 (L4),
    % que é perpendicular à garra:  x3d = xd - Lc*cos(alpha3), ...
    alpha3 = -psi;
    x3d = dFwd - D*cos(alpha3) - L(4)*cos(alpha3 + pi/2);
    y3d = hJ2  - D*sin(alpha3) - L(4)*sin(alpha3 + pi/2);

    % Passo 3: braço planar de 2 elos (J2 -> J3 -> pulso) até (x3d, y3d)
    [theta1, theta2, ok] = movement.ik_2dof(x3d, y3d, L(2), L(3), family);
    if ~ok
        return;                     % fora de alcance para esta inclinação
    end
    % theta3: o elo J4->J5 faz alpha3 + 90 graus com a horizontal
    theta3 = (alpha3 + pi/2) - theta1 - theta2;

    % Passo 4: passar para as juntas do robô. Na pose zero o braço está
    % vertical (o 1.º elo faz 90 graus com a horizontal), por isso J2 = theta1 - 90
    q = wrap([q1, theta1 - pi/2, theta2, theta3]);

    % Passo 5: validar: limites das juntas, dobra do cotovelo, e o cotovelo
    % (J3) e o pulso (J4, J5) acima da base
    zElbow = L(1) + L(2)*cos(q(2));
    zJ4    = zElbow + L(3)*cos(q(2)+q(3));
    zJ5    = zJ4 + L(4)*cos(q(2)+q(3)+q(4));
    if any(q < qMin(1:4)) || any(q > qMax(1:4)) || abs(q(3)) > elbowMax ...
            || min([zElbow zJ4 zJ5]) <= 0
        q = [];
    end
end