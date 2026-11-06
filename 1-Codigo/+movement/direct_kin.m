function [tip, approach] = direct_kin(q, Links)
% DIRECT_KIN  Cinemática direta do myCobot 280 (cenário mycobot280_vision.ttt)
%
%   [tip, approach] = direct_kin(q, Links)
%
%   q        : [J1 J2 J3 J4] (J5 = J6 = 0)  ou  [J1 ... J6]   [rad]
%   Links    : Links(1) base -> J2 (altura)            0.1316
%              Links(2) J2 -> J3                       0.1104
%              Links(3) J3 -> J4                       0.0960
%              Links(4) J4 -> J5                       0.0731
%              Links(5) desvio lateral J3 -> J4        0.0646
%              Links(6) J5 -> J6                       0.0456
%              Links(7) J6 -> centro de preensão       0.1057
%              (valores medidos no cenário; os valores do datasheet
%               devolvidos por get_RobotCharacteristics() diferem alguns mm)
%
%   tip      : 3x1 posição do centro de preensão da garra, referencial da BASE [m]
%   approach : 3x1 vetor unitário, direção para onde a garra aponta
%
% Referencial da base: origem no eixo de J1 (na base do robô), z para cima.
% Pose zero (todas as juntas a 0): braço vertical, garra horizontal a apontar
% para +y. Eixos das juntas na pose zero: J1 +z, J2/J3/J4 +x, J5 +z, J6 +y.
%
% Como nas aulas (braço planar de 3 elos + rotação da base): J2, J3 e J4 movem
% o braço num plano vertical; soma-se, elo a elo, Lk*cos e Lk*sin do ângulo
% acumulado. Depois J1 roda esse plano em torno de z. J6 roda a garra em
% torno do seu próprio eixo e não muda a posição do centro de preensão.

    % Passo 1: garantir 6 juntas (se vierem só 4, J5 e J6 ficam a 0)
    q = q(:)';
    if numel(q) == 4
        q = [q 0 0];    % bloquear J5 e J6
    end
    L = Links;
    D = L(6) + L(7);    % eixo de J5 -> centro de preensão (ao longo da garra)

    % Passo 2: braço planar no plano do braço (h = em frente, v = para cima).
    % Ângulos medidos a partir da horizontal; na pose zero o braço está
    % vertical, por isso o 1.º elo faz q2 + 90 graus com a horizontal
    t1 = q(2) + pi/2;               % elo J2 -> J3
    t2 = t1 + q(3);                 % elo J3 -> J4
    t3 = t2 + q(4);                 % elo J4 -> J5
    tg = t3 - pi/2;                 % garra: perpendicular ao elo J4 -> J5
    % cotovelo (J3)
    h = L(2)*cos(t1);
    v = L(1) + L(2)*sin(t1);
    % pulso (J4)
    h = h + L(3)*cos(t2);
    v = v + L(3)*sin(t2);
    % J5
    h = h + L(4)*cos(t3);
    v = v + L(4)*sin(t3);
    % centro de preensão: J5 roda a garra para fora do plano (cos(q5) no plano,
    % -sin(q5) para o lado)
    h   = h + D*cos(q(5))*cos(tg);
    v   = v + D*cos(q(5))*sin(tg);
    lat = L(5) - D*sin(q(5));       % desvio lateral do plano do braço

    % Passo 3: J1 roda o plano do braço em torno de z. Com J1 = 0 a frente do
    % braço é +y e o lado (desvio L5) é +x
    fwd  = [-sin(q(1)); cos(q(1)); 0];      % direção "em frente"
    side = [ cos(q(1)); sin(q(1)); 0];      % direção "para o lado"
    tip  = lat*side + h*fwd + [0; 0; v];

    % Passo 4: direção da garra (mesma decomposição, sem os comprimentos)
    approach = -sin(q(5))*side + cos(q(5))*cos(tg)*fwd + [0; 0; cos(q(5))*sin(tg)];
end