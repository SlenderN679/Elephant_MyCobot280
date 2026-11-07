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
% para +y. Ângulos positivos no sentido anti-horário.
%
% Como nas aulas (abordagem geométrica, sem matrizes):
%   1. plano lateral: J2, J3 e J4 movem o braço num plano vertical; soma-se,
%      elo a elo, L*cos e L*sin do ângulo absoluto de cada elo -> (d, h)
%   2. plano horizontal: J1 roda esse plano; projeta-se o desvio lateral l e a
%      distância d em frente nos eixos x e y -> (x, y, z)
% J6 roda a garra em torno do seu próprio eixo e não muda a posição.

    % Passo 1: garantir 6 juntas (se vierem só 4, J5 e J6 ficam a 0)
    q = q(:)';
    if numel(q) == 4
        q = [q 0 0];    % bloquear J5 e J6
    end
    L = Links;
    D = L(6) + L(7);    % J5 -> centro de preensão (ao longo da garra)

    % Passo 2: ângulos absolutos de cada elo, a partir da horizontal.
    % Na pose zero o braço está vertical, por isso o 1.º elo faz q2 + 90 graus
    thetaA = q(2) + pi/2;           % elo L2 (J2 -> J3)
    thetaB = thetaA + q(3);         % elo L3 (J3 -> J4)
    thetaC = thetaB + q(4);         % elo L4 (J4 -> J5)
    alpha  = thetaC - pi/2;         % garra: perpendicular a L4 (-90 graus = para baixo)

    % Passo 3: plano lateral (d = em frente, h = para cima), junta a junta
    % J2 (ombro)
    d = 0;
    h = L(1);
    % J3 (cotovelo)
    d = d + L(2)*cos(thetaA);
    h = h + L(2)*sin(thetaA);
    % J4 (pulso)
    d = d + L(3)*cos(thetaB);
    h = h + L(3)*sin(thetaB);
    % J5
    d = d + L(4)*cos(thetaC);
    h = h + L(4)*sin(thetaC);
    % centro de preensão: J5 roda a garra para fora do plano (cos(q5) fica no
    % plano, sin(q5) passa a desvio lateral)
    d = d + D*cos(q(5))*cos(alpha);
    h = h + D*cos(q(5))*sin(alpha);
    l = L(5) - D*sin(q(5));         % desvio lateral total do plano do braço

    % Passo 4: plano horizontal. O desvio l faz o ângulo q1 com o eixo x e a
    % distância d em frente faz q1 + 90 graus
    x = l*cos(q(1)) + d*cos(q(1) + pi/2);      % = l*cos(q1) - d*sin(q1)
    y = l*sin(q(1)) + d*sin(q(1) + pi/2);      % = l*sin(q1) + d*cos(q1)
    z = h;
    tip = [x; y; z];

    % Passo 5: direção da garra (a mesma decomposição, sem os comprimentos)
    aL = -sin(q(5));                % componente para o lado
    aD =  cos(q(5))*cos(alpha);     % componente em frente
    aH =  cos(q(5))*sin(alpha);     % componente para cima
    approach = [aL*cos(q(1)) + aD*cos(q(1) + pi/2);
                aL*sin(q(1)) + aD*sin(q(1) + pi/2);
                aH];
end