function [tip, approach, T] = direct_kin(q, Links)
% DIRECT_KIN  Cinemática direta do myCobot 280 (cenário mycobot280_vision.ttt)
%
%   [tip, approach, T] = direct_kin(q, Links)
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
%   T        : 4x4 transformação homogénea base -> garra
%
% Referencial da base: origem no eixo de J1 (na base do robô), z para cima.
% Pose zero (todas as juntas a 0): braço vertical, garra horizontal a apontar
% para +y. Eixos das juntas na pose zero: J1 +z, J2/J3/J4 +x, J5 +z, J6 +y.

    % Passo 1: garantir 6 juntas (se vierem só 4, J5 e J6 ficam a 0)
    q = q(:)';
    if numel(q) == 4
        q = [q 0 0];    % bloquear J5 e J6
    end
    L = Links;

    % Passo 2: percorrer o braço da base até à garra. Cada par "rodar a junta,
    % avançar ao longo do elo" é uma matriz 4x4; multiplicá-las por ordem dá a
    % pose da garra no referencial da base:
    %   J1 roda em z, sobe L1 | J2 roda em x, sobe L2 | J3 roda em x, desvio L5 e
    %   sobe L3 | J4 roda em x, sobe L4 | J5 roda em z, avança L6 | J6 roda em y,
    %   avança L7 até ao centro de preensão
    T = aux.matrix.rotz(q(1)) * aux.matrix.trans(0,0,L(1)) ...
      * aux.matrix.rotx(q(2)) * aux.matrix.trans(0,0,L(2)) ...
      * aux.matrix.rotx(q(3)) * aux.matrix.trans(L(5),0,L(3)) ...
      * aux.matrix.rotx(q(4)) * aux.matrix.trans(0,0,L(4)) ...
      * aux.matrix.rotz(q(5)) * aux.matrix.trans(0,L(6),0) ...
      * aux.matrix.roty(q(6)) * aux.matrix.trans(0,L(7),0);

    % Passo 3: a última coluna de T é a posição; a 2.ª coluna é o eixo y da garra,
    % que é para onde a garra aponta
    tip      = T(1:3,4);
    approach = T(1:3,2);
end
