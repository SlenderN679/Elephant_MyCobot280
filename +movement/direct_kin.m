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

    q = q(:)';
    if numel(q) == 4
        q = [q 0 0];    % bloquear J5 e J6
    end
    L = Links;

    T = rotz(q(1)) * trans(0,0,L(1)) ...
      * rotx(q(2)) * trans(0,0,L(2)) ...
      * rotx(q(3)) * trans(L(5),0,L(3)) ...
      * rotx(q(4)) * trans(0,0,L(4)) ...
      * rotz(q(5)) * trans(0,L(6),0) ...
      * roty(q(6)) * trans(0,L(7),0);

    tip      = T(1:3,4);
    approach = T(1:3,2);
end

function M = trans(x,y,z)
    M = eye(4);  
    M(1:3,4) = [x;y;z];
end
function M = rotx(t)
    M = [1 0 0 0; 0 cos(t) -sin(t) 0; 0 sin(t) cos(t) 0; 0 0 0 1];
end
function M = roty(t)
    M = [cos(t) 0 sin(t) 0; 0 1 0 0; -sin(t) 0 cos(t) 0; 0 0 0 1];
end
function M = rotz(t)
    M = [cos(t) -sin(t) 0 0; sin(t) cos(t) 0 0; 0 0 1 0; 0 0 0 1];
end