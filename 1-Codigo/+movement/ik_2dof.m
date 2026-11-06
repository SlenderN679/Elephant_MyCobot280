function [theta1, theta2, isReachable] = ik_2dof(xd, yd, LA, LB, solution_type)
% IK_2DOF  Cinemática inversa de um braço planar de 2 elos (abordagem
%          geométrica das aulas: "Basic of kinematics - 2 DOF planar arm")
%
%   [theta1, theta2, isReachable] = ik_2dof(xd, yd, LA, LB, solution_type)
%
%   (xd, yd)      : ponto desejado, com origem no ombro (1.ª junta) [m]
%   LA, LB        : comprimentos dos dois elos [m]
%   solution_type : 'up'   - cotovelo para cima  (theta2 <= 0)
%                   'down' - cotovelo para baixo (theta2 >= 0)
%   theta1        : ângulo do 1.º elo, medido a partir do eixo x [rad]
%   theta2        : ângulo do 2.º elo em relação ao 1.º [rad]
%   isReachable   : false se o ponto estiver fora do alcance (arg fora de [-1, 1])
%
% Triângulo ombro - cotovelo - ponto, com r = distância do ombro ao ponto:
%   alpha1 = atan2(yd, xd)                              direção do ponto
%   beta1  = acos((LA^2 + r^2 - LB^2) / (2*LA*r))       ângulo no ombro
%   beta2  = acos((LA^2 + LB^2 - r^2) / (2*LA*LB))      ângulo no cotovelo
%   cotovelo para baixo: theta1 = alpha1 - beta1,  theta2 =   pi - beta2
%   cotovelo para cima:  theta1 = alpha1 + beta1,  theta2 = -(pi - beta2)

    theta1 = NaN;  theta2 = NaN;  isReachable = false;

    % Passo 1: distância e direção do ponto a partir do ombro
    r      = sqrt(xd^2 + yd^2);
    alpha1 = atan2(yd, xd);

    % Passo 2: lei dos cossenos (e^2 = a^2 + b^2 - 2ab cos(beta)); só existe
    % solução se os argumentos do acos estiverem em [-1, 1]
    arg1 = (LA^2 + r^2 - LB^2) / (2*LA*r);
    arg2 = (LA^2 + LB^2 - r^2) / (2*LA*LB);
    if r == 0 || abs(arg1) > 1 || abs(arg2) > 1
        return;                         % ponto fora do alcance dos dois elos
    end
    beta1 = acos(arg1);
    beta2 = acos(arg2);

    % Passo 3: escolher uma das duas soluções
    if strcmp(solution_type, 'down')
        theta1 = alpha1 - beta1;        % solução 1: cotovelo para baixo
        theta2 = pi - beta2;
    else
        theta1 = alpha1 + beta1;        % solução 2: cotovelo para cima
        theta2 = -(pi - beta2);
    end
    isReachable = true;
end