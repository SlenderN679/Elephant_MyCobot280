function pBase = world_to_base(pWorld, armPosition, baseYaw)
% WORLD_TO_BASE  Converte uma posição do MUNDO para o referencial da BASE do myCobot
%
%   pBase = world_to_base(pWorld, armPosition)
%   pBase = world_to_base(pWorld, armPosition, baseYaw)
%
%   pWorld      : [x y z] no mundo (ex.: sim_interface.get_target_position)
%   armPosition : [x y z] da base do robô (robot_arm.get_robot_position)
%   baseYaw     : rotação da base em torno de z [rad]. No cenário
%                 mycobot280_vision.ttt o robô está rodado -90 graus (por defeito).
%
%   inverse_kin / direct_kin trabalham no referencial da base, por isso NÃO
%   basta fazer pWorld - armPosition: os eixos também estão rodados.

    if nargin < 3
        baseYaw = -pi/2;
    end
    d = pWorld(:) - armPosition(:);
    c = cos(baseYaw);  s = sin(baseYaw);
    pBase = [ c*d(1) + s*d(2);
             -s*d(1) + c*d(2);
              d(3)];
end