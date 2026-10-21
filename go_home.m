function ok = go_home(robot_arm, sim_interface, Links, qMin, qMax, jointOffset, qHome)
% GO_HOME  Leva o myCobot 280 à pose inicial, seja qual for a pose de partida. 
%          Usa APENAS as leituras das juntas, 
%          por isso funciona igual com o braço simulado e com o real.
%
%   ok = go_home(robot_arm, sim_interface, Links, qMin, qMax)
%   ok = go_home(robot_arm, sim_interface, Links, qMin, qMax, jointOffset)
%   ok = go_home(robot_arm, sim_interface, Links, qMin, qMax, jointOffset, qHome)
%
%   jointOffset : (opcional) 1x6 [rad] calibração: leitura das juntas com o
%                 braço realmente na pos inicial. Os encoders do myCobot real vêm
%                 calibrados de fábrica -> deixar a 0 se não tivermos medido outro valor.
%   qHome       : (opcional) 1x6 pose inicial EM RADIANOS (por defeito todas as
%                 juntas a 0 = braço vertical). Ex.: deg2rad([0 0 90 0 0 0])
%   ok          : true se todas as juntas terminaram a menos de 1 grau da pose inicial
%
%   1. Calcula onde está a garra a partir das leituras das juntas (direct_kin).
%      Se estiver baixa, primeiro sobe-a na vertical, mantendo
%      a inclinação, para não arrastar objetos nem bater na mesa.
%   2. Movimento suave das juntas (todas arrancam e param juntas) até à pose inicial.
%   3. Verifica as leituras das juntas: termina quando estão todas a menos de 1 grau.

    if nargin < 6 || isempty(jointOffset), jointOffset = zeros(1,6); end
    if nargin < 7 || isempty(qHome),       qHome = zeros(1,6);       end   % [rad]
    SAFE_Z      = 0.15;           % altura da garra considerada segura [m, referencial da base]
    LIFT_SPEED  = 0.03;           % [m/s]
    JOINT_SPEED = deg2rad(30);    % [rad/s] da junta que mais se move
    TOL         = deg2rad(1);

    robot_arm.open_hand();
    sim_interface.step_simulation();
    q = robot_arm.get_joints() - jointOffset;
    fprintf('go_home: juntas iniciais [graus] %s\n', sprintf('%7.1f', rad2deg(q)));

    %% 1. subir na vertical se a garra estiver baixa
    [tip, approach] = direct_kin(q, Links);
    if tip(3) < SAFE_Z
        pitch = asin(max(-1, min(1, -approach(3))));   % manter a inclinação atual
        p = tip;  qCmd = q;
        while p(3) < SAFE_Z
            p(3) = min(SAFE_Z, p(3) + LIFT_SPEED*0.05);
            [qi, reach] = inverse_kin(p(1), p(2), p(3), Links, qMin, qMax, [], pitch);
            if ~reach || max(abs(qi(1:4) - qCmd(1:4))) > deg2rad(10)
                break;                  % sem caminho suave para cima: passar ao passo 2
            end
            qCmd(1:4) = qi(1:4);        % J5/J6 ficam como estão
            robot_arm.set_joints(qCmd + jointOffset);
            sim_interface.step_simulation();
        end
    end

    %% 2. movimento suave das juntas até à pose inicial
    move_joints(robot_arm, sim_interface, qHome + jointOffset, JOINT_SPEED);

    %% 3. verificar as leituras
    q  = robot_arm.get_joints() - jointOffset;
    ok = all(abs(q - qHome) < TOL);
    if ok
        fprintf('go_home: pose inicial alcançada\n');
    else
        fprintf('go_home: pose inicial NÃO alcançada, juntas [graus] %s\n', sprintf('%7.1f', rad2deg(q)));
    end
    wiggle(robot_arm, sim_interface, 2, deg2rad(30), 2);
end