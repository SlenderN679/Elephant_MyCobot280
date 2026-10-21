function q = random_start(robot_arm, sim_interface, Links, qMin, qMax, tableZbase)
% RANDOM_START  AUXILIAR DE TESTE: leva o braço a uma pose aleatória, para
%               testar o go_home a partir de outra pose que não a guardada
%               no cenário (o CoppeliaSim arranca sempre o braço nessa pose).
%
%   q = random_start(robot_arm, sim_interface, Links, qMin, qMax, tableZbase)
%
%   tableZbase : altura da mesa no referencial da base do robô [m]
%                (TABLE_Z - BASE_POSITION(3) no main_program)
%   q          : a pose aleatória alcançada [rad]
%
% Escolhe juntas aleatórias (J1..J6) dentro dos limites, aceitando só poses
% em que a garra, o pulso e o cotovelo ficam acima da mesa, e vai para lá
% com um movimento suave das juntas. Não usar com o braço real.

    MARGIN = 0.03;                  % manter tudo pelo menos 3 cm acima da mesa
    for tries = 1:1000
        q = qMin + rand(1,6).*(qMax - qMin);
        q(2:4) = 0.6*q(2:4);         % evitar dobras extremas (colisões com o próprio braço)
        tip   = direct_kin(q, Links);
        zEl   = Links(1) + Links(2)*cos(q(2));
        zJ4   = zEl + Links(3)*cos(q(2)+q(3));
        zJ5   = zJ4 + Links(4)*cos(q(2)+q(3)+q(4));
        if min([tip(3) zEl zJ4 zJ5]) > tableZbase + MARGIN
            break;
        end
    end
    fprintf('random_start: a ir para [graus] %s\n', sprintf('%7.1f', rad2deg(q)));
    move_joints(robot_arm, sim_interface, q, deg2rad(45));
    % deixar estabilizar um momento
    for k = 1:10
        sim_interface.step_simulation();
    end
end