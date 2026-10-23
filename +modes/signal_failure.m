function signal_failure(robot_arm, sim_interface, Links, qMin, qMax, FAIL_POSE)
% SIGNAL_FAILURE  Assinalar uma falha: subir a garra se estiver baixa, ir
%                 para a pose de erro e abanar a garra durante 3 s.
%
%   aux.signal_failure(robot_arm, sim_interface, Links, qMin, qMax, FAIL_POSE)
%
%   FAIL_POSE : pose de erro [GRAUS] (J1..J6)
    movement.go_home(SHAKE_HEAD, robot_arm, sim_interface, Links, qMin, qMax, [], deg2rad(FAIL_POSE));
    aux.wiggle(robot_arm, sim_interface, 3, deg2rad(30), 1);
    disp('>>> ROBÔ PARADO: tarefa falhou (ver mensagem acima) <<<');
end