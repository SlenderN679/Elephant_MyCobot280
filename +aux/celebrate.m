function celebrate(robot_arm, sim_interface, Links, qMin, qMax)
% CELEBRATE  Pequena celebração do myCobot 280 (bloqueante, ~6 s).
%
%   celebrate(robot_arm, sim_interface, Links, qMin, qMax)
%
% Põe o braço na vertical e, durante 6 s:
%   - a base roda devagar 20 graus para cada lado e volta (uma vez)
%   - a garra acena para a frente e para trás (J4 +-20 graus, 3 vezes)
%     e abre/fecha a cada acenar
% Termina com o braço na vertical e a garra aberta.

    T = 6;                                            % duração [s]
    movement.go_home(robot_arm, sim_interface, Links, qMin, qMax, [], zeros(1,6));   % braço na vertical
    disp('celebrate: \o/');

    t0 = sim_interface.get_simulation_time();
    lastHand = -1;
    while true
        t = sim_interface.get_simulation_time() - t0;
        if t > T, break; end
        q = deg2rad([20*sin(2*pi*t/T), 0, 0, 20*sin(2*pi*t/2), 0, 0]);
        robot_arm.set_joints(min(max(q, qMin), qMax));
        % garra: fechada na primeira metade de cada aceno, aberta na segunda
        hand = mod(floor(t), 2);
        if hand ~= lastHand
            if hand == 0, robot_arm.close_hand(); else, robot_arm.open_hand(); end
            lastHand = hand;
        end
        sim_interface.step_simulation();
    end

    robot_arm.open_hand();
    robot_arm.set_joints(zeros(1,6));                 % acabar exatamente na vertical
    sim_interface.step_simulation();
end