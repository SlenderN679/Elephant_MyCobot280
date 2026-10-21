% diag_kinematics.m
% Verifica direct_kin / inverse_kin contra o simulador (só simulação).
% Abrir mycobot280_vision.ttt (simulação parada) e correr este script.
% Leva a garra (a apontar para baixo) 6 cm acima de cada objeto e escreve:
%   - juntas pedidas vs. juntas alcançadas
%   - onde o direct_kin diz que está o centro de preensão
%   - onde o CoppeliaSim diz que está realmente o /graspCenter
function diag_kinematics()
    [sim_interface, err] = simulator_interface(0);
    if err, return; end
    cleanup = onCleanup(@() sim_interface.terminate());
    [robot_arm, err] = mycobot_interface(sim_interface, '/mycobot280', 0);
    if err, return; end
    sim = sim_interface.get_connection().require('sim');
    hGrasp = sim.getObject('/graspCenter');
    hBase  = sim.getObject('/mycobot280');

    [~, ~, qMin, qMax] = robot_arm.get_RobotCharacteristics();
    Links = [0.1316 0.1104 0.0960 0.0731 0.0646 0.0456 0.1057];

    sim_interface.start_simulation();
    names = {'BlueCube','BlueCylinder','RedCube','TargetGreen'};
    for k = 1:4
        armPos = robot_arm.get_robot_position();
        p = world_to_base(sim_interface.get_target_position(k), armPos) + [0;0;0.06];
        [q, ok] = inverse_kin(p(1), p(2), p(3), Links, qMin, qMax);
        if ~ok, fprintf('%s: inalcançável\n', names{k}); continue; end
        robot_arm.set_joints(q);
        for s = 1:80                        % ~4 s de simulação
            sim_interface.step_simulation();
        end
        qr  = robot_arm.get_joints();
        dk  = direct_kin(qr, Links);         % modelo, com as juntas realmente alcançadas
        gW  = sim.getObjectPosition(hGrasp, -1);  gW = [gW{:}];
        gB  = sim.getObjectPosition(hGrasp, hBase); gB = [gB{:}];
        fprintf('\n== %s ==\n', names{k});
        fprintf(' objetivo (base)          : %7.4f %7.4f %7.4f\n', p);
        fprintf(' q pedido     [graus]     : %s\n', sprintf('%7.1f', rad2deg(q)));
        fprintf(' q alcançado  [graus]     : %s\n', sprintf('%7.1f', rad2deg(qr)));
        fprintf(' direct_kin (base)        : %7.4f %7.4f %7.4f\n', dk);
        fprintf(' graspCenter (base)       : %7.4f %7.4f %7.4f   <- do simulador\n', gB);
        fprintf(' graspCenter (mundo)      : %7.4f %7.4f %7.4f\n', gW);
        fprintf(' world_to_base(gW)        : %s\n', sprintf('%8.4f', world_to_base(gW, armPos)));
    end
end