function ok = pick_and_place(robot_arm, sim_interface, cfg, objWorld, placeWorld, objYaw)
% PICK_AND_PLACE  Apanha um objeto e pousa-o noutro sítio (bloqueante).
%
%   ok = pick_and_place(robot_arm, sim_interface, cfg, objWorld, placeWorld)
%   ok = pick_and_place(robot_arm, sim_interface, cfg, objWorld, placeWorld, objYaw)
%
%   cfg        : configuração (ver main_program)
%   objWorld   : [x y z] centro do objeto, referencial do MUNDO [m]
%                (sim_interface.get_target_position, ou o sistema de visão)
%   placeWorld : [x y z] ponto de pouso (superfície do alvo), referencial do MUNDO [m]
%   objYaw     : (opcional) orientação do cubo no MUNDO [rad] para alinhar os
%                dedos com as faces ([] = cilindro / desconhecida)
%   ok         : true se todas as fases correram bem
%
% Fases:
%   1. por cima do objeto (movimento suave das juntas)
%   2. descer na vertical          3. fechar a garra
%   4. subir na vertical           5. por cima do destino (movimento das juntas)
%   6. descer na vertical (até cfg.DROP_HEIGHT acima do pouso, se cfg.DROP_ON_PLACE)
%   7. abrir a garra
%   8. subir na vertical

    if nargin < 6, objYaw = []; end
    ok = false;

    % posições no referencial da base do robô
    pObj  = world_to_base(objWorld,   cfg.BASE_POSITION, cfg.BASE_YAW);
    pTgt  = world_to_base(placeWorld, cfg.BASE_POSITION, cfg.BASE_YAW);
    yaw = [];
    if ~isempty(objYaw)
        yaw = objYaw - cfg.BASE_YAW;           % orientação do cubo no referencial da base
    end
    up = [0; 0; cfg.LIFT];

    % altura do centro do objeto acima da mesa (metade da altura do objeto);
    % ao pousar, o centro fica essa altura acima da superfície do destino
    objHalf = objWorld(3) - cfg.TABLE_Z;
    pPlace  = [pTgt(1:2); pTgt(3) + objHalf + 0.005];

    % largar o objeto um pouco acima do ponto de pouso, em vez de o pousar
    % e empurrar (útil ao empilhar: a garra não força os blocos uns contra
    % os outros). Se o campo não existir em cfg, pousa normalmente.
    if isfield(cfg, 'DROP_ON_PLACE') && cfg.DROP_ON_PLACE
        pPlace(3) = pPlace(3) + cfg.DROP_HEIGHT;
    end

    fprintf('pick_and_place: objeto em [%.3f %.3f %.3f] (base)\n', pObj);

    % 1. por cima do objeto: movimento suave no espaço das juntas
    above = pObj + up;
    [qA, reach] = inverse_kin(above(1), above(2), above(3), cfg.Links, cfg.qMin, cfg.qMax, yaw);
    if ~reach
        fprintf('pick_and_place: objeto inalcançável\n');  return;
    end
    move_joints(robot_arm, sim_interface, qA);
    robot_arm.open_hand();
    if ~move_linear(robot_arm, sim_interface, cfg, above, cfg.V_SLOW, yaw), return; end

    % 2-4. descer, fechar, subir
    if ~move_linear(robot_arm, sim_interface, cfg, pObj, cfg.V_SLOW, yaw), return; end
    robot_arm.close_hand();
    wait_time(sim_interface, cfg.T_CLOSE);
    if ~move_linear(robot_arm, sim_interface, cfg, pObj + up, cfg.V_SLOW, yaw), return; end

    % 5. ir para cima do destino: movimento suave das juntas (uma linha reta
    %    pode passar perto do eixo da base, onde a cinemática inversa muda de
    %    solução e as juntas dariam saltos), depois acerto em linha reta
    aboveT = pPlace + up;
    [qT, reach] = inverse_kin(aboveT(1), aboveT(2), aboveT(3), cfg.Links, cfg.qMin, cfg.qMax, yaw);
    if ~reach
        fprintf('pick_and_place: destino inalcançável\n');  return;
    end
    move_joints(robot_arm, sim_interface, qT);
    if ~move_linear(robot_arm, sim_interface, cfg, aboveT, cfg.V_SLOW, yaw), return; end

    % 6-8. descer, abrir, subir
    if ~move_linear(robot_arm, sim_interface, cfg, pPlace, cfg.V_SLOW, yaw), return; end
    robot_arm.open_hand();
    wait_time(sim_interface, cfg.T_OPEN);
    if ~move_linear(robot_arm, sim_interface, cfg, pPlace + up, cfg.V_SLOW, yaw), return; end

    ok = true;
    fprintf('pick_and_place: concluído\n');
end

function wait_time(sim_interface, T)
% espera T segundos de simulação (dá tempo à garra para abrir/fechar)
    t0 = sim_interface.get_simulation_time();
    while sim_interface.get_simulation_time() - t0 < T
        sim_interface.step_simulation();
    end
end