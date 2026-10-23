function ok = move_linear(robot_arm, sim_interface, cfg, goal, V, yaw, maxTime, ikOpts)
% MOVE_LINEAR  Leva o centro de preensão da garra em LINHA RETA até goal (bloqueante).
%
%   ok = move_linear(robot_arm, sim_interface, cfg, goal, V)
%   ok = move_linear(robot_arm, sim_interface, cfg, goal, V, yaw)
%   ok = move_linear(robot_arm, sim_interface, cfg, goal, V, yaw, maxTime)
%   ok = move_linear(robot_arm, sim_interface, cfg, goal, V, yaw, maxTime, ikOpts)
%
%   cfg     : configuração (ver main_program: Links, qMin, qMax, ACC, TABLE_Z,
%             BASE_POSITION, FINGER_EXTRA, CLEARANCE)
%   goal    : 3x1 ponto de destino no referencial da BASE [m]
%   V       : velocidade máxima [m/s]
%   yaw     : (opcional) orientação do cubo para alinhar J6 ([] = J6 a 0)
%   maxTime : (opcional) tempo máximo de simulação [s] (por defeito 15)
%   ikOpts  : (opcional) família e inclinação fixa da garra (de plan_vertical);
%             sem ele, inverse_kin escolhe a solução em cada ponto
%   ok      : true se a garra terminou a menos de 3 mm do objetivo
%
% Em cada passo de simulação:
%   1. o ponto de referência cmd avança na direção do objetivo, acelerando
%      e travando com cfg.ACC (para parar exatamente no objetivo);
%   2. calcula-se inverse_kin para cmd e envia-se para o braço;
%   3. ao chegar, compara-se a posição real da garra (direct_kin das juntas
%      lidas) com o objetivo e corrige-se o erro (juntas a descair por
%      causa da gravidade).

    if nargin < 6, yaw = []; end
    if nargin < 7 || isempty(maxTime), maxTime = 15; end
    if nargin < 8, ikOpts = []; end
    TOL     = 0.003;          % erro de posição aceite [m]
    SETTLE  = 4;              % tempo máximo à espera do erro < TOL depois de chegar [s]

    % proteção da mesa: as pontas dos dedos nunca abaixo da mesa + folga
    floorBase = cfg.TABLE_Z - cfg.BASE_POSITION(3);
    goal = goal(:);
    goal(3) = max(goal(3), floorBase + cfg.FINGER_EXTRA + cfg.CLEARANCE);

    q    = robot_arm.get_joints();
    cmd  = movement.direct_kin(q, cfg.Links);          % começa onde a garra está
    corr = zeros(3,1);                        % correção da gravidade
    vCur = 0;                                 % velocidade atual [m/s]
    t0   = sim_interface.get_simulation_time();
    tPrev = t0;  tReached = -1;
    ok = false;

    while true
        t  = sim_interface.get_simulation_time();
        dt = max(t - tPrev, 1e-3);  tPrev = t;
        q   = robot_arm.get_joints();
        tip = movement.direct_kin(q, cfg.Links);       % onde a garra está realmente

        % avançar o ponto de referência (perfil de velocidade suave)
        d = goal - cmd;  n = norm(d);
        vCur = min([V, vCur + cfg.ACC*dt, sqrt(2*cfg.ACC*n)]);
        if n > max(vCur*dt, 1e-4)
            cmd = cmd + d/n*vCur*dt;
            reached = false;
        else
            cmd = goal;  reached = true;  vCur = 0;
        end

        % correção da gravidade (só depois de chegar; ganho pequeno, sem solavancos)
        if reached
            corr = corr + 0.15*(goal - tip);
            if norm(corr) > 0.03, corr = corr/norm(corr)*0.03; end
        end

        p = cmd + corr;
        if isempty(ikOpts)
            [qi, reach] = movement.inverse_kin(p(1), p(2), p(3), cfg.Links, cfg.qMin, cfg.qMax, yaw);
        else
            [qi, reach] = movement.inverse_kin(p(1), p(2), p(3), cfg.Links, cfg.qMin, cfg.qMax, yaw, ...
                                      ikOpts.pitch, ikOpts.family, true);
        end
        if reach
            if robot_arm.set_joints(qi) == 1
                return;                       % fora dos limites: desistir
            end
        end
        sim_interface.step_simulation();

        % terminado?
        if reached && tReached < 0, tReached = t; end
        if reached && norm(goal - tip) < TOL
            ok = true;  return;
        end
        if (reached && t - tReached > SETTLE) || t - t0 > maxTime
            fprintf('move_linear: objetivo não alcançado (erro %.1f mm)\n', 1000*norm(goal - tip));
            return;
        end
    end
end