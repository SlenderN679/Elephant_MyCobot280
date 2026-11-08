function ok = move_joints(robot_arm, sim_interface, qTarget, jointSpeed, timeout)
% MOVE_JOINTS  Movimento suave no espaço das juntas do myCobot 280 (bloqueante).
%
%   ok = move_joints(robot_arm, sim_interface, qTarget)
%   ok = move_joints(robot_arm, sim_interface, qTarget, jointSpeed, timeout)
%
%   qTarget    : 1x6 juntas [rad]
%   jointSpeed : velocidade da junta que mais se move [rad/s] (por defeito 30 graus/s)
%   timeout    : [s] de tempo de simulação (por defeito 20)
%   ok         : true se todas as juntas terminaram a menos de 1 grau de qTarget
%
% Todas as juntas arrancam e param ao mesmo tempo, com um perfil suave:
% velocidade nula no início e no fim, sem solavancos. Usar para
% movimentos grandes (ex.: home -> por cima do objeto); para movimentos
% curtos perto dos objetos usar trajetórias cartesianas em linha reta
% (inverse_kin a cada passo).

    if nargin < 4 || isempty(jointSpeed)
        jointSpeed = deg2rad(30); 
    end
    if nargin < 5 || isempty(timeout)  
        timeout = 20; 
    end
    TOL = deg2rad(1);

    % Passo 1: diferença entre a pose atual e a pretendida e duração do movimento
    % (a junta que mais se move anda a jointSpeed)
    q0 = robot_arm.get_joints();
    dq = qTarget - q0;                      % as juntas têm limites: sem dar a volta
    T  = 1.5 * max(abs(dq)) / jointSpeed;   % 1.5: no perfil em S a velocidade máxima = 1.5x a média
    tS = sim_interface.get_simulation_time();
    ok = false;
    % Passo 2: a cada passo de simulação, enviar a pose intermédia q0 + s*dq,
    % com s a ir de 0 a 1 segundo um perfil em S (começa e acaba devagar)
    while true
        t = sim_interface.get_simulation_time();
        s = 1;
        if T > 0, s = min(1, (t - tS)/T); end
        s = s*s*(3 - 2*s);                  % arranque e paragem suaves
        robot_arm.set_joints(q0 + s*dq);
        sim_interface.step_simulation();

        % Passo 3: terminar quando o perfil acabou e as juntas chegaram (ou timeout)
        q = robot_arm.get_joints();
        if s >= 1 && max(abs(q - qTarget)) < TOL
            ok = true;
            break;
        end
        if t - tS > T + timeout
            break;
        end
    end
end