function ok = wiggle(robot_arm, sim_interface, duration, amplitude, frequency)
% WIGGLE  Abana a garra: roda J6 para um lado e para o outro durante algum
%         tempo e volta ao ângulo inicial (bloqueante).
%
%   ok = wiggle(robot_arm, sim_interface)
%   ok = wiggle(robot_arm, sim_interface, duration, amplitude, frequency)
%
%   duration  : duração [s] de tempo de simulação (por defeito 1.5)
%   amplitude : rotação máxima de J6 para cada lado [rad] (por defeito 10 graus)
%   frequency : oscilações por segundo [Hz] (por defeito 2)
%   ok        : false se algum comando saiu dos limites das juntas
%
% Só J6 se mexe (rotação da garra em torno do seu próprio eixo), por isso o
% centro de preensão fica no mesmo sítio. A amplitude cresce e diminui de
% forma suave no início e no fim, para não dar solavancos.
% Ex.: wiggle(robot_arm, sim_interface, 2, deg2rad(15), 1.5);

    if nargin < 3 || isempty(duration),  duration  = 1.5;          end
    if nargin < 4 || isempty(amplitude), amplitude = deg2rad(10);  end
    if nargin < 5 || isempty(frequency), frequency = 2;            end

    q0 = robot_arm.get_joints();             % pose atual (as outras juntas ficam fixas)
    t0 = sim_interface.get_simulation_time();
    ok = true;

    while true
        t = sim_interface.get_simulation_time() - t0;
        if t >= duration
            break;
        end
        env = sin(pi * t / duration);        % envolvente: 0 -> 1 -> 0 (suave)
        q = q0;
        q(6) = q0(6) + amplitude * env * sin(2*pi*frequency*t);
        if robot_arm.set_joints(q) == 1
            ok = false;                      % fora dos limites: parar
            break;
        end
        sim_interface.step_simulation();
    end

    % voltar exatamente ao ângulo inicial de J6
    robot_arm.set_joints(q0);
    sim_interface.step_simulation();
end