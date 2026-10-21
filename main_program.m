%   main_program.m
%   Main program for the control of manipulators
%   Author: Luís Louro, luislouro@algoritmi.uminho.pt
%           Estela Bicho, estela.bicho@dei.uminho.pt
%   % Copyright (C) 2025
%   2025/02/05
%--------------------------------------------------------------------------
% Before running this script, open the scenario in CoppeliaSim, e.g
% Do not run simulation!
%--------------------------------------------------------------------------


function main()
    clear
    % need to choose the arm to control
    robot_name = '/mycobot280';
    ROSWORKING = 0;
    VISIONWORKING = 0;
    
    %Number of targets for each colour of box
    BLUE_CUBE = 1;
    BLUE_CYLINDER = 2;
    RED_CYLINDER = 3;
    TARGET_GREEN = 4;
    BLOCK1 = 5;
    BLOCK2 = 6;
    BLOCK3 = 7;
    BLOCK_H = 0.0192;   % block height without studs [m]
%% ======================================================================
% Configuração do exemplo pick-and-place
%======================================================================
    DEMO_PICK_AND_PLACE = 1;                % 1 - executa as tarefas de TASKS (pick-and-place)
                                            
    % Lista de tarefas: cada linha = [objeto, destino, desvio_x, desvio_y]
    %   objeto  : BLUE_CUBE, BLUE_CYLINDER ou RED_CYLINDER (é o /RedCube)
    %   destino : índice do alvo onde pousar (ex.: TARGET_GREEN)
    %   desvio  : deslocamento do ponto de pouso em relação ao centro do
    %             destino [m] (para pousar vários objetos lado a lado)
    TASKS = [ BLOCK1, TARGET_GREEN, 0, 0, 1;    % level 1 (on the target)
              BLOCK2, TARGET_GREEN, 0, 0, 2;    % on top of BLOCK1
              BLOCK3, TARGET_GREEN, 0, 0, 3 ];
    % exemplo com os três objetos lado a lado junto ao alvo verde (desvios
    % para longe da base do robô; perto da base o braço fica muito dobrado):
    % TASKS = [ BLUE_CUBE,     TARGET_GREEN,  0.00, 0.00;
    %           BLUE_CYLINDER, TARGET_GREEN,  0.00, 0.06;
    %           RED_CYLINDER,  TARGET_GREEN,  0.06, 0.00 ];
    IS_CUBE = @(obj) obj ~= BLUE_CYLINDER;  % cubos: J6 alinha os dedos com as faces
    RANDOM_START = 1;                       % 1 - (teste, só simulação) começar numa pose aleatória do braço
    SHOW_ZONES = 1;                         % 1 - desenhar na mesa as zonas de alcance e a zona morta de J1
    CELEBRATE = 1;                          % 1 - dança de celebração quando todas as tarefas correm bem
    END_WHEN_DONE = 1;                      % 1 - parar a simulação quando todas as tarefas acabarem

    % Desimpedir o alvo antes das tarefas: os objetos que estejam dentro de
    % CLEAR_RADIUS do destino são levados para os pontos CLEAR_SPOTS
    CLEAR_TARGET = 1;                       % 1 - desimpedir o alvo no início
    CLEAR_RADIUS = 0.05;                    % raio da zona a desimpedir [m]
    PICKABLE = [BLUE_CUBE BLUE_CYLINDER RED_CYLINDER BLOCK1 BLOCK2 BLOCK3];  % objetos que se podem mover
    % metade da altura de cada objeto [m] (para os pousar à altura certa,
    % mesmo quando são tirados do topo de uma pilha)
    HALF_H = zeros(1, max(PICKABLE));
    HALF_H([BLUE_CUBE BLUE_CYLINDER RED_CYLINDER]) = 0.04/2;   % cubos e cilindro do cenário: 4 cm
    HALF_H([BLOCK1 BLOCK2 BLOCK3]) = BLOCK_H/2;                % blocos
    CLEAR_SPOTS = [ -0.22 -0.03;            % pontos de estacionamento [x y] no MUNDO [m]
                    -0.17 -0.10;            % (alcançáveis com a garra na vertical;
                    -0.10 -0.16;            %  verificar que estão livres no vosso cenário)
                    -0.03 -0.17 ];
    % Montagem - o que se sabe do espaço de trabalho. O código abaixo não usa
    % informação escondida do simulador, por isso serve também para o braço
    % real: o estado do robô vem só das leituras das juntas (encoders) e os
    % objetos de get_target_position (sistema de visão). 
    % Valores por defeito = mycobot280_vision.ttt;
    % !!!MEDIR na montagem real!!!
    BASE_POSITION = [-0.0497 0.0165 0.03]; % origem da base do robô [m]
    BASE_YAW      = -pi/2;                 % rotação da base do robô [rad]
    TABLE_Z       = 0;                     % altura da superfície da mesa [m]
    USE_OBJECT_YAW = (ROSWORKING == 0);    % orientação do cubo (alinhamento de J6): lida do simulador;
    HOME_POSE     = [0 0 -90 0 0 0];       % pose inicial [GRAUS] (J1..J6); [0 0 0 0 0 0] = braço vertical
    FAIL_POSE     = [0 0 0 90 0 0];        % pose de ERRO [GRAUS]: braço vertical com a garra a
                                           % apontar para cima; o robô vai para lá, abana a garra
                                           % e para as tarefas quando uma tarefa falha
%% ======================================================================
% /Configuração do exemplo pick-and-place/
%======================================================================

    %----------------------------------------------------------------------
    % Creation of a communication class object with the simulator
    % Try to connect to simulator
    % Input:
    % VISIONWORKING - enable commucation with vision system
    % Output:
    % sim_interface - pointer to class simulator_interface
    % error_sim = 1 - impossible to connect to simulator
    %----------------------------------------------------------------------
    [sim_interface,error_sim] = simulator_interface(VISIONWORKING);
    if(error_sim==1)
        return;
    end
    
    cleanup = onCleanup(@()myCleanupFun(sim_interface));
    
    %----------------------------------------------------------------------
    %Creation of a communication class object with the manipulator arm
    % Input:
    % sim - pointer to class simulator_interface
    % robot_name - name of arm CoppeliaSim model
    % ROSWORKING - enable commucation with real arm using ROS
    % Output:
    % robot_arm - pointer to class arm_interface
    % error_man = 1 - impossible to connect to simulator
    %----------------------------------------------------------------------
    [robot_arm,error_man] = mycobot_interface(sim_interface,robot_name,ROSWORKING);
    if error_man == 1
        sim_interface.terminate();
        return;
    end
    
    sim_interface.start_simulation(); % start simulation in CoppeliaSim

    % [ACRESCENTADO] desenhar as zonas de alcance na mesa (só visual)
    if SHOW_ZONES == 1
        draw_zones(sim_interface, BASE_POSITION, BASE_YAW, TABLE_Z, deg2rad(165), ...
                   [0.1316 0.1104 0.0960 0.0731 0.0646 0.0456 0.1057]);
    end
    
    %----------------------------------------------------------------------
    % Informations of arm characteristics
    % Output:
    % nJoints - number of arm joints.
    % Links - dimensions of the links between the axes of rotation
    % MinPositionJoint - array with minimum position for joint 1-6
    % MaxPositionJoint - array with maximum position for joint 1-6
    %----------------------------------------------------------------------
    [nJoints,Links,MinPositionJoint,MaxPositionJoint] = robot_arm.get_RobotCharacteristics();

%% ======================================================================
% Inicialização do exemplo pick-and-place
%======================================================================
    % Comprimentos dos elos medidos em mycobot280_vision.ttt
    % (os valores acima são os do datasheet; estes coincidem exatamente com
    % o robô simulado). Ver direct_kin.m para o significado de cada um.
    Links = [0.1316 0.1104 0.0960 0.0731 0.0646 0.0456 0.1057];

    % TESTE (só simulação): começar numa pose aleatória em vez
    % da guardada no cenário, para verificar que o go_home funciona de qualquer pose
    if RANDOM_START == 1 && ROSWORKING == 0
        random_start(robot_arm, sim_interface, Links, MinPositionJoint, MaxPositionJoint, ...
                     TABLE_Z - BASE_POSITION(3));
        pause(3);
    end

    % Levar o braço à pose inicial de forma suave,
    % seja qual for a pose de partida: primeiro sobe a garra se
    % estiver baixa, depois move todas as juntas em conjunto e espera que chegue.
    if ~go_home(robot_arm, sim_interface, Links, MinPositionJoint, MaxPositionJoint, [], deg2rad(HOME_POSE))
        disp('AVISO: pose inicial não alcançada');
    end
    %celebrate(robot_arm, sim_interface, Links, MinPositionJoint, MaxPositionJoint);
    
%% ======================================================================
% /Inicialização do exemplo pick-and-place/
%======================================================================
    %Initialize arm with all the joint with 0 degrees
    armJoints(1)=HOME_POSE(1)*pi/180; 
    armJoints(2)=HOME_POSE(2)*pi/180;    
    armJoints(3)=HOME_POSE(3)*pi/180;    
    armJoints(4)=HOME_POSE(4)*pi/180;     
    armJoints(5)=HOME_POSE(5)*pi/180;
    armJoints(6)=HOME_POSE(6)*pi/180;
    %------------------------------------------------------------------
    % Function to send values to arm joints
    % Input:
    % armJoints - array with joint angles (1..4)
    % Output:
    % error - true if joint value outside the limits of the joint
    %-----------------------------------------------------------------
    error = robot_arm.set_joints(armJoints) %send value for arm Joints in rad
    if error==1
        return;
    end

    %------------------------------------------------------------------
    % Functions that allows open the hand
    %------------------------------------------------------------------
    robot_arm.open_hand();      %send Open Hand

%% ======================================================================
% Configuração do movimento (estrutura cfg, usada por pick_and_place / move_linear)
%======================================================================
    cfg.Links = Links;  cfg.qMin = MinPositionJoint;  cfg.qMax = MaxPositionJoint;
    cfg.BASE_POSITION = BASE_POSITION;  cfg.BASE_YAW = BASE_YAW;  cfg.TABLE_Z = TABLE_Z;
    cfg.LIFT   = 0.1;        % elevação por cima dos objetos [m]
    cfg.V_FAST = 0.06;        % velocidade da garra entre poses [m/s]
    cfg.V_SLOW = 0.01;        % velocidade da garra a descer/subir junto aos objetos [m/s]
    cfg.ACC    = 0.10;        % aceleração/travagem [m/s^2] (movimentos suaves)
    cfg.FINGER_EXTRA = 0.015; % quanto as pontas dos dedos passam do centro de preensão [m]
    cfg.CLEARANCE    = 0.005; % folga entre as pontas dos dedos e a mesa [m]
    cfg.T_CLOSE = 1.5;        % tempo para a garra fechar [s]
    cfg.T_OPEN  = 1.0;        % tempo para a garra abrir [s]
    cfg.DROP_ON_PLACE = 1;    % 1 - larga o objeto um pouco acima do destino (empilhar sem forçar)
                              % 0 - pousa o objeto e só depois abre a garra
    cfg.DROP_HEIGHT   = 0.015;% altura da queda quando DROP_ON_PLACE = 1 [m]
    cfg.CLEAR_SPACING = 0.05; % distância mínima livre à volta de um ponto de estacionamento [m]
    cfg.CHECK_COLLISIONS = 1; % 1 - rejeitar poses em que o braço choca consigo próprio (arm_collision)

    iTask = 1;                % próxima tarefa da lista TASKS
    PRINT_PERIOD = 1.0;       % intervalo entre a escrita do tempo de simulação [s]
    lastPrint = -inf;
%% ======================================================================
% /Configuração do movimento/
%======================================================================

    start = tic;
    told=-1
    m=1;
    stop=0;
    sim_interface.step_simulation();
    while stop==0
        %----------------------------------------------------------------------
        % Read Joints function
        % Output:
        % ReadArmJoints - get joint value (rad) for arm
        %_-----------------------------------------------------------------
        ReadArmJoints = robot_arm.get_joints();
        ReadArmJointsDegree = ReadArmJoints * 180/pi; %convert to degrees
    
        %_-----------------------------------------------------------------
        %Read Arm base position
        % Output:
        % armPosition - get robot position
        %_-----------------------------------------------------------------
        armPosition = robot_arm.get_robot_position();
    
        %_-----------------------------------------------------------------
        % Get information from cameraVISIONWORKING
        % Output:
        % image - matrix with image information
        % resolution - resolution information 
        %_-----------------------------------------------------------------
        if VISIONWORKING==1
            [image,resolution] = sim_interface.get_camera_information();
            imshow(image)
            drawnow;
        end

        %_-----------------------------------------------------------------
        % Get Target position information
        % Input:
        % ID of the object (BLUE_CUBE,BLUE_CYLINDER,RED_CYLINDER)
        % Output:
        % targetPosition - get target position
        %_-----------------------------------------------------------------
        %targetPosition=sim_interface.get_target_position(BLUE_CUBE);
        targetPosition=sim_interface.get_target_position(BLUE_CYLINDER);
        %targetPosition=sim_interface.get_target_position(RED_CUBE);
        targetPosition1=sim_interface.get_target_position(TARGET_GREEN);
    
        %----------------------------------------------------------------------
        % --- YOUR CODE --- %
        %-----------------------------------------------------------------

    %% ======================================================================
    % Exemplo pick-and-place: uma tarefa da lista TASKS por iteração
    %======================================================================
        if DEMO_PICK_AND_PLACE == 1
            if iTask <= size(TASKS, 1)
                % antes da primeira tarefa: desimpedir o destino
                if iTask == 1 && CLEAR_TARGET == 1
                    [~, okC, whyC] = clear_area(robot_arm, sim_interface, cfg, PICKABLE, ...
                               sim_interface.get_target_position(TASKS(1, 2)), CLEAR_RADIUS, ...
                               CLEAR_SPOTS, IS_CUBE, USE_OBJECT_YAW, HALF_H(PICKABLE));
                    if ~okC
                        fprintf('ERRO ao desimpedir o alvo: %s\n', whyC);
                        signal_failure(robot_arm, sim_interface, Links, MinPositionJoint, MaxPositionJoint, FAIL_POSE);
                        DEMO_PICK_AND_PLACE = 0;     % parar as tarefas (a simulação continua)
                    end
                end

                if DEMO_PICK_AND_PLACE == 1     % (0 se o desimpedir do alvo falhou)
                    obj = TASKS(iTask, 1);
                    fprintf('\n--- tarefa %d de %d ---\n', iTask, size(TASKS, 1));

                    % onde está o objeto e onde o pousar (referencial do MUNDO)
                    objPos   = sim_interface.get_target_position(obj);
                    placePos = sim_interface.get_target_position(TASKS(iTask, 2)) ...
                               + [TASKS(iTask, 3), TASKS(iTask, 4), (TASKS(iTask, 5) - 1)*BLOCK_H];

                    % orientação do cubo (para alinhar os dedos com as faces)
                    objYaw = [];
                    if IS_CUBE(obj) && USE_OBJECT_YAW
                        o = sim_interface.get_target_orientation(obj);
                        objYaw = o(3);
                    end

                    [okT, whyT] = pick_and_place(robot_arm, sim_interface, cfg, objPos, placePos, objYaw, HALF_H(obj));
                    if ~okT
                        % falhou: assinalar indo para a pose de erro e parar as tarefas
                        fprintf('ERRO na tarefa %d: %s\n', iTask, whyT);
                        signal_failure(robot_arm, sim_interface, Links, MinPositionJoint, MaxPositionJoint, FAIL_POSE);
                        DEMO_PICK_AND_PLACE = 0;         % parar as tarefas (a simulação continua)
                        if END_WHEN_DONE == 1
                            stop = 1;   % sai do ciclo while -> sim_interface.terminate() para a simulação
                        end
                    end
                    iTask = iTask + 1;
                end
            else
                % todas as tarefas feitas: celebrar, voltar à pose inicial e parar o exemplo
                if CELEBRATE == 1
                    celebrate(robot_arm, sim_interface, Links, MinPositionJoint, MaxPositionJoint);
                end
                go_home(robot_arm, sim_interface, Links, MinPositionJoint, MaxPositionJoint, [], deg2rad(HOME_POSE));
                pause(2);
                DEMO_PICK_AND_PLACE = 0;
                if END_WHEN_DONE == 1
                    stop = 1;   % sai do ciclo while -> sim_interface.terminate() para a simulação
                end
            end
        end
    %% ======================================================================
    % /Exemplo pick-and-place/
    %======================================================================

        %------------------------------------------------------------------
        % Function to send values to arm joints
        % Input:
        % armJoints - array with joint angles (1..4)
        % Output:
        % error - true if joint value outside the limits of the joint
        %-----------------------------------------------------------------
        %     armJoints(1)=0*pi/180;
        %     armJoints(2)=0*pi/180;
        %     armJoints(3)=0*pi/180;
        %     armJoints(4)=0*pi/180;
        %     armJoints(5)=0*pi/180;
        %     armJoints(6)=0*pi/180;
        %     %------------------------------------------------------------------
        %     % Function to send values to arm joints
        %     % Input:
        %     % armJoints - array with joint angles (1..4)
        %     % Output:
        %     % error - true if joint value outside the limits of the joint
        %     %-----------------------------------------------------------------
        %     error = robot_arm.set_joints(armJoints) %send value for arm Joints in rad
        %     if error==1
        %         return;
        %     end
        % end

        %------------------------------------------------------------------
        % Functions that allows open/close the hand
        %------------------------------------------------------------------
        % robot_arm.open_hand();     %open
        % robot_arm.close_hand();    %close

        m=m+1;
        %%------------- END OF YOUR CODE -------------
        sim_interface.step_simulation();
        time = toc(start);
        if time<0.05
            pause(0.05-time);
        end
        start = tic;
        t = sim_interface.get_simulation_time();
        if t==told
            sim_interface.terminate();   % era sim.terminate()
            disp("Simulation stopped");
            return
        end
        told=t;
        if t - lastPrint >= PRINT_PERIOD - 1e-6   % escreve só a cada PRINT_PERIOD s
            fprintf('Simulation time: %.2f [s]\n', t);
            lastPrint = t;
        end
        %----------------------------------------------------------------------
    end
    sim_interface.terminate();
    disp('stop simulation')
end

%% ======================================================================
% Assinalar uma falha: subir a garra se estiver baixa, ir para a pose de
% erro e abanar a garra durante 3 s
%======================================================================
function signal_failure(robot_arm, sim_interface, Links, qMin, qMax, FAIL_POSE)
    go_home(robot_arm, sim_interface, Links, qMin, qMax, [], deg2rad(FAIL_POSE));
    wiggle(robot_arm, sim_interface, 3, deg2rad(30), 1);
    disp('>>> ROBÔ PARADO: tarefa falhou (ver mensagem acima) <<<');
end

function myCleanupFun(sim_interface)
    disp('stop simulation')
    sim_interface.terminate();
    return
end