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
% MODO DE FUNCIONAMENTO - escolher AQUI (só um):
%   1 - PILHA PELA CÂMARA: executa a lista TASKS (empilhar no alvo verde),
%       com as posições dadas pela câmara                         [por defeito]
%   2 - PILHA DE PARES: vigia a mesa; quando entra um bloco da mesma cor de
%       outro (um par), empilha um em cima do outro
%   3 - PEDIDO DE COR: vigia a mesa; um bloco novo da mesma cor de outro é um
%       pedido (pode estar fora do alcance): a peça dessa cor ao alcance vai
%       para a pilha do alvo verde
% (todos fazem primeiro o desimpedir do alvo, se CLEAR_TARGET = 1)
%======================================================================
    MODE = 3;
    MODE_NAMES = {'pilha pela câmara', 'pilha de pares', 'pedido de cor'};
    fprintf('MODO %d: %s\n', MODE, MODE_NAMES{MODE});
    USE_VISION = 1;                         % só no modo 1: 0 = posições do simulador
                                            % (get_target_position) em vez da câmara
    if MODE ~= 1, USE_VISION = 1; end       % os modos de vigiar precisam da câmara
    WATCH_MODE = (MODE == 2 || MODE == 3);  % vigiar em vez das TASKS
    WATCH_KIND = 'request';                 % 'pairs' (modo 2) ou 'request' (modo 3)
    if MODE == 2, WATCH_KIND = 'pairs'; end

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
    ERASE_DRAWINGS = 1;                     % 1 - apagar os desenhos (zonas, marcas da visão) no fim

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

    % [ACRESCENTADO] Visão (vision_colors.m): posições dos objetos pela câmara
    %   /Vision_sensor em vez de get_target_position. A orientação (yaw) dos
    %   cubos também vem da câmara (retângulo mínimo da face de cima).
    % (USE_VISION é escolhido pelo MODE, no início do programa)
    COMPARE_VISION = 1;                    % 1 - escrever o erro visão vs. simulador (só testes)
    SHOW_VISION   = 1;                     % 1 - mostrar o resultado da visão (figura + marcas na mesa)
    OBJ_NAME = {'BlueCube','BlueCylinder','RedCylinder','TargetGreen','Block1','Block2','Block3'};
    % cor de cada objeto (índice = número do objeto). !!!CONFIRMAR no cenário!!!
    OBJ_COLOR = cell(1, BLOCK3);
    OBJ_COLOR{BLUE_CUBE}     = 'blue';
    OBJ_COLOR{BLUE_CYLINDER} = 'blue';
    OBJ_COLOR{RED_CYLINDER}  = 'red';
    OBJ_COLOR{TARGET_GREEN}  = 'green';
    OBJ_COLOR{BLOCK1}        = 'orange';
    OBJ_COLOR{BLOCK2}        = 'pink';
    OBJ_COLOR{BLOCK3}        = 'yellow';
    % se há várias peças da mesma cor: qual é, por área na imagem (1 = a maior)
    OBJ_RANK = ones(1, BLOCK3);
    OBJ_RANK(BLUE_CYLINDER) = 2;           % o cubo azul aparece maior que o cilindro
    USE_VISION_YAW = 1;                    % 1 - alinhar a garra com o yaw medido pela câmara

    % [ACRESCENTADO] Modo VIGIAR (watch_pairs.m): em vez das TASKS, o braço fica
    %   em VISION_POSE a fotografar a mesa. Um bloco que ENTRA na vista com a
    %   MESMA COR de um que já lá está (um par) é um PEDIDO: indica a cor da
    %   próxima peça da pilha do alvo verde. O robô apanha a peça dessa cor ao
    %   seu alcance mais perto da base e empilha-a no alvo. O bloco do pedido
    %   pode estar fora do alcance (é só um sinal).
    %   Corre até se parar a simulação. Precisa de USE_VISION = 1.
    % (WATCH_MODE é escolhido pelo MODE, no início do programa)
    WATCH_COLORS  = {'yellow', 'pink', 'orange'};   % cores dos blocos a emparelhar
    WATCH_PERIOD  = 1.0;                   % intervalo entre fotografias [s de simulação]
    HOME_POSE     = [0 0 -90 0 0 0];       % pose inicial [GRAUS] (J1..J6); [0 0 0 0 0 0] = braço vertical
    VISION_POSE   = [0 0 0 90 0 0];        % [ACRESCENTADO] pose para a fotografia [GRAUS] (fora da vista da câmara)
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
    
    cleanup = onCleanup(@()myCleanupFun(sim_interface, ERASE_DRAWINGS));
    
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

    % desenhar as zonas de alcance na mesa (só visual)
    if SHOW_ZONES == 1
        draw_zones(sim_interface, BASE_POSITION, BASE_YAW, TABLE_Z, deg2rad(165), ...
                   [0.1316 0.1104 0.0960 0.0731 0.0646 0.0456 0.1057]);
    else
        % apagar desenhos de corridas anteriores (draw_zones também o faz)
        simD = sim_interface.get_connection().require('sim');
        try, simD.removeDrawingObject(simD.handle_all); catch, end
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
    cfg.SHOW_CLEAR = 1;       % [ACRESCENTADO] 1 - desenhar na mesa a zona a desimpedir, os
                              % estacionamentos e a peça a retirar (clear_area)

    iTask = 1;                % próxima tarefa da lista TASKS

%% ======================================================================
% Posições dos objetos (POS: linha = número do objeto, MUNDO)
%======================================================================
    % Com visão: uma fotografia com o braço na vertical (fora da vista da
    % câmara), antes de mexer em qualquer peça. Depois as posições são
    % atualizadas pelo próprio programa sempre que move uma peça (uma peça
    % empilhada tapa a de baixo, por isso não se volta a fotografar).
    ALL_OBJ = 1:BLOCK3;
    HALF_ALL = [HALF_H, zeros(1, BLOCK3 - numel(HALF_H))];
    HALF_ALL(TARGET_GREEN) = 0.001;        % o alvo é plano
    POS = zeros(BLOCK3, 3);
    YAW = nan(BLOCK3, 1);                  % orientação de cada objeto [rad] (NaN = desconhecida)
    if USE_VISION == 1
        % (as poses estão em GRAUS: go_home quer radianos -> deg2rad)
        go_home(robot_arm, sim_interface, Links, MinPositionJoint, MaxPositionJoint, [], deg2rad(VISION_POSE));
        % (com try/catch: um erro na visão não deve fechar a simulação; o
        %  onCleanup do programa termina a simulação em qualquer erro)
        try
            [POS, found, pix, img, det, YAW] = vision_snapshot(sim_interface, ALL_OBJ, OBJ_COLOR(ALL_OBJ), ...
                                           OBJ_RANK(ALL_OBJ), HALF_ALL(ALL_OBJ), TABLE_Z);
        catch errV
            fprintf(2, 'ERRO na visão: %s\n', errV.message);
            fprintf(2, '  em %s (linha %d)\n', errV.stack(1).name, errV.stack(1).line);
            POS = nan(BLOCK3, 3);  found = false(BLOCK3, 1);
            pix = nan(BLOCK3, 2);  img = [];  det = [];  YAW = nan(BLOCK3, 1);
        end
        if USE_VISION_YAW ~= 1
            YAW(:) = NaN;                    % não usar a orientação da câmara
        end
        truth = [];  truthYaw = [];
        if COMPARE_VISION == 1           % só no simulador: posições reais, para ver o erro
            truth = zeros(BLOCK3, 3);  truthYaw = zeros(BLOCK3, 1);
            for k = ALL_OBJ
                truth(k,:) = sim_interface.get_target_position(k);
                o = sim_interface.get_target_orientation(k);
                truthYaw(k) = o(3);
            end
        end
        for k = ALL_OBJ
            if found(k)
                fprintf('visão: %-12s (%s) em [%.3f %.3f %.3f]', OBJ_NAME{k}, OBJ_COLOR{k}, POS(k,:));
                if ~isempty(truth)
                    fprintf('   erro %.1f mm', 1000*norm(POS(k,1:2) - truth(k,1:2)));
                end
                if ~isnan(YAW(k)) && IS_CUBE(k)
                    fprintf('   yaw %5.1f°', rad2deg(YAW(k)));
                    if ~isempty(truthYaw)    % erro a menos de 90 graus (face quadrada)
                        fprintf(' (erro %.1f°)', rad2deg(mod(YAW(k) - truthYaw(k) + pi/4, pi/2) - pi/4));
                    end
                end
                fprintf('\n');
            else
                fprintf('visão: %-12s (%s) NÃO ENCONTRADO\n', OBJ_NAME{k}, OBJ_COLOR{k});
            end
        end
        % [ACRESCENTADO] feedback visual: imagem com as deteções + marcas na mesa
        if SHOW_VISION == 1
            try
                show_vision(sim_interface, img, det, OBJ_NAME(ALL_OBJ), OBJ_COLOR(ALL_OBJ), ...
                            POS, pix, found, truth, YAW, truthYaw);
            catch errS          % o feedback é só visual: avisar e continuar
                fprintf(2, 'AVISO show_vision: %s\n', errS.message);
                fprintf(2, '  em %s (linha %d)\n', errS.stack(1).name, errS.stack(1).line);
            end
        end
        % sem as peças das tarefas (ou o destino) não se pode continuar
        if WATCH_MODE == 1
            % [ACRESCENTADO] modo vigiar: não há TASKS; só o alvo interessa (para o desimpedir)
            needed = [];
            if CLEAR_TARGET == 1 && ~found(TARGET_GREEN)
                disp('AVISO: a visão não encontrou o alvo; não vou desimpedir a zona do alvo');
                CLEAR_TARGET = 0;
            end
        else
            needed = unique([TASKS(:,1); TASKS(:,2)])';
        end
        if any(~found(needed))
            fprintf('ERRO: a visão não encontrou o(s) objeto(s) %s\n', mat2str(needed(~found(needed))));
            signal_failure(robot_arm, sim_interface, Links, MinPositionJoint, MaxPositionJoint, FAIL_POSE);
            DEMO_PICK_AND_PLACE = 0;
        end
        % peças não encontradas ficam fora da zona a desimpedir
        PICKABLE = PICKABLE(found(PICKABLE));
        if DEMO_PICK_AND_PLACE == 1      % (se falhou, fica na pose de erro)
            go_home(robot_arm, sim_interface, Links, MinPositionJoint, MaxPositionJoint, [], deg2rad(HOME_POSE));
        end
    else
        for k = ALL_OBJ
            POS(k,:) = sim_interface.get_target_position(k);
        end
    end
    getPos = @(id) POS(id, :);             % posições guardadas (usadas por clear_area)

    % [ACRESCENTADO] de onde vem a orientação dos cubos: câmara ou simulador
    % (a peça mantém o yaw quando é mudada de sítio: a garra pousa-a com o
    %  mesmo ângulo com que a apanhou, por isso YAW não precisa de ser atualizado)
    if USE_VISION == 1
        yawSource = @(id) YAW(id);
    else
        yawSource = USE_OBJECT_YAW;
    end

    % [ACRESCENTADO] modo vigiar: sem TASKS; braço fora da vista da câmara
    if WATCH_MODE == 1
        DEMO_PICK_AND_PLACE = 0;
        [~, W] = watch_pairs();            % estado inicial
        W.colors = WATCH_COLORS;  W.tableZ = TABLE_Z;  W.blockH = BLOCK_H;
        W.base = BASE_POSITION(1:2);       % para escolher o bloco mais perto do robô
        W.reachFn = @(p) can_reach(cfg, p);% só apanhar peças que o robô alcança
        W.mode = WATCH_KIND;               % 'pairs' ou 'request' (escolhido pelo MODE)
        W.placeExtra = 0.005 + (cfg.DROP_ON_PLACE == 1)*cfg.DROP_HEIGHT;   % como em pick_and_place
        W.target = POS(TARGET_GREEN, :);   % pedido de cor: a pilha faz-se no alvo verde
        if strcmp(WATCH_KIND, 'request') && any(isnan(W.target))
            disp('ERRO: a visão não encontrou o alvo verde; o modo vigiar não pode empilhar');
            signal_failure(robot_arm, sim_interface, Links, MinPositionJoint, MaxPositionJoint, FAIL_POSE);
            WATCH_MODE = 0;
        end
        hWatch = [];                       % marcas da última fotografia (CoppeliaSim)
        % desimpedir a zona do alvo antes de começar a vigiar (como nas TASKS)
        if CLEAR_TARGET == 1 && WATCH_MODE == 1
            [~, okC, whyC] = clear_area(robot_arm, sim_interface, cfg, PICKABLE, ...
                       POS(TARGET_GREEN, :), CLEAR_RADIUS, ...
                       CLEAR_SPOTS, IS_CUBE, yawSource, HALF_H(PICKABLE), getPos);
            if ~okC
                fprintf('ERRO ao desimpedir o alvo: %s\n', whyC);
                signal_failure(robot_arm, sim_interface, Links, MinPositionJoint, MaxPositionJoint, FAIL_POSE);
                % pedido de cor: sem o alvo livre não há pilha -> parar.
                % pilha de pares: não precisa do alvo -> continuar a vigiar
                if strcmp(WATCH_KIND, 'request'), WATCH_MODE = 0; end
            end
        end
        if WATCH_MODE == 1
            go_home(robot_arm, sim_interface, Links, MinPositionJoint, MaxPositionJoint, [], deg2rad(VISION_POSE));
            lastWatch = -inf;
            disp('modo vigiar: à espera de pedidos (blocos da mesma cor)...');
        end
    end
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
    % [ACRESCENTADO] Modo vigiar: uma fotografia a cada WATCH_PERIOD segundos
    %======================================================================
        if WATCH_MODE == 1
            tW = sim_interface.get_simulation_time();
            if tW - lastWatch >= WATCH_PERIOD
                lastWatch = tW;
                [act, W, view] = watch_pairs(sim_interface, W);
                % mostrar cada fotografia (figura + marcas na mesa, como no início)
                if SHOW_VISION == 1 && ~isempty(view)
                    try
                        hWatch = show_vision(sim_interface, view.img, view.det, view.names, ...
                                     view.colors, view.pos, view.pix, view.found, [], view.yaw, [], ...
                                     hWatch, sprintf('Vigiar: t = %.1f s, %d blocos', tW, numel(view.names)));
                    catch errS
                        fprintf(2, 'AVISO show_vision: %s\n', errS.message);
                    end
                end
                if ~isempty(act)
                    if strcmp(WATCH_KIND, 'pairs')
                        fprintf('\n--- par %s: bloco em [%.3f %.3f] -> em cima de [%.3f %.3f] (nível %d) ---\n', ...
                                act.color, act.pick(1:2), act.place(1:2), act.level);
                    else
                        fprintf('\n--- pedido %s (bloco em [%.3f %.3f]): peça em [%.3f %.3f] -> pilha verde, nível %d ---\n', ...
                                act.color, act.req, act.pick(1:2), act.level);
                    end
                    yawW = [];
                    if USE_VISION_YAW == 1 && ~isnan(act.yaw), yawW = act.yaw; end
                    [okW, whyW] = pick_and_place(robot_arm, sim_interface, cfg, act.pick, act.place, yawW, BLOCK_H/2);
                    if ~okW
                        % não deu (ex.: fora do alcance): avisar, ignorar esse bloco e continuar
                        fprintf('AVISO: não consegui levar a peça %s para a pilha (%s)\n', act.color, whyW);
                    end
                    [~, W] = watch_pairs([], W, act, okW);
                    % voltar para fora da vista da câmara antes da próxima fotografia
                    go_home(robot_arm, sim_interface, Links, MinPositionJoint, MaxPositionJoint, [], deg2rad(VISION_POSE));
                    lastWatch = sim_interface.get_simulation_time();
                end
            end
        end

    %% ======================================================================
    % Exemplo pick-and-place: uma tarefa da lista TASKS por iteração
    %======================================================================
        if DEMO_PICK_AND_PLACE == 1
            if iTask <= size(TASKS, 1)
                % antes da primeira tarefa: desimpedir o destino
                if iTask == 1 && CLEAR_TARGET == 1
                    [~, okC, whyC, parked] = clear_area(robot_arm, sim_interface, cfg, PICKABLE, ...
                               POS(TASKS(1, 2), :), CLEAR_RADIUS, ...
                               CLEAR_SPOTS, IS_CUBE, yawSource, HALF_H(PICKABLE), getPos);
                    % [ACRESCENTADO] guardar onde ficaram as peças retiradas
                    for m = 1:size(parked, 1)
                        POS(parked(m,1), :) = parked(m, 2:4);
                    end
                    getPos = @(id) POS(id, :);
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
                    % [ACRESCENTADO] posições guardadas (visão ou simulador), não lidas de novo
                    objPos   = POS(obj, :);
                    placePos = POS(TASKS(iTask, 2), :) ...
                               + [TASKS(iTask, 3), TASKS(iTask, 4), (TASKS(iTask, 5) - 1)*BLOCK_H];

                    % orientação do cubo (para alinhar os dedos com as faces)
                    objYaw = [];
                    if IS_CUBE(obj) && USE_VISION == 1
                        if ~isnan(YAW(obj)), objYaw = YAW(obj); end    % [ACRESCENTADO] yaw da câmara
                    elseif IS_CUBE(obj) && USE_OBJECT_YAW
                        o = sim_interface.get_target_orientation(obj);
                        objYaw = o(3);
                    end

                    [okT, whyT] = pick_and_place(robot_arm, sim_interface, cfg, objPos, placePos, objYaw, HALF_H(obj));
                    if ~okT
                        % falhou: assinalar indo para a pose de erro e parar as tarefas
                        fprintf('ERRO na tarefa %d: %s\n', iTask, whyT);
                        signal_failure(robot_arm, sim_interface, Links, MinPositionJoint, MaxPositionJoint, FAIL_POSE);
                        DEMO_PICK_AND_PLACE = 0;         % parar as tarefas (a simulação continua)
                    else
                        % [ACRESCENTADO] a peça está agora no destino (guardar o seu centro)
                        POS(obj, :) = [placePos(1:2), TABLE_Z + (TASKS(iTask, 5) - 1)*BLOCK_H + HALF_H(obj)];
                        getPos = @(id) POS(id, :);
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
            if ERASE_DRAWINGS == 1, erase_drawings(sim_interface); end   % [ACRESCENTADO]
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
    if ERASE_DRAWINGS == 1, erase_drawings(sim_interface); end   % [ACRESCENTADO]
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

function myCleanupFun(sim_interface, eraseDrawings)
    % [ACRESCENTADO] corre também quando há um erro ou Ctrl+C: apagar os desenhos
    if nargin > 1 && eraseDrawings == 1
        erase_drawings(sim_interface);
    end
    disp('stop simulation')
    sim_interface.terminate();
    return
end

%% ======================================================================
% [ACRESCENTADO] Apagar todos os desenhos da cena (zonas de alcance,
% marcas da visão). Os desenhos feitos pela API remota não desaparecem
% sozinhos quando a simulação para.
%======================================================================
function erase_drawings(sim_interface)
    try
        sim = sim_interface.get_connection().require('sim');
        sim.removeDrawingObject(sim.handle_all);
    catch
        % ligação já fechada: não há nada a fazer
    end
end