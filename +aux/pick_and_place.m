function [ok, why, placeWorld] = pick_and_place(robot_arm, sim_interface, cfg, objWorld, placeWorld, objYaw, objHalf, checkPlace, pickColor)
% PICK_AND_PLACE  Apanha um objeto e pousa-o noutro sítio (bloqueante).
%
%   ok = pick_and_place(robot_arm, sim_interface, cfg, objWorld, placeWorld)
%   ok = pick_and_place(robot_arm, sim_interface, cfg, objWorld, placeWorld, objYaw)
%   [ok, why] = pick_and_place(robot_arm, sim_interface, cfg, objWorld, placeWorld, objYaw, objHalf)
%   [ok, why, placeUsed] = pick_and_place(..., objHalf, checkPlace)
%   [ok, why, placeUsed] = pick_and_place(..., objHalf, checkPlace, pickColor)
%
%   cfg        : configuração (ver main_program)
%   objWorld   : [x y z] centro do objeto, referencial do MUNDO [m]
%                (sim_interface.get_target_position, ou o sistema de visão)
%   placeWorld : [x y z] ponto de pouso (superfície do alvo), referencial do MUNDO [m]
%   objYaw     : (opcional) orientação do cubo no MUNDO [rad] para alinhar os
%                dedos com as faces ([] = cilindro / desconhecida)
%   objHalf    : (opcional) metade da altura do objeto [m]. Por defeito é a
%                altura atual do centro acima da mesa, o que só está certo se
%                o objeto estiver pousado na mesa (não numa pilha)
%   checkPlace : (opcional) antes de ir para o destino, confirmar com a câmara
%                onde está a pilha (vision.check_place) e corrigir o ponto de
%                pouso se foi mexida. Só se cfg.CHECK_PLACE = 1.
%                  true          - procura manchas de qualquer cor
%                  'pink' / {..} - SÓ a cor do que está no topo da pilha
%                                  ('green' = o alvo vazio). Recomendado: uma
%                                  peça vizinha (ex.: o cubo vermelho ao lado)
%                                  nunca é confundida com a pilha
%                Usar ao empilhar; não ao estacionar (clear_area).
%   pickColor  : (opcional) cor da peça a apanhar: antes de ir buscá-la, uma
%                fotografia confirma onde está (pode ter sido empurrada desde a
%                fotografia inicial) e apanha-a no sítio novo. Só se
%                cfg.CHECK_PLACE = 1. '' = não confirmar
%   ok         : true se todas as fases correram bem
%   why        : texto com o motivo, se falhou
%   placeUsed  : ponto de pouso usado (MUNDO), depois de uma eventual correção
%
% Antes de mexer o braço planeia-se cada movimento vertical (plan_vertical):
% inclinação fixa da garra, com o cotovelo para cima e sem colisões do
% braço consigo próprio (arm_collision). Assim o braço chega também a
% objetos perto da base (garra inclinada para a base). Se algum ponto não
% tiver solução (ex.: na zona morta de J1), a tarefa nem começa (ok = false).
%
% Fases:
%   0b. (pickColor) fotografia: a peça ainda está onde estava?
%   1. por cima do objeto (movimento suave das juntas)
%   2. descer na vertical          3. fechar a garra
%   4. subir na vertical           4b. (checkPlace) fotografia: a pilha mexeu-se?
%                                      (se não a vê, roda J1 para o lado oposto ao
%                                       destino e repete a fotografia)
%   5. por cima do destino (movimento das juntas)
%   6. descer na vertical (até cfg.DROP_HEIGHT acima do pouso, se cfg.DROP_ON_PLACE)
%   7. abrir a garra               8. subir na vertical

    if nargin < 6, objYaw = []; end
    if nargin < 7 || isempty(objHalf)
        objHalf = objWorld(3) - cfg.TABLE_Z;   % objeto pousado na mesa
    end
    if nargin < 8 || isempty(checkPlace), checkPlace = false; end
    if nargin < 9, pickColor = ''; end
    checkR = struct();                         % opções de vision.check_place
    if ischar(checkPlace) || iscell(checkPlace) || isstring(checkPlace)
        checkR.colors = cellstr(checkPlace);   % só a cor do topo da pilha
        checkPlace = true;
    end
    checkPlace = checkPlace && isfield(cfg, 'CHECK_PLACE') && cfg.CHECK_PLACE == 1;
    ok = false;  why = '';

    % PREPARAÇÃO: converter as posições para a base e calcular os pontos de
    % apanhar (pGrasp) e de largar (pPlace), nunca abaixo da mesa
    % posições no referencial da base do robô
    pObj  = aux.world_to_base(objWorld,   cfg.BASE_POSITION, cfg.BASE_YAW);
    pTgt  = aux.world_to_base(placeWorld, cfg.BASE_POSITION, cfg.BASE_YAW);
    yaw = [];
    if ~isempty(objYaw)
        yaw = objYaw - cfg.BASE_YAW;           % orientação do cubo no referencial da base
    end
    up = [0; 0; cfg.LIFT];

    % ao pousar, o centro do objeto fica metade da sua altura acima da
    % superfície do destino
    pPlace = [pTgt(1:2); pTgt(3) + objHalf + 0.005];
    % largar o objeto um pouco acima do ponto de pouso (empilhar sem forçar)
    if isfield(cfg, 'DROP_ON_PLACE') && cfg.DROP_ON_PLACE
        pPlace(3) = pPlace(3) + cfg.DROP_HEIGHT;
    end
    % o centro de preensão nunca abaixo da mesa + pontas dos dedos + folga
    floorZ = cfg.TABLE_Z - cfg.BASE_POSITION(3) + cfg.FINGER_EXTRA + cfg.CLEARANCE;
    pGrasp = [pObj(1:2); max(pObj(3), floorZ)];
    pPlace(3) = max(pPlace(3), floorZ);

    fprintf('pick_and_place: objeto no mundo [%.3f %.3f %.3f], a %.0f cm do eixo da base\n', ...
            objWorld(1), objWorld(2), objWorld(3), 100*hypot(pObj(1), pObj(2)));

    % 0. planear as duas descidas ANTES de mexer o braço
    qNow = robot_arm.get_joints();
    [optPick, okP, qPickTop] = movement.plan_vertical(pGrasp, pGrasp + up, cfg, yaw);
    if ~okP
        why = sprintf('objeto inalcançável sem colisões, a %.0f cm do eixo da base (mundo x=%.3f y=%.3f)', ...
                      100*hypot(pObj(1), pObj(2)), objWorld(1), objWorld(2));
        fprintf('pick_and_place: %s\n', why);  return;
    end
    [optPlace, okD, qPlaceTop] = movement.plan_vertical(pPlace, pPlace + up, cfg, yaw);
    if ~okD
        why = sprintf('destino inalcançável sem colisões, a %.0f cm do eixo da base (mundo x=%.3f y=%.3f)', ...
                      100*hypot(pTgt(1), pTgt(2)), placeWorld(1), placeWorld(2));
        fprintf('pick_and_place: %s\n', why);  return;
    end
    fprintf(['pick_and_place: apanhar com a garra a %.0f graus (J1 roda %.0f graus), ' ...
             'pousar a %.0f graus (J1 roda %.0f graus)\n'], ...
            rad2deg(optPick.pitch), rad2deg(abs(qPickTop(1) - qNow(1))), ...
            rad2deg(optPlace.pitch), rad2deg(abs(qPlaceTop(1) - qPickTop(1))));

    % 0b. a peça ainda está onde a fotografia inicial a viu? (pode ter sido
    %     empurrada). Fotografia com o braço onde está; se mudou mais de 3 mm,
    %     apanhá-la no sítio novo (a garra só aperta bem uma peça centrada)
    if ~isempty(pickColor) && isfield(cfg, 'CHECK_PLACE') && cfg.CHECK_PLACE == 1
        Rk = struct('colors', {cellstr(pickColor)}, 'what', 'peça', ...
                    'minMove', 0.003, 'minMoveWeak', 0.010);
        if abs(2*objHalf - 0.0192) > 0.004, Rk.blockSize = []; end   % não é um bloco
        tipB = movement.direct_kin(qNow, cfg.Links);
        [newTop, infoK] = vision.check_place(sim_interface, [objWorld(1:2), objWorld(3) + objHalf], ...
                              base_to_world(tipB, cfg.BASE_POSITION, cfg.BASE_YAW), Rk);
        if strcmp(infoK, 'moved')
            objTry = [newTop(1:2), objWorld(3)];
            pTry   = aux.world_to_base(objTry, cfg.BASE_POSITION, cfg.BASE_YAW);
            gTry   = [pTry(1:2); max(pTry(3), floorZ)];
            [optTry, okTry, qTry] = movement.plan_vertical(gTry, gTry + up, cfg, yaw);
            if okTry
                objWorld = objTry;  pObj = pTry;  pGrasp = gTry;  optPick = optTry;  qPickTop = qTry;
            else
                fprintf('check_place: o novo ponto da peça não tem solução; apanho no ponto planeado\n');
            end
        end
    end

    % 1. por cima do objeto
    if ~go_above(robot_arm, sim_interface, cfg, pGrasp + up, yaw, optPick)
        why = 'não chegou por cima do objeto';  return;
    end
    robot_arm.open_hand();

    % 2-4. descer, fechar, subir
    if ~movement.move_linear(robot_arm, sim_interface, cfg, pGrasp, cfg.V_SLOW, yaw, [], optPick)
        why = 'não chegou ao objeto';  return;
    end
    robot_arm.close_hand();
    wait_time(sim_interface, cfg.T_CLOSE);
    if ~movement.move_linear(robot_arm, sim_interface, cfg, pGrasp + up, cfg.V_SLOW, yaw, [], optPick)
        why = 'não conseguiu subir com o objeto';  return;
    end

    % 4b. a pilha ainda está onde estava? (fotografia com o braço aqui, por cima
    %     da peça apanhada; não vai à VISION_POSE). Se mudou, refazer o plano
    %     da descida para o novo ponto
    if checkPlace
        tipB = movement.direct_kin(robot_arm.get_joints(), cfg.Links);
        [newPlace, info] = vision.check_place(sim_interface, placeWorld, ...
                               base_to_world(tipB, cfg.BASE_POSITION, cfg.BASE_YAW), checkR);
        if strcmp(info, 'lost')
            % não a viu: provavelmente o braço está a tapá-la. Só neste caso,
            % rodar só J1 (à mesma altura, com a peça) para o lado oposto ao
            % destino e tirar outra fotografia
            q = robot_arm.get_joints();
            q(1) = away_from(qPlaceTop(1), cfg.qMin(1), cfg.qMax(1));
            fprintf('check_place: talvez o braço a tape; a rodar J1 para %.0f graus (lado oposto ao destino)\n', ...
                    rad2deg(q(1)));
            movement.move_joints(robot_arm, sim_interface, q);
            tipB = movement.direct_kin(robot_arm.get_joints(), cfg.Links);
            [newPlace, info] = vision.check_place(sim_interface, placeWorld, ...
                                   base_to_world(tipB, cfg.BASE_POSITION, cfg.BASE_YAW), checkR);
        end
        if strcmp(info, 'lost')
            fprintf('check_place: pouso no ponto planeado\n');
        end
        if strcmp(info, 'moved')
            pNew = aux.world_to_base(newPlace, cfg.BASE_POSITION, cfg.BASE_YAW);
            pTry = [pNew(1:2); pPlace(3)];
            [optTry, okTry] = movement.plan_vertical(pTry, pTry + up, cfg, yaw);
            if okTry
                placeWorld = newPlace;  pPlace = pTry;  optPlace = optTry;
            else
                fprintf('check_place: o novo ponto não tem solução; pouso no ponto planeado\n');
            end
        end
    end

    % 5. por cima do destino
    if ~go_above(robot_arm, sim_interface, cfg, pPlace + up, yaw, optPlace)
        why = 'não chegou por cima do destino';  return;
    end

    % 6-8. descer, abrir, subir
    if ~movement.move_linear(robot_arm, sim_interface, cfg, pPlace, cfg.V_SLOW, yaw, [], optPlace)
        why = 'não chegou ao destino';  return;
    end
    robot_arm.open_hand();
    wait_time(sim_interface, cfg.T_OPEN);
    if ~movement.move_linear(robot_arm, sim_interface, cfg, pPlace + up, cfg.V_SLOW, yaw, [], optPlace)
        why = 'não conseguiu subir depois de pousar';  return;
    end

    % tudo correu bem
    ok = true;
    fprintf('pick_and_place: concluído\n');
end

function ok = go_above(robot_arm, sim_interface, cfg, pAbove, yaw, ikOpts)
% vai para pAbove com um movimento suave das juntas, com a família e a
% inclinação planeadas. Se o braço estiver na outra família (cotovelo
% dobrado para o outro lado), passa primeiro pela vertical, lá em cima,
% para a mudança não varrer a mesa.
    [q, ok] = movement.inverse_kin(pAbove(1), pAbove(2), pAbove(3), cfg.Links, cfg.qMin, cfg.qMax, ...
                          yaw, ikOpts.pitch, ikOpts.family, true);
    if ~ok, return; end
    qNow = robot_arm.get_joints();
    if sign(qNow(3)) ~= sign(q(3)) && abs(qNow(3)) > deg2rad(5)
        movement.move_joints(robot_arm, sim_interface, [qNow(1) 0 0 0 0 qNow(6)]);   % braço na vertical
    end
    movement.move_joints(robot_arm, sim_interface, q);
    ok = movement.move_linear(robot_arm, sim_interface, cfg, pAbove, cfg.V_SLOW, yaw, [], ikOpts);
end

function q1 = away_from(q1Dest, q1Min, q1Max)
% ângulo de J1 o mais afastado possível de q1Dest (idealmente o oposto, +180
% graus), dentro dos limites de J1 (com 5 graus de margem)
    m = deg2rad(5);
    cand = linspace(q1Min + m, q1Max - m, 121);
    d = abs(atan2(sin(cand - q1Dest), cos(cand - q1Dest)));   % distância angular
    [~, i] = max(d);
    q1 = cand(i);
end

function pW = base_to_world(pB, armPosition, baseYaw)
% o inverso de aux.world_to_base: ponto da base -> MUNDO
    c = cos(baseYaw);  s = sin(baseYaw);
    pW = armPosition(:)' + [c*pB(1) - s*pB(2), s*pB(1) + c*pB(2), pB(3)];
end

function wait_time(sim_interface, T)
% espera T segundos de simulação (dá tempo à garra para abrir/fechar)
    t0 = sim_interface.get_simulation_time();
    while sim_interface.get_simulation_time() - t0 < T
        sim_interface.step_simulation();
    end
end