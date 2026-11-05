function [S, status] = diagnostico(action, S, ctx, SHAKE)
% DIAGNOSTICO  MODO 0: diagnóstico completo (só no simulador).
%              Junta num só modo todos os testes que usam informação que só
%              o simulador tem:
%                1. CINEMÁTICA E PREENSÃO - para cada peça, faz o mesmo que o
%                   pick_and_place até ao momento de fechar a garra (planeia a
%                   descida com plan_vertical, vai por cima, desce em linha
%                   reta com a garra aberta) e mede:
%                     - modelo : direct_kin das juntas lidas vs. /graspCenter real
%                     - alcance: ponto pedido vs. /graspCenter real
%                     - juntas : juntas pedidas vs. alcançadas
%                     - garra-peça: /graspCenter real vs. centro REAL da peça
%                       mais perto (se a garra fechasse, apanhava-a ao centro?)
%                     - yaw real da peça vs. yaw medido pela câmara
%                   Se CLOSE_TEST = 1: fecha a garra, sobe 3 cm e mede onde a
%                   peça ficou DENTRO da garra (desvio e inclinação); depois
%                   volta a pousá-la no mesmo sítio e abre. Assim vê-se se a
%                   garra apanha cada peça ao centro (o que falha no modo 4).
%                   Com CLOSE_TEST = 0 sobe sem fechar (não mexe nas peças).
%                2. VISÃO - no fim, vision.vision_check (posições, tamanhos,
%                   formas, cores lavadas, sinal da câmara).
%              No fim escreve uma tabela com tudo e volta à pose inicial.
%
%   [S, status] = modes.diagnostico('init', [], ctx)
%   [S, status] = modes.diagnostico('step', S, ctx)    % uma peça por chamada
%
%   status : 'running' ou 'finished'
%
% Que peças visitar (TARGETS, abaixo):
%   'visao'  - as peças que a câmara vê na mesa (uma fotografia no início):
%              os cubos e blocos que estão mesmo lá, incluindo clones e as
%              peças do modelo, sem nome no simulador
%   'nomes'  - os objetos com nome do simulator_interface (BlueCube, Block1,
%              ...), nas posições reais; os que estão fora da mesa ficam
%              "inalcançáveis"
%
% Usa informação que só o simulador tem (/graspCenter, posições reais):
% serve para validar o modelo do braço, a preensão e a visão, nunca para
% controlar o robô real.

    SETTLE = 40;            % passos de simulação para estabilizar (~2 s)
    TOL_MM = 5;             % erro [mm] acima do qual se escreve "VER"
    TARGETS = 'visao';      % 'visao' (peças vistas pela câmara) ou 'nomes' (objetos do simulador)
    COLORS  = {'red', 'orange', 'yellow', 'green', 'blue', 'pink'};   % cores procuradas ('visao')
    PIECES  = '^/(Block|BlueCube|RedCube|BlueCylinder)';   % nomes das peças reais (para garra-peça)
    CLOSE_TEST = 1;         % 1 - fechar a garra, levantar 3 cm e medir a peça na garra

    status = 'running';
    switch action
        case 'init'
            sim = ctx.sim_interface.get_connection().require('sim');
            % INIT: centro de preensão real, base do robô e lista de peças
            S.hGrasp = sim.getObject('/graspCenter');
            S.hBase  = sim.getObject(ctx.robot_name);
            [S.names, S.tops, S.half, S.yaw] = make_targets(TARGETS, COLORS, ctx);
            S.k = 1;
            % resultados: modelo, alcance [mm], juntas [graus], garra-peça [mm],
            %             dx, dy [mm], yaw real, yaw da câmara [graus]
            S.res = nan(numel(S.names), 10);        % + na garra: desvio [mm], inclinação [graus]
            fprintf('\n=== MODO 0: DIAGNÓSTICO (%d peças, alvos: %s) ===\n', numel(S.names), TARGETS);

        case 'step'
            sim = ctx.sim_interface.get_connection().require('sim');
            cfg = ctx.cfg;
            n = numel(S.names);
            if S.k <= n
                k = S.k;  S.k = S.k + 1;
                fprintf('\n== %d. %s ==\n', k, S.names{k});

                % 1. os mesmos pontos que o pick_and_place: centro de preensão
                %    no centro da peça (nunca abaixo da mesa + dedos + folga)
                %    e o ponto LIFT acima; inclinação fixa com plan_vertical
                cW = S.tops(k, :) - [0 0 S.half(k)];                  % centro da peça (mundo)
                pObj = aux.world_to_base(cW, ctx.BASE_POSITION, ctx.BASE_YAW);
                floorZ = cfg.TABLE_Z - cfg.BASE_POSITION(3) + cfg.FINGER_EXTRA + cfg.CLEARANCE;
                pGrasp = [pObj(1:2); max(pObj(3), floorZ)];
                up = [0; 0; cfg.LIFT];
                yawB = [];
                if ~isnan(S.yaw(k)), yawB = S.yaw(k) - ctx.BASE_YAW; end
                [opt, ok, qTop] = movement.plan_vertical(pGrasp, pGrasp + up, cfg, yawB);
                if ~ok
                    fprintf(' inalcançável (a %.0f cm do eixo da base)\n', 100*hypot(pGrasp(1), pGrasp(2)));
                    return;
                end
                [qG, okG] = movement.inverse_kin(pGrasp(1), pGrasp(2), pGrasp(3), ctx.Links, ctx.qMin, ctx.qMax, ...
                                                 yawB, opt.pitch, opt.family, true);

                % 2. ir pela vertical, por cima da peça, abrir e descer em linha reta
                movement.go_home(0, ctx.robot_arm, ctx.sim_interface, ctx.Links, ctx.qMin, ctx.qMax, ...
                                 [], zeros(1, 6));
                movement.move_joints(ctx.robot_arm, ctx.sim_interface, qTop);
                movement.move_linear(ctx.robot_arm, ctx.sim_interface, cfg, pGrasp + up, cfg.V_SLOW, yawB, [], opt);
                ctx.robot_arm.open_hand();
                movement.move_linear(ctx.robot_arm, ctx.sim_interface, cfg, pGrasp, cfg.V_SLOW, yawB, [], opt);
                for s = 1:SETTLE
                    ctx.sim_interface.step_simulation();
                end

                % 3. medir: modelo, alcance e juntas
                qr = ctx.robot_arm.get_joints();
                dk = movement.direct_kin(qr, ctx.Links);                 % modelo (base)
                gB = to_vec(sim.getObjectPosition(S.hGrasp, S.hBase));   % real (base)
                gW = to_vec(sim.getObjectPosition(S.hGrasp, -1));        % real (mundo)
                eModel = 1000*norm(dk(:) - gB(:));
                eReach = 1000*norm(pGrasp(:) - gB(:));
                eJoint = NaN;
                if okG, eJoint = rad2deg(max(abs(qr(:) - qG(:)))); end
                fprintf(' pedido (base)        : %7.4f %7.4f %7.4f   (garra a %.0f graus)\n', pGrasp, rad2deg(opt.pitch));
                fprintf(' direct_kin (base)    : %7.4f %7.4f %7.4f\n', dk(1:3));
                fprintf(' graspCenter (base)   : %7.4f %7.4f %7.4f   <- simulador\n', gB);
                fprintf(' erros: modelo %.1f mm %s | alcance %.1f mm %s | juntas %.1f graus\n', ...
                        eModel, flag(eModel, TOL_MM), eReach, flag(eReach, TOL_MM), eJoint);

                % 4. garra vs. peça REAL mais perto: se fechasse aqui, apanhava-a ao centro?
                [pName, pW, pYaw, hP] = nearest_piece(sim, gW, PIECES);
                eGrip = NaN;  dxy = [NaN NaN];
                if ~isempty(pName) && hypot(pW(1) - gW(1), pW(2) - gW(2)) < 0.03
                    dxy = 1000*(gW(1:2) - pW(1:2));
                    eGrip = norm(dxy);
                    fprintf(' garra - peça real    : (%5.1f, %5.1f) mm = %.1f mm %s   [%s]\n', ...
                            dxy, eGrip, flag(eGrip, TOL_MM), pName);
                    yC = S.yaw(k);
                    if ~isnan(yC)
                        fprintf(' yaw: real %.1f°, câmara %.1f° (diferença a menos de 90°: %.1f°)\n', ...
                                rad2deg(pYaw), rad2deg(yC), rad2deg(mod(yC - pYaw + pi/4, pi/2) - pi/4));
                    end
                else
                    fprintf(' garra - peça real    : nenhuma peça a menos de 3 cm da garra\n');
                end
                S.res(k, 1:8) = [eModel, eReach, eJoint, eGrip, dxy, rad2deg(pYawOrNaN(pName, pYaw)), rad2deg(S.yaw(k))];

                % 5. (CLOSE_TEST) fechar, levantar 3 cm e ver onde a peça ficou na garra;
                %    depois pousá-la outra vez no mesmo sítio e abrir
                if CLOSE_TEST == 1 && ~isempty(pName) && ~isnan(eGrip)
                    % dedos da garra antes de fechar (no referencial do graspCenter)
                    if ~isfield(S, 'fingers'), S.fingers = finger_handles(sim, S.hGrasp); end
                    f0 = finger_pos(sim, S.fingers, S.hGrasp);
                    [a0, t0] = in_grip(sim, hP, S.hGrasp);        % peça na garra: aberta
                    ctx.robot_arm.close_hand();
                    wait_steps(ctx.sim_interface, cfg.T_CLOSE);
                    [a1, t1] = in_grip(sim, hP, S.hGrasp);        % fechada, ainda na mesa
                    movement.move_linear(ctx.robot_arm, ctx.sim_interface, cfg, pGrasp + [0; 0; 0.03], cfg.V_SLOW, yawB, [], opt);
                    wait_steps(ctx.sim_interface, 0.5);
                    gW2 = to_vec(sim.getObjectPosition(S.hGrasp, -1));
                    pW2 = to_vec(sim.getObjectPosition(hP, -1));
                    o2  = to_vec(sim.getObjectOrientation(hP, -1));
                    held = 1000*(pW2 - gW2);                         % peça - garra [mm]
                    % no referencial do graspCenter (x, y da garra): mostra se o desvio
                    % é na direção em que os dedos fecham ou ao longo deles
                    [a2, t2] = in_grip(sim, hP, S.hGrasp);        % levantada 3 cm
                    fprintf(' peça no ref. da garra    x      y      z [mm]   inclinação (para x, para y)\n');
                    fprintf('   aberta            : %6.1f %6.1f %6.1f      (%5.1f°, %5.1f°)\n', a0, t0);
                    fprintf('   fechada (na mesa) : %6.1f %6.1f %6.1f      (%5.1f°, %5.1f°)\n', a1, t1);
                    fprintf('   levantada 3 cm    : %6.1f %6.1f %6.1f      (%5.1f°, %5.1f°)\n', a2, t2);
                    fprintf('   rotação da peça (yaw) ao fechar: %.1f°\n', rad2deg(mod(o2(3) - pYaw + pi/4, pi/2) - pi/4));
                    f1 = finger_pos(sim, S.fingers, S.hGrasp);
                    for i = find(vecnorm(f1 - f0, 2, 2)' > 0.001)
                        fprintf(' dedo %-28s aberto (%5.1f, %5.1f, %5.1f) -> fechado (%5.1f, %5.1f, %5.1f) mm\n', ...
                                S.fingers(i).name, 1000*f0(i, :), 1000*f1(i, :));
                    end
                    tilt = rad2deg(acos(max(-1, min(1, cos(o2(1))*cos(o2(2))))));
                    if pW2(3) < cfg.TABLE_Z + 0.015
                        fprintf(' na garra            : a peça NÃO subiu (ficou na mesa)  <- VER\n');
                        S.res(k, 9:10) = [NaN NaN];
                    else
                        fprintf(' na garra            : peça - garra = (%5.1f, %5.1f, %5.1f) mm, inclinada %.0f°  %s\n', ...
                                held, tilt, flag(max(norm(held(1:2)), tilt/2), TOL_MM));
                        S.res(k, 9:10) = [norm(held(1:2)), tilt];
                    end
                    % voltar a pousar e abrir
                    movement.move_linear(ctx.robot_arm, ctx.sim_interface, cfg, pGrasp + [0; 0; 0.003], cfg.V_SLOW, yawB, [], opt);
                    ctx.robot_arm.open_hand();
                    wait_steps(ctx.sim_interface, cfg.T_OPEN);
                end

                % 6. subir (com a garra aberta)
                movement.move_linear(ctx.robot_arm, ctx.sim_interface, cfg, pGrasp + up, cfg.V_SLOW, yawB, [], opt);
            else
                % resumo
                fprintf('\n=== RESUMO DO DIAGNÓSTICO ===\n');
                fprintf('%-22s %9s %9s %8s %11s %17s %13s %16s\n', 'peça', 'modelo', 'alcance', 'juntas', ...
                        'garra-peça', '(dx, dy) [mm]', 'yaw real/cam', 'na garra');
                for k = 1:n
                    r = S.res(k, :);
                    if isnan(r(1))
                        fprintf('%-22s %9s\n', S.names{k}, 'inalcançável');
                    else
                        fprintf('%-22s %6.1f mm %6.1f mm %6.1f° %8.1f mm   (%5.1f, %5.1f) %6.1f°/%5.1f° %6.1f mm %3.0f°%s\n', ...
                                S.names{k}, r(1:4), r(5:6), r(7), r(8), r(9), r(10), flag2(max(r(4), r(9)), TOL_MM));
                    end
                end
                r = S.res(~isnan(S.res(:, 1)), :);
                if ~isempty(r)
                    fprintf('%-22s %6.1f mm %6.1f mm %6.1f° %8.1f mm\n', 'MÁXIMO', max(r(:, 1:4), [], 1));
                    fprintf(['modelo     = direct_kin vs. graspCenter real (erro do modelo do braço)\n' ...
                             'alcance    = ponto pedido vs. graspCenter real (inclui o atraso das juntas)\n' ...
                             'garra-peça = graspCenter real vs. centro real da peça (apanha-a ao centro?)\n' ...
                             'na garra   = depois de fechar e subir 3 cm: desvio da peça e inclinação\n']);
                end
                % 2. visão
                movement.go_home(ctx.SHAKE, ctx.robot_arm, ctx.sim_interface, ctx.Links, ctx.qMin, ctx.qMax, ...
                                 [], deg2rad(ctx.VISION_POSE));
                vision.vision_check(ctx.sim_interface);
                movement.go_home(ctx.SHAKE, ctx.robot_arm, ctx.sim_interface, ctx.Links, ctx.qMin, ctx.qMax, ...
                                 [], deg2rad(ctx.HOME_POSE));
                status = 'finished';
            end
    end
end

% -------------------------------------------------------------------------
function [names, tops, half, yaw] = make_targets(TARGETS, COLORS, ctx)
% peças a visitar: nome, ponto da face de cima (MUNDO), meia altura e yaw
% (da câmara; NaN = desconhecido)
    if strcmpi(TARGETS, 'nomes')
        % objetos com nome do simulador, nas posições reais
        n = numel(ctx.OBJ_NAME);
        names = ctx.OBJ_NAME(:)';  tops = zeros(n, 3);  half = ctx.HALF_ALL(1:n);  yaw = nan(1, n);
        for k = 1:n
            tops(k, :) = ctx.sim_interface.get_target_position(k);
            tops(k, 3) = tops(k, 3) + half(k);
        end
        return;
    end
    % 'visao': braço fora da vista, uma fotografia, uma peça por mancha;
    % manchas com mais de 36 mm são cubos/cilindros de 4 cm, as outras blocos
    movement.go_home(ctx.SHAKE, ctx.robot_arm, ctx.sim_interface, ctx.Links, ctx.qMin, ctx.qMax, ...
                     [], deg2rad(ctx.VISION_POSE));
    det = vision.vision_colors(ctx.sim_interface, COLORS);
    n = numel(det);
    names = cell(1, n);  tops = zeros(n, 3);  half = zeros(1, n);  yaw = nan(1, n);
    for j = 1:n
        if strcmpi(det(j).color, 'green')
            half(j) = 0.0005;                          % o alvo é plano
        elseif ~isnan(det(j).size) && det(j).size > 0.036
            half(j) = 0.02;                            % cubo / cilindro de 4 cm
        else
            half(j) = ctx.BLOCK_H/2;                   % bloco
        end
        tops(j, :) = vision.at_height(det(j), ctx.TABLE_Z + 2*half(j));
        yaw(j) = det(j).yaw;
        names{j} = sprintf('%s (%.0f, %.0f)', det(j).color, 1000*tops(j, 1:2));
    end
    fprintf('diagnóstico: a câmara viu %d peça(s)\n', n);
end

function [name, p, yaw, hBest] = nearest_piece(sim, gW, PIECES)
% peça real (forma com nome de peça) mais perto da garra, em x, y
    name = '';  p = [NaN NaN NaN];  yaw = NaN;  best = inf;  hBest = [];
    try
        sh = to_vec(sim.getObjectsInTree(sim.handle_scene, sim.object_shape_type));
    catch
        return;
    end
    for h = sh
        a = char(sim.getObjectAlias(h, 1));
        if isempty(regexp(a, PIECES, 'once')), continue; end
        q = to_vec(sim.getObjectPosition(h, -1));
        d = hypot(q(1) - gW(1), q(2) - gW(2));
        if d < best
            best = d;  name = a;  p = q;  hBest = h;
            o = to_vec(sim.getObjectOrientation(h, -1));  yaw = o(3);
        end
    end
end

function wait_steps(sim_interface, T)
% espera T segundos de simulação
    t0 = sim_interface.get_simulation_time();
    while sim_interface.get_simulation_time() - t0 < T
        sim_interface.step_simulation();
    end
end

function F = finger_handles(sim, hGrasp)
% objetos da garra: tudo o que está pendurado no mesmo elo que o graspCenter
% (sobe 2 níveis para apanhar a garra inteira). Mostra os nomes uma vez e
% guarda as formas (shapes) que se mexem ao fechar, que são os dedos
    F = struct('h', {}, 'name', {});
    try
        h0 = sim.getObjectParent(hGrasp);
        h1 = sim.getObjectParent(h0);  if h1 ~= -1, h0 = h1; end
        all = to_vec(sim.getObjectsInTree(h0, sim.handle_all, 0));
    catch
        fprintf(' (não consegui ler a árvore da garra)\n');  return;
    end
    fprintf(' objetos da garra (%d):', numel(all));
    for h = all
        a = char(sim.getObjectAlias(h, 1));
        fprintf(' %s', a);
        if sim.getObjectType(h) == sim.object_shape_type
            F(end+1) = struct('h', h, 'name', a);                 %#ok<AGROW>
        end
    end
    fprintf('\n');
end

function [p, t] = in_grip(sim, hP, hG)
% posição da peça no referencial da garra [mm] e inclinação do seu eixo z
% para o x e para o y da garra [graus]
    m = to_vec(sim.getObjectMatrix(hP, hG));
    p = 1000*m([4 8 12]);
    z = m([3 7 11]);                                   % eixo z da peça visto da garra
    t = rad2deg([atan2(z(1), z(3)), atan2(z(2), z(3))]);
end

function P = finger_pos(sim, F, hRef)
% posição de cada dedo no referencial hRef [m]
    P = nan(numel(F), 3);
    for i = 1:numel(F)
        P(i, :) = to_vec(sim.getObjectPosition(F(i).h, hRef));
    end
end

function y = pYawOrNaN(name, yaw)
    if isempty(name), y = NaN; else, y = yaw; end
end

function v = to_vec(c)
% célula (ou vetor) de números -> vetor linha
    if iscell(c), v = cell2mat(c(:)'); else, v = double(c(:)'); end
end

% marca '<- VER' quando o erro passa da tolerância
function s = flag(e, tol)
    if e > tol, s = '<- VER'; else, s = 'OK'; end
end

function s = flag2(e, tol)
    if ~isnan(e) && e > tol, s = '  <- VER'; else, s = ''; end
end