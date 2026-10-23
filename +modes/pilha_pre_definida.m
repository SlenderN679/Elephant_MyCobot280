function [S, status] = pilha_pre_definida(action, S, ctx)
% PILHA_PRE_DEFINIDA  MODO 1: executa a lista TASKS (empilhar no alvo verde),
%                     uma tarefa por chamada de 'step'.
%
%   [S, status] = modes.pilha_pre_definida('init', [], ctx)
%   [S, status] = modes.pilha_pre_definida('step', S, ctx)
%
%   ctx    : contexto criado em main_program (robô, simulador, cfg, objetos,
%            posições guardadas POS/YAW, fotografia inicial VIS, opções)
%   S      : estado do modo (tarefa seguinte, posições atualizadas, desenhos)
%   status : 'running'  - ainda há trabalho
%            'finished' - todas as tarefas feitas (já celebrou e voltou a casa)
%            'failed'   - uma tarefa falhou (já assinalou a falha)
%
% As posições vêm de ctx.POS (câmara ou simulador, lidas uma vez no início)
% e são atualizadas aqui sempre que uma peça muda de sítio.

    status = 'running';
    switch action
        case 'init'
            S = struct('iTask', 1, 'POS', ctx.POS, 'hPlan', ctx.hPlan);

        case 'step'
            T = ctx.TASKS;
            if S.iTask <= size(T, 1)
                if ctx.SHOW_VISION == 1                 % plano das TASKS
                    S.hPlan = show_tasks(ctx, S);
                end
                obj = T(S.iTask, 1);
                fprintf('\n--- tarefa %d de %d ---\n', S.iTask, size(T, 1));

                % onde está o objeto e onde o pousar (referencial do MUNDO)
                objPos   = S.POS(obj, :);
                placePos = S.POS(T(S.iTask, 2), :) ...
                           + [T(S.iTask, 3), T(S.iTask, 4), (T(S.iTask, 5) - 1)*ctx.BLOCK_H];

                % orientação do cubo (para alinhar os dedos com as faces)
                objYaw = [];
                if ctx.IS_CUBE(obj) && ctx.USE_VISION == 1
                    if ~isnan(ctx.YAW(obj)), objYaw = ctx.YAW(obj); end   % yaw da câmara
                elseif ctx.IS_CUBE(obj) && ctx.USE_OBJECT_YAW
                    o = ctx.sim_interface.get_target_orientation(obj);
                    objYaw = o(3);
                end

                [okT, whyT] = aux.pick_and_place(ctx.robot_arm, ctx.sim_interface, ctx.cfg, ...
                                                 objPos, placePos, objYaw, ctx.HALF_H(obj));
                if ~okT
                    fprintf('ERRO na tarefa %d: %s\n', S.iTask, whyT);
                    aux.signal_failure(ctx.robot_arm, ctx.sim_interface, ctx.Links, ...
                                       ctx.qMin, ctx.qMax, ctx.FAIL_POSE);
                    status = 'failed';
                    return;
                end
                % a peça está agora no destino (guardar o seu centro)
                S.POS(obj, :) = [placePos(1:2), ctx.TABLE_Z + (T(S.iTask, 5) - 1)*ctx.BLOCK_H + ctx.HALF_H(obj)];
                S.iTask = S.iTask + 1;
            else
                % todas as tarefas feitas: mostrar, celebrar e voltar à pose inicial
                if ctx.SHOW_VISION == 1
                    S.hPlan = show_tasks(ctx, S);
                end
                if ctx.CELEBRATE == 1
                    aux.celebrate(ctx.robot_arm, ctx.sim_interface, ctx.Links, ctx.qMin, ctx.qMax);
                end
                movement.go_home(SHAKE_HEAD, ctx.robot_arm, ctx.sim_interface, ctx.Links, ctx.qMin, ctx.qMax, ...
                                 [], deg2rad(ctx.HOME_POSE));
                status = 'finished';
            end
    end
end

% =========================================================================
% Visualização: plano das TASKS sobre a fotografia inicial (as marcas seguem
% as posições guardadas em S.POS: uma peça já empilhada aparece no alvo,
% mesmo que a imagem seja a de antes)
% =========================================================================
function hOut = show_tasks(ctx, S)
    hOut = S.hPlan;
    try
        POS = S.POS;  T = ctx.TASKS;  iTask = S.iTask;
        n = size(POS, 1);
        top = [POS(:,1:2), POS(:,3) + ctx.HALF_ALL(:)];      % face de cima de cada objeto
        pix = aux.world2pix(ctx.VIS.cam, top);
        found = all(~isnan(POS), 2);
        names = ctx.OBJ_NAME(1:n);
        steps = struct('from', {}, 'to', {}, 'color', {}, 'label', {});
        for i = 1:size(T, 1)
            obj = T(i, 1);  tgt = T(i, 2);
            to = POS(tgt, :) + [T(i, 3), T(i, 4), (T(i, 5) - 1)*ctx.BLOCK_H];
            from = POS(obj, :);
            if i < iTask, from = to; end                     % já feita: a peça está no destino
            steps(end+1) = struct('from', from, 'to', to, 'color', ctx.OBJ_COLOR{obj}, ...
                                  'label', sprintf('%s -> nível %d', ctx.OBJ_NAME{obj}, T(i, 5))); %#ok<AGROW>
            if i < iTask, names{obj} = sprintf('%s (nível %d)', ctx.OBJ_NAME{obj}, T(i, 5)); end
        end
        if ctx.USE_VISION == 1, src = 'câmara'; else, src = 'simulador'; end
        info = {'MODO 1: PILHA PRÉ-DEFINIDA', sprintf('posições: %s', src)};
        extras = vision.plan_view(ctx.VIS.cam, steps, iTask, info);
        hOut = drawing.show_vision(ctx.sim_interface, ctx.VIS.img, ctx.VIS.det, names, ...
                   ctx.OBJ_COLOR(1:n), POS, pix, found, ctx.VIS.truth, ctx.YAW, ctx.VIS.truthYaw, ...
                   S.hPlan, sprintf('Modo 1: tarefa %d de %d', min(iTask, size(T, 1)), size(T, 1)), extras);
    catch errS
        fprintf(2, 'AVISO show_tasks: %s\n', errS.message);
    end
end