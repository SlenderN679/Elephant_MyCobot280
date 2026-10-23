function [S, status] = copiar_pilha(action, S, ctx)
% COPIAR_PILHA  MODO 4: com UMA fotografia reconhece um modelo deitado na
%               mesa (fila de blocos encaixados = pilha na horizontal) e
%               constrói-o na vertical no alvo verde com outras peças das
%               mesmas cores (vision.copy_stack faz o plano).
%
%   [S, status] = modes.copiar_pilha('init', [], ctx)   % fotografia + plano
%   [S, status] = modes.copiar_pilha('step', S, ctx)    % um passo do plano
%
%   status : 'running'  - ainda há passos
%            'finished' - modelo copiado (já celebrou e voltou a casa)
%            'failed'   - sem modelo, faltam peças ou um passo falhou

    status = 'running';
    switch action
        case 'init'
            [~, ~, R] = vision.copy_stack();                % configuração por defeito
            R.colors = ctx.WATCH_COLORS;  R.tableZ = ctx.TABLE_Z;  R.blockH = ctx.BLOCK_H;
            R.base = ctx.BASE_POSITION(1:2);  R.target = ctx.POS(ctx.TARGET_GREEN, :);
            R.bottom = ctx.COPY_BOTTOM;  R.gap = ctx.COPY_GAP;
            R.reachFn = @(p) aux.can_reach(ctx.cfg, p);
            movement.go_home(SHAKE_HEAD, ctx.robot_arm, ctx.sim_interface, ctx.Links, ctx.qMin, ctx.qMax, ...
                             [], deg2rad(ctx.VISION_POSE));
            [plan, view, R] = vision.copy_stack(ctx.sim_interface, R);
            S = struct('plan', plan, 'view', view, 'R', R, 'iStep', 1, 'hPlan', ctx.hPlan);
            if ctx.SHOW_VISION == 1
                S.hPlan = show_copy(ctx, S);
            end
            if isempty(plan)
                fprintf('ERRO (copiar pilha): %s\n', R.why);
                aux.signal_failure(ctx.robot_arm, ctx.sim_interface, ctx.Links, ...
                                   ctx.qMin, ctx.qMax, ctx.FAIL_POSE);
                status = 'failed';
            end

        case 'step'
            if S.iStep <= numel(S.plan)
                st = S.plan(S.iStep);
                if ctx.SHOW_VISION == 1
                    S.hPlan = show_copy(ctx, S);
                end
                fprintf('\n--- copiar: passo %d de %d: peça %s -> pilha verde, nível %d ---\n', ...
                        S.iStep, numel(S.plan), st.color, st.level);
                yawC = [];
                if ctx.USE_VISION_YAW == 1 && ~isnan(st.yaw), yawC = st.yaw; end
                [ok, why] = aux.pick_and_place(ctx.robot_arm, ctx.sim_interface, ctx.cfg, ...
                                               st.pick, st.place, yawC, ctx.BLOCK_H/2);
                if ~ok
                    fprintf('ERRO no passo %d: %s\n', S.iStep, why);
                    aux.signal_failure(ctx.robot_arm, ctx.sim_interface, ctx.Links, ...
                                       ctx.qMin, ctx.qMax, ctx.FAIL_POSE);
                    status = 'failed';
                    return;
                end
                % a peça está agora na pilha: atualizar a vista (sem nova fotografia)
                k = st.blob;
                S.view.pos(k,:) = [st.place(1:2), st.place(3) + ctx.BLOCK_H/2];
                S.view.pix(k,:) = aux.world2pix(S.view.cam, [st.place(1:2), st.place(3) + ctx.BLOCK_H]);
                S.view.names{k} = sprintf('%s: pilha verde (nível %d)', st.color, st.level);
                S.iStep = S.iStep + 1;
            else
                % modelo copiado: mostrar, celebrar e voltar à pose inicial
                if ctx.SHOW_VISION == 1
                    S.hPlan = show_copy(ctx, S);
                end
                disp('copiar pilha: modelo copiado!');
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
% Visualização: o modelo (seta da base para o topo) e uma seta de cada peça
% escolhida para a pilha do alvo
% =========================================================================
function hOut = show_copy(ctx, S)
    hOut = S.hPlan;
    try
        V = S.view;  plan = S.plan;  R = S.R;  iStep = S.iStep;
        model = struct('from', {}, 'to', {}, 'color', {}, 'label', {}, 'state', {});
        if numel(V.model) >= 2
            a = V.model(1);  b = V.model(end);
            model(1) = struct('from', V.pos(a,:), 'to', V.pos(b,:), 'color', 'white', ...
                              'label', 'modelo: base -> topo', 'state', 'model');
        end
        steps = struct('from', {}, 'to', {}, 'color', {}, 'label', {});
        for i = 1:numel(plan)
            from = plan(i).pick;
            if i < iStep, from = plan(i).place; end          % já feita
            steps(end+1) = struct('from', from, 'to', plan(i).place, 'color', plan(i).color, ...
                                  'label', sprintf('%s -> nível %d', plan(i).color, plan(i).level)); %#ok<AGROW>
        end
        info = {'MODO 4: COPIAR PILHA (1 fotografia)'};
        if isempty(R.recipe)
            info{end+1} = 'modelo: não encontrado';
        else
            info{end+1} = ['modelo: ' strjoin(R.recipe, ' > ')];
        end
        if ~isempty(R.why), info{end+1} = ['ERRO: ' R.why]; end
        extras = vision.plan_view(V.cam, steps, iStep, info, model);
        hOut = drawing.show_vision(ctx.sim_interface, V.img, V.det, V.names, V.colors, V.pos, V.pix, ...
                   V.found, [], V.yaw, [], S.hPlan, ...
                   sprintf('Modo 4: passo %d de %d', min(iStep, numel(plan)), numel(plan)), extras);
    catch errS
        fprintf(2, 'AVISO show_copy: %s\n', errS.message);
    end
end