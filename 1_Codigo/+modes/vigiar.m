function [S, status] = vigiar(action, S, ctx, kind)
% VIGIAR  Parte comum dos MODOS 2 e 3: o braço fica fora da vista da câmara
%         (VISION_POSE) e tira uma fotografia a cada ctx.WATCH_PERIOD
%         segundos (vision.watch_pairs decide o que fazer).
%
%   [S, status] = modes.vigiar('init', [], ctx, kind)
%   [S, status] = modes.vigiar('step', S, ctx, kind)
%
%   kind   : 'pairs'   - pilha de pares (modes.pilha_pares)
%            'request' - pedido de cor  (modes.pedido_cor)
%   status : sempre 'running' (vigia até se parar a simulação)
%
% Chamado por modes.pilha_pares e modes.pedido_cor; não é um modo sozinho.

    status = 'running';
    switch action
        case 'init'
            % INIT: configurar o vigia (vision.watch_pairs) e pôr o braço fora da vista
            [~, W] = vision.watch_pairs();                   % estado inicial
            W.colors = ctx.WATCH_COLORS;  W.tableZ = ctx.TABLE_Z;  W.blockH = ctx.BLOCK_H;
            W.base = ctx.BASE_POSITION(1:2);                 % para escolher o bloco mais perto do robô
            W.reachFn = @(p) aux.can_reach(ctx.cfg, p);      % só apanhar peças que o robô alcança
            W.mode = kind;
            W.placeExtra = 0.005 + (ctx.cfg.DROP_ON_PLACE == 1)*ctx.cfg.DROP_HEIGHT;   % como em pick_and_place
            W.target = ctx.POS(ctx.TARGET_GREEN, :);         % pedido de cor: pilha no alvo verde
            movement.go_home(ctx.SHAKE, ctx.robot_arm, ctx.sim_interface, ctx.Links, ctx.qMin, ctx.qMax, ...
                             [], deg2rad(ctx.VISION_POSE));
            S = struct('W', W, 'lastWatch', -inf, 'hWatch', [], 'topColor', 'green');
            disp('modo vigiar: à espera de blocos novos da mesma cor...');

        case 'step'
            % STEP: 1. só tira fotografia a cada WATCH_PERIOD segundos
            tW = ctx.sim_interface.get_simulation_time();
            if tW - S.lastWatch < ctx.WATCH_PERIOD, return; end   % ainda não é hora da fotografia
            S.lastWatch = tW;

            % 2. fotografia: watch_pairs compara com as anteriores e devolve uma ação
            %    (act) se apareceu um par / pedido, ou [] se não há nada a fazer
            [act, S.W, view] = vision.watch_pairs(ctx.sim_interface, S.W);
            % mostrar cada fotografia (figura + marcas na mesa)
            if ctx.SHOW_VISION == 1 && ~isempty(view)
                try
                    S.hWatch = drawing.show_vision(ctx.sim_interface, view.img, view.det, view.names, ...
                                 view.colors, view.pos, view.pix, view.found, [], view.yaw, [], ...
                                 S.hWatch, sprintf('Modo %d (%s): t = %.1f s, %d blocos', ...
                                 ctx.MODE, ctx.MODE_NAME, tW, numel(view.names)), view.extras);
                catch errS
                    fprintf(2, 'AVISO show_vision: %s\n', errS.message);
                end
            end
            if isempty(act), return; end

            % 3. executar a ação: apanhar e pousar; depois avisar watch_pairs do
            %    resultado e voltar a pôr o braço fora da vista
            if strcmp(kind, 'pairs')
                fprintf('\n--- par %s: bloco em [%.3f %.3f] -> em cima de [%.3f %.3f] (nível %d) ---\n', ...
                        act.color, act.pick(1:2), act.place(1:2), act.level);
            else
                fprintf('\n--- pedido %s (bloco em [%.3f %.3f]): peça em [%.3f %.3f] -> pilha verde, nível %d ---\n', ...
                        act.color, act.req, act.pick(1:2), act.level);
            end
            yawW = [];
            if ctx.USE_VISION_YAW == 1 && ~isnan(act.yaw), yawW = act.yaw; end
            % cor do topo da pilha de destino (só essa cor conta antes de pousar):
            % pedido de cor - a última peça pousada no alvo (verde se vazio);
            % pares - um bloco da mesma cor
            if strcmp(S.W.mode, 'request'), topW = S.topColor; else, topW = act.color; end
            [okW, whyW, placeUsed] = aux.pick_and_place(ctx.robot_arm, ctx.sim_interface, ctx.cfg, ...
                                             act.pick, act.place, yawW, ctx.BLOCK_H/2, topW, act.color);
            if okW && norm(placeUsed(1:2) - act.place(1:2)) > 0
                % a pilha tinha sido mexida: guardar onde ficou
                if strcmp(S.W.mode, 'request')
                    S.W.target(1:2) = S.W.target(1:2) + placeUsed(1:2) - act.place(1:2);
                end
                act.place(1:2) = placeUsed(1:2);
            end
            if okW && strcmp(S.W.mode, 'request'), S.topColor = act.color; end
            if ~okW
                % não deu (ex.: fora do alcance): avisar, ignorar esse bloco e continuar
                fprintf('AVISO: não consegui levar a peça %s para a pilha (%s)\n', act.color, whyW);
            end
            [~, S.W] = vision.watch_pairs([], S.W, act, okW);
            % voltar para fora da vista da câmara antes da próxima fotografia
            movement.go_home(ctx.SHAKE, ctx.robot_arm, ctx.sim_interface, ctx.Links, ctx.qMin, ctx.qMax, ...
                             [], deg2rad(ctx.VISION_POSE));
            S.lastWatch = ctx.sim_interface.get_simulation_time();
    end
end