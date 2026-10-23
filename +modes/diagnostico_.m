function [S, status] = diagnostico(action, S, ctx)
% DIAGNOSTICO  MODO 0: diagnóstico de cinemática (só no simulador).
%              Leva a garra, a apontar para baixo, 6 cm acima de cada objeto
%              e compara o modelo com o simulador:
%                - juntas pedidas (inverse_kin) vs. juntas alcançadas;
%                - centro de preensão segundo o modelo (direct_kin, com as
%                  juntas alcançadas) vs. o /graspCenter real do CoppeliaSim;
%                - ponto pedido vs. ponto alcançado.
%              No fim escreve um resumo com os erros e volta à pose inicial.
%
%   [S, status] = modes.diagnostico('init', [], ctx)
%   [S, status] = modes.diagnostico('step', S, ctx)    % um objeto por chamada
%
%   status : 'running' ou 'finished'
%
% Usa informação que só o simulador tem (/graspCenter, posições reais dos
% objetos): serve para validar direct_kin / inverse_kin / world_to_base,
% não para o braço real. Substitui o antigo aux.diag_kinematics.

    UP     = 0.06;          % altura acima do centro de cada objeto [m]
    SPEED  = deg2rad(30);   % velocidade das juntas [rad/s]
    SETTLE = 40;            % passos de simulação para estabilizar (~2 s)
    TOL_MM = 5;             % erro [mm] acima do qual se escreve "VER"

    status = 'running';
    switch action
        case 'init'
            sim = ctx.sim_interface.get_connection().require('sim');
            S.hGrasp = sim.getObject('/graspCenter');
            S.hBase  = sim.getObject(ctx.robot_name);
            S.k = 1;                                 % próximo objeto
            S.res = nan(numel(ctx.OBJ_NAME), 3);     % erros [mm]: modelo, alcance, juntas [graus]
            fprintf('\n=== MODO 0: DIAGNÓSTICO DE CINEMÁTICA (%d objetos) ===\n', numel(ctx.OBJ_NAME));

        case 'step'
            sim = ctx.sim_interface.get_connection().require('sim');
            n = numel(ctx.OBJ_NAME);
            if S.k <= n
                k = S.k;  S.k = S.k + 1;
                name = ctx.OBJ_NAME{k};
                pW = ctx.sim_interface.get_target_position(k);              % posição real (MUNDO)
                p  = aux.world_to_base(pW, ctx.BASE_POSITION, ctx.BASE_YAW) + [0; 0; UP];
                [q, ok] = movement.inverse_kin(p(1), p(2), p(3), ctx.Links, ctx.qMin, ctx.qMax);
                fprintf('\n== %d. %s ==\n', k, name);
                if ~ok
                    fprintf(' inalcançável (a %.0f cm do eixo da base)\n', 100*hypot(p(1), p(2)));
                    return;
                end
                % ir pela vertical (evita varrer a mesa) e depois até ao ponto
                movement.go_home(SHAKE_HEAD, ctx.robot_arm, ctx.sim_interface, ctx.Links, ctx.qMin, ctx.qMax, ...
                                 [], zeros(1, 6));
                movement.move_joints(ctx.robot_arm, ctx.sim_interface, q, SPEED);
                for s = 1:SETTLE
                    ctx.sim_interface.step_simulation();
                end

                qr = ctx.robot_arm.get_joints();
                dk = movement.direct_kin(qr, ctx.Links);              % modelo (base)
                gB = to_vec(sim.getObjectPosition(S.hGrasp, S.hBase));  % simulador (base)
                gW = to_vec(sim.getObjectPosition(S.hGrasp, -1));       % simulador (mundo)
                eModel = 1000*norm(dk(:) - gB(:));                    % modelo vs. simulador
                eReach = 1000*norm(p(:) - gB(:));                     % pedido vs. alcançado
                eJoint = rad2deg(max(abs(qr(:) - q(:))));
                S.res(k, :) = [eModel, eReach, eJoint];

                fprintf(' objetivo (base)      : %7.4f %7.4f %7.4f\n', p);
                fprintf(' q pedido    [graus]  : %s\n', sprintf('%7.1f', rad2deg(q)));
                fprintf(' q alcançado [graus]  : %s\n', sprintf('%7.1f', rad2deg(qr)));
                fprintf(' direct_kin (base)    : %7.4f %7.4f %7.4f\n', dk(1:3));
                fprintf(' graspCenter (base)   : %7.4f %7.4f %7.4f   <- simulador\n', gB);
                fprintf(' graspCenter (mundo)  : %7.4f %7.4f %7.4f\n', gW);
                fprintf(' world_to_base(mundo) : %7.4f %7.4f %7.4f\n', ...
                        aux.world_to_base(gW, ctx.BASE_POSITION, ctx.BASE_YAW));
                fprintf(' erros: modelo %.1f mm %s | alcance %.1f mm %s | juntas %.1f graus\n', ...
                        eModel, flag(eModel, TOL_MM), eReach, flag(eReach, TOL_MM), eJoint);
            else
                % resumo
                fprintf('\n=== RESUMO DO DIAGNÓSTICO ===\n');
                fprintf('%-14s %12s %12s %12s\n', 'objeto', 'modelo [mm]', 'alcance [mm]', 'juntas [°]');
                for k = 1:n
                    if isnan(S.res(k, 1))
                        fprintf('%-14s %12s\n', ctx.OBJ_NAME{k}, 'inalcançável');
                    else
                        fprintf('%-14s %12.1f %12.1f %12.1f\n', ctx.OBJ_NAME{k}, S.res(k, :));
                    end
                end
                r = S.res(~isnan(S.res(:, 1)), :);
                if ~isempty(r)
                    fprintf('%-14s %12.1f %12.1f %12.1f\n', 'MÁXIMO', max(r, [], 1));
                    fprintf(['modelo  = direct_kin vs. graspCenter real (erro do modelo do braço)\n' ...
                             'alcance = ponto pedido vs. graspCenter real (inclui o atraso das juntas)\n']);
                end
                movement.go_home(SHAKE_HEAD, ctx.robot_arm, ctx.sim_interface, ctx.Links, ctx.qMin, ctx.qMax, ...
                                 [], deg2rad(ctx.HOME_POSE));
                status = 'finished';
            end
    end
end

% -------------------------------------------------------------------------
function v = to_vec(c)
% célula (ou vetor) de números -> vetor linha
    if iscell(c), v = cell2mat(c(:)'); else, v = double(c(:)'); end
end

function s = flag(e, tol)
    if e > tol, s = '<- VER'; else, s = 'OK'; end
end