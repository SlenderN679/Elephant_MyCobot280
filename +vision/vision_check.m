function det = vision_check(sim_interface, wanted)
% VISION_CHECK  Testar a visão (SÓ no simulador): tira uma fotografia com
%               vision_colors, compara cada mancha com a posição real dos
%               objetos do cenário e mostra a imagem com as deteções.
%
%   vision.vision_check()                  % liga ao CoppeliaSim, arranca, mede e pára
%   vision.vision_check(sim_interface)     % usa a ligação já aberta
%   vision.vision_check([], 'red')         % só uma cor (ou {'red','blue'})
%
% Antes de correr sozinho: abrir mycobot280_vision.ttt no CoppeliaSim (sem
% correr a simulação) e pôr o braço fora da vista da câmara.
%
% Escreve, para cada mancha: cor, área, tamanho e forma da face de cima,
% posição estimada, objeto real mais perto e o erro (OK até TOL_MM).
% O tamanho e a forma servem para preencher OBJ_SIZE / OBJ_SHAPE no
% main_program (separar objetos da mesma cor).
% No fim verifica se outro SIGN do modelo da câmara daria erros menores.
% (O código de controlo nunca usa as posições reais: isto é só um teste.)

    TOL_MM      = 10;      % erro [mm] acima do qual se escreve FALHOU
    ROUND_FILL  = 0.88;    % fill abaixo disto = face redonda (igual a vision_snapshot)
    TRUTH_NAMES = {'BlueCube','BlueCylinder','RedCube','TargetGreen','Block1','Block2','Block3','Block4'};
    if nargin < 2, wanted = []; end

    % ---- ligação ao simulador (se não foi dada) ----
    if nargin < 1 || isempty(sim_interface)
        [sim_interface, err] = simulator_interface(0);
        if err == 1, det = []; return; end
        cleanup = onCleanup(@() sim_interface.terminate());   %#ok<NASGU>
        sim_interface.start_simulation();
    end

    % ---- fotografia ----
    [det, img, cam] = vision.vision_colors(sim_interface, wanted);
    fprintf('Sensor: %dx%d px, posição [%.3f %.3f %.3f]\n', cam.W, cam.H, cam.pc);
    if isempty(det)
        fprintf('Nenhuma peça encontrada (ver limiares de cor / MIN_AREA em vision_colors).\n');
        return;
    end

    % ---- posições reais no simulador ----
    nT = min(sim_interface.TARGET_Number, numel(TRUTH_NAMES));
    T = zeros(nT, 3);
    for k = 1:nT
        T(k, :) = sim_interface.get_target_position(k);
    end

    % ---- deteções vs real ----
    fprintf('\n%-7s %6s %8s %-16s %-19s %-13s %8s\n', 'cor', 'área', 'tamanho', 'forma', ...
            'estimado x y [mm]', 'objeto', 'erro');
    used = false(nT, 1);
    for i = 1:numel(det)
        d = det(i);
        [dmin, j] = min(hypot(T(:,1) - d.world(1), T(:,2) - d.world(2)));
        used(j) = true;
        if dmin*1000 <= TOL_MM, verdict = 'OK'; else, verdict = 'FALHOU'; end
        if isnan(d.fill),            shape = '?';
        elseif d.fill < ROUND_FILL,  shape = sprintf('redonda (%.2f)', d.fill);
        else,                        shape = sprintf('quadrada (%.2f)', d.fill);
        end
        fprintf('%-7s %6d %5.1f mm %-16s [%7.1f %7.1f]   %-13s %5.1f mm %s\n', ...
                d.color, d.area, 1000*d.size, shape, 1000*d.world(1:2), TRUTH_NAMES{j}, ...
                1000*dmin, verdict);
    end
    for j = find(~used)'
        fprintf('NÃO DETETADO: %s (real x = %.1f, y = %.1f mm)\n', TRUTH_NAMES{j}, 1000*T(j,1:2));
    end
    fprintf(['(tamanho medido à altura de um bloco; o erro também: peças mais altas\n' ...
             ' que um bloco aparecem um pouco deslocadas, o vision_snapshot corrige isso)\n']);

    % ---- diagnóstico dos sinais do modelo da câmara ----
    combos = [-1 -1; 1 -1; -1 1; 1 1];
    score = zeros(4, 1);
    for c = 1:4
        cc = cam;  cc.sgn = combos(c, :);
        for i = 1:numel(det)
            P = vision.cam_map(cc, det(i).pixel, det(i).world(3));
            if any(isnan(P)), score(c) = score(c) + 1; continue; end
            score(c) = score(c) + min(hypot(T(:,1) - P(1), T(:,2) - P(2)));
        end
    end
    score = score / numel(det);
    cur = all(combos == cam.sgn, 2);
    [~, best] = min(score);
    if ~cur(best) && score(best) < 0.5*score(cur)
        fprintf('\nNOTA: com SIGN = [%d %d] os erros ficam muito menores (%.1f mm em vez de %.1f mm). Experimenta mudar SIGN em vision_colors.\n', ...
                combos(best,:), 1000*score(best), 1000*score(cur));
    end

    % ---- figura ----
    figure('Name', 'vision_check'); image(img); axis image; hold on;
    for i = 1:numel(det)
        plot(det(i).pixel(1) + 0.5, det(i).pixel(2) + 0.5, 'k+', 'MarkerSize', 12, 'LineWidth', 2);
        text(det(i).pixel(1) + 8, det(i).pixel(2), ...
             sprintf('%s  %.1f mm\n(%.0f, %.0f) mm', det(i).color, 1000*det(i).size, ...
                     1000*det(i).world(1), 1000*det(i).world(2)), ...
             'Color', 'w', 'BackgroundColor', 'k', 'FontSize', 8, 'Interpreter', 'none');
    end
    title('Peças localizadas por cor'); drawnow;
end