function compare_vision_vs_reference()
% COMPARE_VISION_VS_REFERENCE  Compara a posição obtida por VISÃO (rgb -> hsv -> cor)
% com a posição de REFERÊNCIA do simulador (sim_interface.get_target_position),
% objeto a objeto, tal como se faz no main_program.
%
% Abrir mycobot280_vision.ttt (sem correr a simulação) e correr este ficheiro.
% Editar OBJ_COLOR: a cor de cada objeto, pela ordem de TargetNames
% (BlueCube, BlueCylinder, RedCube, TargetGreen, Block1, Block2, Block3).
    OBJ_NAMES = {'BlueCube','BlueCylinder','RedCube','TargetGreen','Block1','Block2','Block3'};
    OBJ_COLOR = {'blue','blue','red','green','yellow','pink','orange'};   % <- confirmar nos blocos
    TOL_MM    = 10;
    
    [sim_interface, err] = simulator_interface(0);
    if err == 1, return; end
    cleanup = onCleanup(@() sim_interface.terminate()); %#ok<NASGU>
    sim_interface.start_simulation();
    
    % uma única captura com todas as cores
    det = vision_colors(sim_interface, unique(OBJ_COLOR), true);
    
    fprintf('\n%-13s %-7s %-20s %-20s %-14s %s\n', 'objeto', 'cor', 'visão x y [mm]', ...
        'referência x y [mm]', 'dx dy [mm]', 'erro');
    
    for k = 1:numel(OBJ_NAMES)
        ref = sim_interface.get_target_position(k);          % referência (simulador)
        cand = det(strcmp({det.color}, OBJ_COLOR{k}));
        if isempty(cand)
            fprintf('%-13s %-7s NÃO DETETADO   referência [%7.1f %7.1f]\n', ...
                OBJ_NAMES{k}, OBJ_COLOR{k}, 1000*ref(1:2));
            continue;
        end
        % se houver várias manchas da mesma cor (ex.: cubo e cilindro azuis),
        % usa-se a referência APENAS para escolher qual é qual
        W = vertcat(cand.world);
        [~, j] = min(hypot(W(:,1) - ref(1), W(:,2) - ref(2)));
        est = W(j, :);
        e = 1000*hypot(est(1) - ref(1), est(2) - ref(2));
        if e <= TOL_MM, v = 'OK'; else, v = 'FALHOU'; end
        
        fprintf('%-13s %-7s [%7.1f %7.1f]    [%7.1f %7.1f]    [%5.1f %5.1f]   %5.1f mm  %s\n', ...
            OBJ_NAMES{k}, OBJ_COLOR{k}, 1000*est(1:2), 1000*ref(1:2), ...
            1000*(est(1:2) - ref(1:2)), e, v);
    end
end