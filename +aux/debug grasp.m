function debug_grasp(sim_interface, robot_arm, cfg, pGraspBase, yaw)
% DEBUG_GRASP  (SÓ NO SIMULADOR, para testes) Com a garra em baixo, pronta a
%              fechar, compara onde está a garra com onde está o bloco mais
%              perto, usando as posições reais do CoppeliaSim.
%
%   aux.debug_grasp(sim_interface, robot_arm, cfg, pGraspBase, yaw)
%
% Escreve: o ponto pedido, o /graspCenter real, o bloco (forma cujo nome
% começa por "Block") mais perto e a diferença garra - bloco em x, y, z, mais
% a rotação real do bloco e a pedida. O controlo nunca usa nada disto.
    try
        sim = sim_interface.get_connection().require('sim');
        c = @(v) double(cell2mat_safe(v));
        % ponto pedido (base -> mundo)
        b = cfg.BASE_YAW;  pB = pGraspBase(:)';
        want = cfg.BASE_POSITION(:)' + [cos(b)*pB(1) - sin(b)*pB(2), sin(b)*pB(1) + cos(b)*pB(2), pB(3)];
        g = c(sim.getObject('/graspCenter'));
        gW = c(sim.getObjectPosition(g, -1));
        % bloco mais perto da garra
        shapes = c(sim.getObjectsInTree(sim.handle_scene, sim.object_shape_type));
        best = [];  bd = inf;  name = '';
        for h = shapes(:)'
            a = char(sim.getObjectAlias(h, 1));
            if isempty(regexp(a, '/Block', 'once')), continue; end
            p = c(sim.getObjectPosition(h, -1));
            d = hypot(p(1) - gW(1), p(2) - gW(2));
            if d < bd, bd = d;  best = h;  name = a;  bp = p; end
        end
        fprintf('DEBUG garra: pedido [%.4f %.4f %.4f], graspCenter real [%.4f %.4f %.4f] (erro %.1f mm)\n', ...
                want, gW, 1000*norm(want - gW));
        if ~isempty(best)
            o = c(sim.getObjectOrientation(best, -1));
            fprintf('DEBUG garra: bloco mais perto %s em [%.4f %.4f %.4f]; garra - bloco = (%.1f, %.1f, %.1f) mm; yaw real do bloco %.1f°', ...
                    name, bp, 1000*(gW - bp), rad2deg(o(3)));
            if ~isempty(yaw), fprintf(', yaw pedido (base) %.1f°', rad2deg(yaw)); end
            q = robot_arm.get_joints();
            fprintf(', J6 %.1f°\n', rad2deg(q(6)));
        end
    catch err
        fprintf('DEBUG garra: não consegui ler o simulador (%s)\n', err.message);
    end
end

function v = cell2mat_safe(x)
    if iscell(x), v = cell2mat(x(:)'); else, v = x(:)'; end
end