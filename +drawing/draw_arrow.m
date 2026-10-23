function h = draw_arrow(sim_interface, p0, p1, color)
% desenha uma seta entre 2 pontos
    h = 0;
    try
        sim = drawing.get_sim(sim_interface);
        z = 0.008;              % altura baixa acima do chão
        d = p1(1:2) - p0(1:2);  % vetor distancia entre os pontos
        L = norm(d);            % distancia entre vetores
        if L < 1e-6             % se a distancia for pequena não é desenhado
            return; 
        end
        u = d / L;              % fator para normalizar o vetor
        nrm = [-u(2) u(1)];     % vetor normalizado oposto
        tip = 0.02;             % tamanho da ponta
        b = p1(1:2) - tip*u;    % base da ponta
        h = double(sim.addDrawingObject(sim.drawing_lines, 4, 0, -1, 3, color));
        sim.addDrawingObjectItem(h, [p0(1:2) z, p1(1:2) z]);            % desenha o corpo da seta
        sim.addDrawingObjectItem(h, [p1(1:2) z, b + 0.6*tip*nrm z]);    % desenha 1/2 da ponta
        sim.addDrawingObjectItem(h, [p1(1:2) z, b - 0.6*tip*nrm z]);    % desenha 1/2 da ponta
    catch
    end
end