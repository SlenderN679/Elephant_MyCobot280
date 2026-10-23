function h = draw_cross(sim_interface, p, r, color)
% X por cima de um objeto
    h = 0;
    try
        sim = drawing.get_sim(sim_interface);
        z = max(2*p(3), 0) + 0.004;             % um pouco acima da face de cima
        h = double(sim.addDrawingObject(sim.drawing_lines, 5, 0, -1, 2, color));
        sim.addDrawingObjectItem(h, [p(1) - r p(2) - r z, p(1) + r p(2) + r z]);    % desenha 1/2 da cruz
        sim.addDrawingObjectItem(h, [p(1) - r p(2) + r z, p(1) + r p(2) - r z]);    % desenha 1/2 da cruz
    catch

    end
end