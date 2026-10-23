function h = draw_disc(sim_interface, c, r, color)
% disco semitransparente de raio r centrado em c (na mesa)
    h = 0;
    try
        sim = drawing.get_sim(sim_interface);
        z = 0.006;  a = linspace(0, 2*pi, 41);
        h = double(sim.addDrawingObject(bitor(sim.drawing_triangles, sim.drawing_50percenttransparency), ...
                                        1, 0, -1, numel(a), color));
        for i = 1:numel(a) - 1
            sim.addDrawingObjectItem(h, [c(1) c(2) z, c(1) + r*cos(a(i)) c(2) + r*sin(a(i)) z, ...
                                         c(1) + r*cos(a(i+1)) c(2) + r*sin(a(i+1)) z]);
        end
    catch
    end
end