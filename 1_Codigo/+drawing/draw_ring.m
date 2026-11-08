function h = draw_ring(sim_interface, c, r, color, width)
% circunferência de raio r centrada em c 
    %
    %   h = drawing.draw_ring(sim_interface, c, r, color, width)
    %
    %   c : centro [x y ...] no MUNDO [m];  r : raio [m];  color : [r g b]
    %   width : espessura da linha [pixeis]
    %   h : identificador do desenho (para drawing.erase); 0 se falhar
    %
    % A circunferência é feita de 40 segmentos de reta.
    h = 0;
    try
        sim = drawing.get_sim(sim_interface);
        z = 0.007;  a = linspace(0, 2*pi, 41);
        h = double(sim.addDrawingObject(sim.drawing_lines, width, 0, -1, numel(a), color));
        for i = 1:numel(a) - 1
            sim.addDrawingObjectItem(h, [c(1) + r*cos(a(i))   c(2) + r*sin(a(i))   z, ...
                                         c(1) + r*cos(a(i+1)) c(2) + r*sin(a(i+1)) z]);
        end
    catch
    end
end