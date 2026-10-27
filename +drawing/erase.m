function erase(sim_interface, handles)
% ERASE  Apaga os desenhos indicados (identificadores devolvidos pelas funções
%        draw_*). Ex.: drawing.erase(sim_interface, [hZone hMove])

    try
        sim = drawing.get_sim(sim_interface);
        for h = handles(handles > 0)
            sim.removeDrawingObject(h);
        end
    catch
    end
end