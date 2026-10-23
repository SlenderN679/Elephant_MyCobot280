function erase(sim_interface, handles)
%ERASE Summary of this function goes here

% apagar desenhos
    try
        sim = drawing.get_sim(sim_interface);
        for h = handles(handles > 0)
            sim.removeDrawingObject(h);
        end
    catch
    end
end