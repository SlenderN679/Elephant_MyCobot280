%% ======================================================================
% Apagar todos os desenhos da cena (zonas de alcance,
% marcas da visão). Os desenhos feitos pela API remota não desaparecem
% sozinhos quando a simulação para.
%======================================================================
function erase_drawings(sim_interface)
    try
        sim = sim_interface.get_connection().require('sim');
        sim.removeDrawingObject(sim.handle_all);
    catch
        %
    end
end