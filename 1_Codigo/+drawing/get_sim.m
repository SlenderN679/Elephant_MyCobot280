function sim = get_sim(sim_interface)
% GET_SIM  Objeto 'sim' da API remota do CoppeliaSim, usado
%          pelas funções de desenho da pasta +drawing:
%          sim = drawing.get_sim(sim_interface)
    sim = sim_interface.get_connection().require('sim');
end