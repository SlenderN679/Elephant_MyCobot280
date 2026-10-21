
function pos = get_target_position_vision(sim_interface, color, n, zTop)
% GET_TARGET_POSITION_VISION  Versão "por visão" de sim_interface.get_target_position.
%
%   pos = get_target_position_vision(sim_interface, 'yellow')
%   pos = get_target_position_vision(sim_interface, 'blue', 2)   % 2.ª maior mancha azul
%
%   Devolve [x y z] no referencial do MUNDO (como get_target_position), mas
%   calculado SÓ a partir da imagem: rgb2hsv -> máscara da cor -> centro da
%   mancha -> raio da câmara cortado com o plano das peças (ver vision_colors.m).
%   n : (opcional) qual das manchas dessa cor, ordenadas da maior para a menor
%       (por defeito 1). Útil quando há duas peças da mesma cor.
%   zTop : (opcional) [ACRESCENTADO] altura [m] da face de cima da peça no
%          mundo, se for diferente do Z_PLANE de vision_colors (ex.: cubos de
%          4 cm). O ponto devolvido é então o da face de cima dessa peça.
%   pos = [] se não houver nenhuma peça dessa cor.
    if nargin < 3 || isempty(n), n = 1; end
    if nargin < 4, zTop = []; end
    det = vision_colors(sim_interface, color, true);
    if numel(det) < n
        pos = [];  return;
    end
    [~, order] = sort([det.area], 'descend');
    pos = det(order(n)).world;
    if ~isempty(zTop)                       % recalcular o ponto na altura zTop
        c = det(order(n)).cam;
        pos = c + (zTop - c(3)) / (pos(3) - c(3)) * (pos - c);
    end
end