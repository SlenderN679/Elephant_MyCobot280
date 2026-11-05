function p = at_height(d, z)
% AT_HEIGHT  Ponto onde o raio da câmara que passa pela mancha d corta a altura z.
%
%   p = vision.at_height(d, z)
%
%   d : uma deteção de vision_colors (usa d.cam = câmara e d.world = ponto do
%       mesmo raio à altura Z_PLANE de vision_colors)
%   z : altura [m] no MUNDO (ex.: a face de cima da peça)
%   p : 1 x 3 ponto [x y z] no MUNDO
%
% Porquê: vision_colors calcula sempre o ponto à altura de um bloco (Z_PLANE).
% Uma peça mais alta (cubo, pilha) está no MESMO raio, mas mais acima: andar
% ao longo do raio câmara -> ponto até à altura certa.
    c = d.cam(:)';  w = d.world(:)';
    p = c + (z - c(3)) / (w(3) - c(3)) * (w - c);
end