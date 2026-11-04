function out = cam_map(cam, X, z)
% CAM_MAP  Modelo da câmara nos dois sentidos (pinhole):
%          MUNDO -> pixel  e  pixel -> MUNDO.
%
%   uv = vision.cam_map(cam, P)        % pontos do MUNDO -> pixeis
%   P  = vision.cam_map(cam, uv, z)    % pixeis -> pontos do MUNDO à altura z
%
%   cam : modelo da câmara (W, H, f, Rc, pc, sgn), devolvido por vision_colors
%   P   : N x 3 pontos [x y z] no MUNDO [m]
%   uv  : N x 2 pixeis [coluna linha], na mesma convenção que det.pixel
%   z   : altura do plano no MUNDO [m] (escalar ou um valor por pixel)
%   NaN onde não há resposta (ponto atrás da câmara / raio que não corta o plano)
%
% Câmara do CoppeliaSim: olha segundo +z do sensor, +y do sensor aponta para o
% topo da imagem e +x do sensor aponta para a ESQUERDA da imagem (sgn = [-1 -1]).
% Um pixel (u, v) corresponde à direção do sensor
%     ray = [sgn(1)*(u - W/2)/f ; sgn(2)*(v - H/2)/f ; 1]
% e os dois sentidos são esta mesma conta, para a frente e para trás.

    if nargin < 3
        % ---- MUNDO -> pixel ----
        % passar para o referencial da câmara (Rc' roda o mundo para a câmara),
        % dividir pela profundidade r(3) (perspetiva) e converter em pixeis
        % com a distância focal f, a partir do centro da imagem
        out = nan(size(X, 1), 2);
        if isempty(cam) || isempty(X), return; end
        r = cam.Rc' * (X' - cam.pc(:));              % 3 x N, referencial do sensor
        ok = r(3,:) > 0;                             % só pontos à frente da câmara
        out(ok, :) = [cam.W/2 + cam.sgn(1)*cam.f*r(1,ok)./r(3,ok); ...
                      cam.H/2 + cam.sgn(2)*cam.f*r(2,ok)./r(3,ok)]';
    else
        % ---- pixel -> MUNDO à altura z ----
        % 1. direção do raio no referencial do sensor   2. rodar para o mundo (Rc)
        % 3. andar ao longo do raio até à altura z
        u = X(:,1)';  v = X(:,2)';
        ray = [cam.sgn(1)*(u - cam.W/2)/cam.f; cam.sgn(2)*(v - cam.H/2)/cam.f; ones(1, numel(u))];
        d   = cam.Rc * ray;                          % direções dos raios no mundo
        t   = (z(:)' - cam.pc(3)) ./ d(3,:);
        t(abs(d(3,:)) < 1e-9 | t <= 0) = NaN;        % paralelo ao plano ou plano atrás da câmara
        out = (cam.pc(:) + t .* d)';
    end
end