function uv = world2pix(cam, P)
% WORLD2PIX  Ponto(s) do MUNDO -> pixel [coluna linha] da imagem
%            da câmara (o inverso de pix2world em vision_colors.m).
%
%   uv = world2pix(cam, P)
%
%   cam : modelo da câmara devolvido por vision_colors / vision_snapshot
%   P   : N x 3 pontos [x y z] no MUNDO [m]
%   uv  : N x 2 pixeis [coluna linha], na mesma convenção que det.pixel
%         (NaN se o ponto está atrás da câmara)
    uv = nan(size(P, 1), 2);
    if isempty(cam), return; end
    for i = 1:size(P, 1)
        r = cam.Rc' * (P(i,:)' - cam.pc(:));   % ponto no referencial do sensor
        if r(3) <= 0, continue; end
        r = r / r(3);
        uv(i,:) = [cam.W/2 + cam.sgn(1)*r(1)*cam.f, cam.H/2 + cam.sgn(2)*r(2)*cam.f];
    end
end