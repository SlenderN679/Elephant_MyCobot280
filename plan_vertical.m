function [ikOpts, ok, qTop] = plan_vertical(pLow, pHigh, cfg, yaw, qRef)
% PLAN_VERTICAL  Escolhe uma inclinação FIXA da garra para um movimento
%                vertical (descer até um objeto / subir), com o braço na
%                pose normal "cotovelo para cima".
%
%   [ikOpts, ok, qTop] = plan_vertical(pLow, pHigh, cfg, yaw)
%   [ikOpts, ok, qTop] = plan_vertical(pLow, pHigh, cfg, yaw, qRef)
%
%   pLow, pHigh : 3x1 pontos de baixo e de cima, referencial da BASE [m]
%                 (mesmo x, y; só muda a altura)
%   yaw         : orientação do cubo (para J6), ou []
%   qRef        : (opcional, não usado; mantido por compatibilidade)
%   ikOpts      : estrutura para move_linear / inverse_kin:
%                   ikOpts.family = 'front' (cotovelo para cima)
%                   ikOpts.pitch  = inclinação da garra [rad] (fixa)
%   ok          : false se nenhuma inclinação funciona
%   qTop        : juntas no ponto de cima (pHigh)
%
% Procura a inclinação mais próxima da vertical (90 graus), entre
% cfg.MIN_GRASP_PITCH e 135 graus, que funcione em TODA a descida: de 2 em
% 2 mm verifica-se que há solução com o cotovelo para cima, que nenhuma
% junta salta mais de 5 graus e (se cfg.CHECK_COLLISIONS) que o braço não
% choca consigo próprio (arm_collision).
%
% Porquê uma inclinação fixa: perto da base, deixar o inverse_kin procurar
% a melhor inclinação em cada ponto faz a solução mudar de ramo a meio da
% descida (J4 bate no limite de +-165 graus) e as juntas dão saltos.

    if nargin < 5, qRef = []; end %#ok<NASGU>
    minPitch = deg2rad(45);
    if isfield(cfg, 'MIN_GRASP_PITCH'), minPitch = cfg.MIN_GRASP_PITCH; end
    checkColl = ~isfield(cfg, 'CHECK_COLLISIONS') || cfg.CHECK_COLLISIONS;
    maxPitch = deg2rad(135);
    MAX_JUMP = deg2rad(5);
    DZ       = 0.002;

    zs = pHigh(3):-DZ:pLow(3);
    if isempty(zs) || zs(end) > pLow(3), zs(end+1) = pLow(3); end

    ikOpts = struct('family', 'front', 'pitch', pi/2);
    ok = false;  qTop = [];
    for d = deg2rad(0:5:90)
        if d == 0, cands = pi/2; else, cands = [pi/2 + d, pi/2 - d]; end
        for psi = cands
            if psi > maxPitch || psi < minPitch, continue; end
            good = true;  qPrev = [];  qT = [];
            for z = zs
                [q, reach] = inverse_kin(pLow(1), pLow(2), z, cfg.Links, cfg.qMin, cfg.qMax, ...
                                         yaw, psi, 'front', true);
                if ~reach || (~isempty(qPrev) && max(abs(q(1:4) - qPrev(1:4))) > MAX_JUMP)
                    good = false;  break;
                end
                if checkColl && arm_collision(q, cfg.Links)
                    good = false;  break;
                end
                if isempty(qT), qT = q; end
                qPrev = q;
            end
            if good
                ikOpts.pitch = psi;
                ok = true;  qTop = qT;
                return;
            end
        end
    end
end