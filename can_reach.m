function ok = can_reach(cfg, pWorld)
% CAN_REACH  true se o robô consegue descer na vertical até ao ponto pWorld
%            (centro de preensão, referencial do MUNDO) e voltar a subir
%            cfg.LIFT, sem colisões - o mesmo teste que pick_and_place faz
%            antes de mexer o braço (plan_vertical). Não mexe o robô.
%
%   ok = can_reach(cfg, pWorld)
%
% Ex.: ver se um bloco pousado na mesa se pode apanhar:
%   can_reach(cfg, [x y cfg.TABLE_Z + BLOCK_H/2])

    p = world_to_base(pWorld, cfg.BASE_POSITION, cfg.BASE_YAW);
    % o centro de preensão nunca abaixo da mesa + pontas dos dedos + folga
    floorZ = cfg.TABLE_Z - cfg.BASE_POSITION(3) + cfg.FINGER_EXTRA + cfg.CLEARANCE;
    p = [p(1:2); max(p(3), floorZ)];
    [~, ok] = plan_vertical(p, p + [0; 0; cfg.LIFT], cfg, []);
end