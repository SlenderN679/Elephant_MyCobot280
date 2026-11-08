function [POS, YAW, found, VIS] = locate_objects(sim_interface, OBJ, tableZ, opts)
% LOCATE_OBJECTS  Fotografia inicial: posição e orientação de cada objeto com
%                 nome, escrita na consola (com o erro, se houver simulador).
%
%   [POS, YAW, found, VIS] = vision.locate_objects(sim_interface, OBJ, tableZ, opts)
%
%   OBJ    : descrição dos objetos (um elemento por objeto, índice = número do
%            objeto), campos: name, color, shape, size, rank, halfH
%   tableZ : altura da mesa [m]
%   opts   : useYaw  - 1 = usar o yaw medido pela câmara (0 = YAW fica NaN)
%            compare - 1 = ler as posições reais do simulador e escrever o erro
%                      (só testes; o controlo nunca as usa)
%            isCube  - função @(k) true se o objeto k é um cubo (para o yaw)
%   POS    : N x 3 centro de cada objeto no MUNDO [m] (NaN se não foi encontrado)
%   YAW    : N x 1 orientação [rad] (NaN = desconhecida)
%   found  : N x 1 true se o objeto foi encontrado
%   VIS    : dados da fotografia para mostrar depois (img, det, cam, pix,
%            truth, truthYaw)
%
% Passos: 1. fotografia (vision_snapshot)   2. posições reais (só testes)
%         3. uma linha por objeto na consola
% Um erro na visão não para o programa: os objetos ficam "não encontrados".

    n = numel(OBJ);
    % 1. fotografia: que mancha é cada objeto e a sua posição
    try
        [POS, found, pix, img, det, YAW, cam] = vision.vision_snapshot(sim_interface, 1:n, ...
            {OBJ.color}, [OBJ.rank], [OBJ.halfH], tableZ, {OBJ.shape}, [OBJ.size]);
    catch errV
        fprintf(2, 'ERRO na visão: %s\n', errV.message);
        fprintf(2, '  em %s (linha %d)\n', errV.stack(1).name, errV.stack(1).line);
        POS = nan(n, 3);  found = false(n, 1);  pix = nan(n, 2);
        img = [];  det = [];  YAW = nan(n, 1);  cam = [];
    end
    if ~opts.useYaw
        YAW(:) = NaN;                          % não usar a orientação da câmara
    end

    % 2. só no simulador: posições reais, para ver o erro
    truth = [];  truthYaw = [];
    if opts.compare
        truth = zeros(n, 3);  truthYaw = zeros(n, 1);
        for k = 1:n
            truth(k,:) = sim_interface.get_target_position(k);
            o = sim_interface.get_target_orientation(k);
            truthYaw(k) = o(3);
        end
    end

    % 3. uma linha por objeto
    for k = 1:n
        if ~found(k)
            fprintf('visão: %-12s (%s) NÃO ENCONTRADO\n', OBJ(k).name, OBJ(k).color);
            continue;
        end
        fprintf('visão: %-12s (%s) em [%.3f %.3f %.3f]', OBJ(k).name, OBJ(k).color, POS(k,:));
        if ~isempty(truth)
            fprintf('   erro %.1f mm', 1000*norm(POS(k,1:2) - truth(k,1:2)));
        end
        if ~isnan(YAW(k)) && opts.isCube(k)
            fprintf('   yaw %5.1f°', rad2deg(YAW(k)));
            if ~isempty(truthYaw)              % erro a menos de 90 graus (face quadrada)
                fprintf(' (erro %.1f°)', rad2deg(mod(YAW(k) - truthYaw(k) + pi/4, pi/2) - pi/4));
            end
        end
        fprintf('\n');
    end

    VIS = struct('img', img, 'det', {det}, 'cam', cam, 'pix', pix, ...
                 'truth', truth, 'truthYaw', truthYaw);
end