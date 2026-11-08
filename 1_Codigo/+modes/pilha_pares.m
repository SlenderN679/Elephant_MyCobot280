function [S, status] = pilha_pares(action, S, ctx)
% PILHA_PARES  MODO 2: vigia a mesa; quando entra um bloco da mesma cor de
%              outro (um par), apanha o bloco solto mais perto da base e
%              empilha-o em cima do outro (os dois têm de estar ao alcance).
%
%   [S, status] = modes.pilha_pares('init', [], ctx)
%   [S, status] = modes.pilha_pares('step', S, ctx)
%
%   status : sempre 'running' (vigia até se parar a simulação)
% A lógica está em modes.vigiar (comum aos modos 2 e 3) e vision.watch_pairs.
    [S, status] = modes.vigiar(action, S, ctx, 'pairs');
end