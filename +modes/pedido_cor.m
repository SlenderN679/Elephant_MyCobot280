function [S, status] = pedido_cor(action, S, ctx)
% PEDIDO_COR  MODO 3: vigia a mesa; um bloco novo da mesma cor de outro é um
%             PEDIDO (pode estar fora do alcance): a peça dessa cor ao alcance
%             mais perto da base vai para a pilha do alvo verde.
%
%   [S, status] = modes.pedido_cor('init', [], ctx)
%   [S, status] = modes.pedido_cor('step', S, ctx)
%
%   status : sempre 'running' (vigia até se parar a simulação)
% A lógica está em modes.vigiar (comum aos modos 2 e 3) e vision.watch_pairs.
    [S, status] = modes.vigiar(action, S, ctx, 'request');
end