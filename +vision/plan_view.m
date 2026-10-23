function extras = plan_view(cam, steps, iNext, info, more)
% PLAN_VIEW  [ACRESCENTADO] Plano de uma lista de movimentos (TASKS no modo 1,
%            passos do modelo no modo 4) para show_vision: uma seta por
%            movimento - feitos a cinzento, o próximo grosso, os outros
%            tracejados - e o painel de estado.
%
%   extras = plan_view(cam, steps, iNext, info)
%   extras = plan_view(cam, steps, iNext, info, more)
%
%   cam   : modelo da câmara (vision_colors); [] se não há câmara
%   steps : struct array com from, to ([x y z] no MUNDO), color, label
%   iNext : índice do próximo passo (numel(steps)+1 = tudo feito)
%   info  : cell com as linhas do painel (o progresso é acrescentado aqui)
%   more  : (opcional) setas extra, mesmo formato que extras.arrows
%           (ex.: o modelo no modo 4)

    if nargin < 5, more = []; end
    arrows = struct('from', {}, 'to', {}, 'color', {}, 'label', {}, 'state', {});
    for i = 1:numel(steps)
        if i < iNext,      st = 'done';
        elseif i == iNext, st = 'next';
        else,              st = 'todo';
        end
        arrows(end+1) = struct('from', steps(i).from, 'to', steps(i).to, ...
                               'color', steps(i).color, 'label', steps(i).label, ...
                               'state', st);                               %#ok<AGROW>
    end
    for i = 1:numel(more)
        arrows(end+1) = more(i);                                           %#ok<AGROW>
    end
    % progresso, uma linha por passo: [x] feito, [>] a seguir, [ ] por fazer
    for i = 1:numel(steps)
        if i < iNext,      mark = '[x]';
        elseif i == iNext, mark = '[>]';
        else,              mark = '[ ]';
        end
        info{end+1} = sprintf('%s %d. %s', mark, i, steps(i).label);      %#ok<AGROW>
    end
    if iNext > numel(steps) && ~isempty(steps)
        info{end+1} = 'concluído!';
    end
    extras = struct('cam', cam, 'arrows', arrows, 'info', {info});
end