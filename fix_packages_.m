function fix_packages(root, apply)
% FIX_PACKAGES  [ACRESCENTADO] Acrescenta o nome do pacote às chamadas de
%               funções que estão em pastas +pacote (ex.: direct_kin(...) ->
%               movement.direct_kin(...)), em TODOS os .m do projeto.
%
%   fix_packages()            % só mostra o que mudaria (não altera nada)
%   fix_packages([], true)    % aplica as alterações
%
% Num pacote, até as chamadas entre funções do MESMO pacote precisam do
% prefixo. Este script:
%   - lê os nomes das funções de cada pasta +pacote da pasta do projeto;
%   - em cada .m (pasta do projeto e pastas +pacote) troca  nome(  por
%     pacote.nome(  e  @nome  por  @pacote.nome;
%   - não mexe em linhas 'function ...', em linhas de comentário, nem em
%     nomes que já têm um ponto antes (sim.xxx, movement.xxx, ...).
% FAZER UMA CÓPIA DA PASTA ANTES de aplicar.

    if nargin < 1 || isempty(root), root = fileparts(mfilename('fullpath')); end
    if nargin < 2, apply = false; end
    me = [mfilename '.m'];

    % 1. funções de cada pacote
    names = {};  pkgs = {};
    P = dir(fullfile(root, '+*'));
    P = P([P.isdir]);
    for i = 1:numel(P)
        F = dir(fullfile(root, P(i).name, '*.m'));
        for j = 1:numel(F)
            nm = F(j).name(1:end-2);
            k = find(strcmp(names, nm), 1);
            if ~isempty(k)
                fprintf('AVISO: %s existe em +%s e em %s (fica +%s)\n', nm, pkgs{k}, P(i).name, pkgs{k});
                continue;
            end
            names{end+1} = nm;  pkgs{end+1} = P(i).name(2:end);   %#ok<AGROW>
        end
    end
    fprintf('%d funções em %d pacotes\n', numel(names), numel(P));

    % 2. ficheiros a corrigir: os da pasta do projeto e os dos pacotes
    files = dir(fullfile(root, '*.m'));
    list = fullfile(root, {files.name});
    for i = 1:numel(P)
        F = dir(fullfile(root, P(i).name, '*.m'));
        list = [list, fullfile(root, P(i).name, {F.name})];          %#ok<AGROW>
    end
    list = list(~endsWith(list, me));

    total = 0;
    for f = 1:numel(list)
        txt = fileread(list{f});
        crlf = ~isempty(strfind(txt, sprintf('\r\n')));
        L = regexp(txt, '\r?\n', 'split');
        nChg = 0;
        for i = 1:numel(L)
            line = L{i};
            if ~isempty(regexp(line, '^\s*(function(?!\w)|%)', 'once')), continue; end
            % separar o comentário no fim da linha (o % fora de texto entre plicas)
            [code, rest] = split_comment(line);
            new = code;
            for k = 1:numel(names)
                nm = names{k};  pk = pkgs{k};
                new = regexprep(new, ['(?<![\w.''"])' nm '(?=\s*\()'], [pk '.' nm]);
                new = regexprep(new, ['(?<=@)' nm '(?!\w)'], [pk '.' nm]);
            end
            if ~strcmp(new, code)
                nChg = nChg + 1;
                if ~apply, fprintf('  %s:%d  %s\n', short(list{f}, root), i, strtrim([new rest])); end
                L{i} = [new rest];
            end
        end
        if nChg > 0
            total = total + nChg;
            fprintf('%-45s %3d linha(s)\n', short(list{f}, root), nChg);
            if apply
                if crlf, eol = sprintf('\r\n'); else, eol = newline; end
                fid = fopen(list{f}, 'w');
                fwrite(fid, strjoin(L, eol));
                fclose(fid);
            end
        end
    end
    if apply
        fprintf('feito: %d linha(s) alterada(s)\n', total);
    else
        fprintf('%d linha(s) a alterar. Para aplicar: fix_packages([], true)\n', total);
    end
end

function [code, rest] = split_comment(line)
% parte a linha no primeiro % que não está dentro de texto entre plicas
    inStr = false;  code = line;  rest = '';
    for c = 1:numel(line)
        ch = line(c);
        if ch == ''''
            % plica de texto ou transposta (depois de letra, número, ), ] ou .)
            if inStr || c == 1 || isempty(regexp(line(c-1), '[\w\)\]\.''}]', 'once'))
                inStr = ~inStr;
            end
        elseif ch == '%' && ~inStr
            code = line(1:c-1);  rest = line(c:end);
            return;
        end
    end
end

function s = short(p, root)
    s = strrep(p, [root filesep], '');
end