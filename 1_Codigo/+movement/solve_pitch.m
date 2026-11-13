function [psi, ok] = solve_pitch(x, y, z, Links, qMin, qMax, prefPitch, family)
% SOLVE_PITCH  Escolhe a inclinação da garra para a cinemática inversa
%
%   [psi, ok] = solve_pitch(x, y, z, Links, qMin, qMax, prefPitch, family)
%
%   (x,y,z)   : posição desejada do centro de preensão, referencial da BASE [m]
%   Links     : conexões entre juntas (ver direct_kin.m)
%   qMin/qMax : limites das juntas [rad]
%   prefPitch : inclinação preferida da garra [rad] (pi/2 = vertical para baixo)
%   family    : 'up' (cotovelo para cima) ou 'down' (cotovelo para baixo)
%   psi       : inclinação escolhida [rad] (NaN se nenhuma tiver solução)
%   ok        : false se nenhuma inclinação tiver solução
%
% Com a garra numa inclinação fixa, o plano lateral fica com 2 incógnitas e
% 2 equações (braço de 2 elos). Esta função escolhe essa inclinação: testa a
% preferida e, se não tiver solução, afasta-se de 5 em 5 graus para os dois
% lados (primeiro para a base, > 90 graus, boa para objetos perto da base;
% depois para fora, < 90 graus, objetos longe). Cada inclinação é testada com
% inverse_kin(..., psi, family, true), que resolve os ângulos das juntas e
% verifica as condições de existência para essa inclinação.

    psi = NaN;  ok = false;

    % Passo 1: limites da inclinação da garra
    PITCH_MAX = deg2rad(135);           % garra no máximo 45 graus inclinada para a base
    PITCH_MIN = deg2rad(30);            % garra sempre pelo menos 30 graus abaixo da horizontal
                                        % (a apontar para cima não serve para apanhar nada)
    STEP      = deg2rad(5);

    % uma inclinação "funciona" se a cinemática inversa tiver solução com ela
    works = @(p) has_solution(x, y, z, Links, qMin, qMax, p, family);

    % Passo 2: procurar uma inclinação da garra que tenha solução, começando pela
    % preferida e afastando-se de 5 em 5 graus (primeiro para a base, depois para fora)
    for d = 0:STEP:(PITCH_MAX - PITCH_MIN)
        % candidatos com desvio d da inclinação preferida: primeiro para a
        % base (+d), depois para fora (-d)
        if d == 0
            ang = prefPitch;
        else
            ang = [prefPitch + d, prefPitch - d];
        end
        for p = ang
            if p > PITCH_MAX || p < PITCH_MIN
                continue;
            end
            if ~works(p)
                continue;
            end

            % Passo 3: os candidatos vão de 5 em 5 graus. Procura-se por bisseção
            % a inclinação mais próxima da preferida que ainda funciona, para a
            % garra inclinar de forma contínua em vez de saltar 5 graus
            if d > 0
                good = p;  bad = p - sign(p - prefPitch)*STEP;
                for it = 1:15
                    mid = (good + bad)/2;
                    if works(mid), good = mid; else, bad = mid; end
                end
                p = good;
            end
            psi = p;  ok = true;
            return;
        end
    end
end

% --------------------------------------------------------------------------
% Testa uma inclinação: inverse_kin com a inclinação fixa resolve o plano
% lateral e verifica os limites das juntas, a dobra do cotovelo e as alturas
function ok = has_solution(x, y, z, Links, qMin, qMax, p, family)
% true se a cinemática inversa tiver solução com a inclinação p
    [~, ok] = movement.inverse_kin(x, y, z, Links, qMin, qMax, [], p, family, true);
end