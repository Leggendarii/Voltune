function [C, Vdc] = DC_Cap(V_LL, Sn)

% HP: ma = 1, ripprel 10%
Vdc = V_LL  * 1.6330;
function y = round2digit(x)
    % Arrotonda x al secondo digit significativo
    n = floor(log10(abs(x)));       % ordine di grandezza (esponente base 10)
    y = round(x, -n+1);              % arrotonda in base alle cifre significative
end

Vdc = round2digit(Vdc);
Iac = Sn/(V_LL*sqrt(3));
C = Iac / (6 * 50 * 0.1 * Vdc);
% C = Sn / (0.05 * 2500 * Vdc^2);
end