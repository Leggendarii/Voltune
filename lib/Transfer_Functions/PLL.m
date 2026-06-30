function [kp_pll, ki_pll, openL_PLL, BW_pll, Pm_pll] = PLL(parameters, speed)
    % PLL Calculates the inner loop controller gains and bandwidth.
    %
    % INPUT:
    %   parameters      - Structure containing system parameters:
    %                     C_filt   - Filter capacitance (F).
    %                     L_vsc    - VSC inductance (H).
    %                     L_grid    - Grid inductance (H).
    %                     R_vsc    - VSC resistance (Ohm).
    %                     R_grid    - Grid resistance (Ohm).
    %                     omega_b   - Base angular frequency (rad/s).
    %   speed           - Desired speed for tuning (Hz).
    %
    % OUTPUT:
    %   kp_pll         - Proportional gain of the inner loop (ras/s/V).
    %   ki_pll         - Integral gain of the inner loop (ras/s/Vs).
    %   openL_PLL      - Closed-loop transfer function of the PLL.
    %   BW_pll         - Bandwidth of the PLL (Hz).
    %   Pm_pll         - Phase margin of the PLL (degrees).
    %
    % AUTHOR:
    %   Nicolae Darii (DTU and SGRE)
    %
    % LICENSED UNDER:
    %   MIT License (see LICENSE file in the repository).
    %

    % Extract parameters from the input structure
    C_filt = parameters.C_filt; % Filter capacitance
    L_vsc = parameters.L_vsc;   % VSC inductance
    L_grid = parameters.L_grid;  % Grid inductance
    R_vsc = parameters.R_vsc;    % VSC resistance
    R_grid = parameters.R_grid;   % Grid resistance
    omega_b = parameters.omega_b; % Base angular frequency

    V_base = parameters.V_base;

    % Define the numerator and denominator for the transfer function
    num_pll = L_grid * omega_b; % Numerator
    den_pll = [C_filt * L_grid * L_vsc, ...
               (C_filt * L_grid * R_vsc + C_filt * L_vsc * R_grid), ...
               (-3 * C_filt * L_grid * L_vsc * omega_b^2 + L_grid + L_vsc + C_filt * R_grid * R_vsc), ...
               (R_grid + R_vsc - C_filt * L_grid * R_vsc * omega_b^2 - C_filt * L_vsc * R_grid * omega_b^2)]; % Denominator
    
    % num_pll = [L_grid  R_grid];
    % den_pll = [C_filt*L_vsc*L_grid (C_filt*L_vsc*R_grid + C_filt*L_grid*R_vsc) (L_vsc + L_grid + C_filt*R_vsc*R_grid) (R_vsc + R_grid)];
    % Create the transfer function for the inner loop
    G_pll = tf(num_pll, den_pll);

    % Define the variable s for Laplace transform
    s = tf('s'); 
    integral = 1/s; % Integral term for the PI controller
    
    % Tune the PI controller for the inner loop
    C_pll = pidtune(integral * G_pll, "PI", 2 * pi * speed);

    % Extract the proportional and integral gains
    kp_pll = C_pll.Kp; % Proportional gain (A/V)
    ki_pll = C_pll.Ki; % Integral gain (A/Vs)

    % Calculate the closed-loop transfer function of the inner loop
    openL_PLL = C_pll * integral * G_pll;
    PLL = feedback(openL_PLL, 1);
    
    % Calculate the bandwidth of the inner loop
    BW_pll = bandwidth(PLL) / (2 * pi); % Convert to Hz

    % Calculate the phase margin of the inner loop
    [~, Pm_pll, ~, ~] = margin(openL_PLL);
end