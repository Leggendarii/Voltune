function [kp_inner, ki_inner, inner_loop, BW_in] = Inner_Loop(parameters, speed)
    % INNER_LOOP Calculates the inner loop controller gains and bandwidth.
    %
    % INPUT:
    %   parameters      - Structure containing system parameters:
    %                     C_filt   - Filter capacitance (F).
    %                     L_vsc    - VSC inductance (H).
    %                     L_grid   - Grid inductance (H).
    %                     R_vsc    - VSC resistance (Ohm).
    %                     R_grid   - Grid resistance (Ohm).
    %   speed           - Desired speed for tuning (Hz).
    %
    % OUTPUT:
    %   kp_inner       - Proportional gain of the inner loop (A/V).
    %   ki_inner       - Integral gain of the inner loop (A/Vs).
    %   inner_loop     - Closed-loop transfer function of the inner loop.
    %   BW_in          - Bandwidth of the inner loop (Hz).
    %
    % AUTHOR:
    %   Nicolae Darii (DTU and SGRE)
    %
    % LICENSED UNDER:
    %   MIT License (see LICENSE file in the repository).
    %

    % Extract parameters from the input structure
    C_filt = parameters.C_filt;
    L_vsc = parameters.L_vsc;
    L_grid = parameters.L_grid;
    R_vsc = parameters.R_vsc;
    R_grid = parameters.R_grid;

    % Define the numerator and denominator for the transfer function
    num_in = [C_filt * L_grid, C_filt * R_grid, 1];
    den_in = [C_filt * L_grid * L_vsc, ...
               C_filt * (L_vsc * R_grid + L_grid * R_vsc), ...
               (L_vsc + L_grid + C_filt * R_vsc * R_grid), ...
               (R_vsc + R_grid)];
    
    % Create the transfer function for the inner loop
    G_in = tf(num_in, den_in);
    
    % Tune the PI controller for the inner loop
    C_in = pidtune(G_in, "PI", 2 * pi * speed);

    % Extract the proportional and integral gains
    kp_inner = C_in.Kp; % Proportional gain (A/V)
    ki_inner = C_in.Ki; % Integral gain (A/Vs)

    % Calculate the closed-loop transfer function of the inner loop
    inner_loop = feedback(C_in * G_in, 1);
    
    % Calculate the bandwidth of the inner loop
    BW_in = bandwidth(inner_loop) / (2 * pi);
end