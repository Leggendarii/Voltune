function [kp_outer_V, ki_outer_V, openL_V, BW_out_V, Pm_out_V] = Outer_Loop_V(parameters, inner_loop, speed)
    % OUTER_LOOP_V Calculates the outer loop voltage controller gains and performance metrics.
    %
    % INPUT:
    %   parameters      - Structure containing system parameters:
    %                     C_filt    - Filter capacitance (F).
    %                     L_grid    - Grid inductance (H).
    %                     R_grid    - Grid resistance (Ohm).
    %                     omega_b   - Base angular frequency (rad/s).
    %   inner_loop      - Closed-loop transfer function of the inner loop.
    %   speed           - Desired speed for tuning (Hz).
    %
    % OUTPUT:
    %   kp_outer_V     - Proportional gain of the outer voltage loop (V/A).
    %   ki_outer_V     - Integral gain of the outer voltage loop (V/As).
    %   openL_V        - Open-loop transfer function of the outer voltage loop.
    %   BW_out_V       - Bandwidth of the outer voltage loop (Hz).
    %   Pm_out_V       - Phase margin of the outer voltage loop (degrees).
    %
    % AUTHOR:
    %   Nicolae Darii (DTU and SGRE)
    %
    % LICENSED UNDER:
    %   MIT License (see LICENSE file in the repository).
    %

    % Extract parameters from the input structure
    C_filt = parameters.C_filt;
    L_grid = parameters.L_grid;
    R_grid = parameters.R_grid;
    omega_b = parameters.omega_b;

    % Define the numerator and denominator for the outer voltage loop transfer function
    num_out = -L_grid * omega_b;
    den_out = [C_filt * L_grid, C_filt * R_grid, (-C_filt * L_grid * omega_b^2 + 1)];

    %%% The values that matter are the ones from PoC to right !! %%%
    
    % Create the transfer function for the outer voltage loop
    G_out = tf(num_out, den_out);
    
    % Tune the PI controller for the outer voltage loop
    C_out = pidtune(inner_loop * G_out, "PI", 2 * pi * speed);

    % Extract the proportional and integral gains
    kp_outer_V = C_out.Kp; % Proportional gain (V/A)
    ki_outer_V = C_out.Ki; % Integral gain (V/As)

    % Calculate the open-loop transfer function of the outer voltage loop
    openL_V = C_out * inner_loop * G_out;
    
    % Calculate the closed-loop transfer function of the outer voltage loop
    outer_loop_V = feedback(openL_V, 1);
    
    % Calculate the bandwidth of the outer voltage loop
    BW_out_V = bandwidth(outer_loop_V) / (2 * pi);

    % Calculate the phase margin of the outer voltage loop
    [~, Pm_out_V, ~, ~] = margin(openL_V);
end