function [kp_outer_P, ki_outer_P, openL_P, BW_out_P, Pm_out_P] = Outer_Loop_P(parameters, inner_loop, speed)
    % OUTER_LOOP_P Computes proportional and integral gains for the outer power control loop, as well as key performance metrics.
    %
    % INPUTS:
    %   parameters  - Struct containing system parameters:
    %                 V_base    : Base voltage (V)
    %                 C_filt    : Filter capacitance (F)
    %                 L_grid    : Grid inductance (H)
    %                 R_grid    : Grid resistance (Ohm)
    %                 omega_b   : Base angular frequency (rad/s)
    %   inner_loop  - Transfer function of the closed inner control loop
    %   speed       - Target tuning speed (Hz)
    %
    % OUTPUTS:
    %   kp_outer_P  - Proportional gain of the outer power loop controller (A/W)
    %   ki_outer_P  - Integral gain of the outer power loop controller (A/Ws)
    %   openL_P     - Open-loop transfer function of the outer power loop
    %   BW_out_P    - Bandwidth of the outer power loop (Hz)
    %   Pm_out_P    - Phase margin of the outer power loop (degrees)
    %
    % AUTHOR:
    %   Nicolae Darii (DTU and SGRE)
    %
    % LICENSE:
    %   MIT License (see LICENSE file in the repository)
    %
    % Extract system parameters from the input structure
    V_base = parameters.V_base;
    C_filt = parameters.C_filt;
    L_grid = parameters.L_grid;
    R_grid = parameters.R_grid;
    omega_b = parameters.omega_b;
    
    % Formulate numerator and denominator for the outer power loop transfer function
    num_out_P = V_base *(sqrt(3) / sqrt(2));
    den_out_P = [C_filt * L_grid, C_filt * R_grid, (-C_filt * L_grid * omega_b^2 + 1)];

    % Create the transfer function for the outer power loop
    G_out_P = tf(num_out_P, den_out_P);

    % Tune the PI controller for the outer power loop
    C_out_P = pidtune(inner_loop * G_out_P, "PI", 2 * pi * speed);

    % Extract the proportional and integral gains
    kp_outer_P = C_out_P.Kp; % Proportional gain (V/A)
    ki_outer_P = C_out_P.Ki; % Integral gain (V/As)

    % Calculate the open-loop transfer function for the outer power loop
    openL_P = C_out_P * inner_loop * G_out_P;

    % Calculate the closed-loop transfer function for the outer power loop
    outer_loop_P = feedback(openL_P, 1);

    % Determine bandwidth of the outer power loop
    BW_out_P = bandwidth(outer_loop_P) / (2 * pi);

    % Determine phase margin of the outer power loop
    [~, Pm_out_P, ~, ~] = margin(openL_P);
end
