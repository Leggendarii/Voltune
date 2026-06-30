function [kp_outer_DC, ki_outer_DC, openL_DC, BW_out_DC, Pm_out_DC] = Outer_Loop_DC(parameters, inner_loop, speed)
    % OUTER_LOOP_DC - Computes the PI controller gains for the outer DC loop
    %                 and evaluates its dynamic performance.
    %
    % INPUT:
    %   parameters   - Structure with system parameters:
    %                  .V_base  : DC side base voltage (V)
    %                  .P_base  : Base power (W)
    %                  .omega_b : Base angular frequency (rad/s) [not used here]
    %   inner_loop   - Closed-loop transfer function of the inner loop
    %                  (typically current or AC voltage control).
    %   speed        - Desired tuning frequency for the PI controller (Hz).
    %
    % OUTPUT:
    %   kp_outer_DC  - Proportional gain of the outer DC loop (A/V)
    %   ki_outer_DC  - Integral gain of the outer DC loop (A/Vs)
    %   openL_DC     - Open-loop transfer function of the outer DC loop
    %   BW_out_DC    - Bandwidth of the outer DC loop (Hz)
    %   Pm_out_DC    - Phase margin of the outer DC loop (degrees)
    %
    % AUTHOR:
    %   Nicolae Darii (DTU and SGRE)
    %
    % LICENSE:
    %   MIT License (see LICENSE file in the repository).
    %
    
    % Extract parameters
    V_base = parameters.V_base;
    P_base = parameters.P_base;
    C_filt = parameters.C_filt;
    L_grid = parameters.L_grid;
    R_grid = parameters.R_grid;
    omega_b = parameters.omega_b;
    
    % Compute equivalent DC capacitance and voltage
    [C_dc, Vdc_base] = DC_Cap(V_base, P_base);  % (F, V)
    
    % Model of the DC link capacitor dynamics (1 / (C*s))
    % num_out_DC = 1;        
    % den_out_DC = [C_dc, 0];

    num_out_DC = V_base *(sqrt(3) / sqrt(2)) / (C_dc*Vdc_base);   
    den_out_DC = [C_filt * L_grid, C_filt * R_grid, (-C_filt * L_grid * omega_b^2 + 1), 0];

    G_out_DC = tf(num_out_DC, den_out_DC);
    
    % Automatic PI tuning on the combined system (inner loop * DC dynamics)
    opts = pidtuneOptions('DesignFocus','disturbance-rejection');
    C_out_DC = pidtune(inner_loop * G_out_DC, "PI", 2*pi*speed, opts);
    
    % Extract PI controller gains
    kp_outer_DC = C_out_DC.Kp;
    ki_outer_DC = C_out_DC.Ki;
    
    % Open-loop transfer function of the outer DC loop
    openL_DC = C_out_DC * inner_loop * G_out_DC;
    
    % Closed-loop transfer function of the outer DC loop
    outer_loop_DC = feedback(openL_DC, 1);
    
    % Bandwidth of the outer loop (Hz)
    BW_out_DC = bandwidth(outer_loop_DC) / (2 * pi);
    
    % Phase margin (degrees)
    [~, Pm_out_DC, ~, ~] = margin(openL_DC);
end
