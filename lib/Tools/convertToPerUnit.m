function [kp_inner_pu, ki_inner_pu, kp_outer_pu, ki_outer_pu, kp_outer_P_pu, ki_outer_P_pu, kp_outer_DC_pu, ki_outer_DC_pu] = convertToPerUnit(P_base, V_base, Vdc_base, kp_inner, ki_inner, kp_outer_V, ki_outer_V, kp_outer_P, ki_outer_P, kp_outer_DC, ki_outer_DC)
    % CONVERT_TO_PER_UNIT Converts controller gains to per unit values.
    %
    % INPUT:
    %   P_base          - Base power value (scalar).
    %   V_base          - Base voltage value (scalar).
    %   kp_inner        - Proportional gain of the inner loop (scalar).
    %   ki_inner        - Integral gain of the inner loop (scalar).
    %   kp_outer_V      - Proportional gain of the outer voltage loop (scalar).
    %   ki_outer_V      - Integral gain of the outer voltage loop (scalar).
    %   kp_outer_P      - Proportional gain of the outer power loop (scalar).
    %   ki_outer_P      - Integral gain of the outer power loop (scalar).
    %
    % OUTPUT:
    %   kp_inner_pu     - Proportional gain of the inner loop in per unit (scalar).
    %   ki_inner_pu     - Integral gain of the inner loop in per unit (scalar).
    %   kp_outer_pu     - Proportional gain of the outer voltage loop in per unit (scalar).
    %   ki_outer_pu     - Integral gain of the outer voltage loop in per unit (scalar).
    %   kp_outer_P_pu   - Proportional gain of the outer power loop in per unit (scalar).
    %   ki_outer_P_pu   - Integral gain of the outer power loop in per unit (scalar).
    %
    % AUTHOR:
    %   Nicolae Darii (DTU and SGRE)
    %
    % LICENSED UNDER:
    %   MIT License (see LICENSE file in the repository).
    %

    % Calculate base current in per unit
    I_base_p = (sqrt(2)/sqrt(3)) * P_base / V_base;
    
    % Calculate base voltage in per unit
    V_base_p = V_base * (sqrt(2)/sqrt(3));

    % Convert inner loop gains to per unit
    kp_inner_pu = kp_inner * (I_base_p / V_base);
    ki_inner_pu = ki_inner * (I_base_p / V_base);

    % Convert outer voltage loop gains to per unit
    kp_outer_pu = kp_outer_V * (V_base_p / I_base_p);
    ki_outer_pu = ki_outer_V * (V_base_p / I_base_p);

    % Convert outer power loop gains to per unit
    kp_outer_P_pu = kp_outer_P * (P_base / I_base_p);
    ki_outer_P_pu = ki_outer_P * (P_base / I_base_p);

    % Convert outer DC loop gains to per unit
    kp_outer_DC_pu = kp_outer_DC * (Vdc_base / I_base_p);
    ki_outer_DC_pu = ki_outer_DC * (Vdc_base / I_base_p);
end