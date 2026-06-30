function plot_and_tables(BW_in, BW_out_V, BW_out_P, BW_out_DC, BW_pll, ...
                         Pm_out_V, Pm_out_P, PM_out_DC, Pm_pll, ...
                         kp_inner_pu, ki_inner_pu, ...
                         kp_outer_V_pu, ki_outer_V_pu, ...
                         kp_outer_P_pu, ki_outer_P_pu, ...
                         kp_outer_DC_pu, ki_outer_DC_pu, ...
                         kp_pll, ki_pll, ...
                         openL_V, inner_loop, openL_P, openL_pll, openL_DC, ...
                         plotGraphs)
    % PLOT_AND_TABLES Displays performance metrics and controller gains in tables,
    % and optionally plots system responses including DC loop.
    %
    % INPUT:
    %   BW_out_DC       - Bandwidth of the outer DC loop (Hz).
    %   PM_out_DC       - Phase margin of the outer DC loop (degrees).
    %   kp_outer_DC_pu  - Proportional gain of the outer DC loop in per unit.
    %   ki_outer_DC_pu  - Integral gain of the outer DC loop in per unit.
    %   openL_DC        - Open-loop transfer function for the outer DC loop.
    %
    % Updated performance table
    T_perf = table( ...
        [BW_in; BW_out_V; BW_out_P; BW_out_DC; BW_pll], ...
        [NaN; Pm_out_V; Pm_out_P; PM_out_DC; Pm_pll], ...
        'VariableNames', {'Bandwidth [Hz]', 'PhaseMargin [deg]'}, ...
        'RowNames', {'Inner', 'Outer V', 'Outer P', 'Outer DC', 'PLL'});
    disp("=== Performance ===");
    disp(T_perf);
    % Updated gains table
    T_gains = table( ...
        [kp_inner_pu; abs(kp_outer_V_pu); kp_outer_P_pu; kp_outer_DC_pu; kp_pll], ...
        [1/ki_inner_pu; abs(1/ki_outer_V_pu); abs(1/ki_outer_P_pu); 1/ki_outer_DC_pu; 1/ki_pll], ...
        'VariableNames', {'Kp [pu]', 'Ti [s]'}, ...
        'RowNames', {'Inner', 'Outer V', 'Outer P', 'Outer DC', 'PLL'});
    disp("=== Controller Gains ===");
    disp(T_gains);
    % Control for plotting
    if plotGraphs
        % Analytical performance figure
        figure;
        % Bode plot
        subplot(2, 2, 1);
        bode(feedback(openL_V, 1));
        hold on;
        bode(inner_loop);
        bode(feedback(openL_P, 1), '--');
        bode(feedback(openL_pll, 1));
        bode(feedback(openL_DC, 1), '-.');
        legend("Outer Loop V", "Inner Loop", "Outer Loop P", "PLL", "Outer Loop DC");
        title("Bode Plot");
        % Step response
        subplot(2, 2, 2);
        step(feedback(openL_V, 1));
        hold on;
        step(inner_loop);
        step(feedback(openL_P, 1), '--');
        step(feedback(openL_pll, 1));
        step(feedback(openL_DC, 1), '-.');
        legend("Outer Loop V", "Inner Loop", "Outer Loop P", "PLL", "Outer Loop DC");
        title("Step Response");
        % Margin plot (occupying both columns in the bottom row)
        subplot(2, 2, [3 4]);
        margin(openL_V);
        hold on;
        margin(openL_P, '--');
        margin(openL_pll);
        margin(openL_DC, '-.');
        title("Open Loop Response V/P/PLL/DC");
        legend("Outer V", "Outer P", "PLL", "Outer DC");
    end
end
