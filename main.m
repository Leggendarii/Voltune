% MAIN SCRIPT FOR VSC Converter Tuning
%
% This script initializes parameters for a VSC converter connected to a 
% grid modelled as Thevenin equivalent. Computes
% the frequency domain response of the control loops, and optionally
% plots the results. It utilizes Thevenin equivalent parameters and
% controller design functions to analyze the system's performance.
%
% AUTHOR:
%   Nicolae Darii (DTU and SGRE)
%
% LICENSED UNDER:
%   MIT License (see LICENSE file in the repository).
%
% FUNCTION:
%   This script serves as the main entry point for the VSC concerter tuning

clear all;  % Clear all variables from the workspace
close all;  % Close all figure windows
clc;        % Clear the command window

%% Load required libraries and datasets
addpath(genpath('lib'));   % Analysis, computation, and plotting functions
addpath(genpath('data'));  % Measurement data files
addpath(genpath('PSCAD Validation')) % PSCAD Folder

%% Parameters Initialization
name = "VSC_test.csv";
parameter = readtable(name);

%%%%%%%%%%%%%% Performance Selection %%%%%%%%%%%% (Here you decide!)
Grid_SCR = 4; 
Grid_XR = 11; %1.96
PLL_BW = 30; % Hz  30
inner_loop_BW = 300; % Hz

% Decide whether to plot or not
plotGraphs = 1; % Set to 1 to enable plotting, 0 to disable
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Nominal Values
Ts = parameter.Value(1)*10^-6;     % Sample time (s)
f_base = parameter.Value(2);       % Base frequency (Hz)
V_base = parameter.Value(3)*10^3;  % Base voltage (L-L RMS) (V)
P_base = parameter.Value(4)*10^6;  % Base power (W)

% VSC Parameters
L_vsc = parameter.Value(5);        % VSC inductance (H)
C_filt = parameter.Value(6);       % Filter capacitance (F)
R_vsc = parameter.Value(7);        % VSC resistance (Ohm)
R_filt = parameter.Value(8);       % Filter resistance (Ohm)
[C_dc, Vdc_base] = DC_Cap(V_base, P_base); % DC capacitor and voltage (F, V)
 
% C_dc = 0.21; %bypass
% Vdc_base = 1.38e3; %bypass

t_charge = 0.5;                    % Time to charge the capacitor activate outer loop DC (s)
I_charge = C_dc * Vdc_base / t_charge;      % Current to charge the DC cap in 0.5s (A)

% Grid Parameters (3 per statcom)
[L_grid, R_grid] = thevenin(Grid_SCR, Grid_XR, V_base, P_base, f_base);  % Thevenin equivalent parameters

% L_grid = 0.3299831645537222; % bypass
% R_grid = 0.04714045207910317;  % bypass
% R_grid = parameter.Value(9);        % VSC resistance (Ohm)
% L_grid = parameter.Value(10);       % Filter resistance (Ohm)

% Base values calculation
omega_b = 2 * pi * f_base;        % Base angular frequency (rad/s)
Z_base = V_base^2 / P_base;       % Base impedance (Ohm)
L_base = Z_base / omega_b;        % Base inductance (H)
C_base = 1 / (Z_base * omega_b);  % Base capacitance (F)

% Create a structure to hold all parameters
parameters = struct( ...
    'L_vsc',    L_vsc, ...        % VSC inductance (H)
    'C_filt',   C_filt, ...       % Filter capacitance (F)
    'R_vsc',    R_vsc, ...        % VSC resistance (Ohm)
    'R_filt',   R_filt, ...       % Filter resistance (Ohm)
    'R_grid',   R_grid, ...       % Grid resistance (Ohm)
    'L_grid',   L_grid, ...       % Grid inductance (H)
    'omega_b',  omega_b, ...      % Base angular frequency (rad/s)
    'V_base',   V_base, ...       % Base voltage (V)
    'P_base',   P_base ...        % Base power (W)
);

%% Frequency Domain Response Calculation
% PLL (Transfer function work in progress)
[kp_pll, ki_pll, openL_PLL, BW_pll, Pm_pll]  = PLL(parameters, PLL_BW);

% Inner Loop Controller Design
[kp_inner, ki_inner, inner_loop, BW_in] = Inner_Loop(parameters, inner_loop_BW);

% Outer Voltage Loop Controller Design
outer_Loop_BW = inner_loop_BW/10; %Hz
[kp_outer_V, ki_outer_V, openL_V, BW_out_V, Pm_out_V] = Outer_Loop_V(parameters, inner_loop, outer_Loop_BW);
closedL_V = feedback(openL_V, 1);

% Outer Power Loop Controller Design
[kp_outer_P, ki_outer_P, openL_P, BW_out_P, Pm_out_P] = Outer_Loop_P(parameters, inner_loop, outer_Loop_BW);
closedL_P = feedback(openL_P, 1);

% Outer DC Loop Controller Design
[kp_outer_DC, ki_outer_DC, openL_DC, BW_out_DC, Pm_out_DC] = Outer_Loop_DC(parameters, inner_loop, outer_Loop_BW);
closedL_DC = feedback(1, openL_DC);

% Convert gains to per unit
[kp_inner_pu, ki_inner_pu, kp_outer_V_pu, ki_outer_V_pu, kp_outer_P_pu, ki_outer_P_pu, kp_outer_DC_pu, ki_outer_DC_pu] = ...
    convertToPerUnit(P_base, V_base, Vdc_base, kp_inner, ki_inner, kp_outer_V, ki_outer_V, kp_outer_P, ki_outer_P, kp_outer_DC, ki_outer_DC);

%% Plotting and Displaying Results
% Decide whether to plot or not
% plotGraphs = 1; % Set to 1 to enable plotting, 0 to disable

% Call the function to display tables and plots
plot_and_tables(BW_in, BW_out_V, BW_out_P, BW_out_DC, BW_pll, ...
                         Pm_out_V, Pm_out_P, Pm_out_DC, Pm_pll, ...
                         kp_inner_pu, ki_inner_pu, ...
                         kp_outer_V_pu, ki_outer_V_pu, ...
                         kp_outer_P_pu, ki_outer_P_pu, ...
                         kp_outer_DC_pu, ki_outer_DC_pu, ...
                         kp_pll, ki_pll, ...
                         openL_V, inner_loop, openL_P, openL_PLL, openL_DC, ...
                         plotGraphs)

%% Write to CSV
parameter.Value(9) = R_grid;
parameter.Value(10) = L_grid;
parameter.Value(11) = kp_outer_V_pu;
parameter.Value(12) = abs(1/ki_outer_V_pu);
parameter.Value(13) = kp_outer_P_pu;
parameter.Value(14) = abs(1/ki_outer_P_pu);
parameter.Value(15) = kp_inner_pu;
parameter.Value(16) = abs(1/ki_inner_pu);
parameter.Value(17) = kp_pll;
parameter.Value(18) = abs(1/ki_pll);
parameter.Value(19) = kp_outer_DC_pu;
parameter.Value(20) = abs(1/ki_outer_DC_pu);

% name = 'full_parameters';
writetable(parameter, fullfile('data', name));

% outputPath = 'C:\Grey_boxing\Tools in Matalb\VSC_Tuning\data\full_parameters.csv';  % Percorso completo
% writetable(parameter, outputPath);  % Scrivi la tabella nel file CSV
disp("=== Updated values in CSV ===");

%% Simulink validation

P_DC_Selector = 1; % 1: P reference, 0: DC reference

if P_DC_Selector == 1
    
    Power = 1;
    DC = 0;

    kp_outer = kp_outer_P_pu;
    ki_outer = ki_outer_P_pu;
    BW_out = BW_out_P;
    Pm_out = Pm_out_P;

    load_system('tuning');
else

    Power = 0;
    DC = 1;

    kp_outer = kp_outer_DC_pu;
    ki_outer = ki_outer_DC_pu;
    BW_out = BW_out_DC;
    Pm_out = Pm_out_DC;

    load_system('tuning');
end

%% Run simulink with the new gains to validate
modelName = 'tuning';
open_system(modelName);
sim(modelName);

