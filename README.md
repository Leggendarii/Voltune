# Voltune

Voltune is a MATLAB/Simulink workflow for tuning grid-connected voltage source converter (VSC) controllers and validating the resulting gains in EMT simulation.

## What the main script does

The repository entry point is `main.m`. It:

- loads project libraries, datasets, and PSCAD validation assets
- reads converter and system parameters from a CSV input file
- sets grid and controller targets such as SCR, X/R ratio, PLL bandwidth, and inner-loop bandwidth
- computes base quantities and Thevenin-equivalent grid parameters
- designs the PLL, inner current loop, and outer voltage, power, and DC-link control loops
- converts controller gains to per-unit values
- optionally plots loop responses and summary tables
- writes the updated tuning values back to the CSV file
- opens and runs the `tuning.slx` Simulink model for validation

## Typical workflow

1. Open `main.m` in MATLAB.
2. Set the desired performance targets:
   - `Grid_SCR`
   - `Grid_XR`
   - `PLL_BW`
   - `inner_loop_BW`
3. Choose whether plots should be generated with `plotGraphs`.
4. Run the script.
5. Review the calculated gains, generated plots, updated CSV values, and Simulink validation results.

## Repository structure

```
Voltune/
├── main.m                        # entry point – tuning and validation workflow
├── scam.m                        # helper script
├── tuning.slx                    # Simulink model used for EMT validation
├── data/
│   ├── VSC_test.csv              # converter and system parameter table (read/written by main.m)
│   └── scheme.cir                # PSCAD Thevenin-equivalent circuit for EMT validation
└── lib/
    ├── Tools/
    │   ├── DC_Cap.m              # estimates DC-link capacitance and DC voltage from AC ratings
    │   ├── convertToPerUnit.m    # converts all controller gains to per-unit values
    │   ├── plot_and_tables.m     # generates loop-response plots and summary tables
    │   └── thevenin.m            # computes Thevenin-equivalent grid inductance and resistance
    └── Transfer_Functions/
        ├── PLL.m                 # designs PLL PI gains and returns open-loop TF and bandwidth
        ├── Inner_Loop.m          # designs inner current-loop PI gains and returns open-loop TF
        ├── Outer_Loop_V.m        # designs outer voltage-loop PI gains and performance metrics
        ├── Outer_Loop_P.m        # designs outer active-power-loop PI gains and performance metrics
        └── Outer_Loop_DC.m       # designs outer DC-link-loop PI gains and performance metrics
```

## Inputs and outputs

### Inputs

- `data/VSC_test.csv` – converter and system parameter table read by `main.m`
- `lib/Tools/` – utility functions for Thevenin parameters, per-unit conversion, DC capacitor sizing, and plotting
- `lib/Transfer_Functions/` – per-controller PI design functions (PLL, inner loop, outer V/P/DC loops)
- `tuning.slx` – Simulink model used for closed-loop validation

### Outputs

- updated grid and controller values written back to `data/VSC_test.csv`
- loop bandwidth and phase-margin results displayed through plots/tables
- Simulink validation results from the `tuning` model

## Main configurable selections

The current script exposes the following key user selections near the top of `main.m`:

- grid strength (`Grid_SCR`)
- grid X/R ratio (`Grid_XR`)
- PLL bandwidth (`PLL_BW`)
- inner current-loop bandwidth (`inner_loop_BW`)
- power-vs-DC validation mode (`P_DC_Selector`)
- plot enable/disable (`plotGraphs`)

## Requirements

- MATLAB
- Simulink
- the project files referenced by `main.m`, including the tuning model and supporting data/functions

## License

This project is licensed under the MIT License. See `LICENSE`.
