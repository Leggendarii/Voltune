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

## Inputs and outputs

### Inputs

- A parameter table loaded from `VSC_test.csv`
- MATLAB helper code under the project library paths
- The `tuning.slx` Simulink model used for validation

### Outputs

- updated grid and controller values written back to the CSV data file
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
