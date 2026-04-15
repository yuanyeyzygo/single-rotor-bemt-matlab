# single-rotor-bemt-matlab

A self-contained MATLAB demo for single-rotor blade element momentum theory (BEMT) trim analysis with optional flap dynamics and multiple blade-definition options.

This repository provides a compact single-file implementation for:

- single-rotor force and moment calculation
- trim iteration for flap states and induced velocity
- linear airfoil model
- optional C81 `.txt` airfoil lookup
- linear or lookup-based chord definition
- linear or lookup-based pretwist definition

## Features

- Self-contained MATLAB implementation in one `.m` file
- Explicit model selection:
  - `rotor.airfoil_model = 'linear'` or `'c81txt'`
  - `rotor.chord_model = 'linear'` or `'lookup'`
  - `rotor.pretwist_model = 'linear'` or `'lookup'`
- Clear failure behaviour:
  - if lookup files are selected but missing, the code stops with an error
- Computes:
  - rotor forces and moments in body axes
  - power
  - induced velocity
  - flap angle/rate histories over one revolution

## File

- `Rotor_model.m` – complete demo and all helper functions in a single MATLAB file

## Requirements

- MATLAB
- `griddedInterpolant` support for C81 table interpolation

No additional toolbox is intentionally required beyond standard MATLAB functionality used in this script.

## Quick start

Open MATLAB in the repository folder and run:

```matlab
run('Rotor_model.m')
```

The current default example uses:

- linear airfoil model
- linear chord model
- linear pretwist model

So it should run directly without external lookup files.

## Current default example

The script defines:

- a single rotor with:
  - `R = 1.4 m`
  - `Nb = 4`
  - `omega = 90 rad/s`
- zero body velocity and angular motion
- collective pitch:
  - `theta0_deg = 15`

The script then solves for:

- initial flap states
- induced velocity
- forces and moments
- power

## Optional input models

### 1. Airfoil model

Two options are supported:

#### Linear model
Use:
```matlab
rotor.airfoil_model = 'linear';
```

#### C81 text tables
Use:
```matlab
rotor.airfoil_model = 'c81txt';
```

Then provide files such as:

- `CS1_cl.txt`
- `CS1_cd.txt`
- `CS2_cl.txt`
- `CS2_cd.txt`
- etc.

The parser expects the following format:

- first row: Mach grid
- each following row: angle of attack followed by aerodynamic coefficients at each Mach number

Example:

```text
0.0 0.1 0.2 0.3
-10 0.10 0.11 0.12 0.13
-5  0.30 0.31 0.32 0.33
0   0.50 0.51 0.52 0.53
5   0.70 0.71 0.72 0.73
10  0.90 0.91 0.92 0.93
```

### 2. Chord model

Two options are supported:

#### Linear model
```matlab
rotor.chord_model = 'linear';
```

#### Lookup model
```matlab
rotor.chord_model = 'lookup';
```

This requires:

- `chord_interp.mat`

The chosen variable inside the `.mat` file must be callable with one input `x`, where `x` is the nondimensional radial location.

### 3. Pretwist model

Two options are supported:

#### Linear model
```matlab
rotor.pretwist_model = 'linear';
```

#### Lookup model
```matlab
rotor.pretwist_model = 'lookup';
```

This requires:

- `pretwist_interp.mat`

The chosen variable inside the `.mat` file must be callable with one input `x`, where `x` is the nondimensional radial location.

## Notes on spanwise sectioning

The script includes:

```matlab
rotor.section_r_end = [1 1 1 1 1 1];
rotor.section_airfoil_id = [1 1 1 1 1 1];
```

This is kept exactly as in the current original demo so that the public version stays consistent with the author’s working file.

Users can modify these arrays if they want different airfoil sections across the blade span.

## Important limitations

This repository is intended as a compact demo / baseline implementation.

It is **not** a high-fidelity validated production solver.

Current limitations include:

- single-rotor demo only
- simplified induced inflow treatment using a single induced velocity unknown
- no dynamic inflow model
- no free-wake model
- no unsteady dynamic stall model
- no full multi-rotor interaction model

Therefore, users should treat this code as:

- a research demo
- a teaching / development baseline
- a starting point for further modelling

## Why the code is kept in one file

The code is intentionally kept in a single `.m` file in this repository version to preserve the original implementation with minimal modification.

Future versions may separate the helper functions into individual files for improved maintainability.

## License

This repository is released under the MIT License.

## Citation

If this repository is useful in academic work, please cite the repository and acknowledge the original author.

## Author

Ye Yuan
