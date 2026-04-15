# Single-Rotor BEMT MATLAB Demo

A self-contained MATLAB demo for single-rotor blade element momentum theory (BEMT) trim analysis with optional flap dynamics and configurable blade aerodynamic / geometric definitions.

This repository provides a compact single-file implementation for:

- single-rotor force and moment calculation
- trim iteration for flap states and induced velocity
- linear airfoil modelling
- optional C81 `.txt` airfoil lookup
- linear or lookup-based chord definition
- linear or lookup-based pretwist definition

## Features

- Self-contained MATLAB implementation in one `.m` file
- Explicit model selection in the rotor definition
- Supports:
  - `rotor.airfoil_model = 'linear'` or `'c81txt'`
  - `rotor.chord_model = 'linear'` or `'lookup'`
  - `rotor.pretwist_model = 'linear'` or `'lookup'`
- Clear error behaviour:
  - if lookup or C81 input files are selected but missing, the code stops with an error
- Computes:
  - rotor forces and moments in body axes
  - induced velocity
  - power
  - flap angle and flap rate histories over one revolution

## Repository contents

- `Rotor_model.m` – complete demo and all helper functions in a single MATLAB file
- `chord_interp.mat` – optional nonlinear chord lookup file
- `pretwist_interp.mat` – optional nonlinear pretwist lookup file
- optional C81 airfoil text tables such as:
  - `CS1_cl.txt`, `CS1_cd.txt`
  - `CS2_cl.txt`, `CS2_cd.txt`
  - etc.

## Requirements

- MATLAB
- `griddedInterpolant` support for table interpolation

No additional toolbox is intentionally required beyond the standard MATLAB functionality used in the script.

## Quick start

Open MATLAB in the repository folder and run:

```matlab
run('Rotor_model.m')
```

The current default example uses:

- linear airfoil model
- linear chord model
- linear pretwist model

So the script should run directly without any external lookup files.

## Current default example

The script currently defines:

- rotor radius: `R = 1.4 m`
- number of blades: `Nb = 4`
- rotational speed: `omega = 90 rad/s`
- air density: `rho = 1.225 kg/m^3`
- collective pitch: `theta0_deg = 15`

The body state is initially set to zero velocity, zero angular velocity, zero linear acceleration, and zero angular acceleration.

The solver then trims for:

- initial flap angles
- initial flap rates
- induced velocity

and returns:

- forces and moments
- power
- convergence information

## Model selection

The script uses explicit model selection:

```matlab
rotor.airfoil_model = 'linear';
rotor.chord_model = 'linear';
rotor.pretwist_model = 'linear';
```

Available options are described below.

## Airfoil model

Two airfoil options are supported.

### 1. Linear airfoil model

Use:

```matlab
rotor.airfoil_model = 'linear';
```

This uses a simple linear lift and quadratic drag model defined in the script.

### 2. C81 text-table airfoil model

Use:

```matlab
rotor.airfoil_model = 'c81txt';
```

This requires C81-style text files, for example:

- `CS1_cl.txt`
- `CS1_cd.txt`
- `CS2_cl.txt`
- `CS2_cd.txt`
- etc.

The parser expects the following structure:

- first row: Mach grid
- each following row: angle of attack followed by aerodynamic coefficients at each Mach number

Example format:

```text
0.0 0.1 0.2 0.3
-10 0.10 0.11 0.12 0.13
-5  0.30 0.31 0.32 0.33
0   0.50 0.51 0.52 0.53
5   0.70 0.71 0.72 0.73
10  0.90 0.91 0.92 0.93
```

The code checks for:

- missing files
- invalid row lengths
- non-finite values
- duplicated Mach values
- duplicated angle-of-attack values

before building the interpolation object.

## Chord model

Two chord options are supported.

### 1. Linear chord model

Use:

```matlab
rotor.chord_model = 'linear';
```

This uses the root and tip chord values defined in the script and interpolates linearly along the span.

### 2. Lookup chord model

Use:

```matlab
rotor.chord_model = 'lookup';
```

This requires:

- `chord_interp.mat`

In the current repository setup, `chord_interp.mat` contains a MATLAB `griddedInterpolant` object named:

- `F`

The code loads this object and evaluates it using a single input `x`, where:

- `x` is the nondimensional radial location along the blade span

That is, the lookup variable is expected to behave like:

```matlab
value = F(x)
```

The chord lookup unit is specified in the script as:

```matlab
rotor.chord.lookup_unit = 'c_over_R';
```

This is interpreted case-insensitively by the code.

## Pretwist model

Two pretwist options are supported.

### 1. Linear pretwist model

Use:

```matlab
rotor.pretwist_model = 'linear';
```

This uses the root and tip pretwist values defined in the script and interpolates linearly along the span.

### 2. Lookup pretwist model

Use:

```matlab
rotor.pretwist_model = 'lookup';
```

This requires:

- `pretwist_interp.mat`

In the current repository setup, `pretwist_interp.mat` contains a MATLAB `griddedInterpolant` object named:

- `pre_twist`

The code loads this object and evaluates it using a single input `x`, where:

- `x` is the nondimensional radial location along the blade span

That is, the lookup variable is expected to behave like:

```matlab
value = pre_twist(x)
```

The returned value is interpreted as blade pretwist in degrees.

## Spanwise sectioning

The script includes spanwise section definitions through:

```matlab
rotor.section_r_end = [1 1 1 1 1 1];
rotor.section_airfoil_id = [1 1 1 1 1 1];
```

These are kept as they appear in the current working version of the demo.

Users can modify these arrays if they want different airfoil sections to be applied over different radial ranges.

## Output

After running the script, MATLAB prints:

- convergence flag
- iteration count
- residual norm
- power
- induced velocity
- forces and moments
- initial flap angles
- initial flap rates

## Notes on structure

The entire implementation is intentionally kept in a single MATLAB file in this version in order to preserve the original working structure with minimal modification.

A future version may separate the helper functions into individual files for easier maintenance and extension.

## Important limitations

This repository is intended as a compact demo / baseline implementation.

It is **not** a high-fidelity validated production solver.

Current limitations include:

- single-rotor demo only
- simplified induced inflow treatment using a single induced velocity unknown
- no dynamic inflow model
- no free-wake model
- no dynamic stall model
- no full multi-rotor aerodynamic interaction model

Therefore, this code should be treated as:

- a research demo
- a teaching / development baseline
- a starting point for further modelling work

## License

This repository is released under the MIT License.

## Citation and acknowledgement

If this repository is useful in academic work, please cite the repository and acknowledge the original author.

## Author

Ye Yuan  
University of Glasgow
