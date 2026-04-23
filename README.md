# Single-Rotor BEMT MATLAB / Simulink Demo

A compact research demo for single-rotor blade element momentum theory (BEMT) analysis with flap-trim solving, configurable aerodynamic and geometric models, and a Simulink-compatible preprocessing workflow.

This repository contains two complementary paths:

1. Standalone MATLAB workflow  
   A single-file MATLAB implementation for direct rotor force and moment calculation and trim solving.

2. Preprocessed Simulink workflow  
   An offline preprocessing script that converts airfoil, chord, and pretwist inputs into numeric arrays suitable for a code-generation-friendly runtime implementation in Simulink.

The repository is intended as a baseline research and development model, not a high-fidelity production solver.

--------------------------------------------------
Repository structure
--------------------------------------------------

.
├── Rotor_model.m
├── Data_input.m
├── Simulink_one_rotor.slx
├── chord_interp.mat              (optional, for lookup chord mode)
├── pretwist_interp.mat           (optional, for lookup pretwist mode)
├── CS1_cl.txt ... CS6_cl.txt     (optional, for C81 lift tables)
├── CS1_cd.txt ... CS6_cd.txt     (optional, for C81 drag tables)
└── README.md

Main files

Rotor_model.m
Standalone MATLAB script containing:
- rotor definition
- operating-state definition
- model selection
- flap / induced-velocity trim iteration
- force / moment calculation
- helper functions for:
  - C81 table parsing
  - chord evaluation
  - pretwist evaluation
  - airfoil evaluation

This file is self-contained and is the easiest entry point if you want to inspect or modify the aerodynamic model directly.

Data_input.m
Offline preprocessing script for the Simulink workflow.

This script:
- reads C81 airfoil text tables
- samples or remaps lookup data for chord and pretwist
- assembles a numeric-only rotor structure
- writes the processed data into rotor.mat

This is useful when the runtime model should avoid unsupported features such as runtime file parsing, function handles, or interpolant objects.

Simulink_one_rotor.slx
Simulink implementation of the one-rotor model using preprocessed numeric data.

This model is intended for:
- integration into larger Simulink studies
- code-generation-friendly testing
- future control and system-level simulation work

--------------------------------------------------
What the model does
--------------------------------------------------

The model computes single-rotor aerodynamic loads while solving for a trimmed periodic flap state.

More specifically, it can compute:
- rotor forces in body axes
- rotor moments in body axes
- rotor torque and power
- induced velocity
- flap-angle histories over one revolution
- flap-rate histories over one revolution
- angle-of-attack histories over one revolution

The trim procedure solves for:
- initial flap angles
- initial flap rates
- a single induced-velocity unknown

so that the blade state becomes periodic over one rotor revolution and the scalar inflow balance is satisfied.

--------------------------------------------------
Modelling options
--------------------------------------------------

1. Airfoil model

Two airfoil representations are available.

Linear airfoil model
A simple low-order model defined by:
- lift-curve slope
- zero-lift angle
- parasitic drag coefficient
- quadratic drag factor

This is the simplest mode and is useful for testing, debugging, and baseline comparisons.

C81 text-table lookup
A table-based airfoil option using text files such as:
- CS1_cl.txt, CS1_cd.txt
- CS2_cl.txt, CS2_cd.txt
- ...
- CS6_cl.txt, CS6_cd.txt

Expected format:

0.0 0.1 0.2 0.3
-10 0.10 0.11 0.12 0.13
-5  0.30 0.31 0.32 0.33
0   0.50 0.51 0.52 0.53
5   0.70 0.71 0.72 0.73
10  0.90 0.91 0.92 0.93

where:
- the first row is the Mach grid
- each following row begins with angle of attack
- the remaining entries are the aerodynamic coefficients at each Mach number

The code includes checks for:
- missing files
- invalid row lengths
- duplicate angle-of-attack values
- duplicate Mach values
- non-finite values

before constructing the lookup representation.

2. Chord model

Two chord representations are available.

Linear chord model
Chord is interpolated linearly from root to tip using user-defined values.

Lookup chord model
Chord is loaded from:
- chord_interp.mat

Expected variable:
- F

The lookup should behave like:

value = F(x)

where x is the nondimensional radial position along the blade span.

Depending on the chosen unit convention, the chord data may be interpreted as either:
- physical chord in metres
- nondimensional c/R

3. Pretwist model

Two pretwist representations are available.

Linear pretwist model
Pretwist is interpolated linearly from root to tip in degrees.

Lookup pretwist model
Pretwist is loaded from:
- pretwist_interp.mat

Expected variable:
- pre_twist

The lookup should behave like:

value = pre_twist(x)

where x is the nondimensional radial position.

The returned value is interpreted as pretwist angle in degrees.

--------------------------------------------------
Spanwise sectioning
--------------------------------------------------

The blade can be divided into multiple radial sections using:
- rotor.section_r_end
- rotor.section_airfoil_id

This allows different airfoil definitions to be assigned to different parts of the blade span.

For example, a root section, mid-span section, and tip section may each use different airfoil data if needed.

--------------------------------------------------
Standalone MATLAB workflow
--------------------------------------------------

The standalone solver is the simplest way to run the model.

Quick start

Open MATLAB in the repository folder and run:

run('Rotor_model.m')

The script defines:
- the rotor geometry and physical parameters
- the vehicle or hub state
- the control input
- the solver settings
- the aerodynamic and geometric model selections

It then runs the trim solve and prints the main outputs to the MATLAB console.

Typical outputs

After execution, the standalone script prints:
- convergence flag
- number of iterations
- residual norm
- power
- induced velocity
- body-axis force and moment vector
- initial flap angles
- initial flap rates

--------------------------------------------------
Simulink workflow
--------------------------------------------------

The Simulink path is intended for cases where you want the model to operate with preprocessed numeric data rather than runtime file parsing.

Step 1: preprocess the lookup data

Run:

run('Data_input.m')

This script prepares numeric rotor data and writes:

rotor.mat

Step 2: open and run the Simulink model

open_system('Simulink_one_rotor.slx')

Then run the model from Simulink as usual.

Why this path exists

This workflow is useful when you want to avoid runtime features that are inconvenient for code generation or embedded-style execution, such as:
- runtime text parsing
- griddedInterpolant objects at runtime
- function handles at runtime
- dynamic lookup construction inside the execution loop

Instead, those data are prepared offline and passed to the runtime solver in numeric form.

--------------------------------------------------
Important consistency note
--------------------------------------------------

This repository contains two related but different workflows:
- the standalone MATLAB script uses explicit string-based model selection
- the preprocessing and Simulink path uses numeric mode codes and preprocessed arrays

If you want to compare the two workflows directly, make sure the following are aligned in both paths:
- airfoil mode
- chord mode
- chord unit convention
- pretwist mode
- rotor operating state
- control input
- lookup files used
- rotor geometry and physical parameters

If these are not aligned, the two workflows can produce different results even though they represent the same overall rotor model structure.

--------------------------------------------------
Default example parameters
--------------------------------------------------

A representative example setup in this repository uses values such as:
- rotor radius: R = 1.4 m
- number of blades: Nb = 4
- rotational speed: omega = 90 rad/s
- air density: rho = 1.225 kg/m^3
- root cutout: 0.1R
- spanwise blade elements: 10
- azimuthal steps per revolution: 72
- collective pitch input: theta0_deg = 15

These values can be edited directly in the scripts.

--------------------------------------------------
Solver description
--------------------------------------------------

The model uses a blade-element-based force integration over:
- spanwise blade elements
- azimuthal stations over one rotor revolution

The flap and inflow trim is solved iteratively.
The unknown vector includes:
- blade initial flap angles
- blade initial flap rates
- one induced velocity term

A finite-difference Jacobian is used together with a damped Newton-style update to reduce the trim residual.

This makes the model compact and easy to inspect, while still retaining the key coupling between:
- blade kinematics
- rotor inflow
- aerodynamic loading
- flap response

--------------------------------------------------
Intended use cases
--------------------------------------------------

This repository is suitable for:
- aerodynamic model prototyping
- rotorcraft teaching demonstrations
- debugging of BEMT formulations
- comparison between standalone MATLAB and Simulink implementations
- testing of flap-trim solvers
- building a baseline model before moving to more advanced approaches

Examples of downstream extensions include:
- dynamic inflow models
- free-wake models
- multi-rotor interaction models
- controller integration
- real-time or reduced-order implementations
- parameter-identification studies

--------------------------------------------------
Limitations
--------------------------------------------------

This is a compact research demo and has several important limitations.

Current limitations include:
- single-rotor model only
- one induced-velocity unknown
- no dynamic inflow model
- no free-wake model
- no dynamic stall model
- no rotor-rotor aerodynamic interaction
- no comprehensive validation package included in the repository
- no guarantee of production-level robustness across all operating conditions

Accordingly, the repository should be treated as:
- a baseline implementation
- a research and teaching tool
- a starting point for further development

rather than a final high-fidelity solver.

--------------------------------------------------
Suggested workflow for users
--------------------------------------------------

If you are new to the repository, the recommended path is:
1. run the standalone MATLAB script first
2. verify that the basic solver converges
3. switch between linear and lookup modes
4. run the preprocessing script
5. compare the Simulink and standalone results
6. modify the rotor geometry and operating condition as needed

This makes it easier to debug the model before embedding it in a larger Simulink environment.

--------------------------------------------------
License
--------------------------------------------------

This repository is released under the MIT License.

--------------------------------------------------
Citation and acknowledgement
--------------------------------------------------

If this repository is useful in academic or engineering work, please cite the repository appropriately and acknowledge the original author.

If you adapt or extend the model, it is also helpful to indicate:
- what has been changed
- which workflow was used
- which airfoil, chord, and pretwist data were employed

to improve reproducibility.

--------------------------------------------------
Author
--------------------------------------------------

Ye Yuan
University of Glasgow
