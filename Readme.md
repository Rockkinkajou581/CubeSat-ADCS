# ADCS — Attitude Determination and Control

Attitude determination and control system for PVDX CubeSat for Brown Space Engineering.

Upstream team repository: https://github.com/BrownSpaceEngineering/PVDX-ADCS

## Signal chain

NASA returns the orbital elements of the satellite. These are propagated to find ECI position. The magnetosphere finds the reference magnetic field at the satellite's ECI position. This is fed into the MUKF along with photodiode and magnetometer measurements. The returning estimated quaternion and omega are fed into pointing_error to find the desired quaternion, which is then fed into the PD controller to command a torque.

`propagate_orbital_elements` → `magnetosphere` → `MUKF` → `pointing_error` →
`PD_controller`

`Bdot` is not part of that chain — it's a separate detumble mode that runs on
magnetometer data alone, before the pointing loop takes over.

## Demo

MATLAB Satellite Scenario Viewer from Simulink model, 50x real time, tracking Providence Rhode Island.

<p align="center">
  <img src="MATLAB/docs/providence-pointing.gif" width="600" alt="Providence pointing"><br>
  <em>Sensor footprint tracking Providence, 50× real time</em>
</p>

## Running it

Open [`CubeSatSimulationProject.prj`](MATLAB/Simulink/CubeSat%20Simulation%20Project-3/CubeSatSimulationProject.prj),
then open `CubesatModel/asbCubeSat.slx` and run. The algorithms in
[`MATLAB/Algorithms/`](MATLAB/Algorithms) are called from MATLAB Function blocks
inside that model.

Requires MATLAB, Simulink, Aerospace Blockset, and Aerospace Toolbox.

To run the Monte Carlo study, open the file in matlab with the Simulink model open, then call

```matlab
r = runCubeSatMonteCarlo(NumTrials=100, StopTime=900, SettleTolDeg=1.5);
summary(r)
```

## Repository layout

| Path | Contents |
| --- | --- |
| [`MATLAB/Algorithms/`](MATLAB/Algorithms) | The algorithms, as standalone `.m` files |
| [`MATLAB/Simulink/CubeSat Simulation Project-3/`](MATLAB/Simulink/CubeSat%20Simulation%20Project-3) | The full Simulink project and model |
| [`MATLAB/docs/`](MATLAB/docs) | Demo gif |
| [`C/`](C) | C port, other algorithms to be added |


## What I did / Explanation

### Orbit propagation — [`propagate_orbital_elements.m`](MATLAB/Algorithms/propagate_orbital_elements.m), [`test_propagate_orbital_elements.m`](MATLAB/Algorithms/test_propagate_orbital_elements.m)
- Finds orbital elements of satellite at some future time through solving Kepler's equation with Newton's method. Tested against Aerospace Toolbox functions `propagateOrbit` / `ijk2keplerian` with an eccentricity between 0.05-0.95 with 1 minute steps over a 2 hour arc, accurate to 1e-8 deg.
- Ported to C in [`C/propagate_orbital_elements.c`](C/propagate_orbital_elements.c) for flight software, with a `testKepler` helper that feeds 2 hours of true anomaly to a text file to diff against the MATLAB version.

### Geomagnetic field — [`magnetosphere.m`](MATLAB/Algorithms/magnetosphere.m)

WMM2025 spherical-harmonic field model, originally written by @contextneeded.
- I integrated it into the Simulink control model with the UKF and pointing (feeds into it the reference ECI vector), debugged Simulink compile issues and NED/ECI bug, and tested it against MATLAB's wrldmagm function. 

### Attitude estimation — [`MUKF.m`](MATLAB/Algorithms/MUKF.m)

Multiplicative unscented Kalman filter, 6-state error
(3 attitude + 3 gyro bias), 13 sigma points, switching between a two-vector
(sun + magnetometer) and magnetometer-only measurement mode, written by @david-man. I got it to work with the PD controller in the Simulink model. Specific fixes I did:

- **`omega_icrf2b` 0 in Simulink** - fed in w_eci2b output from the dynamics block, changing the convention from NED and reconfigured quaternion outputs to match, to avoid quaternion finite difference inaccuracy
- **Quaternion backwards bug** - identified and helped fix a bug in quaternions being inverted (active vs passive convention) causing w_estimate to be off.
- **1 vector model drift with PD** - debugged an issue with 1 vec model drifting up to 30 deg of accuracy when used with PD controller due to generated w_measured bug
- **Verified Accuracy with PD controller** - wrote a small script [`Error.m`](MATLAB/Algorithms/Error.m) to find rotation error between q_est and q_true; tested MUKF accuracy over 2 hour orbits with 1 vector mode (magnetometer only), 2 vector, nadir vs Providence pointing, and PD controller fed w_estimate vs w_true. Ran 100 simulation Monte Carlo tests with random initial orientations and velocity to test PD settling with MUKF.

### Pointing error — [`pointing_error.m`](MATLAB/Algorithms/pointing_error.m)
- Builds the desired body frame for a ground-station target (Providence,
RI) from the satellite's ECI position and velocity, converts it to a
quaternion, and returns the error quaternion in body coordinates.
- Handles quaternion sign flip at 180 degree: the commanded quaternion is compared against the previous timestep's via a persistent variable and sign-flipped when the dot product goes negative, so the target attitude doesn't jump discontinuously between equivalent representations.
- Tested against Aerospace Toolbox nadir-pointing block. Added Providence pointing capability by converting Providence ECEF coordinates to ECI each timestep.

### Pointing control — [`PD_controller.m`](MATLAB/Algorithms/PD_controller.m)

Co-developed with @aPizzaRat.
- Converts the error quaternion and body rates to axis-angle, applies proportional and derivative gains per axis, scales by the measured inertia tensor, and saturates the commanded torque.
- Takes the shortest-rotation so that when the scalar of q_error is negative, it negates the quaternion and has small-angle guards on both the axis extraction and the rate normalization.

### Detumbling — [`Bdot.m`](MATLAB/Algorithms/Bdot.m)

Implemented B-dot control law
- Uses finite-differences in the body-frame magnetic field
to command a magnetic dipole opposing the rate of change.

### Monte Carlo verification — [`runCubeSatMonteCarlo.m`](MATLAB/Simulink/CubeSat%20Simulation%20Project-3/runCubeSatMonteCarlo.m)

Wrote with Claude. 
- Configurable trial count, stop time, attitude sampling mode (uniform or
axis-angle with tilt bounds), rate range, settling tolerance, steady-state
window, and RNG seed; returns one row per trial and plots the summary.
Initial conditions are injected with `Simulink.SimulationInput`, so the model on
disk is never modified.

## Simulink model

[`MATLAB/Simulink/CubeSat Simulation Project-3/`](MATLAB/Simulink/CubeSat%20Simulation%20Project-3) is built on the MathWorks Aerospace Blockset
CubeSat Simulation project. I integrated all the above algorithms into the full control loop for the satellite.
