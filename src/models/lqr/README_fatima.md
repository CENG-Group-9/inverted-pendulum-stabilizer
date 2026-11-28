# Hybrid LQR Inverted Pendulum Controller

**Project:** CSE396 – Inverted Pendulum Stabilizer
**Group:** Group 9 – Gebze Technical University
**Contributor:** Fatima Zehra

## Overview

This module implements a hybrid controller for inverted
pendulum swing-up and stabilization using the shared
MuJoCo simulation environment.

The controller combines energy-based swing-up with
discrete-time LQR stabilization, velocity-aware switching
and predictive track protection.

## Features

- Nonlinear mathematical modelling and linearization
- Discrete-time LQR stabilization
- Energy-based swing-up
- Velocity-aware hybrid switching
- Predictive track protection
- Swing-up parameter optimization
- Randomized friction robustness testing
- Sensor noise robustness testing
- Interactive MuJoCo demonstration

## Environment Setup

From the repository root:

```bash
conda env create -f src/models/environment.yml
conda activate inverted-pendulum
cd src/models
```

Skip environment creation if it already exists.

## Usage

Run the following commands from `src/models`.

### Graphical Demonstration

```bash
python -m lqr.fatima_demo
```

With randomized friction:

```bash
python -m lqr.fatima_demo --randomize-friction
```

### Performance Evaluation

```bash
python -m lqr.fatima_evaluate --episodes 20
```

With randomized friction:

```bash
python -m lqr.fatima_evaluate --episodes 20 --randomize-friction --seed 500
```

### Mathematical Model Validation

```bash
python -m lqr.fatima_validate
```

### Sensor Noise Evaluation

```bash
python -m lqr.fatima_noise_test --episodes 20 --seed 700
```

## Experimental Results

| Test | Successful Episodes | Track Failures |
|---|---:|---:|
| Nominal friction | 20/20 | 0 |
| Randomized friction | 39/40 | 0 |
| Sensor noise evaluation | 160/160 | 0 |
| Graphical demonstration | 17/17 | 0 |

Under combined high sensor noise and randomized friction:

- Mean swing-up time: 1.996 seconds
- Mean balancing angle RMS error: 0.2536 degrees
- Mean cart position RMS error: 6.2498 mm
- Mean motor saturation: 0.69%

The sensor noise evaluation includes eight conditions
with 20 episodes per condition, using seeds 700–719.

Experimental CSV files are available in `src/models`.

## Limitations

- Validation has been performed in simulation only.
- Sensor noise levels are assumed experimental values.
- Hardware delays and sensor bias have not been tested.
- Some friction conditions can cause delayed or
  unsuccessful swing-up, including seed 610.
- Predictive track protection is not a formal safety
  guarantee.

## Future Work

- Update physical parameters after hardware measurements.
- Recalculate LQR gains and retune controller parameters
  if necessary.
- Integrate with ESP32 firmware and the project GUI.
- Perform physical hardware validation.

The implementation uses the group's shared simulation
environment and supports future collaborative development.