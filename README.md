# Quantum Drone Routing

This project explores energy-constrained waypoint selection and routing for drones using classical and quantum optimization methods.

The long-term goal is to develop an optimization module that can select useful waypoints under a limited battery budget, determine an efficient route, and later connect the solver output to a simulated and real UAV system.

## Problem Overview

Each candidate waypoint has:

- A coordinate
- A reward or mission priority
- An estimated energy cost

The goal is to select useful waypoints while remaining within the drone's battery budget.

This problem is related to:

- Knapsack optimization for waypoint selection
- Traveling Salesman Problem (TSP) for route ordering
- Orienteering for combining reward-based selection and routing under an energy constraint

The current prototype uses synthetic waypoint rewards and estimated energy values for algorithm development and testing. These values can later be replaced or calibrated using real drone measurements.

## Classical Routing Baseline

`main.py` implements a brute-force classical routing baseline.

Current features include:

- Waypoint coordinates and rewards
- Euclidean distance calculation
- Distance-based travel energy
- Takeoff and landing energy
- Return-to-start energy
- Battery budget checking
- Brute-force route permutation search
- Reward maximization
- Lower-energy tie-breaking for equal-reward routes

### Current Classical Result

Battery budget: `25`

Best route:

`Start -> A -> D -> Start`

Energy: `22.74`

Reward: `15`

A 2D route visualization is generated using Matplotlib and saved as:

`best_route.png`

Classical results are also recorded in:

`results.csv`

## QUBO Formulation

`qubo_model.py` implements the first QUBO formulation of the waypoint-selection problem.

QUBO stands for:

**Quadratic Unconstrained Binary Optimization**

Each waypoint uses a binary decision variable:

- `1` = selected
- `0` = not selected

The current QUBO objective combines:

- Negative reward for reward maximization
- Battery constraint penalty
- Binary slack variables to represent unused battery capacity

### Current QUBO Model

- 5 waypoint variables
- 8 slack variables
- 13 total binary variables
- 91 QUBO terms
- 8192 possible binary states

An exact brute-force verifier evaluates all 8192 states.

### Exact QUBO Result

Selected waypoints:

`A, B`

Scaled energy:

`183 / 250`

Reward:

`20`

Constraint status:

`Valid`

## QAOA Prototype

`qaoa_solver.py` converts the QUBO model into a Qiskit `QuadraticProgram` and solves it using the Quantum Approximate Optimization Algorithm (QAOA).

Current tools include:

- Qiskit
- StatevectorSampler
- COBYLA optimizer
- Fixed initialization for reproducibility

The current QAOA experiments are simulated locally and do not use physical quantum hardware.

### QAOA Comparison

| Solver | Selected Waypoints | Scaled Energy | Reward | Constraint |
|---|---|---:|---:|---|
| Exact brute-force | A, B | 183 / 250 | 20 | Valid |
| QAOA p=1 | B, D | 220 / 250 | 19 | Valid |
| QAOA p=2 | B | 129 / 250 | 12 | Valid |

Under the current settings, QAOA with `p=1` produced a feasible near-optimal solution one reward point below the exact optimum.

Increasing the QAOA depth to `p=2` did not improve the solution under the current optimizer and initialization settings.

The comparison results are saved in:

`qaoa_comparison.csv`

## Current Limitation

The classical and quantum prototypes currently solve slightly different versions of the problem.

The classical solver includes route ordering and route-dependent travel energy.

The current QUBO model focuses on waypoint selection only.

Because of this, the current classical and QAOA results should not yet be treated as a direct classical-versus-quantum performance comparison.

## Current Progress

Completed:

- Classical brute-force routing baseline
- Distance-based energy model
- Takeoff and landing energy
- Return-to-start energy
- Battery budget checking
- Reward-based route optimization
- 2D route visualization
- CSV result logging
- QUBO waypoint-selection formulation
- Binary slack-variable formulation
- Exact QUBO verification
- Qiskit integration
- QAOA prototype
- QAOA p=1 and p=2 experiments
- Exact vs QAOA result comparison

## Next Steps

- Complete literature review and research summary
- Set up PX4 SITL and Gazebo
- Create a clean solver interface:

  `waypoints + battery budget -> selected/ordered waypoint list`

- Connect solver output to a simulated drone mission
- Extend the QUBO formulation toward route ordering
- Compare classical and quantum methods using the same optimization model
- Calibrate the energy model using real drone measurements
- Later connect the optimization module with Jetson Orin Nano, Pixhawk 6, and the physical UAV platform

## Run

Run the classical solver:

```bash
python main.py
