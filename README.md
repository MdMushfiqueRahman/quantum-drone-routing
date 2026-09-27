# Quantum Drone Routing

This project explores energy-constrained waypoint selection and routing for drones using both classical and quantum optimization methods.

## What is implemented so far

- Defined waypoint coordinates and rewards
- Calculated Euclidean distance between waypoints
- Created a simple distance-based energy model
- Added takeoff and landing energy costs
- Added return-to-start energy calculation
- Added battery budget checking
- Implemented a brute-force classical route search
- Added reward maximization
- Added a tie-breaker to prefer lower-energy routes when rewards are equal

## Current Work

The current focus is to build a working classical baseline first, then formulate the problem as a QUBO and test quantum optimization approaches such as QAOA.

## Next Steps

- Add 2D route visualization with Matplotlib
- Develop the QUBO formulation
- Implement a QAOA-based solver
- Compare classical and quantum results
- Integrate the solver with PX4 SITL / Gazebo

## Run

```bash
python main.py