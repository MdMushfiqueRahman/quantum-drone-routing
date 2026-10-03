# Quantum Drone Routing

Hybrid classical and quantum optimization for energy-constrained UAV mission routing.

This project investigates how a drone can choose and order mission waypoints while respecting limited energy and mission-time budgets. The current work combines classical optimization, QUBO formulation, exact verification, and QAOA simulation in Qiskit.

The project is being developed as a proof-of-concept for UAV mission planning and future integration with simulated and physical autonomous systems.

> **Current status:** Classical routing, direct routing QUBO formulation, exact QUBO verification, and QAOA `p=1` / `p=2` experiments are working. A clean classical solver interface for future UAV / SITL integration is now implemented. The next phase is connecting this interface to ArduPilot / ArduCopter SITL.

---

## Research Goal

A UAV may have more possible mission locations than it can visit with its available battery and time.

The optimization problem is therefore:

**Which waypoints should the UAV visit, and in what order, to maximize mission reward while satisfying route continuity, energy, and time constraints?**

Each client or waypoint can contain information such as:

- Mission reward or priority
- Location
- Collection energy
- Collection time
- Travel energy
- Travel time

This combines ideas from the Traveling Salesman Problem, Orienteering, Knapsack optimization, and resource-constrained vehicle routing.

The long-term interface is:

```text
Mission waypoints + resource budgets
                |
                v
     Classical / Quantum Solver
                |
                v
      Validated waypoint route
                |
                v
        UAV mission system
```

---

## Project Development

The repository currently contains two stages of development.

### 1. Initial Waypoint Prototype

The original prototype uses five synthetic waypoints and focuses on understanding reward-based waypoint selection and distance-dependent energy consumption.

It includes:

- Euclidean waypoint distance
- Travel-energy estimation
- Takeoff and landing energy
- Battery-budget validation
- Brute-force route enumeration
- Reward maximization
- Lower-energy tie breaking
- Matplotlib route visualization
- CSV experiment output
- Initial selection-based QUBO and QAOA experiments

Example classical result:

```text
Battery budget: 25

Best route:
Start -> A -> D -> Start

Energy: 22.74
Reward: 15
```

The visualization is stored in:

```text
best_route.png
```

This prototype was useful for developing the first energy model and solver workflow, but its original QUBO formulation focused mainly on waypoint selection rather than complete route continuity.

---

## 2. Distributed Routing Formulation

The newer implementation is located in:

```text
distributed_routing/
```

This version directly models UAV routing with path-continuity, energy, and time constraints.

The current teaching instance contains:

```text
Base: 0

Clients:
A
B
C

Rewards:
A = 8
B = 6
C = 10

Energy budget = 12 Wh
Time budget   = 10 min
```

Synthetic values are intentionally used at this stage so the optimization model can be verified before introducing real UAV telemetry and measured energy data.

---

## Classical Routing Verification

`distributed_routing/classical_solver.py` enumerates all possible ordered non-empty client subsets.

For three clients, there are only 15 possible routes, allowing the optimum to be verified exactly.

### Exact Classical Result

```text
Maximum reward: 16

Optimal routes:

0 -> B -> C -> 0
0 -> C -> B -> 0

Energy: 12 Wh
Time:   10 min
```

Both routes achieve the maximum feasible reward while exactly using the available energy and time budgets.

This solver provides a classical ground-truth reference for validating the QUBO formulation.

---

## Direct Routing QUBO

`distributed_routing/qubo_model.py` converts the routing problem into a Quadratic Unconstrained Binary Optimization model.

The direct formulation includes:

| Variable type | Count |
|---|---:|
| Directed route-edge variables | 12 |
| Client-selection variables | 3 |
| Energy slack bits | 4 |
| Time slack bits | 4 |
| **Total binary variables** | **23** |

The QUBO combines:

- Negative mission reward
- Base departure constraint
- Base return constraint
- Client arrival/departure consistency
- Small-instance subtour prevention
- Energy-budget equality using binary slack
- Time-budget equality using binary slack

A penalty weight of:

```text
P = 25
```

is used for the current teaching instance.

### Exact QUBO Verification

The QUBO was verified using exact classical enumeration.

```text
Minimum QUBO score: -16
Best reward: 16
Energy: 12 Wh
Time: 10 min

Optimal routes:

0 -> B -> C -> 0
0 -> C -> B -> 0
```

For the optimum:

```text
Reward contribution = -16
Route penalty       = 0
Loop penalty        = 0
Energy penalty      = 0
Time penalty        = 0
```

The QUBO optimum therefore matches the independent classical routing solution.

---

## QAOA Experiment

To keep the first local-simulator QAOA experiment computationally manageable, a reduced two-client instance containing only clients `A` and `C` is used.

`distributed_routing/qaoa_solver.py` implements this experiment.

### Reduced Problem

```text
Nodes: 0, A, C

Energy budget = 12 Wh
Time budget   = 10 min
Penalty P     = 25
```

The direct binary encoding contains:

| Variable type | Count |
|---|---:|
| Directed route-edge variables | 6 |
| Client-selection variables | 2 |
| Energy slack bits | 4 |
| Time slack bits | 4 |
| **Total logical qubits** | **16** |

The resulting Ising Hamiltonian currently contains 121 Pauli terms.

---

## Exact Reference for the QAOA Instance

Exact enumeration of all `2^16 = 65,536` binary assignments gives:

```text
Optimal route:
0 -> C -> 0

Reward: 10
Energy: 10 Wh
Time: 9 min

Energy slack: 2
Time slack:   1

QUBO score: -10
```

This exact result is used as the reference when analyzing QAOA samples.

---

## QAOA Implementation

The quantum optimization experiment uses:

- Qiskit
- QAOA
- `StatevectorSampler`
- COBYLA classical optimizer
- QAOA circuit simulation
- AerSimulator for final measurement sampling
- Fixed random seed for reproducibility

QAOA is a hybrid quantum-classical algorithm:

```text
QUBO
  |
  v
Ising Hamiltonian
  |
  v
Parameterized QAOA circuit
  |
  v
Quantum-circuit simulation
  |
  v
Classical COBYLA parameter update
  |
  v
Optimized circuit
  |
  v
Measurement samples
  |
  v
Route + constraint validation
```

All current quantum-circuit execution is simulated locally on a classical computer.

No physical quantum hardware is currently used.

---

## QAOA p=1 and p=2 Results

Current experiment settings:

```text
Shots: 1024
Seed: 42
COBYLA maximum iterations: 30
```

### p = 1

The first QAOA layer experiment sampled the same physical mission route as the exact solution:

```text
Route:
0 -> C -> 0

Reward: 10
Energy: 10 Wh
Time: 9 min
```

Representative result:

```text
Mission-feasible sample fraction: ~0.0107
Exact QUBO optimum sampled: No
```

The route itself was optimal for the original UAV mission, but the associated slack bits did not represent the exact QUBO optimum.

### p = 2

Increasing QAOA depth to two layers produced:

```text
Route:
0 -> C -> 0

Reward: 10
Energy: 10 Wh
Time: 9 min
```

Current representative metrics:

```text
Mission-feasible samples: 15 / 1024
Mission-feasible sample fraction: 0.0146
QUBO-feasible sample fraction: 0.0
Optimal-sample fraction: 0.0
Cost-function evaluations: 30
Total local simulation time: approximately 21 seconds
```

The `p=2` experiment therefore found the same optimal physical UAV mission and produced a slightly larger mission-feasible sample fraction than the current `p=1` experiment.

However, neither experiment sampled the exact 16-bit QUBO optimum under the current settings.

This distinction is important:

```text
Mission feasible
!=
Exact QUBO optimum
```

A sampled route may satisfy the original drone mission constraints even when its binary slack variables do not satisfy the full QUBO equality encoding.

---

## QAOA Runtime Optimization

During early experiments, the first QAOA objective evaluation was extremely slow even for the 16-qubit model.

Runtime diagnostics isolated the bottleneck to:

```text
qaoa.compute_minimum_eigenvalue(...)
```

The issue was reduced by using a flattened `QAOAAnsatz`, avoiding large parameter-binding overhead from the nested circuit representation.

After this change, the complete QAOA experiment runs in practical time on a local laptop.

For the current `p=2` configuration, the circuit contains:

```text
Logical qubits: 16
QAOA parameters: 4
Hamiltonian Pauli terms: 121
```

This optimization changes the circuit representation used by Qiskit but does not change the mathematical QUBO problem.

---

## Validation Strategy

Quantum samples are never accepted only because they have a low penalized objective value.

Each sampled state is decoded and independently checked against the original mission constraints.

The implementation distinguishes:

| Validation level | Meaning |
|---|---|
| Mission feasible | Route structure, energy budget, and time budget are valid |
| QUBO feasible | Route constraints and slack-variable equalities are valid |
| Exact QUBO optimum | Sample exactly matches the minimum-QUBO reference state |

This keeps experimental reporting transparent and avoids treating an invalid binary assignment as a valid UAV mission.

---

## Classical Solver Interface

`distributed_routing/solver_interface.py` provides a clean mission-planning entry point for future UAV / SITL code.

- **Input:** energy and time budgets.
- Uses the verified exact classical routing baseline to search all feasible routes.
- Returns **one deterministic optimal route**.
- **Tie-breaking** among equal-reward routes: lower energy, then lower time, then lexicographic route order.
- Independently validates route structure, energy, and time (recomputed from the route) before returning the mission.

The interface intentionally uses the verified classical baseline rather than QAOA; QAOA remains an experimental optimization backend and is not used for UAV control.

Default teaching-instance output:

```text
Route: 0 -> B -> C -> 0
Reward: 16
Energy: 12 Wh
Time: 10 min
Valid: True
```

---

## Repository Structure

```text
quantum-drone-routing/
|
|-- main.py
|   Initial classical five-waypoint routing prototype
|
|-- qubo_model.py
|   Initial waypoint-selection QUBO prototype
|
|-- qaoa_solver.py
|   Initial QAOA waypoint-selection experiment
|
|-- best_route.png
|   Initial classical route visualization
|
|-- results.csv
|   Classical experiment output
|
|-- qaoa_comparison.csv
|   Initial QAOA comparison output
|
|-- distributed_routing/
|   |
|   |-- classical_solver.py
|   |   Exact routing baseline for the teaching instance
|   |
|   |-- qubo_model.py
|   |   23-variable direct routing QUBO and exact verifier
|   |
|   |-- qaoa_solver.py
|   |   Reduced 16-qubit QAOA experiment
|   |
|   `-- solver_interface.py
|       Clean mission-planning interface that returns one deterministic validated route
|
`-- README.md
```

The original files are intentionally retained because they document the progression from waypoint selection toward direct route optimization.

---

## Current Progress

| Milestone | Status |
|---|---|
| Classical waypoint-routing prototype | Complete |
| Distance-based energy model | Complete |
| Route visualization | Complete |
| Classical exact routing baseline | Complete |
| Direct routing QUBO | Complete |
| Energy and time slack encoding | Complete |
| Exact QUBO verification | Complete |
| QUBO-to-Ising conversion | Complete |
| QAOA local simulation | Complete |
| QAOA `p=1` experiment | Complete |
| QAOA `p=2` experiment | Complete |
| Mission-vs-QUBO validation | Complete |
| Clean classical solver interface | Complete |
| ArduPilot SITL integration | Planned |
| Solver-to-UAV mission interface | Planned |
| Real energy-model calibration | Planned |
| Physical UAV integration | Future work |

---

## UAV Integration Direction

The optimization layer is being designed as a high-level mission-planning component.

The intended flow is:

```text
Mission / waypoint data
          |
          v
 Classical or QAOA optimizer
          |
          v
   Route validation layer
          |
          v
 Ordered waypoint mission
          |
          v
 ArduPilot / ArduCopter SITL
          |
          v
 Physical UAV integration
```

The target autonomous platform includes an onboard companion computer and Pixhawk-based flight control.

Future integration work will focus on ArduPilot/ArduCopter simulation before any physical flight testing.

---

## Classical vs Quantum Interpretation

The purpose of the current project is not to claim that QAOA outperforms classical optimization.

For the small teaching instances used here, exact classical enumeration is simple and provides the ground truth.

The current goals are instead to:

1. formulate the UAV routing problem correctly as a QUBO,
2. verify that formulation classically,
3. demonstrate a working quantum optimization pipeline,
4. study how QAOA behaves as the routing problem becomes larger,
5. investigate when a hybrid classical-quantum approach may become useful.

Performance and scalability studies on larger routing instances are future research tasks.

---

## Running the Project

Activate the project Python environment first.

Run the original classical routing prototype:

```bash
python main.py
```

Run the direct-routing classical baseline:

```bash
python distributed_routing/classical_solver.py
```

Run the full direct-routing QUBO verifier:

```bash
python distributed_routing/qubo_model.py
```

Run the reduced QAOA experiment:

```bash
python distributed_routing/qaoa_solver.py
```

---

## Current Research Questions

The project is moving toward several larger questions:

- How should UAV energy consumption be modeled using real telemetry?
- How does QAOA performance change as the number of route variables increases?
- Can the encoding be reduced to use fewer logical qubits?
- How should subtour constraints be represented for larger routing instances?
- When is a quantum or hybrid solver worth invoking instead of a classical method?
- Can optimization be integrated with real-time UAV replanning?
- How should solver output be safely validated before being sent to an autonomous vehicle?

---

## Next Steps

The clean classical solver interface is now implemented. The immediate next integration milestone is **ArduPilot / ArduCopter SITL**, consuming the validated ordered route produced by `solve_mission()`.

The broader development direction is:

```text
Validated optimizer
       |
       v
Clean solver interface
       |
       v
ArduPilot SITL mission
       |
       v
Classical vs QAOA experiments
       |
       v
Realistic energy measurements
       |
       v
Larger and multi-UAV routing problems
```

Planned work includes:

- Connect the implemented solver interface (`solve_mission`: `energy/time budgets -> validated ordered route`) to ArduPilot/ArduCopter SITL
- Visualize optimized missions in simulation
- Replace synthetic energy values with calibrated UAV measurements
- Test larger routing instances
- Study improved QAOA parameter initialization and optimization
- Investigate reduced-variable QUBO encodings
- Compare classical and quantum methods on the same problem instances
- Investigate hybrid decision strategies for choosing when a quantum solver is useful
- Extend toward multi-UAV mission allocation and replanning

---

## Research Status

This repository is an active research prototype.

The current results demonstrate:

```text
Classical routing
      +
Direct QUBO formulation
      +
Exact mathematical verification
      +
QAOA simulation
      +
Explicit mission validation
```

The present experiments are intended as a foundation for larger hybrid quantum-classical UAV routing studies rather than evidence of quantum computational advantage.
