from qiskit.primitives import StatevectorSampler
from qiskit_optimization import QuadraticProgram
from qiskit_optimization.algorithms import MinimumEigenOptimizer
from qiskit_optimization.minimum_eigensolvers import QAOA
from qiskit_optimization.optimizers import COBYLA
from qubo_model import (
    variables,
    qubo,
    waypoint_energy,
    waypoint_rewards,
    battery_budget,
    best_state,
    best_cost
)
import numpy as np
import csv


print("Loaded QUBO variables:", len(variables))
print("Loaded QUBO terms:", len(qubo))

# Create a Qiskit QuadraticProgram
qp = QuadraticProgram()

# Add all binary variables
for variable in variables:
    qp.binary_var(name=variable)

# Separate linear and quadratic QUBO terms
linear = {}
quadratic = {}

for (var_i, var_j), coefficient in qubo.items():
    if var_i == var_j:
        linear[var_i] = coefficient
    else:
        quadratic[(var_i, var_j)] = coefficient

# QUBO is a minimization problem
qp.minimize(
    linear=linear,
    quadratic=quadratic
)

print("\nQiskit QuadraticProgram created successfully")
print("Number of Qiskit variables:", qp.get_num_vars())


# Turn a state dictionary into a clean, human-readable result
def summarize_state(state, qubo_cost):
    # Get the selected real waypoints (ignore the slack variables)
    selected_waypoints = [
        waypoint
        for waypoint in waypoint_rewards
        if state[waypoint] == 1
    ]

    # Calculate the scaled energy and total reward
    selected_energy = sum(
        waypoint_energy[w] for w in selected_waypoints
    )
    selected_reward = sum(
        waypoint_rewards[w] for w in selected_waypoints
    )

    # Check the battery budget constraint
    constraint_valid = selected_energy <= battery_budget

    return (
        selected_waypoints,
        selected_energy,
        selected_reward,
        qubo_cost,
        constraint_valid
    )


# Run QAOA with a chosen number of repetitions (p) and return the result
def run_qaoa(reps):
    # Create the quantum sampler and the classical parameter optimizer
    sampler = StatevectorSampler(seed=42)
    optimizer = COBYLA(maxiter=100)

    # Two parameters (beta and gamma) per repetition, all starting at 0.5
    initial_point = np.full(2 * reps, 0.5)

    # Create QAOA with the requested number of repetitions
    qaoa = QAOA(
        sampler=sampler,
        optimizer=optimizer,
        reps=reps,
        initial_point=initial_point
    )

    # Connect QAOA to the QUBO problem and solve it
    qaoa_optimizer = MinimumEigenOptimizer(qaoa)
    result = qaoa_optimizer.solve(qp)

    # Convert the QAOA bit decisions into a state dictionary
    qaoa_state = {
        variables[i]: int(round(result.x[i]))
        for i in range(len(variables))
    }

    return summarize_state(qaoa_state, result.fval)


# 1. Exact brute-force QUBO result (reuse best_state and best_cost)
exact_result = summarize_state(best_state, best_cost)

# 2. QAOA with reps=1 and 3. QAOA with reps=2
print("\nRunning QAOA (p=1)...")
qaoa_p1_result = run_qaoa(1)

print("Running QAOA (p=2)...")
qaoa_p2_result = run_qaoa(2)

# Collect all results so we can print and save them the same way
comparison = [
    ("Exact brute-force", exact_result),
    ("QAOA p=1", qaoa_p1_result),
    ("QAOA p=2", qaoa_p2_result)
]

# Print a clean comparison
print("\n=== Solver Comparison ===")

for solver_name, result in comparison:
    selected_waypoints, energy, reward, cost, valid = result

    print("\nSolver:", solver_name)
    print("  Selected Waypoints:", selected_waypoints)
    print("  Scaled Energy:", energy)
    print("  Battery Budget:", battery_budget)
    print("  Reward:", reward)
    print("  QUBO Cost:", cost)
    print("  Constraint Valid:", "Valid" if valid else "Invalid")

# Save the comparison to a CSV file
with open("qaoa_comparison.csv", "w", newline="") as file:
    writer = csv.writer(file)

    writer.writerow([
        "Solver",
        "Selected Waypoints",
        "Scaled Energy",
        "Battery Budget",
        "Reward",
        "QUBO Cost",
        "Constraint Valid"
    ])

    for solver_name, result in comparison:
        selected_waypoints, energy, reward, cost, valid = result

        writer.writerow([
            solver_name,
            ", ".join(selected_waypoints),
            energy,
            battery_budget,
            reward,
            cost,
            "Valid" if valid else "Invalid"
        ])

print("\nComparison saved to qaoa_comparison.csv")
