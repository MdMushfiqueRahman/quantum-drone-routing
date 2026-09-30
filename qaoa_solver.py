from qiskit.primitives import StatevectorSampler
from qiskit_optimization import QuadraticProgram
from qiskit_optimization.algorithms import MinimumEigenOptimizer
from qiskit_optimization.minimum_eigensolvers import QAOA
from qiskit_optimization.optimizers import COBYLA
from qubo_model import variables, qubo

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

#Create a StatevectorSampler
sampler = StatevectorSampler(seed=42)
print("StatevectorSampler ready")

# Create the quantum sampler
sampler = StatevectorSampler(seed=42)

# Classical optimizer used to tune QAOA parameters
optimizer = COBYLA(maxiter=100)

# Create QAOA
qaoa = QAOA(
    sampler=sampler,
    optimizer=optimizer,
    reps=1
)

# Connect QAOA to the QUBO optimization problem
qaoa_optimizer = MinimumEigenOptimizer(qaoa)

print("\nRunning QAOA...")

# Solve the problem
result = qaoa_optimizer.solve(qp)

print("\nQAOA Result:")
print(result)

# Extract QAOA waypoint decisions
qaoa_state = {
    variables[i]: int(round(result.x[i]))
    for i in range(len(variables))
}

# Get selected waypoints
selected_waypoints = [
    waypoint
    for waypoint in ["A", "B", "C", "D", "E"]
    if qaoa_state[waypoint] == 1
]

# Calculate energy and reward
from qubo_model import waypoint_energy, waypoint_rewards, battery_budget

selected_energy = sum(
    waypoint_energy[w] for w in selected_waypoints
)

selected_reward = sum(
    waypoint_rewards[w] for w in selected_waypoints
)

print("\nHuman-Readable QAOA Result:")
print("Selected Waypoints:", selected_waypoints)
print("Scaled Energy:", selected_energy)
print("Scaled Battery Budget:", battery_budget)
print("Total Reward:", selected_reward)
print("QUBO Cost:", result.fval)

if selected_energy <= battery_budget:
    print("Constraint Status: Valid")
else:
    print("Constraint Status: Invalid")