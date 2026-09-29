import itertools

# Define the waypoint rewards, energy consumption, and battery budget
waypoint_rewards = {
    "A": 8,
    "B": 12,
    "C": 10,
    "D": 7,
    "E": 9
}

waypoint_energy = {
    "A": 54,
    "B": 129,
    "C": 134,
    "D": 91,
    "E": 128
}

battery_budget = 250

# Define the penalty for violating the battery budget
penalty = 20

# Define the slack weights for the slack variables
slack_weights = [1, 2, 4, 8, 16, 32, 64, 123]

# Define the variables for the QUBO matrix
variables = ["A", "B", "C", "D", "E"]

# Add the slack variables to the variables list
for i in range(len(slack_weights)):
    variables.append(f"s{i}")

coefficients = {}

for waypoint in waypoint_energy:
    coefficients[waypoint] = waypoint_energy[waypoint]

# Add the slack variables to the coefficients dictionary
for i, weight in enumerate(slack_weights):
    coefficients[f"s{i}"] = weight

# Define the QUBO matrix
qubo = {}

# Iterate over the variables and add the linear terms to the QUBO matrix
for variable in variables:
    value = coefficients[variable]

    linear_term = penalty * (
        value**2 - 2 * battery_budget * value
    )

    if variable in waypoint_rewards:
        linear_term = linear_term - waypoint_rewards[variable]

    qubo[(variable, variable)] = linear_term

for i in range(len(variables)):
    for j in range(i + 1, len(variables)):
        variable_i = variables[i]
        variable_j = variables[j]

        quadratic_term = (
            2
            * penalty
            * coefficients[variable_i]
            * coefficients[variable_j]
        )

        qubo[(variable_i, variable_j)] = quadratic_term

print("QUBO model created successfully")
print("Number of variables:", len(variables))
print("Number of QUBO terms:", len(qubo))

# Define the best state and best cost
best_state = None
best_cost = float("inf")

# Iterate over all possible states and find the best state
for bits in itertools.product([0, 1], repeat=len(variables)):
    state = dict(zip(variables, bits))

    cost = 0

    for (var_i, var_j), value in qubo.items():
        cost = cost + value * state[var_i] * state[var_j]

    if cost < best_cost:
        best_cost = cost
        best_state = state

# Get the selected waypoints and the selected energy and reward
selected_waypoints = []

for waypoint in waypoint_rewards:
    if best_state[waypoint] == 1:
        selected_waypoints.append(waypoint)

# Get the selected energy and reward
selected_energy = sum(
    waypoint_energy[waypoint]
    for waypoint in selected_waypoints
)

selected_reward = sum(
    waypoint_rewards[waypoint]
    for waypoint in selected_waypoints
)

# Print the results
print("\nQUBO Verification Result:")
print("Selected Waypoints:", selected_waypoints)
print("Scaled Energy:", selected_energy)
print("Scaled Battery Budget:", battery_budget)
print("Total Reward:", selected_reward)
print("QUBO Cost:", best_cost)
print(
    "Constraint Status:",
    "Valid" if selected_energy <= battery_budget else "Invalid"
)