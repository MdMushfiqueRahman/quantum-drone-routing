import itertools

# Base node is "0". Clients are A, B, and C.
clients = ["A", "B", "C"]

# Reward for visiting each client
reward = {
    "A": 8,
    "B": 6,
    "C": 10
}

# Energy (Wh) needed to collect from each client
collection_energy = {
    "A": 1,
    "B": 1,
    "C": 2
}

# Time (minutes) needed to collect from each client
collection_time = {
    "A": 2,
    "B": 1,
    "C": 3
}

# Symmetric travel energy (Wh) between two nodes
travel_energy = {
    ("0", "A"): 2, # From base node to client A and it's energy is 2 Wh
    ("0", "B"): 3, # From base node to client B and it's energy is 3 Wh
    ("0", "C"): 4, # From base node to client C and it's energy is 4 Wh
    ("A", "B"): 2, # From client A to client B and it's energy is 2 Wh
    ("A", "C"): 3, # From client A to client C and it's energy is 3 Wh
    ("B", "C"): 2 # From client B to client C and it's energy is 2 Wh
}

# Symmetric travel time (minutes) between two nodes
travel_time = {
    ("0", "A"): 1, # From base node to client A and it's time is 1 minute
    ("0", "B"): 2, # From base node to client B and it's time is 2 minutes
    ("0", "C"): 3, # From base node to client C and it's time is 3 minutes
    ("A", "B"): 1, # From client A to client B and it's time is 1 minute
    ("A", "C"): 3, # From client A to client C and it's time is 3 minutes
    ("B", "C"): 1 # From client B to client C and it's time is 1 minute
}

# Budgets / Our Limits: Energy <= 12 Wh and Time <= 10 minutes
energy_budget = 12
time_budget = 10


# Look up a symmetric travel value for an edge between two nodes
def get_travel(values, node1, node2):
    if (node1, node2) in values:
        return values[(node1, node2)]
    return values[(node2, node1)]


# Build the full ordered path: start at 0, visit clients, return to 0
def build_path(route):
    return ["0"] + list(route) + ["0"]


# Calculate the total travel + collection energy for a route
def calculate_energy(route):
    path = build_path(route)

    total = 0

    # Travel energy along each leg of the path
    for i in range(len(path) - 1):
        total = total + get_travel(travel_energy, path[i], path[i + 1])

    # Collection energy at each visited client
    for client in route:
        total = total + collection_energy[client]

    return total


# Calculate the total travel + collection time for a route
def calculate_time(route):
    path = build_path(route)

    total = 0

    # Travel time along each leg of the path
    for i in range(len(path) - 1):
        total = total + get_travel(travel_time, path[i], path[i + 1])

    # Collection time at each visited client
    for client in route:
        total = total + collection_time[client]

    return total


# Calculate the total reward for a route
def calculate_reward(route):
    total = 0

    for client in route:
        total = total + reward[client]

    return total


# Build a readable label like "0 -> B -> C -> 0"
def route_label(route):
    return " -> ".join(build_path(route))


# Enumerate every ordered non-empty subset of the clients using permutations
routes = []

for size in range(1, len(clients) + 1):
    for route in itertools.permutations(clients, size):
        routes.append(route)

# Print all candidate routes
print("All candidate routes:")
print("Number of routes:", len(routes))
print()

best_reward = -1
best_routes = []

for route in routes:
    route_reward = calculate_reward(route)
    route_energy = calculate_energy(route)
    route_time = calculate_time(route)

    feasible = route_energy <= energy_budget and route_time <= time_budget

    print(
        route_label(route),
        "| Reward:", route_reward,
        "| Energy:", route_energy, "Wh",
        "| Time:", route_time, "min",
        "| Feasible:", "Yes" if feasible else "No"
    )

    # Track the best feasible route(s) by maximum reward
    if feasible:
        if route_reward > best_reward:
            best_reward = route_reward
            best_routes = [route]
        elif route_reward == best_reward:
            best_routes.append(route)

# Print the optimal feasible route(s)
print()
print("Best reward:", best_reward)
print("Optimal route(s):")

for route in best_routes:
    print(
        route_label(route),
        "| Energy:", calculate_energy(route), "Wh",
        "| Time:", calculate_time(route), "min"
    )
