import itertools

"""
Clean solver interface between the optimization code and the future
UAV / SITL mission layer.

solve_mission() performs the same exact classical route search as the current
classical baseline (distributed_routing/classical_solver.py) and returns ONE
deterministic optimal route in a simple dictionary that downstream mission code
(e.g. ArduPilot / ArduCopter SITL, added later) can consume.

No ArduPilot, MAVLink, SITL, Qiskit, or QAOA code is included at this stage.
"""



# Teaching-instance data (identical values to classical_solver.py).
clients = ["A", "B", "C"]

reward = {
    "A": 8,
    "B": 6,
    "C": 10
}

collection_energy = {
    "A": 1,
    "B": 1,
    "C": 2
}

collection_time = {
    "A": 2,
    "B": 1,
    "C": 3
}

# Symmetric travel energy (Wh) between two nodes.
travel_energy = {
    ("0", "A"): 2,
    ("0", "B"): 3,
    ("0", "C"): 4,
    ("A", "B"): 2,
    ("A", "C"): 3,
    ("B", "C"): 2
}

# Symmetric travel time (minutes) between two nodes.
travel_time = {
    ("0", "A"): 1,
    ("0", "B"): 2,
    ("0", "C"): 3,
    ("A", "B"): 1,
    ("A", "C"): 3,
    ("B", "C"): 1
}

# Default budgets for the teaching instance.
DEFAULT_ENERGY_BUDGET = 12
DEFAULT_TIME_BUDGET = 10


def get_travel(values, node1, node2):
    """Look up a symmetric travel value for an edge between two nodes."""
    if (node1, node2) in values:
        return values[(node1, node2)]
    return values[(node2, node1)]


def build_path(route):
    """Full ordered path: start at base 0, visit clients, return to 0."""
    return ["0"] + list(route) + ["0"]


def calculate_energy(route):
    """Total travel + collection energy for a route (clients only, no base)."""
    path = build_path(route)
    total = 0
    for i in range(len(path) - 1):
        total += get_travel(travel_energy, path[i], path[i + 1])
    for client in route:
        total += collection_energy[client]
    return total


def calculate_time(route):
    """Total travel + collection time for a route (clients only, no base)."""
    path = build_path(route)
    total = 0
    for i in range(len(path) - 1):
        total += get_travel(travel_time, path[i], path[i + 1])
    for client in route:
        total += collection_time[client]
    return total


def calculate_reward(route):
    """Total reward for a route."""
    return sum(reward[client] for client in route)


def is_valid_mission(path, energy, time, energy_budget, time_budget):
    """Independently validate the original mission constraints for a path.

    Energy and time are recomputed here from the route's middle client sequence
    rather than trusting the values reported by solve_mission().
    """
    # 1. Path must start and end at base 0.
    if len(path) < 2 or path[0] != "0" or path[-1] != "0":
        return False

    middle = path[1:-1]

    # 2. Every middle node must be a known client.
    if any(node not in clients for node in middle):
        return False

    # 3. No client visited more than once.
    if len(set(middle)) != len(middle):
        return False

    # 4. Recompute energy and time independently from the middle sequence.
    recomputed_energy = calculate_energy(middle)
    recomputed_time = calculate_time(middle)

    # 5 + 6. Recomputed totals must respect the budgets.
    if recomputed_energy > energy_budget or recomputed_time > time_budget:
        return False

    # 7. Independent recomputation must match the solver-reported values.
    if recomputed_energy != energy or recomputed_time != time:
        return False

    return True


def solve_mission(energy_budget=DEFAULT_ENERGY_BUDGET, time_budget=DEFAULT_TIME_BUDGET):
    """
    Exact classical route search. Returns one deterministic optimal mission.

    Deterministic tie-breaking among equal-reward feasible routes:
      1. lower energy
      2. then lower time
      3. then lexicographic route order
    """
    # Enumerate every ordered non-empty subset of the clients.
    candidates = []
    for size in range(1, len(clients) + 1):
        for route in itertools.permutations(clients, size):
            candidates.append(route)

    best = None  # (-reward, energy, time, path) ranking key
    best_route = None

    for route in candidates:
        energy = calculate_energy(route)
        time = calculate_time(route)

        # Only feasible routes are eligible.
        if energy > energy_budget or time > time_budget:
            continue

        reward_value = calculate_reward(route)
        path = build_path(route)

        # Maximize reward, then minimize energy, time, and finally the path
        # order lexicographically -> fully deterministic single winner.
        key = (-reward_value, energy, time, path)
        if best is None or key < best:
            best = key
            best_route = route

    if best_route is None:
        # No feasible mission under these budgets.
        return {
            "route": [],
            "reward": 0,
            "energy": 0,
            "time": 0,
            "valid": False,
        }

    path = build_path(best_route)
    energy = calculate_energy(best_route)
    time = calculate_time(best_route)
    reward_value = calculate_reward(best_route)
    valid = is_valid_mission(path, energy, time, energy_budget, time_budget)

    return {
        "route": path,
        "reward": reward_value,
        "energy": energy,
        "time": time,
        "valid": valid,
    }


if __name__ == "__main__":
    result = solve_mission()

    print("Mission result:")
    print("  Route:", " -> ".join(result["route"]) if result["route"] else "NONE")
    print("  Reward:", result["reward"])
    print("  Energy:", result["energy"], "Wh")
    print("  Time:", result["time"], "min")
    print("  Valid:", result["valid"])
