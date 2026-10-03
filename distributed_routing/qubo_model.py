import itertools

# ============================================================
# QUBO MODEL for the 3-client drone-routing teaching problem.
#
# This is the same problem solved in classical_solver.py, but
# here we express it as a QUBO (Quadratic Unconstrained Binary
# Optimization) problem.
#
# In a QUBO we only have binary variables (0 or 1) and we want
# to MINIMIZE a quadratic score. Constraints are not hard rules;
# instead we add "penalty" terms that make the score worse when
# a constraint is broken.
#
# Only the Python standard library is used. No QAOA yet.
# ============================================================


# ------------------------------------------------------------
# PROBLEM DATA
# ------------------------------------------------------------

# Node 0 is the base. A, B, C are the clients.
nodes = ["0", "A", "B", "C"]
clients = ["A", "B", "C"]

# Reward collected for visiting each client
reward = {
    "A": 8,
    "B": 6,
    "C": 10
}

# Energy (Wh) spent collecting the update at each client
collection_energy = {
    "A": 1,
    "B": 1,
    "C": 2
}

# Time (minutes) spent collecting the update at each client
collection_time = {
    "A": 2,
    "B": 1,
    "C": 3
}

# Symmetric travel energy (Wh). Same value in both directions.
travel_energy_sym = {
    ("0", "A"): 2,
    ("0", "B"): 3,
    ("0", "C"): 4,
    ("A", "B"): 2,
    ("A", "C"): 3,
    ("B", "C"): 2
}

# Symmetric travel time (minutes). Same value in both directions.
travel_time_sym = {
    ("0", "A"): 1,
    ("0", "B"): 2,
    ("0", "C"): 3,
    ("A", "B"): 1,
    ("A", "C"): 3,
    ("B", "C"): 1
}

# Budgets for the mission
energy_budget = 12
time_budget = 10

# Penalty weight. A large value makes breaking a constraint
# very expensive, so the minimizer prefers valid routes.
P = 25


# Look up a symmetric travel value for the edge between two nodes.
# Because travel is symmetric, (i, j) and (j, i) share one value.
def get_sym(values, node_i, node_j):
    key = tuple(sorted((node_i, node_j)))
    return values[key]


# ------------------------------------------------------------
# VARIABLES
# ------------------------------------------------------------

# 1. Directed edge variables y_i_j for every i != j.
#    y_0_A = 1 means "fly directly from 0 to A".
#    With 4 nodes there are 4 * 3 = 12 directed edges.
edge_vars = [
    f"y_{i}_{j}"
    for i in nodes
    for j in nodes
    if i != j
]

# 2. Client-selection variables. x_A = 1 means "collect A".
client_vars = [f"x_{c}" for c in clients]

# 3. Energy slack bits with binary weights 1, 2, 4, 8.
#    A 4-bit slack can represent any integer from 0 to 15.
slack_weights = [1, 2, 4, 8]
energy_slack_vars = [f"e_s{k}" for k in range(4)]

# 4. Time slack bits with binary weights 1, 2, 4, 8.
time_slack_vars = [f"t_s{k}" for k in range(4)]

# The full list of 23 binary variables.
all_vars = edge_vars + client_vars + energy_slack_vars + time_slack_vars

# The 15 "structural" variables (edges + client choices) fully
# describe the route. The 8 slack bits are extra helper bits.
structural_vars = edge_vars + client_vars


# Helper to build a directed edge variable name from two nodes.
def edge(i, j):
    return f"y_{i}_{j}"


# ------------------------------------------------------------
# SMALL QUBO HELPER FUNCTIONS
# ------------------------------------------------------------
#
# HERE The QUBO is stored as a dictionary:
#     key   = (variable1, variable2)
#     value = coefficient
#
# For a linear term we use (var, var) because a binary variable
# satisfies x^2 = x, so (x, x) really means a linear x term.
#
# We always store a key in sorted order so that (a, b) and (b, a)
# accumulate into the SAME entry instead of overwriting each other.


# Add a coefficient to a QUBO term, accumulating duplicates.
def add_term(qubo, var1, var2, coefficient):
    key = tuple(sorted((var1, var2)))
    qubo[key] = qubo.get(key, 0) + coefficient


# Add weight * (linear_expression)^2 into the QUBO.
#
# A linear expression is given as:
#   terms    : a dict {variable: coefficient}
#   constant : a plain number added to the sum
#
# Expanding (sum_v c_v * v + k)^2 gives:
#   - quadratic/diagonal terms: c_va * c_vb * va * vb
#   - linear terms from the cross with the constant: 2 * k * c_v * v
#   - a pure constant k^2 (returned so the caller can track the offset)
def add_square(qubo, terms, constant, weight):
    items = list(terms.items())

    # Quadratic part. The double loop visits each off-diagonal pair
    # twice (va,vb) and (vb,va); both map to the same sorted key, so
    # together they correctly contribute 2 * c_va * c_vb.
    # The diagonal (va == va) becomes the linear x term since x^2 = x.
    for (var_a, coef_a) in items:
        for (var_b, coef_b) in items:
            add_term(qubo, var_a, var_b, weight * coef_a * coef_b)

    # Linear part coming from 2 * constant * (sum of terms)
    for (var, coef) in items:
        add_term(qubo, var, var, weight * 2 * constant * coef)

    # Pure constant offset k^2 * weight
    return weight * constant * constant


# ------------------------------------------------------------
# Now BUILD THE QUBO
# ------------------------------------------------------------

def build_qubo():
    qubo = {}
    offset = 0  # constant part of the objective, tracked separately

    # ----- REWARD TERM -----
    # We want MORE reward, but QUBO minimizes, so we add the
    # negative reward: -(8*x_A + 6*x_B + 10*x_C).
    for c in clients:
        add_term(qubo, f"x_{c}", f"x_{c}", -reward[c])

    # ----- ROUTE / DEGREE CONSTRAINTS (all multiplied by P) -----

    # The drone must LEAVE base 0 exactly once:
    #   (sum of y_0_j for j in A,B,C - 1)^2
    leave_terms = {edge("0", j): 1 for j in clients}
    offset += add_square(qubo, leave_terms, -1, P)

    # The drone must RETURN to base 0 exactly once:
    #   (sum of y_i_0 for i in A,B,C - 1)^2
    return_terms = {edge(i, "0"): 1 for i in clients}
    offset += add_square(qubo, return_terms, -1, P)

    # For each client, outgoing edges must equal x_i and incoming
    # edges must equal x_i. So a selected client has exactly one
    # arrival and one departure, and an unselected client has none.
    for i in clients:
        # (outgoing edges from i - x_i)^2
        out_terms = {edge(i, j): 1 for j in nodes if j != i}
        out_terms[f"x_{i}"] = out_terms.get(f"x_{i}", 0) - 1
        offset += add_square(qubo, out_terms, 0, P)

        # (incoming edges to i - x_i)^2
        in_terms = {edge(j, i): 1 for j in nodes if j != i}
        in_terms[f"x_{i}"] = in_terms.get(f"x_{i}", 0) - 1
        offset += add_square(qubo, in_terms, 0, P)

    # ----- DISCONNECTED LOOP PENALTY -----
    # For this small 3-client teaching example we forbid using BOTH
    # directions of a client-client pair at the same time (a tiny
    # 2-node loop that is not connected to the base).
    #
    # NOTE: This simple pairwise trick only works for this small
    # instance. Larger routing problems need general subtour-
    # elimination constraints or explicit ordering (position)
    # variables to rule out all disconnected loops.
    add_term(qubo, edge("A", "B"), edge("B", "A"), P)
    add_term(qubo, edge("A", "C"), edge("C", "A"), P)
    add_term(qubo, edge("B", "C"), edge("C", "B"), P)

    # ----- ENERGY CONSTRAINT -----
    # total_energy = travel energy on used edges + collection energy.
    # We add slack so that: total_energy + slack == 12 (equality).
    #   penalty = P * (total_energy + energy_slack - 12)^2
    energy_terms = {}
    for i in nodes:
        for j in nodes:
            if i != j:
                energy_terms[edge(i, j)] = get_sym(travel_energy_sym, i, j)
    for c in clients:
        energy_terms[f"x_{c}"] = energy_terms.get(f"x_{c}", 0) + collection_energy[c]
    for k, weight in enumerate(slack_weights):
        energy_terms[f"e_s{k}"] = weight
    offset += add_square(qubo, energy_terms, -energy_budget, P)

    # ----- TIME CONSTRAINT -----
    # total_time = travel time on used edges + collection time.
    #   penalty = P * (total_time + time_slack - 10)^2
    time_terms = {}
    for i in nodes:
        for j in nodes:
            if i != j:
                time_terms[edge(i, j)] = get_sym(travel_time_sym, i, j)
    for c in clients:
        time_terms[f"x_{c}"] = time_terms.get(f"x_{c}", 0) + collection_time[c]
    for k, weight in enumerate(slack_weights):
        time_terms[f"t_s{k}"] = weight
    offset += add_square(qubo, time_terms, -time_budget, P)

    return qubo, offset


# Build the QUBO once so it can be reused below.
qubo, qubo_offset = build_qubo()


# ------------------------------------------------------------
# EVALUATING A STATE
# ------------------------------------------------------------

# Evaluate the full QUBO score for a given assignment of all 23
# variables. The constant offset is included so the score matches
# the real objective value.
def evaluate_qubo(state):
    total = qubo_offset
    for (var1, var2), coefficient in qubo.items():
        total += coefficient * state[var1] * state[var2]
    return total


# ------------------------------------------------------------
# COMPONENT BREAKDOWN (for teaching / debugging)
# ------------------------------------------------------------
# These recompute each part of the objective directly from the
# formulas (not from the QUBO dict). Their sum must equal
# evaluate_qubo(state).

def reward_contribution(state):
    return -sum(reward[c] * state[f"x_{c}"] for c in clients)


def route_penalty(state):
    penalty = 0

    # leave base exactly once
    leave = sum(state[edge("0", j)] for j in clients) - 1
    penalty += leave * leave

    # return to base exactly once
    back = sum(state[edge(i, "0")] for i in clients) - 1
    penalty += back * back

    # degree match for every client
    for i in clients:
        out = sum(state[edge(i, j)] for j in nodes if j != i) - state[f"x_{i}"]
        inc = sum(state[edge(j, i)] for j in nodes if j != i) - state[f"x_{i}"]
        penalty += out * out
        penalty += inc * inc

    return P * penalty


def loop_penalty(state):
    pairs = [("A", "B"), ("A", "C"), ("B", "C")]
    total = 0
    for i, j in pairs:
        total += state[edge(i, j)] * state[edge(j, i)]
    return P * total


def total_energy_before_slack(state):
    total = 0
    for i in nodes:
        for j in nodes:
            if i != j:
                total += get_sym(travel_energy_sym, i, j) * state[edge(i, j)]
    for c in clients:
        total += collection_energy[c] * state[f"x_{c}"]
    return total


def total_time_before_slack(state):
    total = 0
    for i in nodes:
        for j in nodes:
            if i != j:
                total += get_sym(travel_time_sym, i, j) * state[edge(i, j)]
    for c in clients:
        total += collection_time[c] * state[f"x_{c}"]
    return total


def energy_slack_value(state):
    return sum(weight * state[f"e_s{k}"] for k, weight in enumerate(slack_weights))


def time_slack_value(state):
    return sum(weight * state[f"t_s{k}"] for k, weight in enumerate(slack_weights))


def energy_penalty(state):
    residual = total_energy_before_slack(state) + energy_slack_value(state) - energy_budget
    return P * residual * residual


def time_penalty(state):
    residual = total_time_before_slack(state) + time_slack_value(state) - time_budget
    return P * residual * residual


# ------------------------------------------------------------
# DECODING A ROUTE
# ------------------------------------------------------------

# Turn the directed edge variables into a readable path, starting
# at base 0 and following edges set to 1 until we return to 0.
def decode_route(state):
    path = ["0"]
    current = "0"

    # At most a few steps for this tiny problem; the guard prevents
    # an infinite loop if the edges do not form a clean tour.
    for _ in range(len(nodes) + 1):
        next_node = None
        for j in nodes:
            if j != current and state.get(edge(current, j), 0) == 1:
                next_node = j
                break

        if next_node is None:
            break

        path.append(next_node)
        current = next_node

        if current == "0":
            break

    return path


def route_label(path):
    return " -> ".join(path)


# ------------------------------------------------------------
# EXACT VERIFICATION
# ------------------------------------------------------------
#
# We do NOT enumerate all 2^23 states. Instead we enumerate the
# 2^15 structural states (12 edge bits + 3 client bits). For each
# structural state the slack bits appear ONLY in the energy and
# time penalties, so we can pick the best slack value independently:
#
#   - energy slack: the integer 0..15 that minimizes
#                   (energy_before + slack - 12)^2
#   - time slack:   the integer 0..15 that minimizes
#                   (time_before + slack - 10)^2
#
# Because we minimize each slack register exactly, this is still an
# exact minimization over the complete 23-variable QUBO.

def best_slack_value(amount_before, budget):
    # Choose slack in 0..15 that makes (amount_before + slack - budget)^2
    # as small as possible.
    best_value = 0
    best_residual = None
    for candidate in range(16):
        residual = (amount_before + candidate - budget) ** 2
        if best_residual is None or residual < best_residual:
            best_residual = residual
            best_value = candidate
    return best_value


def int_to_bits(value):
    # Convert an integer 0..15 into the four slack bits (weights 1,2,4,8).
    return [(value >> k) & 1 for k in range(4)]


def exact_verification():
    best_score = None
    best_states = []

    # Enumerate every structural state: 2^15 = 32768 combinations.
    for bits in itertools.product([0, 1], repeat=len(structural_vars)):
        state = dict(zip(structural_vars, bits))

        # Figure out the energy and time used before adding slack.
        energy_before = total_energy_before_slack(state)
        time_before = total_time_before_slack(state)

        # Pick the slack integers that minimize each budget penalty.
        e_slack = best_slack_value(energy_before, energy_budget)
        t_slack = best_slack_value(time_before, time_budget)

        # Store the chosen slack values as their four binary bits.
        for k, bit in enumerate(int_to_bits(e_slack)):
            state[f"e_s{k}"] = bit
        for k, bit in enumerate(int_to_bits(t_slack)):
            state[f"t_s{k}"] = bit

        # Evaluate the full 23-variable QUBO score.
        score = evaluate_qubo(state)

        if best_score is None or score < best_score:
            best_score = score
            best_states = [state]
        elif score == best_score:
            best_states.append(state)

    return best_score, best_states


# ------------------------------------------------------------
# RUN AND REPORT
# ------------------------------------------------------------

def main():
    print("QUBO MODEL")
    print("Total binary variables:", len(all_vars))
    print("Directed edge variables:", len(edge_vars))
    print("Client variables:", len(client_vars))
    print("Energy slack bits:", len(energy_slack_vars))
    print("Time slack bits:", len(time_slack_vars))
    print("Penalty weight:", P)

    best_score, best_states = exact_verification()

    # Use the first optimal state for the readable summary.
    best_state = best_states[0]

    best_energy = total_energy_before_slack(best_state)
    best_time = total_time_before_slack(best_state)
    best_reward = sum(reward[c] * best_state[f"x_{c}"] for c in clients)
    constraint_valid = best_energy <= energy_budget and best_time <= time_budget

    print()
    print("EXACT QUBO VERIFICATION")
    print("Minimum QUBO score:", best_score)
    print("Best reward:", best_reward)
    print("Energy:", best_energy, "Wh")
    print("Time:", best_time, "min")
    print("Energy slack:", energy_slack_value(best_state))
    print("Time slack:", time_slack_value(best_state))
    print("Constraint valid:", "Yes" if constraint_valid else "No")

    # Decode every optimal state into a readable route (no duplicates).
    print()
    print("Optimal route(s):")
    seen = []
    for state in best_states:
        label = route_label(decode_route(state))
        if label not in seen:
            seen.append(label)
            print(label)

    # Show the individual objective components for one optimum.
    print()
    print("Objective components for an optimum:")
    print("reward contribution =", reward_contribution(best_state))
    print("route penalty       =", route_penalty(best_state))
    print("loop penalty        =", loop_penalty(best_state))
    print("energy penalty      =", energy_penalty(best_state))
    print("time penalty        =", time_penalty(best_state))


if __name__ == "__main__":
    main()
