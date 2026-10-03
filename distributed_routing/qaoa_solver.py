"""
Small QAOA experiment for the A/C drone-routing teaching problem.

This file builds and verifies the QUBO, runs QAOA on a local
Qiskit simulator, and validates the sampled drone routes.
"""

import itertools

# Settings

QAOA_REPS = 1          # number of QAOA layers (p); set to 2 later if desired
QAOA_SHOTS = 1024
SEED = 42
COBYLA_MAXITER = 30


# Problem data

# Node 0 is the base; only clients A and C exist in this first experiment.
nodes = ["0", "A", "C"]
clients = ["A", "C"]

reward = {
    "A": 8,
    "C": 10
}

collection_energy = {
    "A": 1,
    "C": 2
}

collection_time = {
    "A": 2,
    "C": 3
}

# Travel costs are symmetric (same value in both directions).
travel_energy_sym = {
    ("0", "A"): 2,
    ("0", "C"): 4,
    ("A", "C"): 3
}

travel_time_sym = {
    ("0", "A"): 1,
    ("0", "C"): 3,
    ("A", "C"): 3
}

energy_budget = 12
time_budget = 10

# Penalty weight for constraint violations.
P = 25


def get_sym(values, node_i, node_j):
    return values[tuple(sorted((node_i, node_j)))]


# Binary variables
# This ordering is FIXED: each measured qubit maps to one variable by position
# (qubit index i <-> all_vars[i]), so it must never be reordered.

# Directed edges. y_0_A = 1 means "fly directly from 0 to A".
edge_vars = ["y_0_A", "y_0_C", "y_A_0", "y_A_C", "y_C_0", "y_C_A"]

# Client selection. x_A = 1 means "collect A".
client_vars = ["x_A", "x_C"]

# Slack bits let an inequality budget become an equality: the 4 bits with
# weights 1, 2, 4, 8 represent any integer 0..15 of unused budget.
slack_weights = [1, 2, 4, 8]
energy_slack_vars = ["e_s0", "e_s1", "e_s2", "e_s3"]
time_slack_vars = ["t_s0", "t_s1", "t_s2", "t_s3"]

all_vars = edge_vars + client_vars + energy_slack_vars + time_slack_vars
var_index = {name: i for i, name in enumerate(all_vars)}


def edge(i, j):
    return f"y_{i}_{j}"


# QUBO construction
# The QUBO is a dict: key = (var1, var2), value = coefficient.
# A linear term uses (var, var) because binary variables satisfy x^2 = x.
# Keys are stored sorted so (a, b) and (b, a) accumulate instead of overwrite.


def add_term(qubo, var1, var2, coefficient):
    """Add or update a QUBO term."""
    key = tuple(sorted((var1, var2)))
    qubo[key] = qubo.get(key, 0) + coefficient


def add_square(qubo, terms, constant, weight):
    """Add weight * (sum_v c_v*v + constant)^2 to the QUBO.
    Returns the pure constant^2 offset so the caller can track it."""
    items = list(terms.items())

    # Off-diagonal pairs are visited twice (same sorted key) giving 2*c_a*c_b;
    # the diagonal becomes a linear term since x^2 = x.
    for (var_a, coef_a) in items:
        for (var_b, coef_b) in items:
            add_term(qubo, var_a, var_b, weight * coef_a * coef_b)

    for (var, coef) in items:
        add_term(qubo, var, var, weight * 2 * constant * coef)

    return weight * constant * constant


def build_qubo():
    """Build the QUBO dict and its constant offset."""
    qubo = {}
    offset = 0

    # Reward term (negative, since we minimize)
    for c in clients:
        add_term(qubo, f"x_{c}", f"x_{c}", -reward[c])

    # Route / degree constraints (each scaled by P)

    # Leave base exactly once, and return to base exactly once.
    leave_terms = {edge("0", j): 1 for j in clients}
    offset += add_square(qubo, leave_terms, -1, P)

    return_terms = {edge(i, "0"): 1 for i in clients}
    offset += add_square(qubo, return_terms, -1, P)

    # Degree match: a selected client has one arrival and one departure;
    # an unselected client has none.
    for i in clients:
        out_terms = {edge(i, j): 1 for j in nodes if j != i}
        out_terms[f"x_{i}"] = out_terms.get(f"x_{i}", 0) - 1
        offset += add_square(qubo, out_terms, 0, P)

        in_terms = {edge(j, i): 1 for j in nodes if j != i}
        in_terms[f"x_{i}"] = in_terms.get(f"x_{i}", 0) - 1
        offset += add_square(qubo, in_terms, 0, P)

    # Prevent the disconnected A<->C two-node loop. This pairwise trick only
    # suffices for a 2-client instance; larger problems need general
    # subtour-elimination / ordering constraints.
    add_term(qubo, edge("A", "C"), edge("C", "A"), P)

    # Energy equality with slack: P * (total_energy + slack - 12)^2
    energy_terms = {}
    for i in nodes:
        for j in nodes:
            if i != j:
                energy_terms[edge(i, j)] = get_sym(travel_energy_sym, i, j)
    for c in clients:
        energy_terms[f"x_{c}"] = energy_terms.get(f"x_{c}", 0) + collection_energy[c]
    for k, w in enumerate(slack_weights):
        energy_terms[f"e_s{k}"] = w
    offset += add_square(qubo, energy_terms, -energy_budget, P)

    # Time equality with slack: P * (total_time + slack - 10)^2
    time_terms = {}
    for i in nodes:
        for j in nodes:
            if i != j:
                time_terms[edge(i, j)] = get_sym(travel_time_sym, i, j)
    for c in clients:
        time_terms[f"x_{c}"] = time_terms.get(f"x_{c}", 0) + collection_time[c]
    for k, w in enumerate(slack_weights):
        time_terms[f"t_s{k}"] = w
    offset += add_square(qubo, time_terms, -time_budget, P)

    return qubo, offset


qubo, qubo_offset = build_qubo()


def evaluate_qubo(state):
    """QUBO score for a full assignment (offset included)."""
    total = qubo_offset
    for (var1, var2), coefficient in qubo.items():
        total += coefficient * state[var1] * state[var2]
    return total


# Validation
# These recompute the ORIGINAL routing quantities directly, so we never trust
# a low penalized score alone -- we always re-check the real constraints.


def selected_clients(state):
    return [c for c in clients if state[f"x_{c}"] == 1]


def total_reward(state):
    return sum(reward[c] * state[f"x_{c}"] for c in clients)


def route_penalty(state):
    """Zero means a valid tour shape (degrees satisfied)."""
    penalty = 0

    leave = sum(state[edge("0", j)] for j in clients) - 1
    penalty += leave * leave

    back = sum(state[edge(i, "0")] for i in clients) - 1
    penalty += back * back

    for i in clients:
        out = sum(state[edge(i, j)] for j in nodes if j != i) - state[f"x_{i}"]
        inc = sum(state[edge(j, i)] for j in nodes if j != i) - state[f"x_{i}"]
        penalty += out * out
        penalty += inc * inc

    return P * penalty


def loop_penalty(state):
    """Zero means no disconnected A<->C 2-cycle."""
    return P * state[edge("A", "C")] * state[edge("C", "A")]


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
    return sum(w * state[f"e_s{k}"] for k, w in enumerate(slack_weights))


def time_slack_value(state):
    return sum(w * state[f"t_s{k}"] for k, w in enumerate(slack_weights))


def decode_route(state):
    """Follow directed edges from base 0 into a path."""
    path = ["0"]
    current = "0"
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


def constraints_report(state):
    """Validate the ORIGINAL routing problem, not just the penalized score."""
    route_ok = route_penalty(state) == 0 and loop_penalty(state) == 0
    energy_ok = total_energy_before_slack(state) <= energy_budget
    time_ok = total_time_before_slack(state) <= time_budget

    path = decode_route(state)
    # A clean mission starts and ends at 0 and visits exactly the selected clients.
    visits = path[1:-1] if len(path) >= 2 and path[0] == "0" and path[-1] == "0" else []
    tour_ok = (
        len(path) >= 2
        and path[0] == "0"
        and path[-1] == "0"
        and sorted(visits) == sorted(selected_clients(state))
    )

    feasible = route_ok and energy_ok and time_ok and tour_ok

    return {
        "route_ok": route_ok,
        "energy_ok": energy_ok,
        "time_ok": time_ok,
        "tour_ok": tour_ok,
        "feasible": feasible,
        "path": path,
    }


# Bitstring mapping
# Qiskit returns measurement bitstrings where the RIGHTMOST character is
# qubit 0 and the LEFTMOST character is qubit (n-1). We measured qubit i into
# classical bit i, and qubit i corresponds to all_vars[i].
#
# Therefore, for an n-character bitstring s:
#     value of all_vars[i] = s[n - 1 - i]
#
# These two helpers are exact inverses of each other.

def state_to_bitstring(state):
    n = len(all_vars)
    return "".join(str(state[all_vars[i]]) for i in range(n - 1, -1, -1))


def bitstring_to_state(bitstring):
    n = len(all_vars)
    state = {}
    for i in range(n):
        state[all_vars[i]] = int(bitstring[n - 1 - i])
    return state


# Exact verification
# 2^16 = 65,536 states, so plain enumeration is fine and fully exact.

def exact_verification():
    best_score = None
    best_states = []

    for bits in itertools.product([0, 1], repeat=len(all_vars)):
        state = dict(zip(all_vars, bits))
        score = evaluate_qubo(state)
        if best_score is None or score < best_score:
            best_score = score
            best_states = [state]
        elif score == best_score:
            best_states.append(state)

    return best_score, best_states


# QUBO to Ising
# QAOA needs the cost as an Ising Hamiltonian over Pauli-Z operators, not as a
# QUBO over {0,1} variables. We substitute each binary variable:
#
#     x = (1 - z) / 2     where z is a Pauli-Z eigenvalue in {+1, -1}
#                         z = +1  <->  x = 0
#                         z = -1  <->  x = 1
#
# Linear term   c * x_a         -> constant c/2, and Z_a coefficient -c/2
# Quadratic term c * x_a * x_b  -> constant c/4, Z_a: -c/4, Z_b: -c/4, ZZ: +c/4
#
# We fold the QUBO offset into the identity term so the Hamiltonian's energy
# equals the QUBO score (handy for a sanity check). Qubit i == var_index of the
# variable, so the Pauli acts on exactly the qubit we later decode.

def qubo_to_ising():
    from qiskit.quantum_info import SparsePauliOp

    n = len(all_vars)
    const = float(qubo_offset)
    z_coeffs = [0.0] * n
    zz_coeffs = {}

    for (var1, var2), coef in qubo.items():
        i = var_index[var1]
        j = var_index[var2]
        c = float(coef)
        if i == j:
            const += c / 2.0
            z_coeffs[i] += -c / 2.0
        else:
            const += c / 4.0
            z_coeffs[i] += -c / 4.0
            z_coeffs[j] += -c / 4.0
            key = (i, j) if i < j else (j, i)
            zz_coeffs[key] = zz_coeffs.get(key, 0.0) + c / 4.0

    # Build a sparse Pauli list: ("", [], const) is the identity/offset term.
    sparse_list = [("", [], const)]
    for i in range(n):
        if abs(z_coeffs[i]) > 1e-12:
            sparse_list.append(("Z", [i], z_coeffs[i]))
    for (i, j), c in zz_coeffs.items():
        if abs(c) > 1e-12:
            sparse_list.append(("ZZ", [i, j], c))

    return SparsePauliOp.from_sparse_list(sparse_list, num_qubits=n)


# QAOA
# Qiskit is imported lazily here so the classical sections above stay fast and
# dependency-free. QAOA (the quantum algorithm) finds good circuit parameters;
# the Aer statevector simulator (classical execution of a quantum circuit)
# produces the measured samples we decode.

# Minimal "transpiler" that forces QAOA's internal ansatz to flatten=True,
# avoiding the large parameter-binding overhead of the nested circuit.
class FlattenQAOA:
    def run(self, circuit, **kwargs):
        if hasattr(circuit, "flatten"):
            circuit.flatten = True
        return circuit


def run_qaoa(reps, shots, seed, maxiter):
    import time
    import numpy as np
    from qiskit import transpile
    from qiskit.primitives import StatevectorSampler
    from qiskit.circuit.library import QAOAAnsatz
    from qiskit_algorithms.minimum_eigensolvers import QAOA
    from qiskit_algorithms.optimizers import COBYLA
    from qiskit_aer import AerSimulator

    # Same QUBO we verified exactly above, now as an Ising cost Hamiltonian.
    # [DIAGNOSTIC] temporary stage timers to locate the runtime bottleneck.
    print("[1/6] Building Ising Hamiltonian...")
    t0 = time.perf_counter()
    cost_hamiltonian = qubo_to_ising()
    print("Hamiltonian qubits:", cost_hamiltonian.num_qubits)
    print("Hamiltonian Pauli terms:", len(cost_hamiltonian))
    print(f"[1/6] Done in {time.perf_counter() - t0:.3f}s")

    print("[2/6] Creating StatevectorSampler and COBYLA...")
    t0 = time.perf_counter()
    sampler = StatevectorSampler(seed=seed)
    optimizer = COBYLA(maxiter=maxiter)

    # QAOAAnsatz has 2 parameters per layer (one mixer + one cost angle).
    initial_point = np.full(2 * reps, 0.5)

    def qaoa_callback(eval_count, parameters, mean, metadata):
        print(f"      QAOA evaluation {eval_count}: cost = {mean:.4f}")

    qaoa = QAOA(
        sampler=sampler,
        optimizer=optimizer,
        reps=reps,
        initial_point=initial_point,
        callback=qaoa_callback,
        transpiler=FlattenQAOA(),
    )
    print(f"[2/6] Done in {time.perf_counter() - t0:.3f}s")

    # [DIAGNOSTIC] inspect ansatz size/complexity (not transpiled or simulated).
    diagnostic_ansatz = QAOAAnsatz(
        cost_operator=cost_hamiltonian,
        reps=reps,
        flatten=True
    )
    print("QAOA ansatz qubits:", diagnostic_ansatz.num_qubits)
    print("QAOA ansatz parameters:", diagnostic_ansatz.num_parameters)
    print("QAOA ansatz depth before decomposition:", diagnostic_ansatz.depth())

    diagnostic_decomposed = diagnostic_ansatz.decompose()
    print("QAOA ansatz depth after one decomposition:",
          diagnostic_decomposed.depth())

    start = time.time()
    print("[3/6] Starting QAOA parameter optimization...")
    t0 = time.perf_counter()
    result = qaoa.compute_minimum_eigenvalue(cost_hamiltonian)
    print(f"[3/6] Done in {time.perf_counter() - t0:.3f}s")

    # Sample the OPTIMIZED circuit on the local simulator. A fresh ansatz keeps
    # the parameter order aligned with result.optimal_point and gives one clean
    # 16-bit register.
    print("[4/6] Building QAOAAnsatz and assigning parameters...")
    t0 = time.perf_counter()
    ansatz = QAOAAnsatz(cost_operator=cost_hamiltonian, reps=reps)
    bound = ansatz.assign_parameters(result.optimal_point)
    bound.measure_all()
    print(f"[4/6] Done in {time.perf_counter() - t0:.3f}s")

    print("[5/6] Transpiling circuit for Aer simulator...")
    t0 = time.perf_counter()
    simulator = AerSimulator()
    transpiled = transpile(bound, simulator)
    print(f"[5/6] Done in {time.perf_counter() - t0:.3f}s")

    print("[6/6] Running simulator sampling...")
    t0 = time.perf_counter()
    counts = simulator.run(transpiled, shots=shots, seed_simulator=seed).result().get_counts()
    print(f"[6/6] Done in {time.perf_counter() - t0:.3f}s")

    # Runtime measured after sampling, so it covers optimization + sampling.
    runtime = time.time() - start

    return result, counts, runtime


# Sample analysis


def analyze_samples(counts, optimum_bitstrings):
    """Decode samples, keep feasible ones, pick highest-reward feasible mission,
    and measure feasible/optimal sample fractions."""
    total_shots = sum(counts.values())
    feasible_shots = 0
    optimal_shots = 0

    best = None

    for bitstring, count in counts.items():
        state = bitstring_to_state(bitstring)
        report = constraints_report(state)

        if bitstring in optimum_bitstrings:
            optimal_shots += count

        if not report["feasible"]:
            continue

        feasible_shots += count

        r = total_reward(state)
        e = total_energy_before_slack(state)
        t = total_time_before_slack(state)
        score = evaluate_qubo(state)

        candidate = {
            "bitstring": bitstring,
            "route": route_label(report["path"]),
            "reward": r,
            "energy": e,
            "time": t,
            "energy_slack": energy_slack_value(state),
            "time_slack": time_slack_value(state),
            "qubo_score": score,
            "shots": count,
        }

        # Prefer higher reward; break ties by lower energy.
        rank = (r, -e)
        if best is None or rank > best["rank"]:
            candidate["rank"] = rank
            best = candidate

    return {
        "total_shots": total_shots,
        "feasible_shots": feasible_shots,
        "optimal_shots": optimal_shots,
        "best_feasible": best,
    }


# Output

def main():
    print("SMALL QAOA INSTANCE")
    print("Binary variables:", len(all_vars))
    print("QAOA reps:", QAOA_REPS)
    print("Shots:", QAOA_SHOTS)
    print("Seed:", SEED)

    # Show the fixed variable ordering so measured bitstrings can be decoded.
    print("\nVariable ordering (qubit index : variable):")
    for i, name in enumerate(all_vars):
        print(f"  q{i:>2} : {name}")
    print("Bit mapping: in a bitstring s of length n, all_vars[i] = s[n-1-i]")

    # Exact classical reference
    best_score, best_states = exact_verification()
    ref_state = best_states[0]
    ref_report = constraints_report(ref_state)
    optimum_bitstrings = {state_to_bitstring(s) for s in best_states}

    print("\nEXACT QUBO REFERENCE (classical enumeration, not quantum)")
    print("Route:", route_label(ref_report["path"]))
    print("Reward:", total_reward(ref_state))
    print("Energy:", total_energy_before_slack(ref_state), "Wh")
    print("Time:", total_time_before_slack(ref_state), "min")
    print("Energy slack:", energy_slack_value(ref_state))
    print("Time slack:", time_slack_value(ref_state))
    print("QUBO score:", best_score)
    print("Route penalty:", route_penalty(ref_state))
    print("Loop penalty:", loop_penalty(ref_state))
    print("Number of optimal states:", len(best_states))

    # QAOA on local simulator
    print("\nRunning QAOA on local Aer simulator (this is NOT real hardware)...")
    result, counts, runtime = run_qaoa(QAOA_REPS, QAOA_SHOTS, SEED, COBYLA_MAXITER)
    analysis = analyze_samples(counts, optimum_bitstrings)

    print("\nQAOA RESULT")
    best = analysis["best_feasible"]
    if best is None:
        # Honest reporting: QAOA may fail to sample any feasible route.
        print("Best feasible sampled route: NONE FOUND")
        print("QAOA did not sample a feasible mission at these settings.")
        matches = False
    else:
        matches = best["reward"] == total_reward(ref_state) and \
            best["route"] == route_label(ref_report["path"])
        print("Best feasible sampled route:", best["route"])
        print("Reward:", best["reward"])
        print("Energy:", best["energy"], "Wh")
        print("Time:", best["time"], "min")
        print("Energy slack:", best["energy_slack"])
        print("Time slack:", best["time_slack"])
        print("QUBO score:", best["qubo_score"])
        print("Feasible:", "Yes")
        print("Same route/reward as exact mission:", "Yes" if matches else "No")
        print("Exact QUBO optimum sampled:",
              "Yes" if analysis["optimal_shots"] > 0 else "No")

    # Metrics (honest; derived from real samples)
    total_shots = analysis["total_shots"]
    feasible_fraction = analysis["feasible_shots"] / total_shots if total_shots else 0.0
    optimal_fraction = analysis["optimal_shots"] / total_shots if total_shots else 0.0

    print("\nSAMPLE METRICS")
    print("Total shots:", total_shots)
    print("Feasible-sample fraction:", round(feasible_fraction, 4))
    print("Optimal-sample fraction:", round(optimal_fraction, 4))
    print("Cost-function evaluations:", result.cost_function_evals)
    print("QAOA optimizer time (s):", round(result.optimizer_time, 4))
    print("Total QAOA run time incl. sampling (s):", round(runtime, 4))

    print("\nREMINDER: the simulator runs the quantum circuit classically on this")
    print("laptop. This is a teaching proof-of-concept, not quantum advantage.")


if __name__ == "__main__":
    main()
