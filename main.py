import math
import itertools
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import csv
import os

#Define the waypoints
waypoints = [
    {"name": "A", "x": 2, "y": 3, "reward": 8},
    {"name": "B", "x": 5, "y": 7, "reward": 12},
    {"name": "C", "x": 8, "y": 4, "reward": 10},
    {"name": "D", "x": 6, "y": 1, "reward": 7},
    {"name": "E", "x": 3, "y": 8, "reward": 9}
]

start = {"name": "Start",
         "x": 0,
         "y": 0,
         "reward": 0}

battery_budget = 25

#Calculate the distance between two points
def calculate_distance(point1, point2):
    dx = point2["x"] - point1["x"]
    dy = point2["y"] - point1["y"]

    distance = math.sqrt(dx**2 + dy**2)

    return distance

#Test the function
distance_to_a = calculate_distance(start, waypoints[0])

print("Distance from Start to A:", distance_to_a)

#Define the energy rate
energy_rate = 1.5

takeoff_energy = 1.0
landing_energy = 0.5

#Calculate the energy required to travel between two points
def calculate_energy(point1, point2):
    distance = calculate_distance(point1, point2)
    energy = distance * energy_rate
    return energy

#Test the function
energy_to_a = calculate_energy(start, waypoints[0])
print("Energy required to travel from Start to A:", energy_to_a)

#Display the results
print("\nAll Waypoints:")

for waypoint in waypoints:
    distance = calculate_distance(start, waypoint)
    energy = calculate_energy(start, waypoint)

    print(
        waypoint["name"],
        "| Distance:",
        round(distance, 2),
        "| Energy:",
        round(energy, 2),
        "| Reward:",
        waypoint["reward"]
    )

#Calculate the energy required to travel along a route
def calculate_route_energy(route):
    total_energy = 0
    current_point = start

    #Add the takeoff energy once at the beginning
    total_energy = total_energy + takeoff_energy

    for waypoint in route:
        energy = calculate_energy(current_point, waypoint)
        total_energy = total_energy + energy
        current_point = waypoint

    #Add the energy cost to return from the final waypoint back to Start
    total_energy = total_energy + calculate_energy(current_point, start)

    #Add the landing energy once at the end
    total_energy = total_energy + landing_energy

    return total_energy

test_route = [
    waypoints[0],
    waypoints[3],
    waypoints[1]
]

route_energy = calculate_route_energy(test_route)

#Display the results
print("\nTest Route: Start -> A -> D -> B")
print("Total Route Energy:", round(route_energy, 2))

#Calculate the reward for a route
def calculate_route_reward(route):
    total_reward = 0

    for waypoint in route:
        total_reward = total_reward + waypoint["reward"]

    return total_reward

#Check if the route is valid
if route_energy <= battery_budget:
    print("Route Status: Valid")
else:
    print("Route Status: Invalid")

#Display the possible 2-waypoint combinations using itertools.combinations
print("\nPossible 2-waypoint combinations:")

#Calculate the reward for each 2-waypoint combination
for combo in itertools.combinations(waypoints, 2):
    reward = calculate_route_reward(combo)

    print(
        combo[0]["name"],
        "->",
        combo[1]["name"],
        "| Reward:",
        reward
    )

best_route = None
best_reward = -1
best_energy = 0

#Test routes with different numbers of waypoints
for route_length in range(1, len(waypoints) + 1):

    for route in itertools.permutations(waypoints, route_length):

        energy = calculate_route_energy(route)
        reward = calculate_route_reward(route)

        #Check if the route is valid and has the highest reward
        #(break ties by preferring the route with lower energy)
        if energy <= battery_budget and (
            reward > best_reward
            or (reward == best_reward and energy < best_energy)
        ):
            best_route = route
            best_reward = reward
            best_energy = energy

#Display the best route
print("\nBest Route:")

best_route_names = " -> ".join(
    waypoint["name"] for waypoint in best_route
)

print("Route: Start ->", best_route_names, "-> Start")
print("Energy:", round(best_energy, 2))
print("Reward:", best_reward)

#Write the results to a CSV file
#Only write the header the first time (when results.csv does not exist yet)
file_exists = os.path.exists("results.csv")

with open("results.csv", "a", newline="") as file:
    writer = csv.writer(file)

    if not file_exists:
        writer.writerow([
            "Solver",
            "Battery Budget",
            "Best Route",
            "Energy",
            "Reward"
        ])

    writer.writerow([
        "classical_bruteforce",
        battery_budget,
        "Start -> " + best_route_names + " -> Start",
        round(best_energy, 2),
        best_reward
    ])

#Plot the start point
plt.scatter(start["x"], start["y"])
plt.text(start["x"], start["y"], "Start")

#Plot all waypoints
for waypoint in waypoints:
    plt.scatter(waypoint["x"], waypoint["y"])
    plt.text(waypoint["x"], waypoint["y"], waypoint["name"])

#Plot the best route: Start -> waypoints -> Start
route_points = [start] + list(best_route) + [start]

route_x = [point["x"] for point in route_points]
route_y = [point["y"] for point in route_points]

plt.plot(route_x, route_y)

plt.xlabel("X Coordinate")
plt.ylabel("Y Coordinate")
plt.title(
    f"Best Route | Energy: {best_energy:.2f} | "
    f"Reward: {best_reward} | Battery: {battery_budget}"
)
plt.grid()

plt.savefig("best_route.png", dpi=300, bbox_inches="tight")
plt.show()