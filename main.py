import math
import matplotlib.pyplot as plt

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

    for waypoint in route:
        energy = calculate_energy(current_point, waypoint)
        total_energy = total_energy + energy
        current_point = waypoint

    return total_energy



test_route = [
    waypoints[0],
    waypoints[3],
    waypoints[1]
]



route_energy = calculate_route_energy(test_route)



print("\nTest Route: Start -> A -> D -> B")
print("Total Route Energy:", round(route_energy, 2))

#Calculate the reward for a route
def calculate_route_reward(route):
    total_reward = 0

    for waypoint in route:
        total_reward = total_reward + waypoint["reward"]

    return total_reward