import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
from matplotlib.transforms import Affine2D
import csv
import numpy as np

def car_dynamics(q, u, shape):
    # Extract state components
    v, phi, theta = q[2], q[3], q[4]

    # Calculate derivatives
    xdot = v * np.cos(theta)
    ydot = v * np.sin(theta)
    vdot = u[0]
    phidot = u[1]
    thetadot = (v / shape[1]) * np.tan(phi)

    return np.array([xdot, ydot, vdot, phidot, thetadot])

def rk4_step(f, q, u, dt, shape):
    k1 = f(q, u, shape)
    k2 = f(q + 0.5 * dt * k1, u, shape)
    k3 = f(q + 0.5 * dt * k2, u, shape)
    k4 = f(q + dt * k3, u, shape)

    return q + dt * (k1 + 2*k2 + 2*k3 + k4) / 6

def integrate(q0, u, duration, dt, shape):
    time = 0
    q = q0
    while time < duration:
        q = rk4_step(car_dynamics, q, u, dt, shape)
        time += dt
    return q

def read_obstacle_csv(file_name):
    with open(file_name, 'r') as file:
        reader = csv.reader(file)
        obstacles = []
        current_obstacle = []
        for row in reader:
            if row:  # If the row is not empty
                current_obstacle.append(list(map(float, row)))
            else:  # If the row is empty, start a new obstacle
                if current_obstacle:  # Only add if the current obstacle is not empty
                    obstacles.append(current_obstacle)
                    current_obstacle = []
        if current_obstacle:  # Add the last obstacle if file doesn't end with a blank line
            obstacles.append(current_obstacle)
        return obstacles

def read_csv(file_name):
    with open(file_name, 'r') as file:
        reader = csv.reader(file)
        return [list(map(float, row)) for row in reader]

import csv
import numpy as np

def read_txt(file_name, integrate, shape):
    with open(file_name, 'r') as file:
        reader = csv.reader(file, delimiter=' ')
        data = []
        prev_q = None

        for row in reader:
            if prev_q is not None:
                # Control and time from the current row
                u = [float(row[5]), float(row[6])]
                t = float(row[7])

                # Integrating over 5 steps between 0 and t
                for delta in np.linspace(0, t, 10):
                    dt = 0.01  # Assuming a constant time step for each integration
                    new_q = integrate(prev_q, u, delta, dt, shape)
                    data.append(new_q)

            # Update prev_q with the current state for the next iteration
            prev_q = [float(row[0]), float(row[1]), float(row[2]), float(row[3]), float(row[4])]

    return data


                


x1 = 1  # Additional length for the dotted box in the x-direction
y1 = 1  # Additional height for the dotted box in the y-direction

def get_car_patch(center_x, center_y, theta, width, height):
    car = patches.Rectangle((center_x - width / 2, center_y - height / 2), width, height, color='blue')
    t = Affine2D().rotate_around(center_x, center_y, theta) + ax.transData 
    car.set_transform(t)
    
    dotted_box = patches.Rectangle((center_x - (width + x1) / 2, center_y - (height + y1) / 2), width + x1, height + y1, linestyle='dotted', edgecolor='black', facecolor='none')
    dotted_box.set_transform(t)

    return car, dotted_box

global car, dotted_box

# Read data from CSV files
obstacle_data = read_obstacle_csv('/home/nic/dev/ASEN_5254/ASEN_5254_Project/AMP-Tools-public/build/bin/obstacles.csv')
bounds_data = read_csv('/home/nic/dev/ASEN_5254/ASEN_5254_Project/AMP-Tools-public/build/bin/bounds.csv')
start_data = read_csv('/home/nic/dev/ASEN_5254/ASEN_5254_Project/AMP-Tools-public/build/bin/start.csv')
path_data = read_txt('/home/nic/dev/ASEN_5254/ASEN_5254_Project/AMP-Tools-public/build/bin/Output.txt', integrate, [0.8, 0.4])
print(len(path_data))
goal_data = read_csv('/home/nic/dev/ASEN_5254/ASEN_5254_Project/AMP-Tools-public/build/bin/goal.csv')

# Initialize the figure and axis
fig, ax = plt.subplots()
x_min, y_min = bounds_data[0]
x_max, y_max = bounds_data[1]
ax.set_xlim(x_min - 2, x_max + 2)
ax.set_ylim(y_min - 2, y_max + 2)
ax.set_aspect('equal', adjustable='box')
ax.grid(True)

# Car dimensions
car_width = .8
car_height = .4
initial_state = path_data[0]

# Initial car position and orientation
car, dotted_box = get_car_patch(initial_state[0], initial_state[1], initial_state[4], car_width, car_height)
ax.add_patch(car)
ax.add_patch(dotted_box)

# Draw obstacles
for obstacle in obstacle_data:
    polygon = patches.Polygon(obstacle, facecolor='red')
    ax.add_patch(polygon)

# Draw goal
for goal in goal_data:
    circle = patches.Circle(goal, radius=0.5, facecolor='green')
    ax.add_patch(circle)

# Draw start
for start in start_data:
    circle = patches.Circle(start, radius=0.5, facecolor='green')
    ax.add_patch(circle)

# Function to animate the car
def animate(frame):
    global car, dotted_box  # Refer to the global variables
    if frame < len(path_data):
        state = path_data[frame]
        x = state[0]
        y = state[1]
        theta = state[4]
        # Remove the old car and dotted box
        car.remove()
        dotted_box.remove()
        # Create and add new car and dotted box
        car, dotted_box = get_car_patch(x, y, theta, car_width, car_height)
        ax.add_patch(car)
        ax.add_patch(dotted_box)
    return car, dotted_box

# Create animation
anim = FuncAnimation(fig, animate, frames=len(path_data), interval=10, blit=True)

plt.show()
