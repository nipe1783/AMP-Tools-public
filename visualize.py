import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
from matplotlib.transforms import Affine2D
import csv

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

def read_txt(file_name):
    with open(file_name, 'r') as file:
        reader = csv.reader(file, delimiter=' ')
        # Read x, y, and theta (fifth column, index 4)
        return [[float(row[0]), float(row[1]), float(row[4])] for row in reader]

x1 = 0.5  # Additional length for the dotted box in the x-direction
y1 = 0.5  # Additional height for the dotted box in the y-direction

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
path_data = read_txt('/home/nic/dev/ASEN_5254/ASEN_5254_Project/AMP-Tools-public/build/bin/Output.txt')
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
car, dotted_box = get_car_patch(initial_state[0], initial_state[1], initial_state[2], car_width, car_height)
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
        x, y, theta = path_data[frame]
        # Remove the old car and dotted box
        car.remove()
        dotted_box.remove()
        # Create and add new car and dotted box
        car, dotted_box = get_car_patch(x, y, theta, car_width, car_height)
        ax.add_patch(car)
        ax.add_patch(dotted_box)
    return car, dotted_box

# Create animation
anim = FuncAnimation(fig, animate, frames=len(path_data), interval=50, blit=True)

plt.show()
