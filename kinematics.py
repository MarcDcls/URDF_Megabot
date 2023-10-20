import time
import numpy as np
import meshcat.transformations as tf
from placo_utils.visualization import robot_viz, arrow_viz, robot_frame_viz, frame_viz, contacts_viz
import megabot

robot = megabot.load()
solver = megabot.make_solver(robot)

# Initializing T_world_base to identity
T_world_base = np.eye(4)
robot.set_T_world_frame("base", T_world_base)
robot.update_kinematics()

# We set the position of the base in the world
base_task = solver.add_frame_task("base", T_world_base)
base_task.configure("base", "soft", 1.0, 10.0)

# We set the position of legs
for leg in range(1, 5):
    name = f"leg_{leg}"
    T_world_leg = robot.get_T_world_frame(name)
    leg_task = solver.add_position_task(name, T_world_leg[:3, 3])
    leg_task.configure(name, "hard", 1.0)

# Creating dynamics solver
id = megabot.make_dynamics_solver(robot)
id.set_static(True)
id.enable_torque_limits(False)

for leg in range(1, 5):
    name = f"leg_{leg}"
    T_world_leg = robot.get_T_world_frame(name)
    leg_task = id.add_position_task(name, T_world_leg[:3, 3])
    leg_task.configure(name, "hard", 1.0)
    contact = id.add_unilateral_point_contact(leg_task)
    contact.weight_forces = 1e3

# Initializing the viewer
viz = robot_viz(robot)
t: float = 0.0
dt: float = 0.01
start_time = time.time()

while True:
    # Update the body target frame
    T = T_world_base.copy()
    T[0, 3] = np.cos(t * 2) * 0.3
    T[1, 3] = np.sin(t * 2) * 0.3
    base_task.T_world_frame = T

    # Updating the kinematics and solving the IK
    robot.update_kinematics()
    solver.solve(True)

    result = id.solve()
    if result.success:
        contacts_viz(id, ratio=1e-3, radius=0.03)

    viz.display(robot.state.q)
    robot_frame_viz(robot, "base")
    frame_viz("base_target", T, 0.5)

    t += dt
    while time.time() - start_time < t:
        time.sleep(1e-4)
