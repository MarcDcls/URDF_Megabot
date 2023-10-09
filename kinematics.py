import time
import numpy as np
import pinocchio as pin
import placo
from placo_utils.visualization import robot_viz, arrow_viz, robot_frame_viz

robot = placo.RobotWrapper("megabot/", placo.Flags.ignore_collisions)
viz = robot_viz(robot)

initial_z = [117.1, 78.5, 136.9]
limit_z = [36.5, 236.5]

for leg in range(1, 5):
    for r in range(1, 8):
        robot.set_joint_limits(f"l{leg}_r{r}", -np.pi, np.pi)
    for c in range(1, 4):
        limit_min = limit_z[0] - initial_z[c-1]
        limit_max = limit_z[1] - initial_z[c-1]
        robot.set_joint_limits(f"l{leg}_c{c}", limit_min*1e-3, limit_max*1e-3)

solver = robot.make_solver()

# Adding closing loop constraints
for leg in range(1, 5):
    for cl in range(1, 5):
        closing_loop = solver.add_relative_position_task(f"l{leg}_cl{cl}_1", f"l{leg}_cl{cl}_2", np.array([0.0, 0.0, 0.0]))
        closing_loop.configure(f"l{leg}_cl{cl}", "hard", 1.0)
        closing_loop.mask.set_axises("xz")

T_world_base = np.eye(4)
T_world_base[2, 3] = 0.45
base_task = solver.add_frame_task("base", T_world_base)

for k in range(16):
    robot.update_kinematics()
    solver.solve(True)

for leg in range(1, 5):
    name = f"leg_{leg}"
    T_world_leg = robot.get_T_world_frame(name)
    leg_task = solver.add_position_task(name, T_world_leg[:3, 3])
    leg_task.configure(name, "hard", 1.0)

joint_task = solver.add_joint_task("l1_c1", 0.0)

t: float = 0.0
dt: float = 0.01

while True:
    viz.display(robot.state.q)

    robot.update_kinematics()

    T = T_world_base.copy()
    T[0, 3] = np.cos(t*2)*0.2
    T[1, 3] = np.sin(t*2)*0.2
    # T[2, 3] = np.sin(t*3)*0.1 + 0.25
    base_task.T_world_frame = T

    t0 = time.time()
    solver.solve(True)
    t1 = time.time()
    solver.dump_status()
    print(f"Solver time: {(t1-t0)*1e6:.2f} us")

    gravity_torques = placo.GravityTorques(robot)
    contacts = {}

    # Base contact
    for leg in ["leg_1", "leg_2", "leg_3", "leg_4"]:
        contact = gravity_torques.add_contact()
        contact.configure(leg, "point")
        contacts[leg] = contact

    for leg in range(1, 5):
        for r in range(1, 8):
            gravity_torques.set_passive(f"l{leg}_r{r}", True)
        for cl in range(1, 5):
            gravity_torques.add_loop_closing_constraint(f"l{leg}_cl{cl}_1", f"l{leg}_cl{cl}_2", "xz")

    result = gravity_torques.compute()

    if result.success:
        c1 = result.tau[robot.get_joint_v_offset("l1_c1")]
        c2 = result.tau[robot.get_joint_v_offset("l1_c2")]
        c3 = result.tau[robot.get_joint_v_offset("l1_c3")]
        print(f"Torques forces: {c1:.2f} {c2:.2f} {c3:.2f}")

        for leg in ["leg_1", "leg_2", "leg_3", "leg_4"]:
            T_world_leg = robot.get_T_world_frame(leg)
            point = T_world_leg[:3, 3]
            force = contacts[leg].wrench[:3]
            arrow_viz(f"{leg}_force", point, point+force*1e-3, 0xff3366, radius=0.02, head_length=0.05)
    else:
        print("Failed to compute gravity torques")

    # for cl in range(1, 5):
    #     robot_frame_viz(robot, f"l1_cl{cl}_1")
    #     robot_frame_viz(robot, f"l1_cl{cl}_2")

    t += dt
    time.sleep(dt)
