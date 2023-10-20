import numpy as np
import placo


def load() -> placo.RobotWrapper:
    robot = placo.RobotWrapper("megabot/", placo.Flags.ignore_collisions)

    # Here, we handle the zero position of the joints that is not properly handled by OnShape
    # so far. In the future, we hopefully can avoid this hack.
    initial_z = [117.1, 78.5, 136.9]
    limit_z = [36.5, 236.5]

    for leg in range(1, 5):
        for r in range(1, 8):
            robot.set_joint_limits(f"l{leg}_r{r}", -np.pi, np.pi)
        for c in range(1, 4):
            limit_min = limit_z[0] - initial_z[c - 1]
            limit_max = limit_z[1] - initial_z[c - 1]
            robot.set_joint_limits(f"l{leg}_c{c}", limit_min * 1e-3, limit_max * 1e-3)

    return robot


def make_solver(robot: placo.RobotWrapper) -> placo.KinematicsSolver:
    solver = robot.make_solver()

    # Adding hard closing loop constraints to match l*_cl*_1 and l*_cl*_2 in the XZ plane
    for leg in range(1, 5):
        for cl in range(1, 5):
            closing_loop = solver.add_relative_position_task(f"l{leg}_cl{cl}_1", f"l{leg}_cl{cl}_2", np.array([0.0, 0.0, 0.0]))
            closing_loop.configure(f"l{leg}_cl{cl}", "hard", 1.0)
            closing_loop.mask.set_axises("xz")

    return solver
