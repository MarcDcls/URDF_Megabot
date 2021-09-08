import time

import pybullet as p
import pybullet_data
import numpy as np

class Megabot:
    """
        Links : self.legs[leg_id][i] = id du i-ème membre de la patte leg_id
        Joints : self.joint_name_to_id['ij'] = id du j-ème joint de la patte i
    """

    def __init__(
            self,
    ):
        self.id = p.loadURDF("megabot/robot.urdf", [0, 0, 1], p.getQuaternionFromEuler([0, 0, 0]))
        self.connectors = [[16, 8, 6, 9, 12, 17, 15, 72],
                           [34, 26, 24, 27, 30, 35, 33, 73],
                           [52, 44, 42, 45, 48, 53, 51, 74],
                           [70, 62, 60, 63, 66, 71, 69, 75]]
        self._BuildJointNameToIdDict()
        self._CreateClosedKinematicLoops()

    def _BuildJointNameToIdDict(self):
        num_joints = p.getNumJoints(self.id)
        self.joint_name_to_id = {}
        for i in range(num_joints):
            joint_info = p.getJointInfo(self.id, i)
            self.joint_name_to_id[joint_info[1].decode("UTF-8")] = joint_info[0]

    def _CreateClosedKinematicLoops(self):
        for i in range(4):
            self._Constraint(i, 0, [0, 1, 0])
            self._Constraint(i, 1, [1, 0, 0])
            self._Constraint(i, 2, [1, 0, 0])
            self._Constraint(i, 3, [1, 0, 0])

    def _Constraint(self, leg_id, n, axis):
        p.createConstraint(self.id, self.connectors[leg_id][2 * n], self.id, self.connectors[leg_id][2 * n + 1],
                           p.JOINT_POINT2POINT, axis, [0, 0, 0], [0, 0, 0])


################################ SIMULATION ################################

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
planeId = p.loadURDF("plane.urdf")
p.setGravity(0, 0, -10)
robot = Megabot()

for i in range(10000):
    p.stepSimulation()

    # p.changeVisualShape(robot.id, 52, rgbaColor=(0., 1., 0., 1.))
    # p.changeVisualShape(robot.id, 51, rgbaColor=(0., 1., 1., 1.))

    # p.changeVisualShape(robot.id, robot.connectors[0][int(i/200)], rgbaColor=(1., 0., 0., 1.))

    # p.setJointMotorControl2(robot.id, robot.joint_name_to_id['0v31'], targetPosition=-0.0045-np.abs(np.sin(i/300)*0.2), controlMode=p.POSITION_CONTROL, force=100000)

    # time.sleep(1)
    time.sleep(1. / 240.)

p.disconnect()
