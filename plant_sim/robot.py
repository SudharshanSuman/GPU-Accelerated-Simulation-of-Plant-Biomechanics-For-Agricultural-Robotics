"""3-link robot arm with forward kinematics and soft contact against plant nodes."""

import numpy as np

from .config import DT, SUBSTEPS


class SimpleRobotArm:
    def __init__(self, base_pos=(0.35, 0.0, 0.0)):
        self.base = np.array(base_pos, dtype=np.float32)
        self.link_lengths = [0.45, 0.30, 0.18]

    def forward_kinematics(self, t):
        base_height = 0.45

        theta1 = np.deg2rad(-20) - np.deg2rad(50) * (0.5 - 0.5 * np.cos(t * 2 * np.pi))
        theta2 = np.deg2rad(-30) - np.deg2rad(40) * (0.5 - 0.5 * np.cos(t * 2 * np.pi))

        L1, L2, L3 = self.link_lengths

        p0 = self.base.copy()
        p1 = p0 + np.array([0.0, base_height, 0.0])
        p2 = p1 + np.array([-L1 * np.cos(theta1),
                             L1 * np.sin(theta1), 0.0])
        theta_abs = theta1 + theta2
        p3 = p2 + np.array([-L2 * np.cos(theta_abs),
                             L2 * np.sin(theta_abs), 0.0])
        theta_grip = theta_abs + np.deg2rad(-20)
        p4 = p3 + np.array([-L3 * np.cos(theta_grip),
                             L3 * np.sin(theta_grip), 0.0])

        return [p0, p1, p2, p3, p4]

    def get_gripper_pos(self, t):
        return self.forward_kinematics(t)[-1]


def apply_robot_contact(pos, vel, gripper_pos, field, force_strength=40.0):
    contact_radius = 0.08
    for i in range(len(pos)):
        if field["is_root"][i] == 1:
            continue
        diff = pos[i] - gripper_pos
        dist = np.linalg.norm(diff)
        if dist < contact_radius and dist > 1e-6:
            direction   = diff / dist
            penetration = contact_radius - dist
            push = direction * penetration * force_strength
            vel[i] += push * DT / SUBSTEPS
    return vel
