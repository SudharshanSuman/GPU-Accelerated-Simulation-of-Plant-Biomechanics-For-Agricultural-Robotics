"""GPU plant biomechanics package. Heavy deps (open3d, warp) are imported lazily.

Typical usage:
    from plant_sim import build_single_plant, visualize_plant_and_robot
    plant = build_single_plant()
    visualize_plant_and_robot(plant)
"""

from .config import *  # noqa: F401,F403
from .plant import build_single_plant  # noqa: F401


def __getattr__(name):
    if name == "k_apply_forces":
        from .physics import k_apply_forces
        return k_apply_forces
    if name == "pbd_solve":
        from .pbd import pbd_solve
        return pbd_solve
    if name in ("SimpleRobotArm", "apply_robot_contact"):
        from . import robot
        return getattr(robot, name)
    if name in ("make_stem_mesh", "make_leaf_mesh",
                "make_robot_arm_mesh", "make_ground_with_grid"):
        from . import meshes
        return getattr(meshes, name)
    if name == "visualize_plant_and_robot":
        from .simulation import visualize_plant_and_robot
        return visualize_plant_and_robot
    raise AttributeError(f"module 'plant_sim' has no attribute {name!r}")
