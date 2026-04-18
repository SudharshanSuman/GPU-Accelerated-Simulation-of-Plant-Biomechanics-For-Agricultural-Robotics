"""Interactive Open3D visualization loop: GPU forces + PBD constraints + robot."""

import time

import numpy as np
import open3d as o3d
import warp as wp

from .config import (
    STEM_NODES, LEAF_CONFIG,
    DT, SUBSTEPS, PBD_ITERS,
    DAMPING, ROOT_DAMPING, SOIL_DEPTH,
    RHO_AIR, RHO_WATER, CD_STEM, CD_LEAF,
    RAIN_RATE, V_RAIN,
    wind_speed, rain_intensity,
)
from .physics import k_apply_forces
from .pbd import pbd_solve
from .robot import SimpleRobotArm, apply_robot_contact
from .meshes import (
    make_stem_mesh, make_leaf_mesh, make_robot_arm_mesh, make_ground_with_grid,
)


def visualize_plant_and_robot(plant_data, max_frames=600):
    n = len(plant_data["pos"])
    pos_np  = plant_data["pos"].copy()
    vel_np  = plant_data["vel"].copy()
    rest_np = plant_data["rest_pos"]

    pos_gpu  = wp.array(pos_np, dtype=wp.vec3,   device="cuda")
    vel_gpu  = wp.array(vel_np, dtype=wp.vec3,   device="cuda")
    mass_gpu = wp.array(plant_data["mass"],         dtype=wp.float32, device="cuda")
    area_gpu = wp.array(plant_data["frontal_area"], dtype=wp.float32, device="cuda")
    root_gpu = wp.array(plant_data["is_root"],      dtype=wp.int32,   device="cuda")
    leaf_gpu = wp.array(plant_data["is_leaf"],      dtype=wp.int32,   device="cuda")

    robot = SimpleRobotArm(base_pos=(0.35, 0.0, 0.0))

    vis = o3d.visualization.Visualizer()
    vis.create_window(
        window_name="Plant Biomechanics + Agri-Robot (GPU Simulation)",
        width=1280, height=720)

    for g in make_ground_with_grid():
        vis.add_geometry(g)

    coord = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.12)
    vis.add_geometry(coord)

    ropt = vis.get_render_option()
    ropt.background_color = np.array([0.94, 0.96, 0.98])
    ropt.light_on = True
    ropt.mesh_show_back_face = True

    plant_geoms = []
    robot_geoms = []

    ctr = vis.get_view_control()
    ctr.set_lookat([0.2, 0.5, 0.0])
    ctr.set_front([0.2, -0.15, -1.0])
    ctr.set_up([0.0, 1.0, 0.0])
    ctr.set_zoom(0.45)

    sub_dt = DT / SUBSTEPS
    frame  = 0

    print("\n" + "=" * 60)
    print(" 3D Visualization — close window or Ctrl+C to exit")
    print("=" * 60)
    print(" Left drag : rotate  |  Right drag : pan  |  Scroll : zoom")
    print("=" * 60 + "\n")

    try:
        while frame < max_frames:
            t_robot = (frame % 150) / 150.0
            w       = wind_speed(frame)
            rain_on = 1 if rain_intensity(frame) > 0.0 else 0

            for _ in range(SUBSTEPS):
                wp.launch(k_apply_forces, dim=n,
                          inputs=[pos_gpu, vel_gpu, mass_gpu, area_gpu,
                                  root_gpu, leaf_gpu, w, rain_on,
                                  RHO_AIR, RHO_WATER, CD_STEM, CD_LEAF,
                                  RAIN_RATE, V_RAIN,
                                  DAMPING, ROOT_DAMPING, SOIL_DEPTH, sub_dt])

                pos_np = pos_gpu.numpy().copy()
                vel_np = vel_gpu.numpy().copy()

                if not np.all(np.isfinite(pos_np)):
                    pos_np = rest_np.copy()
                    vel_np = np.zeros_like(pos_np)

                pos_np[0] = [0.0, 0.0, 0.0]
                vel_np[0] = [0.0, 0.0, 0.0]

                gripper = robot.get_gripper_pos(t_robot)
                vel_np = apply_robot_contact(pos_np, vel_np, gripper,
                                             plant_data, force_strength=60.0)

                pos_np = pbd_solve(pos_np, rest_np,
                                   plant_data["parents"], plant_data["children"],
                                   plant_data["L0"], plant_data["bend_k"],
                                   plant_data["is_root"], n_iters=PBD_ITERS)

                pos_gpu = wp.array(pos_np, dtype=wp.vec3, device="cuda")
                vel_gpu = wp.array(vel_np, dtype=wp.vec3, device="cuda")

            for g in plant_geoms + robot_geoms:
                vis.remove_geometry(g, reset_bounding_box=False)
            plant_geoms.clear()
            robot_geoms.clear()

            stem_positions = pos_np[:STEM_NODES]
            plant_geoms.extend(make_stem_mesh(stem_positions))

            cursor = STEM_NODES
            for (attach, ang, n_seg) in LEAF_CONFIG:
                leaf_positions = np.vstack([
                    pos_np[attach:attach + 1],
                    pos_np[cursor:cursor + n_seg]
                ])
                plant_geoms.extend(make_leaf_mesh(leaf_positions))
                cursor += n_seg

            joints = robot.forward_kinematics(t_robot)
            robot_geoms.extend(make_robot_arm_mesh(joints))

            for g in plant_geoms + robot_geoms:
                vis.add_geometry(g, reset_bounding_box=False)

            vis.poll_events()
            vis.update_renderer()
            frame += 1
            time.sleep(0.016)

    except KeyboardInterrupt:
        print("\nStopped by user.")

    vis.destroy_window()
    print(f"\nCompleted {frame} frames.")
