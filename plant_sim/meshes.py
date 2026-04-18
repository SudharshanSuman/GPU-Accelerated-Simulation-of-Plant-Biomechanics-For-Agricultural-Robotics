"""Open3D mesh builders for stem, leaves, robot arm, and ground plane."""

import numpy as np
import open3d as o3d

from .config import R_BASE, R_TIP


def _align_cylinder(cyl, p0, p1):
    vec    = p1 - p0
    length = np.linalg.norm(vec)
    if length < 1e-5:
        return None
    z_axis = np.array([0.0, 0.0, 1.0])
    dir_vec = vec / length
    axis   = np.cross(z_axis, dir_vec)
    angle  = np.arccos(np.clip(np.dot(z_axis, dir_vec), -1.0, 1.0))
    if np.linalg.norm(axis) > 1e-6:
        axis /= np.linalg.norm(axis)
        R = o3d.geometry.get_rotation_matrix_from_axis_angle(axis * angle)
        cyl.rotate(R, center=(0, 0, 0))
    cyl.translate((p0 + p1) * 0.5)
    return cyl


def make_stem_mesh(positions, color=(0.11, 0.62, 0.46)):
    meshes = []
    for i in range(len(positions) - 1):
        p0 = np.asarray(positions[i],     dtype=np.float32)
        p1 = np.asarray(positions[i + 1], dtype=np.float32)
        length = np.linalg.norm(p1 - p0)
        if length < 1e-5:
            continue
        t = i / max(len(positions) - 2, 1)
        r = R_BASE + (R_TIP - R_BASE) * t
        cyl = o3d.geometry.TriangleMesh.create_cylinder(
            radius=r * 3.5, height=length, resolution=8)
        cyl.paint_uniform_color(color)
        cyl.compute_vertex_normals()
        _align_cylinder(cyl, p0, p1)
        meshes.append(cyl)
    return meshes


def make_leaf_mesh(positions, color=(0.36, 0.79, 0.65)):
    meshes = []
    for i in range(len(positions) - 1):
        p0 = np.asarray(positions[i],     dtype=np.float32)
        p1 = np.asarray(positions[i + 1], dtype=np.float32)
        length = np.linalg.norm(p1 - p0)
        if length < 1e-5:
            continue
        cyl = o3d.geometry.TriangleMesh.create_cylinder(
            radius=0.008, height=length, resolution=6)
        cyl.paint_uniform_color(color)
        cyl.compute_vertex_normals()
        _align_cylinder(cyl, p0, p1)
        meshes.append(cyl)
    return meshes


def make_robot_arm_mesh(joints,
                        link_color=(0.3, 0.3, 0.35),
                        joint_color=(0.2, 0.2, 0.25),
                        gripper_color=(0.85, 0.25, 0.15)):
    meshes = []
    for p in joints[:-1]:
        sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.025, resolution=10)
        sphere.paint_uniform_color(joint_color)
        sphere.compute_vertex_normals()
        sphere.translate(p)
        meshes.append(sphere)

    gripper = o3d.geometry.TriangleMesh.create_sphere(radius=0.04, resolution=12)
    gripper.paint_uniform_color(gripper_color)
    gripper.compute_vertex_normals()
    gripper.translate(joints[-1])
    meshes.append(gripper)

    for i in range(len(joints) - 1):
        p0, p1 = joints[i], joints[i + 1]
        length = np.linalg.norm(p1 - p0)
        if length < 1e-5:
            continue
        cyl = o3d.geometry.TriangleMesh.create_cylinder(
            radius=0.018, height=length, resolution=10)
        cyl.paint_uniform_color(link_color)
        cyl.compute_vertex_normals()
        _align_cylinder(cyl, p0, p1)
        meshes.append(cyl)

    return meshes


def make_ground_with_grid(size=2.5, color=(0.45, 0.32, 0.20)):
    meshes = []

    ground = o3d.geometry.TriangleMesh.create_box(
        width=size, height=0.01, depth=size)
    ground.translate((-size / 2, -0.01, -size / 2))
    ground.paint_uniform_color(color)
    ground.compute_vertex_normals()
    meshes.append(ground)

    grid_lines = o3d.geometry.LineSet()
    points = []
    lines  = []
    n_lines = 11
    step = size / (n_lines - 1)
    idx = 0
    for i in range(n_lines):
        x = -size / 2 + i * step
        points.append([x, 0.001, -size / 2])
        points.append([x, 0.001,  size / 2])
        lines.append([idx, idx + 1])
        idx += 2
        z = -size / 2 + i * step
        points.append([-size / 2, 0.001, z])
        points.append([ size / 2, 0.001, z])
        lines.append([idx, idx + 1])
        idx += 2

    grid_lines.points = o3d.utility.Vector3dVector(points)
    grid_lines.lines  = o3d.utility.Vector2iVector(lines)
    grid_lines.paint_uniform_color([0.3, 0.22, 0.14])
    meshes.append(grid_lines)

    return meshes
