"""Plant geometry: tapered stem + leaf branches with Young's-modulus stiffness."""

import numpy as np

from .config import (
    STEM_NODES, L0_STEM, R_BASE, R_TIP, RHO_STEM,
    E_STEM, E_LEAF,
    LEAF_CONFIG, L0_LEAF, R_LEAF, A_LEAF_BLADE, RHO_LEAF,
)


def stem_radius(i, n=STEM_NODES):
    t = i / (n - 1)
    return R_BASE + (R_TIP - R_BASE) * t


def area(r):
    return np.pi * r * r


def second_moment(r):
    return np.pi * r ** 4 / 4.0


def flexural_rigidity(E, r):
    return E * second_moment(r)


def compute_pbd_stiffness(E, r, L0):
    EI     = flexural_rigidity(E, r)
    EI_ref = flexural_rigidity(E_STEM, R_BASE)
    ratio  = EI / EI_ref
    return 0.04 + (0.92 - 0.04) * np.clip(ratio, 0.0, 1.0) ** 0.4


def build_single_plant(x_offset=0.0):
    pos, mass, is_root, is_leaf, front_area = [], [], [], [], []
    parents, children, L0_list, bend_k = [], [], [], []

    for i in range(STEM_NODES):
        y = i * L0_STEM
        pos.append([x_offset, y, 0.0])
        is_root.append(1 if i == 0 else 0)
        is_leaf.append(0)
        r = stem_radius(i)
        mass.append(max(RHO_STEM * area(r) * L0_STEM, 1e-5))
        front_area.append(2.0 * r * L0_STEM)
        if i > 0:
            r_edge = 0.5 * (stem_radius(i - 1) + stem_radius(i))
            parents.append(i - 1)
            children.append(i)
            L0_list.append(L0_STEM)
            bend_k.append(compute_pbd_stiffness(E_STEM, r_edge, L0_STEM))

    for (attach, angle_deg, n_seg) in LEAF_CONFIG:
        a = np.radians(angle_deg)
        dx, dy = np.sin(a) * L0_LEAF, np.cos(a) * L0_LEAF
        base = pos[attach]
        prev = attach
        leaf_stiff = compute_pbd_stiffness(E_LEAF, R_LEAF, L0_LEAF)
        for j in range(1, n_seg + 1):
            idx = len(pos)
            pos.append([base[0] + dx * j, base[1] + dy * j, 0.0])
            is_root.append(0)
            is_leaf.append(1)
            mass.append(RHO_LEAF * area(R_LEAF) * L0_LEAF)
            front_area.append(A_LEAF_BLADE)
            parents.append(prev)
            children.append(idx)
            L0_list.append(L0_LEAF)
            bend_k.append(leaf_stiff)
            prev = idx

    pos_np = np.asarray(pos, dtype=np.float32)
    return dict(
        n_nodes=len(pos),
        pos=pos_np,
        rest_pos=pos_np.copy(),
        vel=np.zeros_like(pos_np),
        mass=np.asarray(mass, dtype=np.float32),
        is_root=np.asarray(is_root, dtype=np.int32),
        is_leaf=np.asarray(is_leaf, dtype=np.int32),
        frontal_area=np.asarray(front_area, dtype=np.float32),
        parents=np.asarray(parents, dtype=np.int32),
        children=np.asarray(children, dtype=np.int32),
        L0=np.asarray(L0_list, dtype=np.float32),
        bend_k=np.asarray(bend_k, dtype=np.float32),
    )
