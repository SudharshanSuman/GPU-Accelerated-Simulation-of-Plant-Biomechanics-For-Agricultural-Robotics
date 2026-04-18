"""Position-Based Dynamics constraint solver: edge-length + rest-direction bending."""

import numpy as np


def pbd_solve(pos, rest_pos, parents, children, L0, bend_k, is_root, n_iters=4):
    for _ in range(n_iters):
        for e in range(len(parents)):
            p, c = parents[e], children[e]
            diff = pos[c] - pos[p]
            dist = np.linalg.norm(diff)
            if dist > 1e-8:
                n_vec = diff / dist
                err = dist - L0[e]
                if is_root[p]:
                    pos[c] -= n_vec * err
                else:
                    pos[p] += n_vec * err * 0.5
                    pos[c] -= n_vec * err * 0.5

        for e in range(len(parents)):
            p, c = parents[e], children[e]
            rest_dir = rest_pos[c] - rest_pos[p]
            rest_len = np.linalg.norm(rest_dir)
            if rest_len < 1e-8:
                continue
            rest_dir /= rest_len
            cur_dir = pos[c] - pos[p]
            cur_len = np.linalg.norm(cur_dir)
            if cur_len < 1e-8:
                continue
            target = pos[p] + rest_dir * cur_len
            pos[c] = pos[c] * (1.0 - bend_k[e]) + target * bend_k[e]

    pos[:, 1] = np.maximum(pos[:, 1], 0.0)
    return pos
