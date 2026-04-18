"""GPU force kernel (NVIDIA Warp) — gravity, wind drag, rain, soil damping."""

import warp as wp

from .config import G


@wp.kernel
def k_apply_forces(positions:  wp.array(dtype=wp.vec3),
                   velocities: wp.array(dtype=wp.vec3),
                   mass:       wp.array(dtype=wp.float32),
                   area_arr:   wp.array(dtype=wp.float32),
                   is_root:    wp.array(dtype=wp.int32),
                   is_leaf:    wp.array(dtype=wp.int32),
                   wind_x:     float, rain_on: int,
                   rho_air:    float, rho_w:   float,
                   cd_stem:    float, cd_leaf:  float,
                   rain_rate:  float, v_rain:   float,
                   damping:    float, soil_damp: float,
                   soil_y:     float, dt:       float):
    i = wp.tid()
    if is_root[i] == 1:
        velocities[i] = wp.vec3(0.0, 0.0, 0.0)
        return

    m = mass[i]
    f = wp.vec3(0.0, -G * m, 0.0)

    v_rel = wp.vec3(wind_x, 0.0, 0.0) - velocities[i]
    speed = wp.length(v_rel)
    cd = cd_stem
    if is_leaf[i] == 1:
        cd = cd_leaf
    h_factor = 0.3 + 0.8 * positions[i][1]
    f = f + v_rel * (0.5 * rho_air * cd * area_arr[i] * speed * h_factor)

    if rain_on == 1:
        rf = rho_w * rain_rate * v_rain * area_arr[i]
        if is_leaf[i] == 1:
            rf = rf * 30.0
        else:
            rf = rf * 5.0
        f = f + wp.vec3(0.5 * rf, -rf, 0.0)

    a = f / m
    a_mag = wp.length(a)
    if a_mag > 800.0:
        a = a * (800.0 / a_mag)

    d = damping
    if positions[i][1] < soil_y:
        d = soil_damp

    velocities[i] = (velocities[i] + a * dt) * d
    positions[i]  =  positions[i]  + velocities[i] * dt

    if positions[i][1] < 0.0:
        positions[i]  = wp.vec3(positions[i][0], 0.0, positions[i][2])
        velocities[i] = wp.vec3(velocities[i][0], 0.0, velocities[i][2])
