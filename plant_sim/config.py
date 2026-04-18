"""Simulation constants and environment schedules."""

# Stem geometry and material
STEM_NODES   = 12
STEM_HEIGHT  = 1.0
L0_STEM      = STEM_HEIGHT / (STEM_NODES - 1)
R_BASE       = 0.008
R_TIP        = 0.002
RHO_STEM     = 700.0
CD_STEM      = 1.2

# Young's modulus (Pa)
E_STEM       = 2.0e7
E_LEAF       = 5.0e6

# Leaves: (attach_node, angle_deg, n_segments)
LEAF_CONFIG  = [
    (3,  55, 3), (3, -55, 3),
    (6,  60, 4), (6, -60, 4),
    (9,  50, 3), (9, -50, 3),
]
L0_LEAF      = 0.055
R_LEAF       = 0.0015
A_LEAF_BLADE = 0.005
RHO_LEAF     = 500.0
CD_LEAF      = 2.5

# Environment
G            = 9.81
RHO_AIR      = 1.225
RHO_WATER    = 1000.0
V_RAIN       = 9.0
RAIN_RATE    = 8.0e-5

# Soil / integration
SOIL_DEPTH   = 0.05
ROOT_DAMPING = 0.55
DT           = 1.0 / 60.0
SUBSTEPS     = 8
DAMPING      = 0.97
PBD_ITERS    = 4

# Weather schedule
WIND_CALM   = 0.5
WIND_STRONG = 15.0
RAIN_START  = 50
RAIN_END    = 200


def wind_speed(frame):
    if frame < RAIN_START:      return WIND_STRONG
    if frame < RAIN_END:        return WIND_STRONG * 1.2
    if frame < RAIN_END + 30:   return 3.0
    return WIND_CALM


def rain_intensity(frame):
    if frame < RAIN_START:      return 0.0
    if frame < RAIN_START + 15: return 1.0
    if frame < RAIN_END:        return 2.5
    return 0.0
