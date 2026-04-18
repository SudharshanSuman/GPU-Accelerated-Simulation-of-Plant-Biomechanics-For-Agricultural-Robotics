# GPU-Accelerated Simulation of Plant Biomechanics for Agricultural Robotics

A GPU physics playground (NVIDIA Warp + Open3D) for simulating how real plants
bend, droop, and recover under wind, rain, and robot contact. The end goal is
an agri-robot stack (Isaac Sim + ROS 2 Humble) that can plan gentle grasps
around living plants without damaging them.

## The Big Picture

Three layers, built bottom-up:

1. **Deformable plant model** — stems and leaves bending realistically under
   internal elastic forces.
2. **Environmental forces** — gravity, wind drag, rain impact, soil damping.
3. **Robot arm** — 3-link kinematic arm whose gripper applies contact forces
   to the plant, eventually running inside Isaac Sim with a ROS 2 bridge.

## Project History — What's in Each Notebook

The repo grew notebook-by-notebook, each one adding a physical phenomenon on
top of the previous. They are kept for reference; `Final.ipynb` is the current
entry point.

| Notebook | What it does |
|---|---|
| `Sample Start.ipynb`   | Warp sanity check — initialize CUDA, run a gravity kernel on 10 nodes, confirm `cuda:0` is live. |
| `spring_chain.ipynb`   | First real sim: 12-node stem with variable stiffness, wind + gravity, CPU constraint enforcement. Produces `stem_bending.png`. |
| `leavesplants.ipynb`   | Adds leaf branches with root/leaf flags in the GPU kernel. |
| `Leaves-plants.ipynb`  | Extends to full physics: stem + 6 leaves, rain forces, soil damping, substepping, PBD constraints. Produces `plant_physics_accurate.png`, `plant_rain_soil.png`, `plant_with_leaves.png`. |
| `Multiplant.ipynb`     | Scales to a field of 10 plants running simultaneously on GPU. Produces `field_10_plants.png`. |
| `Final.ipynb`          | **Current entry point.** Modular driver that imports the `plant_sim/` package and launches the interactive Open3D viewer with a 3-link robot arm reaching toward the plant. |

## The `plant_sim/` Package

All simulation logic for `Final.ipynb` is split into importable modules so it
can be reused from Isaac Sim, ROS 2 nodes, or unit tests.

```
plant_sim/
├── __init__.py      # lazy re-exports (heavy deps loaded on demand)
├── config.py        # geometry, material, weather, integration constants
├── plant.py         # build_single_plant() — stem + leaf graph with Young's-modulus bending stiffness
├── physics.py       # k_apply_forces — Warp GPU kernel (gravity, wind drag, rain, soil damping)
├── pbd.py           # pbd_solve — Position-Based Dynamics (edge length + rest-direction bending)
├── robot.py         # SimpleRobotArm forward kinematics + apply_robot_contact soft push
├── meshes.py        # Open3D cylinder / sphere / ground-with-grid builders
└── simulation.py    # visualize_plant_and_robot — interactive viewer loop
```

### Physics Model

- **Plant**: 12 stem nodes + 6 leaf branches (3 pairs at nodes 3, 6, 9) ≈ 32 nodes total.
- **Tapered stem**: radius 4 mm at base → 1.5 mm at tip.
- **Stiffness**: from flexural rigidity `EI = E · πr⁴/4`, mapped to a per-edge
  PBD bending coefficient. `E_STEM = 20 MPa`, `E_LEAF = 5 MPa`.
- **Wind**: height-proportional drag, ramps down after a rain event.
- **Rain**: asymmetric forces (leaves feel ~30×, stem ~5×), active frames 50–200.
- **Soil**: nodes below 5 cm get damping `0.55` vs `0.97` in air.
- **Integration**: `dt = 1/60`, 8 substeps per frame, 4 PBD iterations per substep.
- **Robot contact**: gripper within 8 cm of a node pushes it outward with a soft penetration force.

## Setup

### System check (run once)

```bash
lsb_release -a                  # should say Ubuntu 22.04
nvidia-smi                      # should show RTX 4070, driver 525+
nvcc --version                  # verify CUDA toolkit
python3 --version               # needs 3.10+
```

Install CUDA toolkit if `nvcc` is missing:

```bash
sudo apt update
sudo apt install nvidia-cuda-toolkit -y
```

### Python environment

```bash
python3 -m venv plant_sim_env
source plant_sim_env/bin/activate

pip install warp-lang numpy scipy matplotlib open3d
```

Optional for the target stack:

```bash
sudo apt install ros-humble-desktop        # ROS 2 Humble (Ubuntu 22.04)
# Isaac Sim via NVIDIA Omniverse Launcher
```

### Verify Warp

```python
import warp as wp
wp.init()
print(wp.get_device())   # should print: cuda:0
```

## Running

### Interactive 3D viewer (recommended)

Open `Final.ipynb` in Jupyter and run the cells top-to-bottom, or from Python:

```python
from plant_sim import build_single_plant, visualize_plant_and_robot

plant = build_single_plant()
visualize_plant_and_robot(plant)
```

Viewer controls: **left drag** rotate · **right drag** pan · **scroll** zoom.

### Reproduce earlier figures

Open the legacy notebook and run all cells:

```bash
jupyter notebook spring_chain.ipynb     # → stem_bending.png
jupyter notebook Leaves-plants.ipynb    # → plant_physics_accurate.png
jupyter notebook Multiplant.ipynb       # → field_10_plants.png
```

## Generated Images

| File | Source notebook | Content |
|---|---|---|
| `stem_bending.png`          | `spring_chain.ipynb`   | Early 6-frame wind-only stem |
| `plant_with_leaves.png`     | `Leaves-plants.ipynb`  | Leaves added, variable rigidity |
| `plant_rain_soil.png`       | `Leaves-plants.ipynb`  | Wind + rain + soil damping |
| `plant_physics_accurate.png`| `Leaves-plants.ipynb`  | Stiffness ∝ 1/height, 6-panel timeline |
| `field_10_plants.png`       | `Multiplant.ipynb`     | 10-plant field on GPU |

## Roadmap

- 3D simulation (add Z-axis bending)
- Isaac Sim scene with textured plant meshes
- Robot arm contact forces driving a real URDF
- ROS 2 publishers for plant state and gripper commands

## Hardware / Stack

- NVIDIA Warp 1.12.0 (CUDA 12.9, RTX 4070, sm_89)
- Python 3.10+ with NumPy / SciPy / Matplotlib / Open3D
- Target: Isaac Sim (Omniverse) + ROS 2 Humble on Ubuntu 22.04
