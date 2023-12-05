# panda_w_realsense
This code connects [MoveIt!](https://ros-planning.github.io/moveit_tutorials/) and [IsaacSim](https://developer.nvidia.com/isaac-sim) using Franka Panda robot with realsense D435i RGB-D camera.

Based on ROS1(noetic) and Ubuntu 20.04.

# Requirement
- [IsaacSim](https://developer.nvidia.com/isaac-sim)
- [MoveIT!](https://ros-planning.github.io/moveit_tutorials/)

If you want to use conda environment, folowing below.
1. Go to your IsaacSim path
 
```
cd .local/share/ov/pkg/isaac_sim-2023.1.0-hotfix.1/
```

2. Make your conda env by `environment.yml`.

```
conda env create --name isaac-sim -f environment.yml
```
```
conda activate isaac-sim
```

3. Setup directories to load isaac-sim variables
```
mkdir -p ${CONDA_PREFIX}/etc/conda/activate.d
```
```
printf '%s\n' '#!/bin/bash' '' \
        '# for isaac-sim' \
        'source '${isaacsim_setup_conda_env_script}'' \
        '' \
        '# show icon if not runninng headless' \
        'export RESOURCE_NAME="IsaacSim"' \
        '' > ${CONDA_PREFIX}/etc/conda/activate.d/setenv.sh
```
```
mkdir -p ${CONDA_PREFIX}/etc/conda/deactivate.d
```
```
printf '%s\n' '#!/bin/bash' '' \
        '# for isaac-sim' \
        'unset CARB_APP_PATH' \
        'unset EXP_PATH' \
        'unset ISAAC_PATH' \
        'unset RESOURCE_NAME' \
        '' \
        '# restore paths' \
        'export PYTHONPATH='${cache_pythonpath}'' \
        'export LD_LIBRARY_PATH='${cache_ld_library_path}'' \
        '' > ${CONDA_PREFIX}/etc/conda/deactivate.d/unsetenv.sh
```

# Usage
1. Turn on Ros Master.
```
roscore
```
2. Open IsaacSim by Launcher.
3. Open `panda.usd` in isaac_model folder and Play it.
4. Turn on moveit_config pkg.
```
roslaunch panda_w_realsense_moveit_config demo_isaac.launch
```
