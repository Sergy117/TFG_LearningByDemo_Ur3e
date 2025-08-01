---
Learning and Validation Pipeline
---
## Overview

This directory contains the core scripts for the Learning from Demonstration (LfD) pipeline. These scripts are responsible for processing the raw 3D demonstration data (captured by the vision_system) and training the final policies that the robot will execute.

The workflow is divided into two main stages:

1. **Validation**: Visually inspect and verify the quality of the human-to-robot kinematic mapping on individual demonstrations.

2. **Learning**: Process the entire dataset of validated demonstrations to train a generative model—a Dynamic Movement Primitive (DMP)—for each phase of the task.

## Dependencies

Both scripts in this folder share a common set of Python dependencies.

* **Required Libraries**: numpy, pandas, matplotlib, scipy, pydmps.

* **You can install them via pip**:

```sh
pip install numpy pandas matplotlib scipy pydmps
```
1. **Validation Script**: validate_compare_kinematic_mapping.py

**Purpose**

This script is a crucial visual debugging and validation tool. Its purpose is to load a single human demonstration and render a 3D animation that directly compares the original human motion with the resulting robot imitation. This allows you to verify that the entire kinematic translation pipeline (including calibration, morphological reflection, and angle mapping) is working correctly before committing to training on the full dataset.

**Configuration**

Before running, ensure the following constants at the top of the script are correctly set:

* **DEMONSTRATIONS_BASE_DIR**: The path pattern to find all your demonstration folders.

* **CALIBRATION_MATRIX_FILE**: The path to your T_base_a_camara.npy calibration file.

* **SCALE_FACTORS**: The dictionary used to invert joint directions if needed (e.g., to solve the "arm under the table" problem).

**How to Run**

1. Ensure the configuration paths are correct.

2. Run the script from your terminal:
```sh
python3 validate_compare_kinematic_mapping.py
```
**What it Does**

The script will open a Matplotlib window and display a 3D animation. You will see two skeletons:

* **Blue Skeleton**: Represents the original human demonstration, transformed into the robot's coordinate frame for a fair comparison.

* **Green Skeleton**: Represents the final robot arm pose, calculated by applying Forward Kinematics (FK) to the translated joint angles.

The script provides an interactive menu in the terminal to cycle through your demonstrations (n for next, r for replay, q for quit).

2. **Learning Script:** learn_dmp.py

**Purpose**

This is the main offline training script. It processes the entire dataset of human demonstrations to learn a generalized model for each phase of the task. It implements the final stage of the LfD pipeline, producing the policies that the robot controller will execute.

**Configuration**

Verify the following constants at the top of the script:

* **DEMONSTRATIONS_BASE_DIR**: The path pattern to find all your demonstration folders.

* **CALIBRATION_MATRIX_FILE**: The path to your calibration file.

* **DMP_OUTPUT_DIR**: The name of the directory where the learned models (.pkl files) will be saved.

* **PHASES_TO_LEARN**: A list of the action phases for which a DMP will be trained.

* **NUM_RESAMPLE_POINTS**: The fixed length to which all trajectories will be resampled.

* **N_BASIS_FUNCTIONS**: The number of basis functions for the DMP model, defining its complexity.

**How to Run**

1. Ensure all configuration paths and parameters are correct.

2. Run the script from your terminal:
```sh
python3 learn_dmp.py
```
**What it Does**

The script executes the full learning workflow automatically:

1. It iterates through each phase defined in PHASES_TO_LEARN.

2. For each phase, it finds and processes all corresponding demonstration segments from your dataset.

3. Each demonstration is translated into a robot joint space trajectory using the validated kinematic mapping logic.

4. All trajectories for the phase are resampled to a uniform length.

5. A single average trajectory is computed from all demonstrations.

6. A DMP model is trained to imitate this averaged trajectory.

7. The final trained DMP model, along with its start and goal poses, is saved to a .pkl file in the DMP_OUTPUT_DIR.

**Outputs**

The final output is a set of .pkl files (e.g., dmp_grasping_to_rotating.pkl), one for each task phase. These files contain the learned, generative policies that are ready to be loaded and executed by the robot_player_from_dmp.py script in the robot_control module.

## Recommended Workflow

1. Use validate_compare_kinematic_mapping.py to visually inspect a few of your new demonstrations to ensure the translation quality is high.

2. Once satisfied, run learn_dmp.py once to process the entire dataset and generate the final DMP models.

Proceed to the robot_control system to execute the learned policies.