# FRA371 Kinematics Lab 3

Code template for Lab 3 Part B: differential kinematics, manipulability, and resolved-rate control, using the Robotics Toolbox for Python and the Swift simulator.

## Repository contents

| Path | Description |
| --- | --- |
| `Gxx_Lab3_part_B_main.ipynb` | Main notebook. Complete the sections marked with `*`. |
| `Lab3_part_B_utils.py` | Helper functions for loading the robot, simulation, and plotting. Do not modify this file. |
| `my_robot/` | Robot descriptions. Includes `Dummy_articulated_RRR.urdf`, a 3-joint revolute arm. |
| `requirements.txt` | Python package versions for this lab. |

## Setup(Optional)

You need Python 3.11 or newer. Check your version with:

```
python --version
```

Install the packages from the repository folder:

```
pip install -r requirements.txt
python fix_swift.py
```

Open the notebook in VS Code with the Python and Jupyter extensions, and select the Python environment you installed into as the kernel. Open the repository folder itself, because the robot files are loaded from `my_robot/` relative to the notebook's working directory.


## Using your robot

1. Copy your URDF into `my_robot/`. If it uses meshes, put them in `my_robot/meshes/` and reference them as `package://my_robot/meshes/...`.
2. In the **Robot Preparation** section, set `urdf_file_name` to your file name.
3. For Parts B2 and B3, set `urdf_file_name` to `Dummy_articulated_RRR.urdf`.

## What to complete

| Part | Robot | Task |
| --- | --- | --- |
| B1 | Your robot | Implement `CalcForwardDK`, `CalcInverseDK`, and `CalcManipulability`, then check them against your hand calculations from Part A. |
| B2 | Dummy robot | Implement `ResolvedRateController` and the simulation loop, then plot the results. |
| B3 | Dummy robot | Complete the singularity simulation loop, inspect the plots, and answer the question in the notebook. |

## Submission

Rename the notebook with your group number, for example `G01_Lab3_part_B_main.ipynb`, and fill in the group number and members in the cell near the top. Then, zip lab folder, and rename to FRA371_Lab3_Gxx (xx is group number).
