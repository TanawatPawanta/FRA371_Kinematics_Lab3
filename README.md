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

### Create a virtual environment

A virtual environment keeps this lab's packages separate from other Python projects. In a terminal opened in the repository folder, create one named `FRA371_Lab3_env`:

```
python -m venv FRA371_Lab3_env
```

You only need to do this once.

### Activate the environment

Activate it every time you open a new terminal to work on the lab.

Windows (PowerShell, the default terminal in VS Code):

```
Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass
FRA371_Lab3_env\Scripts\Activate.ps1
```

PowerShell blocks scripts by default, so the first line allows them for this terminal window only. It changes nothing permanently, so run both lines every time you open a new terminal.

Windows (Command Prompt only, not PowerShell):

```
FRA371_Lab3_env\Scripts\activate.bat
```

macOS / Linux:

```
source FRA371_Lab3_env/bin/activate
```

When it is active, the prompt starts with `(FRA371_Lab3_env)`. To leave the environment, run `deactivate`.
### Install the packages

With the environment active, install the packages from the repository folder:

```
pip install -r requirements.txt
python fix_swift.py
```

Open the notebook in VS Code with the Python and Jupyter extensions, and select the `FRA371_Lab3_env` environment as the kernel. Open the repository folder itself, because the robot files are loaded from `my_robot/` relative to the notebook's working directory.

### Delete the environment

To remove the environment, for example to start over with a clean install, first close the notebook (or shut down its kernel) and run `deactivate` if the environment is active. Then, in the repository folder, delete its folder:

Windows (PowerShell):

```
Remove-Item -Recurse -Force FRA371_Lab3_env
```

Windows (Command Prompt):

```
rmdir /s /q FRA371_Lab3_env
```

macOS / Linux:

```
rm -rf FRA371_Lab3_env
```

This only deletes the environment and its installed packages. Your notebook and lab files are not affected. To use the lab again, repeat the steps from **Create a virtual environment**.


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
