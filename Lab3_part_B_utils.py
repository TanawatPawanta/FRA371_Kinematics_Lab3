import numpy as np
import roboticstoolbox as rtb
from roboticstoolbox.robot.ERobot import ERobot
import spatialmath as sm
import spatialgeometry as sg
from swift.Swift import Swift
import plotly.graph_objects as go
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
from pathlib import Path
import builtins, re, time
import swift.SwiftRoute as _route
from roboticstoolbox.robot.Robot import Robot

# Swift's file server passes "/D:/..." to open(), which is invalid on Windows
_route.open = lambda p, *a, **k: builtins.open(p[1:] if re.match(r"^/[A-Za-z]:", p) else p, *a, **k)
"""
Please do not modify this file!!! This file contains utility functions for Lab 3 Part B.
"""

def DefindeRobot(urdf_file_name:str):
    """
    Define the robot from the URDF file.

    Parameters:
    - urdf_file_name: The name of the URDF file (string).

    Returns:
    - robot: The defined robot object (instance of ERobot class).
    """
    # 1. Get the current working directory using pathlib
    HERE = Path.cwd()
    # 2. Read the URDF with tld=HERE so package://my_robot/meshes/... resolves to the local meshes
    links, name, _, _ = Robot.URDF_read(f'my_robot/{urdf_file_name}', tld=HERE)
    # 3. Build the ERobot from the links
    robot = rtb.ERobot(links, name=name)
        
    return robot

def SetupSimulation(robot: ERobot, start_position: list[float])-> Swift:
    """
    Move the robot to its start position and open it in a Swift simulator.

    Parameters:
    - robot: The robot object (instance of Robot class).
    - start_position: Starting [x, y, z] position of the end-effector [m] (list).

    Returns:
    - env: The Swift environment object.
    """
    position_only_mask = [1, 1, 1, 0, 0, 0]
    sol = robot.ikine_LM(
        sm.SE3(*start_position),
        mask=position_only_mask,
        q0=[0.0, 0.0, 0.0],
        joint_limits=True,
    )
    robot.q = sol.q

    env = Swift()
    env.launch(realtime=True, browser="notebook")
    env.add(robot)
    time.sleep(1)   # Give the simulator time to load before the motion starts

    return env

def ManipulabilityViz(robot: ERobot,
                      env: Swift,
                      eigenvalues: np.ndarray, 
                      eigenvectors: np.ndarray,
):
    """
    Visualize the manipulability based on the eigenvalues and eigenvectors.

    Parameters:
    - robot: The robot object (instance of Robot class).
    - env: The Swift environment object.
    - eigenvalues: The eigenvalues of the Jacobian matrix (numpy array).
    - eigenvectors: The eigenvectors of the Jacobian matrix (numpy array).
    """
    ee_position = robot.fkine(robot.q).t
    colors = [
        [0.0, 1.0, 1.0, 0.9], # Cyan
        [1.0, 0.0, 1.0, 0.9], # Magenta
        [1.0, 1.0, 0.0, 0.9]  # Yellow
    ]
    color_list = ['Cyan', 'Magenta', 'Yellow']

    for i in range(3):
        length = abs(eigenvalues[i])
        print(f"Vector {color_list[i]} = Eigenvalue {i}: {np.round(length, 3)}")
        
        # Skip singularities (length of 0)
        if length < 1e-4:
            continue
            
        vec = eigenvectors[:, i]
        
        # --- 2. NATIVE MATH ROTATION ---
        z_axis = vec / np.linalg.norm(vec) 
        
        # Find an orthogonal axis to complete the coordinate frame
        temp = np.array([1.0, 0.0, 0.0]) if abs(z_axis[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
        y_axis = np.cross(z_axis, temp)
        y_axis = y_axis / np.linalg.norm(y_axis)
        x_axis = np.cross(y_axis, z_axis)
        
        # Stack them into a 3x3 rotation matrix
        R = np.column_stack((x_axis, y_axis, z_axis))
        
        # Combine the Rotation and Translation into a single SE3 pose
        arrow_pose = sm.SE3.Rt(R, ee_position)
        
        # --- 3. CREATE AND ADD ARROW ---
        arrow = sg.Arrow(
            length=length, 
            radius=0.005,       
            head_radius=0.0,  
            color=colors[i],
            pose=arrow_pose      
        )
        env.add(arrow)

def UpdateBoxPosition(box, P_current, V, dt):
    """
    Update the position of the box in the environment.

    Parameters:
    - box: The box object (instance of Cuboid class).
    - V: Velocity of the box (numpy array).
    - dt: Time step (float).
    """
    
    P_new = P_current + V * dt
    box.T = sm.SE3(P_new[0], P_new[1], P_new[2])  # Update the pose of the box

def VizRobotWorkSpace(robot:ERobot,num_samples:int=15000):
    #Generate random joint configurations
    q_min = robot.qlim[0, :]
    q_max = robot.qlim[1, :]

    # Manually bound the continuous Joint 1 to one full rotation [-pi, pi]
    if np.isinf(q_min[0]) or q_min[0] < -np.pi: q_min[0] = -np.pi
    if np.isinf(q_max[0]) or q_max[0] > np.pi:  q_max[0] = np.pi

    q_random = np.random.uniform(q_min, q_max, size=(num_samples, robot.n))

    print("Calculating Forward Kinematics...")

    #Compute Forward Kinematics and extract [X, Y, Z]
    positions = robot.fkine(q_random).t 

    print("Generating interactive plot...")

    #Create the Interactive Plotly Figure
    fig = go.Figure(data=[go.Scatter3d(
        x=positions[:, 0],
        y=positions[:, 1],
        z=positions[:, 2],
        mode='markers',
        marker=dict(
            size=2,
            color=positions[:, 2],    # Color points by their Z-height
            colorscale='Viridis',     # A visually pleasing color map
            opacity=0.4               # Slight transparency to see density
        )
    )])

    #Format the layout and camera
    fig.update_layout(
        title="Interactive Robot Reachable Workspace",
        margin=dict(l=0, r=0, b=0, t=40), # Removes unnecessary white space
        scene=dict(
            xaxis_title='X (meters)',
            yaxis_title='Y (meters)',
            zaxis_title='Z (meters)',
            aspectmode='data'  # CRITICAL: Ensures the 3D space isn't artificially stretched
        )
    )

    # Show the plot
    fig.show()

def PlotTrackingError(timestamp: np.ndarray, err_hist: np.ndarray):
    """
    Plot the end-effector position error over time.

    Parameters:
    - timestamp: Time of each step [s] (numpy array).
    - err_hist: Position error of each step [m] (numpy array, N x 3).
    """
    label = ["err_x","err_y","err_z"]
    for i in range(3):
        plt.plot(timestamp,err_hist[:,i],label=label[i])
    plt.xlabel("Time [s]")
    plt.ylabel("Error [m]")
    plt.legend()
    plt.gca().yaxis.set_major_locator(ticker.MultipleLocator(0.01))
    plt.title("Tracking error VS Time")
    plt.show()

def PlotManipulability(timestamp: np.ndarray, M_hist: np.ndarray):
    """
    Plot the manipulability over time.

    Parameters:
    - timestamp: Time of each step [s] (numpy array).
    - M_hist: Manipulability of each step (numpy array).
    """
    plt.plot(timestamp,M_hist)
    plt.xlabel("Time [s]")
    plt.ylabel("Manipulability")
    plt.title("Manipulability VS Time")
    plt.show()

def PlotJointVelocity(timestamp: np.ndarray, q_dot_hist: np.ndarray):
    """
    Plot the joint velocities over time.

    Parameters:
    - timestamp: Time of each step [s] (numpy array).
    - q_dot_hist: Joint velocities of each step (numpy array, N x 3).
    """
    label = ["q1","q2","q3"]
    for i in range(3):
        plt.plot(timestamp,q_dot_hist[:,i],label=label[i])
    plt.xlabel("Time [s]")
    plt.ylabel("Joint velocity [rad/s]")
    plt.legend()
    plt.title("Joint velocity VS Time")
    plt.show()

def SortEigen(eigval_hist: np.ndarray, eigvec_hist: np.ndarray):
    """
    Make the eigenvalues and eigenvectors of each step comparable over time.

    Parameters:
    - eigval_hist: Eigenvalues of each step (numpy array, N x 3).
    - eigvec_hist: Eigenvectors of each step, one per column (numpy array, N x 3 x 3).

    Returns:
    - eigval_sorted: Eigenvalues from largest to smallest (numpy array, N x 3).
    - eigvec_sorted: Eigenvectors in the same order with a consistent sign (numpy array, N x 3 x 3).
    """
    # np.linalg.eig returns no fixed order: sort every step from largest to smallest eigenvalue
    order = np.argsort(eigval_hist, axis=1)[:, ::-1]
    eigval_sorted = np.take_along_axis(eigval_hist, order, axis=1)
    eigvec_sorted = np.take_along_axis(eigvec_hist, order[:, None, :], axis=2)

    # The sign of an eigenvector is arbitrary: keep each one pointing the same way as the previous step
    for k in range(1, len(eigvec_sorted)):
        for i in range(3):
            if eigvec_sorted[k,:,i] @ eigvec_sorted[k-1,:,i] < 0:
                eigvec_sorted[k,:,i] *= -1

    return eigval_sorted, eigvec_sorted

def PlotEigenvalues(timestamp: np.ndarray, eigval_hist: np.ndarray, eigvec_hist: np.ndarray):
    """
    Plot the eigenvalues of J_lin J_lin^T over time.

    Parameters:
    - timestamp: Time of each step [s] (numpy array).
    - eigval_hist: Eigenvalues of each step (numpy array, N x 3).
    - eigvec_hist: Eigenvectors of each step, one per column (numpy array, N x 3 x 3).
    """
    eigval_sorted, _ = SortEigen(eigval_hist, eigvec_hist)

    for i in range(3):
        plt.plot(timestamp,eigval_sorted[:,i],label=f"λ{i+1}")
    plt.yscale("log")   # The smallest eigenvalue is far smaller than the others near the singularity
    plt.xlabel("Time [s]")
    plt.ylabel("Eigenvalue of J_lin J_lin^T")
    plt.legend()
    plt.title("Eigenvalues VS Time")
    plt.show()

def PlotEigenvectors(timestamp: np.ndarray, eigval_hist: np.ndarray, eigvec_hist: np.ndarray):
    """
    Plot the x, y and z components of each eigenvector of J_lin J_lin^T over time.

    Parameters:
    - timestamp: Time of each step [s] (numpy array).
    - eigval_hist: Eigenvalues of each step (numpy array, N x 3).
    - eigvec_hist: Eigenvectors of each step, one per column (numpy array, N x 3 x 3).
    """
    _, eigvec_sorted = SortEigen(eigval_hist, eigvec_hist)

    label = ["x","y","z"]
    fig, axes = plt.subplots(3, 1, sharex=True, sharey=True, figsize=(8, 7))
    for i in range(3):
        for j in range(3):
            axes[i].plot(timestamp,eigvec_sorted[:,j,i],label=label[j])
        axes[i].set_ylabel(f"Eigenvector of λ{i+1}")
        axes[i].legend(loc="upper right")
    axes[2].set_xlabel("Time [s]")
    axes[0].set_title("Eigenvectors VS Time")
    plt.show()