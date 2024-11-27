import os

from compas.data import json_load
from compas.data import json_dump
from compas.geometry import Frame
from compas_robots import RobotModel
from compas_robots import ToolModel

from compas_fab.backends import PyBulletClient
from compas_fab.backends import PyBulletPlanner
from compas_fab.robots import PointAxisWaypoints
from compas_fab.robots import RobotCell
from compas_fab.robots import FrameTarget
from compas_fab.robots import PointAxisTarget
from compas_fab.robots import TargetMode
from compas_fab.backends import MPStartStateInCollisionError
from compas_fab.backends import MPTargetInCollisionError
from compas_fab.backends import CollisionCheckError
from compas_fab.backends import MPNoIKSolutionError
from compas_fab.backends import InverseKinematicsError

from npp2.tasks import PrintingProcess

from utils import get_root_folder_path
from utils import remove_robot_model_visuals

# Define file paths
robot_cell_filename = os.path.join(
    get_root_folder_path(), "design", "241121_victor", "gofa_10_cell_120box.rc"
)
process_filename = os.path.join(
    get_root_folder_path(), "design", "241121_victor", "path_0.process"
)
result_filename = os.path.join(
    get_root_folder_path(), "design", "241121_victor", "path_0.result"
)
# Remove robot and tool visual meshes for better performance (default: True)
remove_visuals = True


def run():
    # Load problem
    process = json_load(process_filename)  # type: PrintingProcess

    # --------------------------------------------------------------

    # Load Robot Cell
    robot_cell = json_load(robot_cell_filename)  # type: RobotCell

    robot_model = robot_cell.robot_model  # type: RobotModel
    if remove_visuals:
        remove_robot_model_visuals(robot_model)

    # Tool Model (Assuming there is only one tool)
    tool_name = robot_cell.tool_ids[0]
    tool_model = robot_cell.tool_models[tool_name]  # type: ToolModel
    if remove_visuals:
        remove_robot_model_visuals(tool_model)

    # --------------------------------------------------------------

    # Create Robot Cell State
    robot_cell_state = process.start_state

    # Load PyBullet Client
    with PyBulletClient() as client:
        planner = PyBulletPlanner(client)
        planner.set_robot_cell(robot_cell, robot_cell_state)
        # input("Press Enter to continue...")

        start_state = process.start_state

        # Check if all the movement has IK solutions.
        movements = process.get_robotic_movements()
        # for i, movement in enumerate(movements):
        #     target = movement.target  # type: PointAxisWaypoints
        #     last_point, last_z = target.target_points_and_axes[-1]
        #     target = PointAxisTarget(last_point, last_z, target.target_mode)
        #     try:
        #         config = planner.inverse_kinematics(
        #             target, start_state, options={"check_collision": True}
        #         )
        #         start_state.robot_configuration.values = config.values
        #         print(
        #             "IK Check OK Movement {}, {} of {}".format(
        #                 i, movement.tag, len(movements)
        #             )
        #         )
        #     except InverseKinematicsError as e:
        #         print("IK Error at movement {}: {}".format(i, e.message))
        #         # Visualize the target without CC
        #         config = planner.inverse_kinematics(
        #             target, start_state, options={"check_collision": False}
        #         )
        #         try:
        #             planner.check_collision(start_state)
        #         except Exception as e:
        #             print(e.message)
        #             input("Press Enter to continue...")

        # Plan Movements
        for i, movement in enumerate(movements):
            # Plan Movement
            print("Planning Movement {}: {}".format(i, movement.tag))
            try:
                trajectory = planner.plan_cartesian_motion(movement.target, start_state)
                print("   Plan = Success")
            except MPStartStateInCollisionError as e:
                print("   Plan = Failed (Start State in Collision)")
                print("   Error: {}".format(e.message))
            except MPTargetInCollisionError as e:
                print("   Plan = Failed (Target in Collision)")
                print("   Error: {}".format(e.message))
            except MPNoIKSolutionError as e:
                print("   Plan = Failed (No IK Solution)")
                print("   Error: {}".format(e.message))
                partial_trajectory = e.partial_trajectory  # type: JointTrajectory
                # Visualize last point
                if partial_trajectory.points:
                    last_config = partial_trajectory.points[-1]
                    start_state.robot_configuration.joint_values = (
                        last_config.joint_values
                    )
                    planner.set_robot_cell_state(start_state)
                    input(
                        "IK Failed Showing last successful Point. Press Enter to continue..."
                    )
                else:
                    input(
                        "IK Failed at the beginning of the movement. Press Enter to continue..."
                    )
                movement.trajectory = partial_trajectory
                break  # Stop planning process
            movement.trajectory = trajectory
            print("   Trajectory Points: {}".format(len(trajectory.points)))
            # Update start state
            start_state.robot_configuration.joint_values = trajectory.points[
                -1
            ].joint_values

    # Save the process
    json_dump(process, result_filename)


run()
