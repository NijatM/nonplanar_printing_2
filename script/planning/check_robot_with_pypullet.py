import os

from compas.data import json_load
from compas_robots import RobotModel
from compas_robots import ToolModel

from compas_fab.backends import PyBulletClient
from compas_fab.backends import PyBulletPlanner
from compas_fab.robots import RobotSemantics
from compas_fab.robots import RobotCell
from compas_fab.robots import FrameTarget
from compas_fab.robots import PointAxisTarget
from compas_fab.robots import TargetMode
from compas_fab.backends import CollisionCheckError

from .utils import get_root_folder_path
from .utils import remove_robot_model_visuals

# Define file paths
robot_model_filename = os.path.join(
    get_root_folder_path(), "robot", "gofa10", "crb15000_10kg_model.json"
)
robot_semantics_filename = os.path.join(
    get_root_folder_path(), "robot", "gofa10", "crb15000_10kg_semantics.json"
)
tool_json_path = os.path.join(
    get_root_folder_path(), "tool", "BioPrint901", "BioPrint901.json"
)

# Remove robot and tool visual meshes for better performance (default: True)
remove_visuals = True


def run():
    # Load Robot Model
    robot_model = json_load(robot_model_filename)  # type: RobotModel
    if remove_visuals:
        remove_robot_model_visuals(robot_model)

    # Load Robot Semantics
    robot_semantics = json_load(robot_semantics_filename)  # type: RobotSemantics

    # Load Robot Tool (Printing Tool)
    tool_model = json_load(tool_json_path)  # type: ToolModel
    if remove_visuals:
        remove_robot_model_visuals(tool_model)

    tool_models = {}
    tool_models[tool_model.name] = tool_model

    # Add collision rigid bodies
    from compas.geometry import Box
    from compas.geometry import Frame
    from compas_fab.robots import RigidBody

    box = Box(0.1, 0.1, 1).to_mesh(triangulated=True)
    box_rb = RigidBody([box], [box])
    rigid_body_models = {}
    rigid_body_models["box"] = box_rb

    # Create RobotCell
    robot_cell = RobotCell(robot_model, robot_semantics, tool_models, rigid_body_models)

    # Create Robot Cell State
    robot_cell_state = robot_cell.default_cell_state()

    # Change location of box
    robot_cell_state.rigid_body_states["box"].frame = Frame(
        [0.5, 0.0, 0.0], [1, 0, 0], [0, 1, 0]
    )

    # Attache tool to robot
    robot_cell_state.tool_states[tool_model.name].attached_to_group = (
        robot_cell.main_group_name
    )
    robot_cell_state.tool_states[tool_model.name].touch_links = ["link_6"]

    # Load PyBullet Client
    with PyBulletClient() as client:
        planner = PyBulletPlanner(client)
        planner.set_robot_cell(robot_cell, robot_cell_state)
        input("Press Enter to continue...")
        count_total = 0
        count_no_collision = 0
        count_ik_success = 0

        for i in range(100):
            # Generate a random Configuration
            robot_cell_state.robot_configuration = robot_cell.random_configuration()
            print("Random Config: {}".format(robot_cell_state.robot_configuration))

            frame = planner.forward_kinematics(robot_cell_state, TargetMode.ROBOT)
            print("   Frame = {}".format(frame))
            count_total += 1

            # Check collision
            try:
                planner.check_collision(robot_cell_state, options={"full_report": True})
                print("   Collision = Free")
                count_no_collision += 1
                collision = False
            except CollisionCheckError as e:
                print("   Collision = Collided")
                collision = True
                print("   Error: {}".format(e.message))

            if not collision:
                target = FrameTarget(frame, TargetMode.ROBOT)
                try:
                    config = planner.inverse_kinematics(
                        target,
                        robot_cell_state,
                        options={"allow_collision": True, "attempts": 100},
                    )
                    print("   IK Result = {}".format(config))
                    count_ik_success += 1
                except:
                    print("   IK Result = Failed")

        print("Total Configurations Generated: {}".format(count_total))
        print("Configurations with no Collision: {}".format(count_no_collision))
        print("IK Success: {}".format(count_ik_success))


import cProfile

cProfile.run("run()", "restats")

import pstats
from pstats import SortKey

p = pstats.Stats("restats")
p.sort_stats(SortKey.CUMULATIVE).print_stats(30)
