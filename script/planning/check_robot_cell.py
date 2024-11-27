import os

from compas.data import json_load
from compas.geometry import Frame
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

from utils import get_root_folder_path
from utils import remove_robot_model_visuals

# Define file paths
robot_cell_filename = os.path.join(
    get_root_folder_path(), "design", "241121_victor", "gofa_10_cell_120box.rc"
)

# Remove robot and tool visual meshes for better performance (default: True)
remove_visuals = True


def run():
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

    # Create Robot Cell State
    robot_cell_state = robot_cell.default_cell_state()

    # Attache tool to robot
    robot_cell_state.set_tool_attached_to_group(
        tool_name,
        robot_cell.main_group_name,
        attachment_frame=Frame.worldXY(),
        touch_links=["link_6"],
    )

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


run()
# import cProfile

# cProfile.run("run()", "restats")

# import pstats
# from pstats import SortKey

# p = pstats.Stats("restats")
# p.sort_stats(SortKey.CUMULATIVE).print_stats(30)
