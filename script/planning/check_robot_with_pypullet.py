import os

from compas.data import json_load
from compas_robots import RobotModel
from compas_robots import ToolModel

from compas_fab.backends import PyBulletClient
from compas_fab.backends import PyBulletPlanner
from compas_fab.robots import RobotCell


current_file_location = os.path.abspath(__file__)
current_folder_path = os.path.dirname(current_file_location)
root_folder_path = os.path.dirname(os.path.dirname(current_folder_path))

# Load Robot Model
robot_model_filename = os.path.join(root_folder_path, "robot", "gofa10", "gofa10.json")
robot_model = json_load(robot_model_filename)  # type: RobotModel

# Load Robot Tool (Printing Tool)
tool_json_path = os.path.join(
    root_folder_path, "tool", "BioPrint901", "BioPrint901.json"
)
tool_model = json_load(tool_json_path)  # type: ToolModel

# Create RobotCell
robot_cell = RobotCell(robot_model, tool_model)
# Load Pybullet Client
with PyBulletClient("gui") as client:
    planner = PyBulletPlanner(client)
    planner.set_robot_cell(robot_cell)

client, robot, robot_uid = load_pybullet_with_robot(
    urdf_filename, srdf_filename, viewer=True
)


# tool = Tool.from_tool_model(tool_model)
# tool.link_name = 'tool0'
# touch_links = ['tool0', 'flange']
# robot.attach_tool(tool,touch_links=touch_links)

# # Add Tool to Pybullet Client
# urdf_package_path = os.path.join('tool', 'BioPrint901')
# add_tool_to_client(client, robot, tool, urdf_package_path, touch_links=touch_links)

# # Load some Collision Meshes
# collision_meshes_path = os.path.join('test', 'design', 'CollisionMesh1_PrintBed.json')
# collision_meshes = json_load(collision_meshes_path)
# for i, mesh in enumerate(collision_meshes):
#     cm = CollisionMesh(mesh, 'static_cm_%i' % i)
#     client.add_collision_mesh(cm, {})

while True:
    configuration = robot.random_configuration()
    print(configuration)

    frame = client.forward_kinematics(robot, configuration, options={})
    print(frame)

    collision = client.check_collisions(robot, configuration)
    print(collision)

    config = client.inverse_kinematics(
        robot, frame, options={"avoid_collisions": False, "attempts": 100}
    )
    print(config)
    if config:
        break

wait_if_gui()
pass
