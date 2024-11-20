import math

import compas_rrc as rrc
from compas.data import json_dump, json_load

configurations = json_load('config_test.json')


def to_degrees(radians):
    return [math.degrees(r) for r in radians]


ros = rrc.RosClient()
ros.run()
abb = rrc.AbbClient(ros, '/rob1')

results = []
for config in configurations:
    joint_values = to_degrees(config.joint_values)
    # send robot to position
    abb.send_and_wait(rrc.MoveToJoints(joint_values, [], 200, zone=rrc.Zone.Z0, feedback_level=1))
    abb.send_and_wait(rrc.WaitTime(0.2))
    # Read frame back
    frame = abb.send_and_wait(rrc.GetFrame())
    results.append((config, frame))

json_dump(results, 'results.json')