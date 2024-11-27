from compas.data import Data
import os

from compas_robots.configuration import Configuration
from compas.geometry import Frame

from compas_fab.robots import JointTrajectory
from compas_fab.robots import RobotCellState

try:
    from typing import Optional, List, Tuple
except:
    pass


class Task(Data):
    def __init__(
        self, task_id=None, execution_result=None, execution_end_time=None, tag=None
    ):
        super(Task, self).__init__()
        self.task_id = task_id or ""  # type: str
        self.execution_result = execution_result
        self.execution_end_time = execution_end_time
        self.tag = tag or "Generic Task"  # type: str

    @property
    def __data__(self):
        data = {
            "task_id": self.task_id,
            "execution_result": self.execution_result,
            "execution_end_time": self.execution_end_time,
            "tag": self.tag,
        }
        return data

    def get_filepath(self, subdir="tasks"):
        # type: (str) -> str
        """Returns the location of the json file when saved externally.
        This is useful to save a Movement Task containing computed trajectory and has a large file size
        e.g.: 'tasks\A2_M2.json'
        """
        return os.path.join(subdir, "%s.json" % self.task_id)

    @property
    def short_summary(self):
        return "{}(#{}, {})".format(self.__class__.__name__, self.task_id, self.tag)


class RoboticMovement(Task):
    """Generic class for movements related to Robot Arm movement.

    `RoboticMovement.allowed_collision_matrix`
    - List of Tuple[object_id, object_id] representing allowable collision between beams and tools

    `RoboticMovement.target_configuration`
    - Optional Robotic Configuration (J / E values) for the target,
    When set, the configuration will become a constraint in the end-state for the path planning.
    When None, the pathplanner should decide for the configuration using IK sampling.
    """

    def __init__(
        self,
        target=None,  # type: Frame # Target of the Robotic Movement
        speed_type="",  # type: str # A string linking to a setting
        trajectory=None,  # type: JointTrajectory
        seed=None,  # type: int
        task_id=None,  # type: str
        execution_result=None,
        execution_end_time=None,
        tag=None,  # type: str
    ):
        super(RoboticMovement, self).__init__(
            task_id=task_id,
            execution_result=execution_result,
            execution_end_time=execution_end_time,
            tag=tag or "Generic Robotic Movement",
        )
        self.target = target  # type: Frame
        self.speed_type = speed_type  # type: str # A string linking to a setting
        self.trajectory = trajectory  # type: JointTrajectory
        self.seed = seed  # or hash(time.time())

    @property
    def __data__(self):
        """Sub class specific data added to the dictionary of the parent class"""
        data = {}
        data["target"] = self.target
        data["speed_type"] = self.speed_type
        data["trajectory"] = self.trajectory
        data["seed"] = self.seed
        data["task_id"] = self.task_id
        data["execution_result"] = self.execution_result
        data["tag"] = self.tag
        return data

    @property
    def short_summary(self):
        return "{}(#{}, {}, has_traj {})".format(
            self.__class__.__name__, self.task_id, self.tag, self.trajectory is not None
        )

    def __str__(self):
        return "Move to %s" % (self.target)


class RoboticFreeMovement(RoboticMovement):

    def __str__(self):
        str = super(RoboticFreeMovement, self).__str__()
        return "Free " + str


class RoboticLinearMovement(RoboticMovement):

    def __str__(self):
        str = super(RoboticLinearMovement, self).__str__()
        return "Linear " + str


class DigitalOutput(Task):
    def __init__(self, tag=None):
        super(DigitalOutput, self).__init__()
        self.tag = tag or "Digital Output"

    @property
    def __data__(self):
        data = {
            "tag": self.tag,
        }
        return data

    def __str__(self):
        return "DigitalOutput " + self.__class__.__name__


class ExtruderOn(DigitalOutput):
    def __init__(self, tag=None):
        super(ExtruderOn, self).__init__()
        self.tag = tag or "Extruder On"


class ExtruderOff(DigitalOutput):
    def __init__(self, tag=None):
        super(ExtruderOff, self).__init__()
        self.tag = tag or "Extruder Off"


class AirAssistOn(DigitalOutput):
    def __init__(self, tag=None):
        super(AirAssistOn, self).__init__()
        self.tag = tag or "Air Assist On"


class AirAssistOff(DigitalOutput):
    def __init__(self, tag=None):
        super(AirAssistOff, self).__init__()
        self.tag = tag or "Air Assist Off"


class PrintingProcess(Data):
    def __init__(self, start_state, tasks):
        super(PrintingProcess, self).__init__()
        self.start_state = start_state  # type: RobotCellState
        self.tasks = tasks  # type: List[Task]

    @property
    def __data__(self):
        data = {
            "start_state": self.start_state,
            "tasks": self.tasks,
        }
        return data

    def get_robotic_movements(self):
        # type: () -> List[RoboticMovement]
        """Returns all the Robotic Movements in the planning problem"""
        return [t for t in self.tasks if isinstance(t, RoboticMovement)]

    def renumber_task_ids(self):
        # type: () -> None
        """Renumber the movement ids in the planning problem"""
        for i, t in enumerate(self.tasks):
            t.task_id = "t%i" % i

    def get_linear_movement_groups(self):
        # type: () -> List[List[RoboticLinearMovement]]
        """Returns a list of list of RoboticLinearMovement"""
        groups = []
        for t in self.get_robotic_movements():
            if isinstance(t, RoboticLinearMovement):
                if len(groups) == 0:
                    groups.append([])
                groups[-1].append(t)
            else:
                if len(groups) > 0:
                    if len(groups[-1]) > 0:
                        groups.append([])
        # Remove empty groups
        if len(groups) > 0:
            if groups[-1] == []:
                groups.pop()
        return groups
