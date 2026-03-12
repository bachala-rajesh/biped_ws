

from enum import IntEnum


class RobotState(IntEnum):
    INIT = 10
    PASSIVE = 20
    STANDBY = 30
    ACTIVE = 40
    FALLEN = 50
    ERROR = 60
    STOP = 70