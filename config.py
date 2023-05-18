from yacs.config import CfgNode

_C = CfgNode()
cfg = _C

_C.UAV_VELOCITY = 5 # m/s
_C.UAV_ANGULAR_RATE_MAX = 20 # deg/s
_C.NUM_POINTS = 100
_C.LARGE_CONSTANT_D = 1000
_C.EPSILON_DELTA = 0.01
_C.DYNAMICS_TOLERANCE = 0.0001

_C.OBSTACLES = 1
# state: x, y, yaw
_C.UAV_START_STATE = [0, 0, 0]
_C.UAV_FINAL_STATE = [100, 0, 0]
