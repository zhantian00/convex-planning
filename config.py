from yacs.config import CfgNode

_C = CfgNode()
cfg = _C

_C.UAV_VELOCITY = 5 # m/s
_C.UAV_ANGULAR_RATE_MAX = 20 # deg/s
_C.NUM_POINTS = 100
_C.LARGE_CONSTANT_D = 1000
_C.RELATIVE_TOLERANCE = 0.01

# state: x, y (in meters), yaw (in deg)
_C.UAV_START_STATE = [0.0, 0.0, 0.0]
_C.UAV_FINAL_STATE = [100.0, 0.0, 0.0]

# x, y, radius
_C.OBSTACLES = [[50, 0, 15], [80, -20, 19], [20, -20, 15], [20, 10, 10],  [80, 15, 15]]

_C.SAVE_NAME = 'test'