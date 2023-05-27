from yacs.config import CfgNode

_C = CfgNode()
cfg = _C

_C.UAV_VELOCITY = 5 # m/s
_C.UAV_ANGULAR_RATE_MAX = 20 # deg/s
_C.NUM_POINTS = 100
_C.LARGE_CONSTANT_D = 1000
_C.DELTA_TOLERANCE = 0.01

# state: x, y (in meters), yaw (in deg)
_C.UAV_START_STATE = [0, 0, 0]
_C.UAV_FINAL_STATE = [100, 0, 0]

# case1 [0, 0, 0], [100, 0, 0]
# case2 [0, 0, 60], [100, 0, 60]
# case3 [0, 0, 60], [100, 0, -60]

# x, y, radius
_C.OBSTACLES = [[50, 0, 10]]

# case0 [[1000, 0, 0]]
# case1 [[50, 0, 10]]
# case2 [[33, 10, 15], [66, -10, 15]]
# case3 [[50, 0, 17], [80, -20, 17], [20, -20, 15], [20, 10, 10],  [80, 15, 15]]

_C.SAVE_NAME = 'test'