from yacs.config import CfgNode

_C = CfgNode()
cfg = _C

_C.UAV_VELOCITY = 5 # m/s
_C.OMEGA_MAX = 20 # deg/s
_C.NUM_POINTS = 100
_C.LARGE_CONSTANT_D = 1000
_C.EPSILON_DELTA = 0.01
