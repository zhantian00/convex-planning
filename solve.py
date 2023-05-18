import numpy as np
import scipy
import ecos
import cvxpy as cp
from cvxpy import Variable, Problem, Minimize
import matplotlib.pyplot as plt
import logging
import time
import argparse
from config import cfg

parser = argparse.ArgumentParser()
parser.add_argument('-l', '--log-level', default='INFO', type=str,
                    choices=['DEBUG', 'INFO', 'WARNING'],
                    help='Configure the logging level.')
parser.add_argument('-a', default=1, type=int, choices=[0,1,2],
                    help='Choose algorithm 1 or 2 (or 0 for test).')
args = parser.parse_args()
logging.basicConfig(format='%(asctime)s - %(levelname)s - %(name)s => %(message)s', level=args.log_level)
logger = logging.getLogger("logger")

def main():
    logger.info(args)
    logger.info('config info: \n{}'.format(cfg))
    if args.a == 0:
        solve_problem_test(cfg)
    elif args.a == 1:
        solve_problem_alg1(cfg)
    else:
        ValueError('Algorithm not defined')

    return

def solve_problem_test(cfg=None):
    x1 = Variable(2)
    x2 = Variable(2)
    y = Variable(1, integer=True)
    z = cp.hstack([x1, x2])

    constraints = [
        x1[0] + x1[1] >= 10,
        x2[0] + x2[1] >= 10,
    ]
    objective = Minimize(cp.sum_squares(z) + cp.sum_squares(y))
    problem = Problem(objective, constraints)

    end = time.time()
    problem.solve(solver=cp.ECOS_BB)
    logger.info('Time consumption: {}s'.format(time.time() - end))
    logger.info('Optimal Problem objective value: {}'.format(problem.value))
    logger.info('x1 {}, x2 {}, y {}'.format(x1.value, x2.value, y.value))
    logger.info(problem.status)
    return

def solve_problem_alg1(cfg):
    # read params
    N = cfg.NUM_POINTS
    M = cfg.OBSTACLES
    D = cfg.LARGE_CONSTANT_D
    V = cfg.UAV_VELOCITY
    CONVERENCE = cfg.EPSILON_DELTA
    UAV_ANGULAR_RATE_MAX = np.deg2rad(cfg.UAV_ANGULAR_RATE_MAX)
    x_0, y_0, yaw_0 = cfg.UAV_START_POSITION
    x_f, y_f, yaw_f = cfg.UAV_FINAL_POSITION
    

    # define optimization variables
    y = Variable(N)
    vartheta = Variable(N)
    u = Variable(N)
    delta = Variable(N)
    z = cp.hstack([y, vartheta, u, delta]).T
    eta = Variable(M)

    # define objective
    c_transpose = np.array([0] * N * 3 + [abs(x_f - x_0) / V] * N)
    objective = Minimize(cp.sum(c_transpose @ z))

    # define constraints
    dx = abs(x_f - x_0) / N
    dynamics_constraint = \
        [cp.abs((y[i+1] - y[i]) / dx - vartheta[i]) <= cfg.DYNAMICS_TOLERANCE for i in range(N-1)] \
        + [cp.abs((vartheta[i+1] - vartheta[i]) / dx - u[i]) <= cfg.DYNAMICS_TOLERANCE for i in range(N-1)]
    terminal_constraint = [
        y[0] == y_0,
        y[N-1] == y_f,
        vartheta[0] == yaw2vartheta(yaw_0),
        vartheta[N-1] == yaw2vartheta(yaw_f)
    ]
    linear_inequality_constraint = dynamics_constraint + terminal_constraint

    genralized_inequality_constraint = [delta[i] >= cp.sqrt(1 + vartheta[i]**2) for i in range(N)]
    discretized_constraint = []
    
    constraints = linear_inequality_constraint + genralized_inequality_constraint + discretized_constraint

    last_delta = np.array([UAV_ANGULAR_RATE_MAX / V] * N)
    converged = False
    end = time.time()
    while not converged:
        # control_constraint = [
        #     cp.abs(u[i]) <= UAV_ANGULAR_RATE_MAX / V * (3 * last_delta[i]**2 * delta[i] - 2 * last_delta[i]**3) for i in range(N)
        # ]
        problem = Problem(objective, constraints)# + control_constraint)
        problem.solve(solver=cp.ECOS_BB, max_iters=1) 
        converged = np.max(np.abs(delta.value - last_delta)) < CONVERENCE
        last_delta = delta.value
        print(delta.value)


    logger.info('Time consumption: {}s'.format(time.time() - end))
    logger.info('Optimal Problem objective value: {}'.format(problem.value))
    logger.info(problem.status)

    return

def yaw2vartheta(yaw):
    return np.tan(yaw)

if __name__ == '__main__':
    main()