import numpy as np
import cvxpy as cp
from cvxpy import Variable, Problem, Minimize, Parameter
import matplotlib.pyplot as plt
import logging
import argparse
from config import cfg

parser = argparse.ArgumentParser()
parser.add_argument('-l', '--log-level', default='INFO', type=str,
                    choices=['DEBUG', 'INFO'],
                    help='Configure the logging level.')
parser.add_argument('-a', '--alg', default=1, type=int, choices=[0,1,2],
                    help='Choose algorithm 1 or 2 (or 0 for test).')
args = parser.parse_args()
logging.basicConfig(format='%(asctime)s - %(levelname)s - %(name)s => %(message)s', level=args.log_level)
logger = logging.getLogger("logger")


def main():
    logger.info(args)
    logger.info('config info: \n{}'.format(cfg))
    if args.alg == 0:
        solve_problem_test(cfg)
    elif args.alg == 1:
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

    problem.solve(solver=cp.ECOS_BB)
    logger.info('Optimal Problem objective value: {}'.format(problem.value))
    logger.info('x1 {}, x2 {}, y {}'.format(x1.value, x2.value, y.value))
    logger.info(problem.status)
    return


def solve_problem_alg1(cfg):
    # read params
    N = cfg.NUM_POINTS
    obstacles = cfg.OBSTACLES
    M = len(obstacles)
    D = cfg.LARGE_CONSTANT_D
    V = cfg.UAV_VELOCITY
    CONVERENCE = cfg.EPSILON_DELTA
    UAV_ANGULAR_RATE_MAX = np.deg2rad(cfg.UAV_ANGULAR_RATE_MAX)
    x_0, y_0, yaw_0 = cfg.UAV_START_STATE
    x_f, y_f, yaw_f = cfg.UAV_FINAL_STATE
    x_traj = np.arange(x_0, x_f, (x_f - x_0)/N)
    # define optimization variables
    # -----------------------------
    y_traj = Variable(N)
    vartheta = Variable(N)
    u = Variable(N)
    delta = Variable(N)
    delta.value = np.array([1.1] * N)
    eta = Variable(M, integer=True)

    last_delta_squred = Parameter(N)
    last_delta_cubic = Parameter(N)
    last_delta_squred.value = np.power(delta.value, 2)
    last_delta_cubic.value = np.power(delta.value, 3)

    dx = abs(x_f - x_0) / N
    # define objective
    # ----------------
    c_transpose = np.array([dx / V] * N)
    objective = Minimize(cp.sum(c_transpose @ delta))

    # define constraints
    # ------------------
    dynamics_constraint = \
        [(y_traj[i+1] - y_traj[i]) / dx == vartheta[i] for i in range(N-1)] \
        + [(vartheta[i+1] - vartheta[i]) / dx == u[i] for i in range(N-1)]
    def yaw2vartheta(yaw): return np.tan(np.deg2rad(yaw))
    terminal_constraint = [
        y_traj[0] == y_0,
        y_traj[N-1] == y_f,
        vartheta[0] == yaw2vartheta(yaw_0),
        vartheta[N-1] == yaw2vartheta(yaw_f)
    ]

    control_constraint = [
        cp.abs(u[i]) <= UAV_ANGULAR_RATE_MAX / V * (3 * last_delta_squred[i] * delta[i] - 2 * last_delta_cubic[i]) for i in range(N)
    ]

    linear_inequality_constraint = dynamics_constraint + terminal_constraint + control_constraint

    generalized_inequality_constraint = [delta[i] >= cp.norm(1 + vartheta[i]) for i in range(N)]

    discretized_constraint = []
    # for j, obs in enumerate(obstacles):
    #     x_obs, y_obs, r_obs = obs
    #     def in_proj_of_obstacle(x): return x <= x_obs + r_obs and x >= x_obs - r_obs 
    #     discretized_constraint += [y_traj[i] >= y_obs + np.sqrt(r_obs**2 - (x_traj[i] - x_obs)**2) + D * (eta[j] - 1) for i in range(N) if in_proj_of_obstacle(x_traj[i])]
    #     discretized_constraint += [y_traj[i] <= y_obs - np.sqrt(r_obs**2 - (x_traj[i] - x_obs)**2) + D * eta[j] for i in range(N) if in_proj_of_obstacle(x_traj[i])]
    #     discretized_constraint += [eta[j] >= 0, eta[j] <= 1]

    #     print([x_traj[i] for i in range(N) if in_proj_of_obstacle(x_traj[i])])
    #     print([y_obs + np.sqrt(r_obs**2 - (x_traj[i] - x_obs)**2) for i in range(N) if in_proj_of_obstacle(x_traj[i])])
    #     print([y_obs - np.sqrt(r_obs**2 - (x_traj[i] - x_obs)**2) for i in range(N) if in_proj_of_obstacle(x_traj[i])])
    
    constraints = linear_inequality_constraint + generalized_inequality_constraint + discretized_constraint
    problem = Problem(objective, constraints)

    # solve by iteration
    # ------------------
    converged = False
    ITER = 0
    total_time = 0
    last_delta = delta.value
    while not converged:
        problem.solve(solver=cp.ECOS_BB, mi_max_iters=1, verbose=True if args.log_level=='DEBUG' else False)
        total_time += problem.solver_stats.solve_time
        converged = np.max(np.abs(delta.value - last_delta)) < CONVERENCE
        last_delta = delta.value
        last_delta_squred.value = np.power(delta.value, 2)
        last_delta_cubic.value = np.power(delta.value, 3)
        ITER += 1
        logger.info('Iters: {}'.format(ITER))
        logger.info('Problem status: {}'.format(problem.status))

    logger.info('Alg time consumption: {}s'.format(total_time))
    logger.info('Optimal Problem objective value: {}'.format(problem.value))
    draw_traj(y_traj.value, vartheta.value, obstacles)
    return


def draw_traj(y, theta, obstacles):
    N = cfg.NUM_POINTS
    x_0, y_0, yaw_0 = cfg.UAV_START_STATE
    x_f, y_f, yaw_f = cfg.UAV_FINAL_STATE
    x = np.arange(x_0, x_f, (x_f - x_0)/N)
    fig, ax = plt.subplots()
    for obs in obstacles:
        x_obs, y_obs, r_obs = obs
        circle = plt.Circle((x_obs, y_obs), r_obs, fill=False)
        ax.add_patch(circle)

    theta = np.arctan(theta)
    for i in range(N):
        if i % 10 == 0 or i == N-1:
            _x = x[i]
            _y = y[i]
            heading = theta[i]
            arrow_length = 1
            dx = arrow_length * np.cos(heading)
            dy = arrow_length * np.sin(heading)
            ax.arrow(_x, _y, dx, dy,head_width=1.5, head_length=1.5, fc='red', ec='red')
    
    plt.plot(x, y, zorder=0)
    ax.set_xlim([0, 100])
    ax.set_ylim([-40, 40])
    plt.axis('equal')
    plt.xlabel("x(m)")
    plt.ylabel("y(m)")
    plt.savefig('./results/{}_fig.png'.format(cfg.SAVE_NAME))


if __name__ == '__main__':
    main()