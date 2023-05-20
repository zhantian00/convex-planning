import numpy as np
import cvxpy as cp
from cvxpy import Variable, Problem, Minimize, transforms
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
    solve_problem(cfg)
    return


def solve_problem(cfg):
    # read params
    N = cfg.NUM_POINTS
    obstacles = cfg.OBSTACLES
    M = len(obstacles)
    D = cfg.LARGE_CONSTANT_D
    V = cfg.UAV_VELOCITY
    UAV_ANGULAR_RATE_MAX = np.deg2rad(cfg.UAV_ANGULAR_RATE_MAX)
    x_0, y_0, yaw_0 = cfg.UAV_START_STATE
    x_f, y_f, yaw_f = cfg.UAV_FINAL_STATE
    x_traj = np.arange(x_0, x_f, (x_f - x_0)/N)
    dx = abs(x_f - x_0) / N
    C = dx / V
    if args.alg == 2:
        CONVERENCE = 10000000
    else:
        CONVERENCE = cfg.RELATIVE_TOLERANCE * C * N
    # define optimization variables
    # -----------------------------
    y_traj = Variable(N)
    vartheta = Variable(N)
    u = Variable(N)
    delta = Variable(N)
    if args.alg == 2:
        delta.value = np.array([1.0] * N)
    else:
        delta.value = np.array([1.1] * N)
    eta = Variable(M, integer=True)

    # define objective
    # ----------------
    objective = Minimize(cp.sum(C * delta))

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

    control_constraint = [cp.abs(u[i]) <= UAV_ANGULAR_RATE_MAX / V * (3 * delta.value[i]**2 * delta[i] - 2 * delta.value[2]**3)  for i in range(N)]

    linear_inequality_constraint = dynamics_constraint + terminal_constraint + control_constraint

    generalized_inequality_constraint = [delta[i] >= cp.norm(1 + vartheta[i]) for i in range(N)]

    discretized_constraint = []
    for j, obs in enumerate(obstacles):
        x_obs, y_obs, r_obs = obs
        def in_proj_of_obstacle(x): return x <= x_obs + r_obs and x >= x_obs - r_obs 
        discretized_constraint += [y_traj[i] >= y_obs + np.sqrt(r_obs**2 - (x_traj[i] - x_obs)**2) + D * (eta[j] - 1) for i in range(N) if in_proj_of_obstacle(x_traj[i])]
        discretized_constraint += [y_traj[i] <= y_obs - np.sqrt(r_obs**2 - (x_traj[i] - x_obs)**2) + D * eta[j] for i in range(N) if in_proj_of_obstacle(x_traj[i])]
        discretized_constraint += [eta[j] >= 0, eta[j] <= 1]

    
    constraints = linear_inequality_constraint + generalized_inequality_constraint + discretized_constraint
    problem = Problem(objective, constraints)

    # solve
    # ------------------
    verbose = True if args.log_level=='DEBUG' else False
    problem.solve(solver='ECOS_BB', max_iters=100, reltol=CONVERENCE, verbose=verbose)
    runtime = problem.solver_stats.solve_time
    logger.info('Iters: {}'.format(problem.solver_stats.num_iters))
    logger.info('Problem status: {}'.format(problem.status))
    logger.info('Alg time consumption: {}s'.format(runtime))
    logger.info('Optimal Problem objective value: {}'.format(problem.value))
    logger.info('Decisions: {}'.format(eta.value))
    draw_traj(y_traj.value, vartheta.value, obstacles)


    logger.debug([np.abs(u.value[i]) <= UAV_ANGULAR_RATE_MAX / V * delta.value[i]**3 for i in range(N)])
    return


def draw_traj(y, theta, obstacles):
    N = cfg.NUM_POINTS
    x_0, y_0, yaw_0 = cfg.UAV_START_STATE
    x_f, y_f, yaw_f = cfg.UAV_FINAL_STATE
    x = np.arange(x_0, x_f, (x_f - x_0)/N)
    fig, ax = plt.subplots()
    for i, obs in enumerate(obstacles):
        x_obs, y_obs, r_obs = obs
        circle = plt.Circle((x_obs, y_obs), r_obs, fill=False)
        ax.add_patch(circle)
        ax.text(x_obs, y_obs, "{}".format(i))

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
    
    plt.plot(x, y, linewidth=3,zorder=0)
    ax.set_xlim([0, 100])
    ax.set_ylim([-40, 40])
    plt.axis('equal')
    plt.xlabel("x(m)")
    plt.ylabel("y(m)")
    plt.savefig('./results/{}_fig.png'.format(cfg.SAVE_NAME))


if __name__ == '__main__':
    main()