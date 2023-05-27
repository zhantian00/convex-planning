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
parser.add_argument('-s', '--solver', default='ECOS_BB', type=str,
                    help='Choose solver.')
parser.add_argument('-a', '--alg', default=1, type=int, choices=[1,2],
                    help='Choose algorithm 1 or 2.')
args = parser.parse_args()
logging.basicConfig(format='%(asctime)s - %(levelname)s - %(name)s => %(message)s', level=args.log_level)
logger = logging.getLogger("logger")
fh = logging.FileHandler('./results/{}.log'.format(cfg.SAVE_NAME))
logger.addHandler(fh)

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
    CONVERENCE = cfg.DELTA_TOLERANCE
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
    linear_inequality_constraint = dynamics_constraint + terminal_constraint

    generalized_inequality_constraint = [cp.SOC(delta[i], cp.hstack([1, vartheta[i]])) for i in range(N)]

    discretized_constraint = []
    for j, obs in enumerate(obstacles):
        x_obs, y_obs, r_obs = obs
        def in_proj_of_obstacle(x): return x <= x_obs + r_obs and x >= x_obs - r_obs 
        discretized_constraint += [y_traj[i] >= y_obs + np.sqrt(r_obs**2 - (x_traj[i] - x_obs)**2) + D * (eta[j] - 1) for i in range(N) if in_proj_of_obstacle(x_traj[i])]
        discretized_constraint += [y_traj[i] <= y_obs - np.sqrt(r_obs**2 - (x_traj[i] - x_obs)**2) + D * eta[j] for i in range(N) if in_proj_of_obstacle(x_traj[i])]
        discretized_constraint += [eta[j] >= 0, eta[j] <= 1]
    
    constraints = linear_inequality_constraint + generalized_inequality_constraint + discretized_constraint

    # solve
    # ------------------
    import time
    converged = False
    runtime_built_in, runtime_solve = 0, 0
    iters = 0
    last_delta = delta.value
    while not converged:
        control_constraint = [cp.abs(u[i]) <= UAV_ANGULAR_RATE_MAX / V * (3 * delta.value[i]**2 * delta[i] - 2 * delta.value[i]**3)  for i in range(N)]
        problem = Problem(objective, constraints + control_constraint)
        end = time.time()
        problem.solve(solver=args.solver)
        runtime_solve += time.time() - end
        runtime_built_in += problem.solver_stats.solve_time
        iters += 1
        logger.info('Iters: {}'.format(iters))
        logger.info('Problem status: {}'.format(problem.status))
        if args.alg == 1 and np.max(np.abs(last_delta - delta.value)) <= CONVERENCE: converged = True
        if args.alg == 2: converged = True
        last_delta = delta.value

    logger.info('--------------------RESULTS--------------------')
    logger.info('Totol iters {}'.format(iters))
    logger.info('Alg time consumption: {}ms (built-in {}ms)'.format(runtime_solve*1000, runtime_built_in*1000))
    logger.info('Optimal Problem objective value: {}'.format(problem.value))
    logger.info('Decisions: {}'.format(np.round(eta.value)))
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
    
    ax.plot(x, y, linewidth=3, zorder=0)
    ax.set_xlim([-5, 105])
    ax.set_ylim([-40, 40])
    plt.xlabel("x(m)")
    plt.ylabel("y(m)")
    plt.savefig('./results/{}_fig.png'.format(cfg.SAVE_NAME))


if __name__ == '__main__':
    main()