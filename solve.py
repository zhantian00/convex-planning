import numpy as np
import scipy
import ecos
import cvxpy as cp
from cvxpy import Variable, Problem, Minimize
import matplotlib.pyplot as plt
import logging
import argparse
from config import cfg

parser = argparse.ArgumentParser()
parser.add_argument('--log-level', default='INFO', type=str,
                    choices= ['DEBUG', 'INFO', 'WARNING'],
                    help='Configure the logging level.')

def main():
    args = parser.parse_args()
    logging.basicConfig(format='%(asctime)s - %(levelname)s - %(name)s => %(message)s', level=args.log_level)
    logger = logging.getLogger("logger")
    logger.info(args)
    logger.info('config info: \n{}'.format(cfg))

if __name__ == '__main__':
    main()