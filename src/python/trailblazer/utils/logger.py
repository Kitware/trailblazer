# Distributed under the Apache License, Version 2.0.
# See accompanying NOTICE file for details.

import sys
import logging

def initialize(log_name='trailblazer', log_level=logging.INFO):
    logger = logging.getLogger()
    logger.setLevel(log_level)

    # create file handler which logs even debug messages
    fh = logging.FileHandler(log_name+'.log', mode='w')
    fh.setLevel(log_level)

    # create console handler with a higher log level
    ch = logging.StreamHandler(stream=sys.stdout)
    ch.setLevel(log_level)

    # create formatter and add it to the handlers
    formatter = logging.Formatter('[%(asctime)s] %(levelname)8s --- %(message)s ' +
                                  '(%(filename)s:%(lineno)s)', datefmt='%Y-%m-%d %H:%M:%S')
    fh.setFormatter(formatter)
    ch.setFormatter(formatter)

    # add the handlers to the logger
    logger.addHandler(ch)
    logger.addHandler(fh)