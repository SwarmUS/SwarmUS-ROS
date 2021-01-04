################################################################################
# Executes clang-format, only to check.
# 
# This script should be run from the root of the catkin workspace
################################################################################

import os
import sys
from config import CLANG_FORMAT_CMD, SWARMUS_SRC_DIR

os.system("python {} --recursive --style=file {}".format(CLANG_FORMAT_CMD, SWARMUS_SRC_DIR))