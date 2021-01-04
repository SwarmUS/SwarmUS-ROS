################################################################################
# Executes clang-format and edits the files which don't comply to the coding 
# standards. 
# 
# Run this before pushing to make sure the build passes.
# 
# This script should be run from the root of the catkin workspace
################################################################################

import os
import sys
from config import CLANG_FORMAT_CMD, SWARMUS_SRC_DIR

os.system("python {} --recursive --in-place --style=file {}".format(CLANG_FORMAT_CMD, SWARMUS_SRC_DIR))