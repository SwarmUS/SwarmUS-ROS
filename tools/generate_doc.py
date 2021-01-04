################################################################################
# Generates Doxygen documentation 
# 
# This generates the documentation in an output folder specified with an argument 
# (in this case, the "doc" folder) :
# `path/to/generate_doc.py doc`
#
# The script will generate one folder for each ROS package found. Every ROS package
# will then have its own standalone doxygen documentation.
# 
# This script should be run from the root of the catkin workspace. The path
# of the output folder will then be relative to the catkin workspace's root.
################################################################################

import sys
import os
from config import ROSDOC_LITE_CMD, SWARMUS_SRC_DIR

def walklevel(some_dir, level=1):
    some_dir = some_dir.rstrip(os.path.sep)
    assert os.path.isdir(some_dir)
    num_sep = some_dir.count(os.path.sep)
    for root, dirs, files in os.walk(some_dir):
        yield root, dirs, files
        num_sep_this = root.count(os.path.sep)
        if num_sep + level <= num_sep_this:
            del dirs[:]

def get_package_list(root_dir):
    packages = []
    for root, dirs, files in walklevel(root_dir, 0):
        for name in dirs:
            packages.append(os.path.join(root, name))

    return packages

def generate_doc_by_pkg(pkg_dir, output_path):
    pkg_name = pkg_dir[pkg_dir.rfind("/") + 1:]
    output_path = output_path + "/" + pkg_name
    print("Generating documentation for {} in {}".format(pkg_name, output_path))
    cmd = ROSDOC_LITE_CMD + " -o " + output_path + " " + pkg_dir
    os.system(cmd)

if __name__ == "__main__":
    output_path = sys.argv[1]

    packages = get_package_list(SWARMUS_SRC_DIR)

    for package in packages:
        generate_doc_by_pkg(package, output_path)