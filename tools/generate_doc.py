import sys
import os

ROSDOC_LITE_CMD = "rosdoc_lite"

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
    # print(cmd)
    os.system(cmd)

if __name__ == "__main__":
    output_path = sys.argv[1]

    root_dir = 'src/SwarmUS-ROS/src' # TODO with env variable?

    packages = get_package_list(root_dir)

    for package in packages:
        generate_doc_by_pkg(package, output_path)