"""Keep pytest away from the launch_testing files in this directory.

pytest 8+ collects the whole directory when handed a single file, so `*.launch.py` gets
imported as a module and dies on the dotted name. Those files belong to add_launch_test.
"""

collect_ignore_glob = ["*.launch.py"]
