import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/palopezw/TitoSync/ros2_ws/src/install/control_pkg'
