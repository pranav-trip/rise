import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ptrip/ros2_ws/src/cam_sim/install/cam_sim'
