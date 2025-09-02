import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/stefano/dev/Costanza/ros2_ws/install/shepherding_control'
