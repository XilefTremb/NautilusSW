import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/xavier/Documents/GitHub/NautilusSW/UTILES/ros2_ws/install/submarine_sim'
