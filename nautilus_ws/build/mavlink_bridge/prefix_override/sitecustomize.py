import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/devs/NautilusSW/nautilus_ws/install/mavlink_bridge'
