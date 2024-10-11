import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/younousse/Documents/Autonomouscar/Programm/autofollow_ws/install/autofollow_lidar_detection'
