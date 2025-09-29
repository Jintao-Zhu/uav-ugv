import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/suda/drone_ugv_ws/src/install/yahboomcar_self_nav'
