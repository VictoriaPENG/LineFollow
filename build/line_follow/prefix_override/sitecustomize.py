import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/yunbo/rdk_backup/dev_ws/src/originbot/Line_follow/install/line_follow'
