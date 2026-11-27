import sys
if sys.prefix == '/home/roshan/Robot-RPi-Core/.venv':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/roshan/Robot-RPi-Core/install/robot_voice'
