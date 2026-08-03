import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/icad/RYUGU-ROV-CompanionComputer/install/ryugu_control'
