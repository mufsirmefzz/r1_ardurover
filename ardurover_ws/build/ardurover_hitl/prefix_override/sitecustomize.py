import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/mufsir/Mufsir/IISC/ardurover_ws/install/ardurover_hitl'
