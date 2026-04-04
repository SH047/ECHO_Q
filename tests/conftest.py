import sys, os
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for pkg in ('echo_q_control','echo_q_utilities'):
    sys.path.insert(0, os.path.join(ROOT,'src',pkg,'src'))
