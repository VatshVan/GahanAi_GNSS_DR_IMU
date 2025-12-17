import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/sedrica/Downloads/gahanai-av-sw-stack-feature-new-vehicle-iitb/install/state_estimator'
