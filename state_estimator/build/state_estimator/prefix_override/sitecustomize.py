import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/vatshvan/GahanAi_GNSS_DR_IMU/state_estimator/install/state_estimator'
