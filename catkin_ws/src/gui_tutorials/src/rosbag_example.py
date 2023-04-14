#!/usr/bin/env python3.8

import subprocess


command = "rosbag record -O velodyne /velodyne_points"
p = subprocess.run(command, shell=True, timeout=6)
p.kill()

# try:
#     s = String()
#     s.data = 'foo'

#     i = Int32()
#     i.data = 42

#     bag.write('/vectornav/IMU')
# finally: 
#     bag.close()