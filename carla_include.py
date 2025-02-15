import glob
import os
import sys
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
import carla
import os
import sys
import time
import glob
import random
import signal
import logging
import threading
import numpy as np
# import open3d as o3d
import matplotlib.pyplot as plt
from plyfile import PlyData
from matplotlib.animation import FuncAnimation