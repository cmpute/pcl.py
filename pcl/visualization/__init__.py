try:
    available = True
    from pcl.visualization._visualization import *
except ImportError:
    available = False

