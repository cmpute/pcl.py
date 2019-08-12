from pcl.PointField import *
from pcl.PointCloud import *

from pcl.common import *
from pcl.filters import *
from pcl.io import *
from pcl.visualization import *

def get_include():
    import pcl, os
    return os.path.join(os.path.dirname(pcl.__file__), 'include')
