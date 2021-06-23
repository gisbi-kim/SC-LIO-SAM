import numpy as np
import pypcd # for the install, use this command: python3.x (use your python ver) -m pip install --user git+https://github.com/DanielPollithy/pypcd.git
from pypcd import pypcd

def make_xyzi_point_cloud(xyzl, label_type='f'):
    """ Make XYZL point cloud from numpy array.
    TODO i labels?
    """
    md = {'version': .7,
          'fields': ['x', 'y', 'z', 'intensity'],
          'count': [1, 1, 1, 1],
          'width': len(xyzl),
          'height': 1,
          'viewpoint': [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
          'points': len(xyzl),
          'data': 'ASCII'}
    if label_type.lower() == 'f':
        md['size'] = [4, 4, 4, 4]
        md['type'] = ['F', 'F', 'F', 'F']
    elif label_type.lower() == 'u':
        md['size'] = [4, 4, 4, 1]
        md['type'] = ['F', 'F', 'F', 'U']
    else:
        raise ValueError('label type must be F or U')
    # TODO use .view()
    xyzl = xyzl.astype(np.float32)
    dt = np.dtype([('x', np.float32), ('y', np.float32), ('z', np.float32),
                   ('intensity', np.float32)])
    pc_data = np.rec.fromarrays([xyzl[:, 0], xyzl[:, 1], xyzl[:, 2],
                                 xyzl[:, 3]], dtype=dt)
    pc = pypcd.PointCloud(md, pc_data)
    return pc
