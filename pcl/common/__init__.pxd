from libcpp cimport bool

cdef inline _ensure_true(bool value, str func):
    if value != True:
        raise RuntimeError("Function %s returned false, please check stderr output!" % func)
cdef inline _ensure_zero(int value, str func):
    if value != 0:
        raise RuntimeError("Function %s returned %d, please check stderr output!" % (func, value))
cdef inline _ensure_exist(str path):
    import os.path as osp
    if not osp.exists(path):
        raise FileNotFoundError("Cannot find file: " + osp.basename(path))
