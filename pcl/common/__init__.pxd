from libcpp cimport bool

cdef inline _ensure_true(bool value, str func):
    if value != True:
        raise RuntimeError("Function %s returned false, please check stderr output!" % func)
cdef inline _ensure_zero(int value, str func):
    if value != 0:
        raise RuntimeError("Function %s returned %d, please check stderr output!" % (func, value))
