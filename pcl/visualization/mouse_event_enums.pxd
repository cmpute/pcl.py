
# Move these declarations into separate file to enable direct access of their values
cdef extern from "pcl/visualization/keyboard_event.h" namespace "pcl::visualization::MouseEvent":
    ctypedef enum Type "pcl::visualization::MouseEvent::Type":
        MouseMove,
        MouseButtonPress,
        MouseButtonRelease,
        MouseScrollDown,
        MouseScrollUp,
        MouseDblClick
    ctypedef enum MouseButton "pcl::visualization::MouseEvent::MouseButton":
        NoButton
        LeftButton
        MiddleButton
        RightButton
        VScroll
