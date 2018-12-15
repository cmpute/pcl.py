from libcpp cimport bool
from libcpp.string cimport string

cdef extern from "pcl/visualization/keyboard_event.h" namespace "pcl::visualization":
    cdef cppclass KeyboardEvent:
        KeyboardEvent(bool action, string &key_sym, unsigned char key, bool alt, bool ctrl, bool shift)
        bool isAltPressed()
        bool isCtrlPressed()
        bool isShiftPressed()
        unsigned char getKeyCode()
        string getKeySym()
        bool keyDown()
        bool keyUp()
