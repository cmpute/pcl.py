from libcpp cimport bool
from .mouse_event_enums cimport Type, MouseButton

cdef extern from "pcl/visualization/keyboard_event.h" namespace "pcl::visualization":
    cdef cppclass MouseEvent:
            
        MouseEvent(Type &type, MouseButton &key_sym,
            unsigned int x, unsigned int y, bool alt, bool ctrl, bool shift, bool selection_mode)
        Type getType()
        void setType(Type &type)
        MouseButton getButton()
        void setButton(MouseButton &button)
        unsigned int getX()
        unsigned int getY()
        unsigned int getKeyboardModifiers()
        bool getSelectionMode()
