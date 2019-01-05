from libcpp cimport bool

cdef extern from "pcl/visualization/keyboard_event.h" namespace "pcl::visualization":
    cdef cppclass MouseEvent:
        enum Type:
            MouseMove,
            MouseButtonPress,
            MouseButtonRelease,
            MouseScrollDown,
            MouseScrollUp,
            MouseDblClick
        enum MouseButton:
            NoButton
            LeftButton
            MiddleButton
            RightButton
            VScroll
            
        MouseEvent(MouseEvent.Type &type, MouseEvent.MouseButton &key_sym,
            unsigned int x, unsigned int y, bool alt, bool ctrl, bool shift, bool selection_mode)
        MouseEvent.Type getType()
        void setType(Type &type)
        MouseEvent.MouseButton getButton()
        void setButton(MouseButton &button)
        unsigned int getX()
        unsigned int getY()
        unsigned int getKeyboardModifiers()
        bool getSelectionMode()
