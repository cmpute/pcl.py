'''
Implementation of following files:
    pcl/visualization/include/pcl/visualization/impl/pcl_visualizer.hpp
    pcl/visualization/include/pcl/visualization/pcl_visualizer.h
    pcl/visualization/include/pcl/visualization/interactor_style.h
    pcl/visualization/include/pcl/visualization/point_picking_event.h
    pcl/visualization/src/pcl_visualizer.cpp
    pcl/visualization/src/interactor_style.cpp
    pcl/visualization/src/point_picking_event.cpp
'''

import abc
try:
    import vtk
except ImportError:
    print('please install vtk properly to use visualization')

########## Callback Functions ###########
class _InteractorKeyboardModifier:
    ALT = 0
    CTRL = 1
    SHIFT = 2

class _Callback(metaclass=abc.ABCMeta):
    '''
    Base class for callback functions
    '''
    @abc.abstractmethod
    def execute(self, obj, event):
        '''
        Execute callback function, derived from vtkCommand
        '''
        pass

class _PointPickingCallback(_Callback):
    '''
    Callback for points picking
    '''
    def execute(self, obj, event):
        pass
    # TODO: Not finished implementing

class _FPSCallback(_Callback):
    '''
    Callback for FPS displaying
    '''
    def __init__(self, visualizer):
        self.actor = None
        self.pcl_visualizer = visualizer
        self.decimated = None

    def execute(self, obj, event):
        pass
    # TODO: Not finished implementing

class _ExitMainLoopTimerCallback(_Callback):
    '''
    Callback for exiting main loop
    '''
    def __init__(self, visualizer):
        self.right_timer_id = None
        self.pcl_visualizer = visualizer

    def execute(self, obj, event):
        pass
    # TODO: Not finished implementing

class _ExitCallBack(_Callback):
    '''
    Callback for exit
    '''
    def __init__(self, visualizer):
        self.pcl_visualizer = visualizer

    def execute(self, obj, event):
        pass
    # TODO: Not finished implementing

########## Standard Visualizer ###########
class VisualizerInteractionStyle(vtk.vtkInteractorStyleRubberBandPick):
    '''
    PCLVisualizerInteractorStyle defines an unique, custom VTK
    based interactory style for PCL Visualizer applications. Besides
    defining the rendering style, we also create a list of custom actions
    that are triggered on different keys being pressed:
    -        p, P   : switch to a point-based representation
    -        w, W   : switch to a wireframe-based representation (where available)
    -        s, S   : switch to a surface-based representation (where available)
    -        j, J   : take a .PNG snapshot of the current window view
    -        c, C   : display current camera/window parameters
    -        f, F   : fly to point mode
    -        e, E   : exit the interactor\
    -        q, Q   : stop and call VTK's TerminateApp
    -       + / -   : increment/decrement overall point size
    -        g, G   : display scale grid (on/off)
    -        u, U   : display lookup table (on/off)
    -  r, R [+ ALT] : reset camera [to viewpoint = {0, 0, 0} . center_{x, y, z}]
    -  CTRL + s, S  : save camera parameters
    -  CTRL + r, R  : restore camera parameters
    -  ALT + s, S   : turn stereo mode on/off
    -  ALT + f, F   : switch between maximized window mode and original size
    -        l, L           : list all available geometric and color handlers for the
                              current actor map
    -  ALT + 0..9 [+ CTRL]  : switch between different geometric handlers (where available)
    -        0..9 [+ CTRL]  : switch between different color handlers (where available)
    -
    -  SHIFT + left click   : select a point
    -        x, X   : toggle rubber band selection mode for left mouse button
    '''
    def __init__(self):
        self.renderer_collection = None
        self.cloud_actor_map = None
        self.shape_actor_map = None
        self.use_vbos = False
        self._init = False
        self._win_height = self._win_width = None
        self._win_pos_x = self.win_pos_y = None
        self._max_win_height = self._max_win_width = None
        self._grid_enabled = None
        self._grid_actor = None
        self._lut_enabled = None
        self._lut_actor = None
        self._snapshot_writer = None
        self._wif = None
        self._mouse_signal = None
        self._keyboard_signal = None
        self._point_picking_singal = None
        self._area_picking_singal = None
        self._stereo_anaglyph_mask_default = None
        self._mouse_callback = None
        self._modifier = None
        self.camera_file = None
        self._camera = None
        self._camera_saved = None
        self.render_window = None
        self._lut_actor_id = ''

    def initialize(self):
        '''
        Initialization routine. Must be called before anything else.
        '''
        self._modifier = _InteractorKeyboardModifier.ALT
        # Set windows size (width, height) to unknown (-1)
        self._win_height = self._win_width = -1
        self._win_pos_x = self.win_pos_y = 0
        self._max_win_height = self._max_win_width = -1

        # Grid is disabled by default
        self._grid_enabled = False
        self._grid_actor = vtk.vtkLegendScaleActor()

        # LUT is disabled by default
        self._lut_enabled = False
        self._lut_actor = vtk.vtkLegendScaleActor()
        self._lut_actor.SetTitle("")
        self._lut_actor.SetOrientationToHorizontal()
        self._lut_actor.SetPosition(0.05, 0.01)
        self._lut_actor.SetWidth(0.9)
        self._lut_actor.SetHeight(0.1)
        self._lut_actor.SetNumberOfLabels(self._lut_actor.GetNumberOfLabels() * 2)
        prop = self._lut_actor.GetLabelTextProperty()
        prop.SetFontSize(10)
        self._lut_actor.SetLabelTextProperty(prop)
        self._lut_actor.SetTitleTextProperty(prop)

        # Create the image filter and PNG writer objects
        self._wif = vtk.vtkWindowToImageFilter()
        self._wif.ReadFrontBufferOff()
        self._snapshot_writer = vtk.vtkPNGWriter()
        self._snapshot_writer.SetInputConnection(self._wif.GetOutputPort)

        self._init = True

        self._stereo_anaglyph_mask_default = True
        # Start in orient mode
        super().CurrentMode = 0

        # Add our own mouse callback before any user callback. Used for accurate point picking.
        self._mouse_callback = _PointPickingCallback()
        self.AddObserver(vtk.vtkCommand.LeftButtonPressEvent, self._mouse_callback.execute)
        self.AddObserver(vtk.vtkCommand.LeftButtonReleaseEvent, self._mouse_callback.execute)

    # TODO: Not finished implementing

class Visualizer():
    '''
    PCL Visualizer
    '''
    def __init__(self, name='', create_interactor=True, style=VisualizerInteractionStyle()):
        self._interactor = vtk.vtkRenderWindowInteractor()
        self._update_fps = _FPSCallback(self)
        self._stopped = False
        self._timer_id = 0
        self._exit_main_loop_timer_callback = None
        self._exit_callback = None
        self._rens = vtk.RendererCollection()
        self._win = None
        self._style = style
        self._cloud_actor_map = dict()
        self._shape_actor_map = dict()
        self._coordinate_actor_map = dict()
        self._camera_set = False
        self._camera_file_loaded = False

        # Create a Renderer
        ren = vtk.vtkRenderer()
        ren.AddObserver(vtk.vtkCommand.EndEvent, self._update_fps)
        self._rens.AddItem(ren)

        # FPS callback
        txt = vtk.vtkTextActor()
        self._update_fps.actor = txt
        self._update_fps.pcl_visualizer = self
        self._update_fps.decimated = False
        ren.AddActor(txt)
        txt.SetInput('0 FPS')

        # Create a RendererWindow
        self._win = vtk.vtkRendererWindow()
        self._win.SetWindowName(name)

        # Get screen size
        scr_size_x, scr_size_y = self._win.GetScreenSize()
        # Set the window size as 1/2 of the screen size
        self._win.SetSize(scr_size_x / 2, scr_size_y / 2)

        # By default, don't use vertex buffer objects
        self._use_vbox = False

        # Add all renderers to the window
        self._rens.InitTraversal()
        while True:
            renderer = self._rens.GetNextItem()
            if renderer is None:
                break
            self._win.AddRenderer(renderer)

        # Set renderer window in case no interactor is created
        self._style.render_window = self._win

        # Create the interactor style
        self._style.initialize()
        self._style.renderer_collection = self._rens
        self._style.cloud_actor_map = self._cloud_actor_map
        self._style.shape_actor_map = self._shape_actor_map
        self._style.user_timers_on()
        self._style.use_vbos = self._use_vbos

        if create_interactor:
            self.create_interactor()

        self._win.SetWindowName(name) # XXX duplicate?

    def __del__(self):
        if self._interactor is not None:
            self._interactor.DestroyTimer(self._timer_id)
        self._rens.RemoveAllItems()

    def create_interactor(self):
        '''
        Create the internal Interactor object.
        '''
        self._interactor = vtk.vtkRendererWindowInteractor()

        self._win.AlphaBitPlanesOff()
        self._win.PointSmoothingOff()
        self._win.LineSmoothingOff()
        self._win.PolygonSmoothingOff()
        self._win.SwapBuffersOn()
        self._win.SetStereoTypeToAnaglyph()

        self._interactor.SetRenderWindow(self._win)
        self._interactor.SetInteractorStyle(self._style)
        self._interactor.SetDesiredUpdateRate(30.0)

        self._interactor.Initialize()
        self._timer_id = self._interactor.CreateRepeatingTimer(5000)

        ppkr = vtk.vtkPointPicker()
        ppkr.SetTolerance(ppkr.GetTolerance() * 2)
        self._interactor.SetPicker(ppkr)

        self._exit_main_loop_timer_callback = _ExitMainLoopTimerCallback(self)
        self._exit_main_loop_timer_callback.right_timer_id = -1
        self._interactor.AddObserver(vtk.vtkCommand.TimerEvent,
                                     self._exit_main_loop_timer_callback)

        self._exit_callback = _ExitCallBack(self)
        self._interactor.AddObserver(vtk.vtkCommand.ExitEvent, self._exit_callback)

        self.reset_stopped_flag()

    def setup_interactor(self, iren, win, style=None):
        '''
        Set up our unique PCL interactor style for a given vtkRenderWindowInteractor object
        attached to a given vtkRenderWindow

        # Parameters
        iren : vtkRenderWindowInteractor
            The vtkRenderWindowInteractor object to set up
        win : vtkRenderWindow
            A vtkRenderWindow object that the interactor is attached to
        '''
        if style is None:
            style = self._style

        win.AlphaBitPlanesOff()
        win.PointSmoothingOff()
        win.LineSmoothingOff()
        win.PolygonSmoothingOff()
        win.SwapBuffersOn()
        win.SetStereoTypeToAnaglyph()

        iren.SetRenderWindow(win)
        iren.SetInteractorStyle(style)
        iren.SetDesiredUpdateRate(30.0)

        iren.Initialize()

        if style != self._style:
            ppkr = vtk.vtkPointPicker()
            ppkr.SetTolerance(ppkr.GetTolerance() * 2)
            iren.SetPicker(ppkr)

    def reset_stopped_flag(self):
        '''
        Set the stopped flag back to false
        '''
        pass

    def save_screen_shot(self, file):
        '''
        Save the current rendered image to disk, as a PNG screenshot.
        '''
        self._style.save_screen_shot(file)

    def save_camera_parameters(self, file):
        '''
        Save the camera parameters to disk, as a .cam file.
        '''
        self._style.save_camera_parameters(file)

    # TODO: Not finished implementing
