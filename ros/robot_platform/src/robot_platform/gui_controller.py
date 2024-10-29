
## Then load sys to get sys.argv.
import sys
import os
import time
import math

## Next import all the Qt bindings into the current namespace, for
## convenience.  This uses the "python_qt_binding" package which hides
## differences between PyQt and PySide, and works if at least one of
## the two is installed.  The RViz Python bindings use
## python_qt_binding internally, so you should use it here as well.
from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
try:
    from python_qt_binding.QtWidgets import *
except ImportError:
    pass
from .log_utils import env2log

## Finally import the RViz bindings themselves.
from rviz import bindings as rviz

from .odometry_helpers import PlatformStatics, create_request
import rospy

from sensor_msgs.msg import Joy, JoyFeedback
from actionlib_msgs.msg import GoalID
from std_msgs.msg import String
from robot_platform.msg import PlatformStatus, MoveRequest
from geometry_msgs.msg import Point
from std_msgs.msg import String

from threading import Lock


duration = 0.1

## The MyViz class is the main container widget.
class MyViz(QWidget):

    ## MyViz Constructor
    ## ^^^^^^^^^^^^^^^^^
    ##
    ## Its constructor creates and configures all the component widgets:
    ## frame, thickness_slider, top_button, and side_button, and adds them
    ## to layouts.
    def __init__(self, name='robot_platform'):
        QWidget.__init__(self)
        rospy.init_node(name, log_level=env2log(), anonymous=True)

        ## rviz.VisualizationFrame is the main container widget of the
        ## regular RViz application, with menus, a toolbar, a status
        ## bar, and many docked subpanels.  In this example, we
        ## disable everything so that the only thing visible is the 3D
        ## render window.
        self.frame = rviz.VisualizationFrame()

        ## The "splash path" is the full path of an image file which
        ## gets shown during loading.  Setting it to the empty string
        ## suppresses that behavior.
        self.frame.setSplashPath( "" )

        ## VisualizationFrame.initialize() must be called before
        ## VisualizationFrame.load().  In fact it must be called
        ## before most interactions with RViz classes because it
        ## instantiates and initializes the VisualizationManager,
        ## which is the central class of RViz.
        self.frame.initialize()

        ## The reader reads config file data into the config object.
        ## VisualizationFrame reads its data from the config object.
        reader = rviz.YamlConfigReader()
        config = rviz.Config()
        

        mapping_rviz = ''
        with open('/home/deck/Documents/ArduinoPlatformController/ros/robot_platform/config/mapping.rviz') as f:
            mapping_rviz = f.read()
            
        reader.readString( config, mapping_rviz )
        self.frame.load( config )

        ## You can also store any other application data you like in
        ## the config object.  Here we read the window title from the
        ## map key called "Title", which has been added by hand to the
        ## config file.
        self.setWindowTitle( config.mapGetChild( "Title" ).getValue() )

        ## Here we disable the menu bar (from the top), status bar
        ## (from the bottom), and the "hide-docks" buttons, which are
        ## the tall skinny buttons on the left and right sides of the
        ## main render window.
        self.frame.setMenuBar( None )
        self.frame.setStatusBar( None )
        self.frame.setHideButtonVisibility( False )

        ## frame.getManager() returns the VisualizationManager
        ## instance, which is a very central class.  It has pointers
        ## to other manager objects and is generally required to make
        ## any changes in an rviz instance.
        self.manager = self.frame.getManager()

        ## Since the config file is part of the source code for this
        ## example, we know that the first display in the list is the
        ## grid we want to control.  Here we just save a reference to
        ## it for later.
        self.grid_display = self.manager.getRootDisplayGroup().getDisplayAt( 0 )
        
        ## Here we create the layout and other widgets in the usual Qt way.
        layout = QVBoxLayout()

        upper_buttons_layout = QHBoxLayout()
        main_view_button = QPushButton( "main" )
        # main_view_button.clicked.connect( self.onTopButtonClick )
        details_view_button = QPushButton( "details" )
        # main_view_button.clicked.connect( self.onTopButtonClick )
        upper_buttons_layout.addWidget( main_view_button )
        upper_buttons_layout.addWidget( details_view_button )
        layout.addLayout( upper_buttons_layout )

        layout.addWidget( self.frame )
        
        # thickness_slider = QSlider( Qt.Horizontal )
        # thickness_slider.setTracking( True )
        # thickness_slider.setMinimum( 1 )
        # thickness_slider.setMaximum( 1000 )
        # thickness_slider.valueChanged.connect( self.onThicknessSliderChanged )
        # layout.addWidget( thickness_slider )
        
        commands_layout = QHBoxLayout()
        stop_command_button = QPushButton( "stop" )
        stop_command_button.clicked.connect( self._send_stop_command )
        rebuild_map_button = QPushButton( "rebuild map" )
        # rebuild_map_button.clicked.connect( self._send_stop_command )
        shutdown_command_button = QPushButton( "shutdown" )
        shutdown_command_button.clicked.connect( self._send_shutdown_command )
        commands_layout.addWidget( stop_command_button )
        commands_layout.addWidget( rebuild_map_button )
        commands_layout.addWidget( shutdown_command_button )
                
        layout.addLayout( commands_layout )
        
        self.setLayout( layout )

        self._last_platform_status = PlatformStatus()
        self._last_joy = Joy()
        self._last_limited_deltas = [0.0] * PlatformStatics.MOTOR_NUM
        self._last_request = None
        self._request = None
        self._left_trigger_was_max = False
        self._last_pan_angle = 0.0
        self._last_tilt_angle = 0.0
        self._request_lock = Lock()
        self._last_request = None
        joy_input_topic = rospy.get_param('~joy_topic')
        joy_output_topic = rospy.get_param('~joy_feedback_topic')
        move_request_output_topic = rospy.get_param('~move_request_output_topic')
        platform_status_input_topic = rospy.get_param('~platform_status_input_topic')
        shutdown_command_output_topic = rospy.get_param('~shutdown_command_output_topic')

        rospy.Subscriber(joy_input_topic, Joy, self._handle_joystick_updates)
        self._joy_feedback_publisher = rospy.Publisher(joy_output_topic, JoyFeedback)
        self._move_request_publisher = rospy.Publisher(move_request_output_topic, MoveRequest)
        rospy.Subscriber(platform_status_input_topic, PlatformStatus, self._handle_platform_status)
        self._shutdown_command_publisher = rospy.Publisher(shutdown_command_output_topic, String, queue_size=10)

        self._cancel_move_publisher = rospy.Publisher('/move_base/cancel', GoalID)

        rospy.Timer(rospy.Duration(duration), self._send_request)
        self._rospy_timer = rospy.Timer(rospy.Duration(1), self._rospy_spin) # start the ros spin after 1 s

    def _rospy_spin(self, *args, **kwargs):
        rospy.spin()
        self._rospy_timer.shutdown()

    def _send_stop_command(self):
        self._cancel_move_publisher.publish(GoalID())

    def _send_shutdown_command(self):
        data = String()
        data.data = 'shutdown'
        self._shutdown_command_publisher.publish(data)
    
    def _get_velocity(self, y_axis:float, left_trigger:float) -> float:
        rel_velocity = -0.25 * y_axis
        
        if rel_velocity < -0.025:
            rel_velocity = rel_velocity
        elif rel_velocity > 0.025:
            rel_velocity = rel_velocity
        else:
            rel_velocity = 0


        velocity = 0.0

        if not self._left_trigger_was_max and abs(left_trigger) > 0.5:
            self._left_trigger_was_max = True

        if self._left_trigger_was_max:
            boost = 0.5 + 3*(1-left_trigger)/2
        else:
            boost = 0.5
    
        if abs(rel_velocity) > PlatformStatics.MOVE_VELOCITY/100.0:
            velocity = round(-PlatformStatics.MOVE_VELOCITY * (rel_velocity * boost), 2)

        return velocity

    def _get_turn_radius(self, x_axis:float) -> float:
        turn_radius = round(-1.95 * x_axis, 2)
        
        if turn_radius < 0:
            turn_radius = max(turn_radius, -1.99)
        elif turn_radius > 0:
            turn_radius = min(turn_radius, 1.99)
        
        if turn_radius > 0.05:
            turn_radius = 2 - turn_radius + (PlatformStatics.ROBOT_WIDTH/4 + 0.05)
        elif turn_radius < -0.05:
            turn_radius = -2 - turn_radius - (PlatformStatics.ROBOT_WIDTH/4 + 0.05)
        else:
            turn_radius = 0
        
        return turn_radius

    def _get_pan_update(self, x_axis:float) -> float:
        if abs(x_axis) < 0.1:
            return None
        yaw_update = self._last_pan_angle + x_axis*0.01
        if yaw_update > math.pi/2:
            yaw_update = math.pi
        if yaw_update < -math.pi/2:
            yaw_update = -math.pi
        self._last_pan_angle = yaw_update
        return yaw_update

    def _get_tilt_update(self, y_axis:float) -> float:
        if abs(y_axis) < 0.1:
            return None
        tilt_update =  self._last_tilt_angle + y_axis*0.01
        if tilt_update > math.pi/2:
            tilt_update = math.pi
        if tilt_update < -math.pi/2:
            tilt_update = -math.pi
        self._last_tilt_angle = tilt_update
        return tilt_update

    def _handle_joystick_updates(self, data:Joy):

        if data.axes:

            velocity = self._get_velocity(data.axes[1], data.axes[3])
            turn_radius = self._get_turn_radius(data.axes[0])

            pan = self._get_pan_update(data.axes[2])
            tilt = self._get_tilt_update(data.axes[3])

            
            turning_point = None
            if abs(turn_radius) > PlatformStatics.MIN_ANGLE_DIFF:
                turning_point = Point()
                turning_point.y = -turn_radius
            
            r = create_request(velocity, 1.5*duration, self._last_platform_status, turning_point, pan, tilt)
            with self._request_lock:
                self._request = r

    def _handle_platform_status(self, status:PlatformStatus):
        self._last_platform_status = status

    def _requests_changed(self, r1, r2) -> bool:
        if not r1:
            return False
        
        requests_equal = r1 == r2

        if requests_equal:
            return abs(r1.motor1.velocity) > 0.001 or \
                abs(r1.motor2.velocity) > 0.001 or \
                abs(r1.motor3.velocity) > 0.001 or \
                abs(r1.motor4.velocity) > 0.001
        else:
            return True


    def _send_request(self, event=None):
        with self._request_lock:
            if self._requests_changed(self._request, self._last_request):
                self._move_request_publisher.publish(self._request)
                self._last_request = self._request


    ## Handle GUI events
    ## ^^^^^^^^^^^^^^^^^
    ##
    ## After the constructor, for this example the class just needs to
    ## respond to GUI events.  Here is the slider callback.
    ## rviz.Display is a subclass of rviz.Property.  Each Property can
    ## have sub-properties, forming a tree.  To change a Property of a
    ## Display, use the subProp() function to walk down the tree to
    ## find the child you need.
    def onThicknessSliderChanged( self, new_value ):
        if self.grid_display != None:
            self.grid_display.subProp( "Line Style" ).subProp( "Line Width" ).setValue( new_value / 1000.0 )

    ## The view buttons just call switchToView() with the name of a saved view.
    def onTopButtonClick( self ):
        self.switchToView( "map" );
        
    def onSideButtonClick( self ):
        self.switchToView( "base_link" );
        
    ## switchToView() works by looping over the views saved in the
    ## ViewManager and looking for one with a matching name.
    ##
    ## view_man.setCurrentFrom() takes the saved view
    ## instance and copies it to set the current view
    ## controller.
    def switchToView( self, view_name ):
        view_man = self.manager.getViewManager()
        for i in range( view_man.getNumViews() ):
            if view_man.getViewAt( i ).getName() == view_name:
                view_man.setCurrentFrom( view_man.getViewAt( i ))
                return
        print( "Did not find view named %s." % view_name )

## Start the Application
## ^^^^^^^^^^^^^^^^^^^^^
##
## That is just about it.  All that's left is the standard Qt
## top-level application code: create a QApplication, instantiate our
## class, and start Qt's main event loop (app.exec_()).
def main():
    app = QApplication( sys.argv )

    myviz = MyViz()
    myviz.resize( 1280, 800 )
    myviz.show()

    sys.exit(app.exec())

if __name__ == '__main__':
    main()