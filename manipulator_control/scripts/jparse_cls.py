import rclpy
from rclpy.Node import Node

# pinocchio
import numpy as np
import pinocchio as pin
from urdf_parser_py.urdf import URDF

# ros2 stuff
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Quaternion, Pose, Point, PoseStamped
from sensor_msgs.msg import JointState

# this is the huge node that does all the jacobian calculations!!


class JParseClass(Node):
    def __init__(self):
        # Initialize any necessary variables or parameters here
        """
        Base link: The base link of the robot.
        End link: The end link of the robot.
        """
        super().__init__('jparse_class')
        
        # parameters
        self.declare_parameter('base_link', 'base_link',
                               description='Base link of the robot')
        self.declare_parameter('end_link', 'end_effector_link',
                               description='End effector link of the robot')
        
        # Get parameters
        self.base_link = self.get_parameter('base_link').get_parameter_value().string_value
        self.end_effector_link = self.get_parameter('end_link').get_parameter_value().string_value

        # Load URDF from parameter server
        urdf = URDF.from_parameter_server(self)
        model = pin.buildModelFromUrdf(urdf.to_xml_string())
        self.data = model.createData()
        self.model = model

        # publishers
        self.marker_pub = self.create_publisher(MarkerArray, '/jparse_ellipsoid_marker', 10)

        # subscribers
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)

        self.J_prev = None
        self.J_prev_time = None

        # Timer: publish every 0.1s
        self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):


        