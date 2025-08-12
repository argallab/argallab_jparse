#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

# pinocchio
import numpy as np
import pinocchio as pin
# from urdf_parser_py.urdf import Robot

# ros2 stuff
# from visualization_msgs.msg import Marker, MarkerArray
# from geometry_msgs.msg import Quaternion, Pose, Point, PoseStamped
# from sensor_msgs.msg import JointState

# this is the huge node that does all the jacobian calculations!!


class JParseClass(Node):
    def __init__(self, base_link="link_base", end_link="link_eef"): # defaults are the xarm ones
        # Initialize any necessary variables or parameters here
        """
        Base link: The base link of the robot.
        End link: The end link of the robot.
        """
        super().__init__('jparse_class')

        self.base_link = base_link
        self.end_link = end_link
        # from IPython import embed; embed()
        # Load URDF from parameter server
        # urdf = Robot.from_parameter_server()
        # model = pin.buildModelFromUrdf(urdf.to_xml_string())
        # self.data = model.createData()
        # self.model = model
        # self.num_joints = 7

        # publishers
        # self.marker_pub = self.create_publisher(MarkerArray, '/jparse_ellipsoid_marker', 10)

        # subscribers
        # self.joint_state_sub = self.create_subscription(
        #     JointState, '/joint_states', self.joint_state_callback, 10)

        self.J_prev = None
        self.J_prev_time = None

    def svd_compose(self,U,S,Vt):
        """
        This function takes SVD: U,S,V and recomposes them for J
        """
        Zero_concat = np.zeros((U.shape[0],Vt.shape[0]-len(S)))
        Sfull = np.zeros((U.shape[1],Vt.shape[0]))
        for row in range(Sfull.shape[0]):
            for col in range(Sfull.shape[1]):
              if row == col:
                  if row < len(S):        
                      Sfull[row,col] = S[row]
        J_new =np.matrix(U)*Sfull*np.matrix(Vt)
        return J_new


    def JParse(self, J=None, jac_nullspace_bool=False, gamma=0.1, singular_direction_gain_position=1,singular_direction_gain_orientation=1, position_dimensions=None, angular_dimensions=None):
        """
        input: Jacobian J (m x n) numpy matrix
        Args:
          - jac_nullspace_bool (default=False): Set this to true of the nullspace of the jacobian is desired
          - gamma (default=0.1): threshold gain for singular directions
          - singular_direction_gain_position (default=1): gain for singular directions in position
          - singular_direction_gain_orientation (default=1): gain for singular directions in orientation
          - position_dimensions (default=None): the number of dimensions for the position *NOTE: if this is not set, then all gains are be set to the singular_directin_gain_position value
          - angular_dimensions (default=None): the number of dimensions for the orientation *NOTE: if this is not set, then all gains are be set to the singular_directin_gain_position value

        Note: if no information is provided for position or angular dimensions, then the rows of the jacobian are used to determine the gain matrix dimension, and the default (or user set) value of singular_direction_gain_position
        is used by default.

        output:
          - J_parse (n x m) numpy matrix
          - (optional) J_safety_nullspace (n x n) numpy matrix : this can be used with a secondary objective like attracting joints to a nominal position (which will not interfere with primary task objective).
                                          This is just the matrix, the joint-space objective must be calculated and multiplied by the user.
        """

        #Perform the SVD decomposition of the jacobian
        U, S, Vt = np.linalg.svd(J)
        #Find the adjusted condition number
        sigma_max = np.max(S)
        adjusted_condition_numbers = [sig / sigma_max for sig in S]

        #Find the projection Jacobian
        U_new_proj = []
        S_new_proj = []
        for col in range(len(S)):
            if S[col] > gamma*sigma_max:
                #Singular row
                U_new_proj.append(np.matrix(U[:,col]).T)
                S_new_proj.append(S[col])
        U_new_proj = np.concatenate(U_new_proj,axis=1) #Careful which numpy version is being used for this!!!!!
        J_proj = self.svd_compose(U_new_proj, S_new_proj, Vt)

        #Find the safety jacboian
        S_new_safety = [s if (s/sigma_max) > gamma else gamma*sigma_max for s in S]
        J_safety = self.svd_compose(U,S_new_safety,Vt)

        #Find the singular direction projection components
        U_new_sing = []
        Phi = [] #these will be the ratio of s_i/s_max
        set_empty_bool = True
        for col in range(len(S)):
            if adjusted_condition_numbers[col] <= gamma:
                set_empty_bool = False
                U_new_sing.append(np.matrix(U[:,col]).T)
                Phi.append(adjusted_condition_numbers[col]/gamma) #division by gamma for s/(s_max * gamma), gives smooth transition for Kp =1.0;

        #set an empty Phi_singular matrix, populate if there were any adjusted
        #condition numbers below the threshold
        Phi_singular = np.zeros(U.shape) #initialize the singular projection matrix

        if set_empty_bool == False:
            #construct the new U, as there were singular directions
            U_new_sing = np.matrix(np.concatenate(U_new_sing,axis=1)) #Careful which numpy version is being used for this!!!!!
            Phi_mat = np.matrix(np.diag(Phi))

            # Now handle the gain conditions
            if position_dimensions == None and angular_dimensions == None:
                #neither dimensions have been set, this is the default case
                gain_dimension = J.shape[0]
                gains = np.array([singular_direction_gain_position]*gain_dimension, dtype=float)
            elif angular_dimensions == None and position_dimensions != None:
                #only position dimensions have been set
                gain_dimension = position_dimensions
                gains = np.array([singular_direction_gain_position]*gain_dimension, dtype=float)
            elif position_dimensions == None and angular_dimensions != None:
                #only angular dimensions have been set
                gain_dimension = angular_dimensions
                gains = np.array([singular_direction_gain_orientation]*gain_dimension, dtype=float)
            else:
                #both position and angular dimensions are filled
                gains = np.array([singular_direction_gain_position]*position_dimensions + [singular_direction_gain_orientation]*angular_dimensions, dtype=float)
            #now put them into a matrix:
            Kp_singular = np.diag(gains)

            # Now put it all together:
            Phi_singular = U_new_sing @ Phi_mat @ U_new_sing.T @ Kp_singular

        #Obtain psuedo-inverse of the safety jacobian and the projection jacobian
        J_safety_pinv = np.linalg.pinv(J_safety)
        J_proj_pinv = np.linalg.pinv(J_proj)

        if set_empty_bool == False:
            J_parse = J_safety_pinv @ J_proj @ J_proj_pinv + J_safety_pinv @ Phi_singular
        else:
            J_parse = J_safety_pinv @ J_proj @ J_proj_pinv

        if jac_nullspace_bool == True:
            #Find the nullspace of the jacobian
            J_safety_nullspace = np.eye(J_safety.shape[1]) - J_safety_pinv @ J_safety
            return J_parse, J_safety_nullspace

        return J_parse
        

def main(args=None):
    rclpy.init(args=args)
    node = JParseClass()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()



"""
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
        pass
"""
        