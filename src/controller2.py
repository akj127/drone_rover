#!/usr/bin/env python
# coding: utf-8




import rospy
import numpy as np
import tf

from math import pi, sqrt
from std_msgs.msg import Float32
from std_srvs.srv import Empty, EmptyResponse
from geometry_msgs.msg import Vector3, Point, PoseStamped, TwistStamped, PointStamped
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import CommandBool, SetMode
from mavros_apriltag_tracking.srv import PIDGains, PIDGainsResponse




class Controller:
    def __init__(self):
        # Error in body x direction (+x is drone's right)
        self.ex_ = 0.0
        # Error in body y direction (+y is  drone's front)
        self.ey_ = 0.0
        # Error in body z direction (+z is  up)

        # Error integral in x
        self.ex_int_ = 0.0
        # Error integral in y
        self.ey_int_ = 0.0
        # Error integral in z


        # Proportional gain for horizontal controller
        self.kP_xy_ = rospy.get_param('~horizontal_controller/kP', 1.5)
        # Integral gain for horizontal controller
        self.kI_xy_ = rospy.get_param('~horizontal_controller/kI', 0.01)
        # Integral gain for vertical controller



        # Controller outputs. Velocity commands in body sudo-frame
        self.body_vx_cmd_ = 0.0
        self.body_vy_cmd_ = 0.0
   
        # Controller outputs in local frame
        self.local_vx_cmd_ = 0.0
        self.local_vy_cmd_ = 0.0

        
        # Maximum horizontal velocity (m/s)
        self.vXYMAX_ = rospy.get_param('~horizontal_controller/vMAX', 1.0)
     
        # Flag for FCU state. True if vehicle is armed and ready
        # Prevents building controller integral part when vehicle is idle on ground
        self.engaged_ = False

        # Subscribe to drone FCU state
        rospy.Subscriber('mavros/state', State, self.cbFCUstate)

        # Service for modifying horizontal PI controller gains 
        rospy.Service('horizontal_controller/pid_gains', PIDGains, self.setHorizontalPIDCallback)
    

    def setHorizontalPIDCallback(self, req):
        if req.p < 0. or req.i < 0.0:
            rospy.logerr("Can not set negative PID gains.")
            return []

        self.kP_xy_ = req.p
        self.kI_xy_ = req.i

        rospy.loginfo("Horizontal controller gains are set to P=%s I=%s", self.kP_xy_, self.kI_xy_)

        return []

    def cbFCUstate(self, msg):
        if msg.armed and msg.mode == 'OFFBOARD' :
            self.engaged_ = True
        else:
            self.engaged_ = False

    def resetIntegrators(self):
        self.ex_int_ = 0.
        self.ey_int_ = 0.


    def computeVelSetpoint(self):
        """
        Computes XY velocity setpoint in body sudo-frame using a PI controller
        """
        # Compute commands
        self.body_vx_cmd_ = self.kP_xy_*self.ex_ + self.kI_xy_*self.ex_int_
        self.body_vy_cmd_ = self.kP_xy_*self.ey_ + self.kI_xy_*self.ey_int_


        # Horizontal velocity constraints
        vel_magnitude = sqrt(self.body_vx_cmd_**2 + self.body_vy_cmd_**2)
        if vel_magnitude > self.vXYMAX_ : # anti-windup scaling      
            scale = self.vXYMAX_/vel_magnitude
            self.body_vx_cmd_ = self.body_vx_cmd_*scale
            self.body_vy_cmd_ = self.body_vy_cmd_*scale
        else:
            if self.engaged_: # if armed & offboard
                self.ex_int_ = self.ex_int_ + self.ex_ # You can divide self.ex_ by the controller rate, but you can just tune self.kI_xy_ for now!
                self.ey_int_ = self.ey_int_ + self.ey_


        return self.body_vx_cmd_, self.body_vy_cmd_





class Commander:
    def __init__(self):
        # Instantiate a setpoint topic structure
        self.setpoint_ = PositionTarget()

        # use velocity and yaw setpoints
        self.setBodyVelMask()

        # Velocity setpoint by user
        self.vel_setpoint_ = Vector3()

        # Position setpoint by user
        self.pos_setpoint_ = Point()

        # Current local velocity
        self.local_vel_ = TwistStamped()

        # Current body velocity
        self.body_vel_ = TwistStamped()

        # Yaw setpoint by user (degrees); will be converted to radians before it's published
        self.yaw_setpoint_ = 0.0

        # Current drone position (local frame)
        self.drone_pos_ = Point()

        # FCU modes
        # self.fcu_mode_ = FCUModes()

        # setpoint publisher (velocity to Pixhawk)
        self.setpoint_pub_ = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=10)

        # Subscriber for user setpoints (yaw in degrees)
        rospy.Subscriber('setpoint/yaw_deg', Float32, self.yawSpCallback)

        # Subscriber to current drone's position
        rospy.Subscriber('mavros/local_position/pose', PoseStamped, self.dronePosCallback)

        # Subscriber to body velocity
        rospy.Subscriber('mavros/local_position/velocity_body', TwistStamped, self.bodyVelCallback)

        # Subscriber to local velocity
        rospy.Subscriber('mavros/local_position/velocity_local', TwistStamped, self.localVelCallback)

        # Service for arming and setting OFFBOARD flight mode
        #rospy.Service('arm_and_offboard', Empty, self.armAndOffboard)

        # Service for autoland
       # rospy.Service('auto_land', Empty, self.autoLand)


    def bodyVelCallback(self, msg):
        self.body_vel_ = msg

    def localVelCallback(self, msg):
        self.local_vel_ = msg

    #def autoLand(self, req):
     #   self.fcu_mode_.setAutoLandMode()

      #  return EmptyResponse()

   # def armAndOffboard(self, req):
    #    self.fcu_mode_.setArm()
     #   self.fcu_mode_.setOffboardMode()
        
      #  return EmptyResponse()

    def dronePosCallback(self, msg):
        self.drone_pos_.x = msg.pose.position.x
        self.drone_pos_.y = msg.pose.position.y


    def velSpCallback(self, msg):
        """
        Velocity setpoint callback
        """
        self.vel_setpoint_.x = msg.x
        self.vel_setpoint_.y = msg.y


        self.setBodyVelMask()
    
    def posSpCallback(self, msg):
        """
        Position setpoint callback
        """
        self.pos_setpoint_.x = msg.x
        self.pos_setpoint_.y = msg.y
   

        self.setLocalPositionMask()

    def yawSpCallback(self, msg):
        """
        Yaw setpoint callback
        """
        self.yaw_setpoint_ = msg.data


    def setLocalPositionMask(self):
        """
        Sets type_mask for position setpoint in local frame +  yaw setpoint
        """
        # FRAME_LOCAL_NED, FRAME_LOCAL_OFFSET_NED, FRAME_BODY_NED, FRAME_BODY_OFFSET_NED
        self.setpoint_.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        self.setpoint_.type_mask = PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ +                                     PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ +                                     PositionTarget.IGNORE_YAW_RATE

    def setBodyVelMask(self):
        """
        Sets type_mask for velocity setpoint in body frame + yaw setpoint
        """
        # FRAME_LOCAL_NED, FRAME_LOCAL_OFFSET_NED, FRAME_BODY_NED, FRAME_BODY_OFFSET_NED
        self.setpoint_.coordinate_frame = PositionTarget.FRAME_BODY_NED
        self.setpoint_.type_mask = PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ +                                     PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ +                                     PositionTarget.IGNORE_YAW_RATE

    def setLocalVelMask(self):
        """
        Sets type_mask for velocity setpoint in local frame + yaw setpoint
        """
        # FRAME_LOCAL_NED, FRAME_LOCAL_OFFSET_NED, FRAME_BODY_NED, FRAME_BODY_OFFSET_NED
        self.setpoint_.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        self.setpoint_.type_mask = PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ +                                     PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ +                                     PositionTarget.IGNORE_YAW_RATE

    def publishSetpoint(self):
        self.setpoint_.header.stamp = rospy.Time.now()

        # Only one type of the following setpoints will be consumed based on the type_mask
        self.setpoint_.position.x = self.pos_setpoint_.x
        self.setpoint_.position.y = self.pos_setpoint_.y
     

        self.setpoint_.velocity.x = self.vel_setpoint_.x
        self.setpoint_.velocity.y = self.vel_setpoint_.y
     

        self.setpoint_.yaw = self.yaw_setpoint_ * pi / 180. # convert to radians

        self.setpoint_pub_.publish(self.setpoint_)



class Tracker:
    def __init__(self):
        # setpoints in local frame
        self.local_xSp_  = 0.0
        self.local_ySp_  = 0.0
    

        # Relative setpoints (i.e. with respect to body horizontal-frame)
        self.relative_xSp_  = 0.0
        self.relative_ySp_  = 0.0

        
        # Flag to select between local vs. relative tracking
        # set False for relative target tracking
        self.local_tracking_ = False

        # Commander object to send velocity setpoints to FCU
        self.commander_ = Commander()

        # Controller object to calculate velocity commands
        self.controller_ =Controller()

        # Subscriber for user setpoints (local position)
        rospy.Subscriber('setpoint/local_pos', Point, self.localPosSpCallback)

        # Subscriber for user setpoints (relative position)
        rospy.Subscriber('setpoint/relative_pos', Point, self.relativePosSpCallback)

        # Publisher for velocity errors in body frame
        self.bodyVel_err_pub_ = rospy.Publisher('analysis/body_vel_err', PointStamped, queue_size=10)

        # Publisher for velocity errors in local frame
        self.localVel_err_pub_ = rospy.Publisher('analysis/local_vel_err', PointStamped, queue_size=10)

        # Publisher for position errors in local frame
        self.localPos_err_pub_ = rospy.Publisher('analysis/local_pos_err', PointStamped, queue_size=10)

        # Publisher for position error between drone and target
        self.relativePos_err_pub_ = rospy.Publisher('analysis/relative_pos_err', PointStamped, queue_size=10)

    def computeControlOutput(self):
        if self.local_tracking_:
            self.controller_.ex_ = self.local_xSp_ - self.commander_.drone_pos_.x
            self.controller_.ey_ = self.local_ySp_ - self.commander_.drone_pos_.y
           
            self.commander_.setLocalVelMask()
        else: # relative tracking
            self.controller_.ex_ = self.relative_xSp_
            self.controller_.ey_ = self.relative_ySp_
         
            self.commander_.setBodyVelMask()

        self.commander_.vel_setpoint_.x, self.commander_.vel_setpoint_.y = self.controller_.computeVelSetpoint()

    def localPosSpCallback(self, msg):
        self.local_xSp_ = msg.x
        self.local_ySp_ = msg.y
      

        # In case we are switching from relative to local tracking
        # to avoid jumps caused by accumulation in the integrators
        if not self.local_tracking_ or self.local_tracking_ is None:
            self.controller_.resetIntegrators()
            self.local_tracking_ = True

    def relativePosSpCallback(self, msg):
        self.relative_xSp_ = msg.x
        self.relative_ySp_ = msg.y

        # In case we are switching from local to relative tracking
        # to avoid jumps caused by accumulation in the integrators
        if self.local_tracking_ or self.local_tracking_ is None:
            self.controller_.resetIntegrators()
            self.local_tracking_ = False

    def publishErrorSignals(self):
        """
        Publishes all error signals for debugging and tuning
        """

        # Local velocity errors
        localVelErr_msg = PointStamped()
        localVelErr_msg.header.stamp = rospy.Time.now()
        localVelErr_msg.point.x = self.commander_.setpoint_.velocity.x - self.commander_.local_vel_.twist.linear.x
        localVelErr_msg.point.y = self.commander_.setpoint_.velocity.y - self.commander_.local_vel_.twist.linear.y
       
        
        self.localVel_err_pub_.publish(localVelErr_msg)

        # Body velocity errors
        # this message uses convention of +x-right, +y-forward, +z-up
        # the setpoint msg follows the same convention
        # However, the feedback signal (actual body vel from mavros) follows +x-forward, +y-left, +z-up
        # Required conversion is done below
        bodyVelErr_msg = PointStamped()
        bodyVelErr_msg.header.stamp = rospy.Time.now()
        bodyVelErr_msg.point.x = self.commander_.setpoint_.velocity.x - (-self.commander_.body_vel_.twist.linear.y)
        bodyVelErr_msg.point.y = self.commander_.setpoint_.velocity.y - self.commander_.body_vel_.twist.linear.x
      

        self.bodyVel_err_pub_.publish(bodyVelErr_msg)

        # Local position errors
        localPosErr_msg = PointStamped()
        localPosErr_msg.header.stamp = rospy.Time.now()
        localPosErr_msg.point.x = self.local_xSp_ - self.commander_.drone_pos_.x
        localPosErr_msg.point.y = self.local_ySp_ - self.commander_.drone_pos_.y
        

        self.localPos_err_pub_.publish(localPosErr_msg)

        # Relative position errors
        relPosErr_msg = PointStamped()
        relPosErr_msg.header.stamp = rospy.Time.now()
        relPosErr_msg.point.x = self.relative_xSp_
        relPosErr_msg.point.y = self.relative_ySp_
        
        
        self.relativePos_err_pub_.publish(relPosErr_msg)

if __name__ == '__main__':
    rospy.init_node('Offboard_control_node', anonymous=True)
    
    tracker = Tracker()

    loop = rospy.Rate(10)
     # Should be created outside ROS while loop 
    K = Controller()

    while not rospy.is_shutdown():

    
    #Example of how to use the PositionController
       
        
        # The following should be inside ROS while loop
        # update errors
        K.ex = relative_position_in_x # body directions
        K.ey = relative_position_in_y
    
        cmd.vel_setpoint_.x, cmd.vel_setpoint_.y = K.computeVelSetpoint()
        cmd.setBodyVelMask()
        # Then, publish command as below (publishSetpoint)
       
        tracker.computeControlOutput()
        tracker.commander_.publishSetpoint()
        tracker.publishErrorSignals()
        #cmd.publishSetpoint()
        loop.sleep()







