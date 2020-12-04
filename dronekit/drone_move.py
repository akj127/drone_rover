# dronekit
from dronekit import connect
from pymavlink import mavutil  # Needed for command message definitions

# ROS shit
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import argparse
import rospy
import cv2

# Controller
from Controller import Controller

# Drone utils
from drone_utils import arm_and_takeoff


bridge = CvBridge()

# Load the dictionary that was used to generate the markers.
dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
# Initialize the detector parameters using default values
parameters = cv2.aruco.DetectorParameters_create()

controller = Controller(0.001, 0.001, 0.001)


def get_aruco_center(cv_image, parameters, dictionary):
    image_shape = cv_image.shape

    center_x = image_shape[1] / 2
    center_y = image_shape[0] / 2

    markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(
        cv_image, dictionary, parameters=parameters)

    try:
        bottom_left_corner = tuple(markerCorners[0][0][0])
        top_right_corner = tuple(markerCorners[0][0][2])
    except IndexError:
        bottom_left_corner = [center_x, center_y]
        top_right_corner = [center_x, center_y]

    rover_x = (bottom_left_corner[0] + top_right_corner[0]) / 2
    rover_y = (bottom_left_corner[1] + top_right_corner[1]) / 2

    rover_x = center_x - rover_x
    rover_y = center_y - rover_y

    return rover_x, rover_y


def getImage(in_image):
    # cv_image is a cv2 image object. proceed forward with merging your code
    global bridge
    cv_image = bridge.imgmsg_to_cv2(in_image, "bgr8")

    rover_x, rover_y = get_aruco_center(cv_image, parameters, dictionary)

    vel_x, vel_y = controller.get_coordinates(rover_x, rover_y)

    print(rover_x, rover_y, vel_x, vel_y)

    send_vel_drone(vel_x, vel_y)

    return cv_image


def send_vel_drone(vel_x, vel_y):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
        0b0000111111000111,  # type_mask (only speeds enabled)
        0, 0, 0,  # x, y, z positions (not used)
        vel_x, vel_y, 0,  # x, y, z velocity in m/s
        # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0, 0,
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    # send command to vehicle on 1 Hz cycle
    vehicle.send_mavlink(msg)


def listener():
    rospy.init_node('listener', anonymous=True)
    rate = rospy.Rate(10)  # 10hz

    rospy.Subscriber("/webcam/image_raw", Image, getImage)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='Control Copter and send commands in GUIDED mode ')
    parser.add_argument('--connect',
                        help="Vehicle connection target string. If not specified, SITL automatically started and used.")
    args = parser.parse_args()

    connection_string = args.connect
    sitl = None

    if not connection_string:
        raise Exception("Conntection string not provided")

    # Connect to the Vehicle
    print('Connecting to vehicle on: %s' % connection_string)
    vehicle = connect(connection_string, wait_ready=True)

    # Arm and take of to altitude of 5 meters
    arm_and_takeoff(vehicle, 10)

    listener()
    rospy.spin()
