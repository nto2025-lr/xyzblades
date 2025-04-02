import rospy
from clover import srv
from std_srvs.srv import Trigger

import numpy as np
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import math

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, Point, Quaternion
import tf.transformations as t
import tf2_ros
import tf2_geometry_msgs

from geometry_msgs.msg import Pose, Point, Quaternion
import tf.transformations as t
import tf2_ros
import tf2_geometry_msgs

import time

import threading

from mavros_msgs.srv import CommandBool, CommandLong
from pymavlink import mavutil

from web_backend import web
import multiprocessing
import time
from random import randint, uniform
from colorama import Fore, Style


# def video_capture():


#     fourcc = cv2.VideoWriter_fourcc(*'XVID')  # Codec (e.g., XVID, MJPG, MP4V)
#     out = cv2.VideoWriter('output.avi', fourcc, 20.0, (640, 480))  # (filename, codec, fps, resolution)
        
#     out.write(frame)
               
 
#     out.release()



if __name__ == '__main__':
    multiprocessing.freeze_support()

    manager = multiprocessing.Manager()
    # video_capture()
    button_flags = manager.dict()
    marks = manager.list()
    voltage = manager.Value('d', 0.0)
    state = manager.Value('i', 0)

def main(button_flags, marks):
    x1, y1 = 3.5, 0.5
    x2, y2 = 0.5, 3

    # xc, yc = (x1 + x2)//2, (y1 + y2)//2 
    # xc2, yc2 = (x1 + xc)//2, (y1 + yc)//2
    # xf, yf = (x1 + xc2)//2, (y1 + yc2)//2
    # xf1, yf1 = ((xc + xc2) + (yc + yc2))//2
    x = []
    y = []
    for i in range(4):
        x.append(x1 + (x2 - x1) / 8 * i * 2 + 1)
        y.append(y1 + (y2 - y1) / 8 * i * 2 + 1)
        

    poly = []


    poly = []

    cords = []

    bridge = CvBridge()

    rospy.init_node('solvePnP')

    send_command = rospy.ServiceProxy('mavros/cmd/command', CommandLong)

    color_debug = rospy.Publisher("/color_debug", Image, queue_size=1)
    mask_debug = rospy.Publisher("/mask_debug", Image, queue_size=1)
    markers_arr_pub = rospy.Publisher("/l22_aero_color/markers_viz", MarkerArray)

    get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
    navigate = rospy.ServiceProxy('navigate', srv.Navigate)
    land = rospy.ServiceProxy('land', Trigger)


    # navigation with code blocking
    def navigate_wait(x=0, y=0, z=1, yaw=float('nan'), speed=0.5, frame_id='aruco_map', auto_arm=False, tolerance=0.2):
        if not button_flags['killswitch']:
            navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

        while not rospy.is_shutdown():
            if button_flags['killswitch']:
                send_command(command=mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, param1=0, param2=21196)
                break

            telem = get_telemetry(frame_id='navigate_target')
            if telem.x ** 2 + telem.y ** 2 + telem.z ** 2 < tolerance ** 2:
                break
            rospy.sleep(0.2)


    while True:
        print('while started')
        while not button_flags['start']:
            pass
        for i in range(len(marks)):
            marks.pop()
        print('armed')

        # Takeoff and flight to the central point
        navigate_wait(frame_id='body', auto_arm=True)
        telemetry = get_telemetry(frame_id='aruco_map')
        xs, ys = telemetry.x, telemetry.y
        # rospy.Subscriber('/main_camera/image_raw_throttled', Image, image_callback)

        # Flight
        for X, Y in zip(x, y):
            if button_flags['stop'] or button_flags['killswitch'] or button_flags['home'] or button_flags['land']:
                    break
            navigate_wait(x=x, y=y, z=0.5, frame_id='aruco_map')
            img = bridge.imgmsg_to_cv2(rospy.wait_for_message('thermal_camera/image_raw', Image), 'bgr8')
            contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            contours = [i for i in contours if cv2.contourArea(i) >25]

            if contours != []:
                cords.append((X,Y))
        
        if button_flags['stop']:
            while True:
                if button_flags['home'] or button_flags['land']:
                    break

        if button_flags['home']:
            navigate_wait(xs, ys, 1, frame_id='aruco_map')
            land()
        
        elif button_flags['land']:
            land()


        button_flags['start'] = False
        button_flags['stop'] = True
        button_flags['home'] = False
        button_flags['land'] = False


if __name__ == '__main__':
    button_flags["start"] = False
    button_flags["stop"] = True
    button_flags["killswitch"] = False
    button_flags["home"] = False
    button_flags["land"] = False
    
    main_thread = multiprocessing.Process(target=main, args=(button_flags, marks))
    main_thread.start()

    try:
        web.button_flags = button_flags
        web.marks = marks
        web.init_batt(voltage, state)

        web.app.run(host='192.168.50.50', port=5000, debug=False)
    except Exception as e:
        print(f'error: {e}')
    finally:
        main_thread.terminate()
        main_thread.join()
