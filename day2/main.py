import rospy
from clover import srv
from std_srvs.srv import Trigger

import numpy as np
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image, Range
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

import requests as rq

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
    response = rq.get("http://157.180.22.113:8000/coords_1.json")
    response = response.json()
    yellow = [[response['nodes'][0]['x'], response['nodes'][0]['y']], [response['nodes'][1]['x'], response['nodes'][1]['y']]]
    yellow_mid = [(yellow[1][0] - yellow[0][0]) / 2 + yellow[0][0], (yellow[1][1] - yellow[0][1]) / 2 + yellow[0][1]]
    yellow_points = response['tappings']
    K = (yellow[0][1] - yellow[0][0]) / (yellow[1][1] - yellow[1][0])
    k =(math.pi / 2 - math.atan(K))
    print(f'response: {response}')
    print(f'yellow: {yellow}')
    print(f'yellow_mid: {yellow_mid}')



    x1, y1 = 6.5, 3.5
    x2, y2 = 4, 0.5

    # xc, yc = (x1 + x2)//2, (y1 + y2)//2
    # xc2, yc2 = (x1 + xc)//2, (y1 + yc)//2
    # xf, yf = (x1 + xc2)//2, (y1 + yc2)//2
    # xf1, yf1 = ((xc + xc2) + (yc + yc2))//2
    x = []
    y = []

    thermal_lines = [[x2, y2, x2 + (x1 - x2) / 4, y2 + (y1 - y2) / 4, 'red'],
                     [x2 + (x1 - x2) / 4, y2 + (y1 - y2) / 4, x2 + (x1 - x2) / 4 * 2, y2 + (y1 - y2) / 4 * 2, 'red'],
                     [x2 + (x1 - x2) / 4 * 2, y2 + (y1 - y2) / 4 * 2, x2 + (x1 - x2) / 4 * 3, y2 + (y1 - y2) / 4 * 3, 'red'],
                     [x2 + (x1 - x2) / 4 * 3, y2 + (y1 - y2) / 4 * 3, x1, y1], 'red']

    print(f'thermal_lines: {thermal_lines}')

    for i in range(4):
        x.append((x1-x2) / 8 + x2 + 2 * i * (x1-x2) / 8)
        y.append((y1-y2) / 8 + y2 + 2 * i * (y1-y2) / 8)

    print(x, y)

    poly = []

    poly = []
    fin = []
    cords_points = []

    cords = []

    bridge = CvBridge()

    rospy.init_node('solvePnP')

    send_command = rospy.ServiceProxy('mavros/cmd/command', CommandLong)

    get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
    navigate = rospy.ServiceProxy('navigate', srv.Navigate)
    land = rospy.ServiceProxy('land', Trigger)

    debug_pub = rospy.Publisher('debug', Image, queue_size=1)

    # Global variables
    points = []
    poly_init = []
    cords_before_front = []
    input_data = [[603, 578], [450, 390], [500, 300]]

    # HSV color range for detection
    low = (0, 80, 135)
    high = (60, 255, 225)

    # low = (0, 47, 98)
    # high = (51, 255, 255)

    camera_matrix = np.array([[332.47884746146343 / 2, 0, 320.0 / 2],
                              [0, 333.1761847948052 / 2, 240.0 / 2],
                              [0, 0, 1]])

    distortion_coefficients = np.array([0.215356885, -0.117472846, -0.000306197672,
                                         -0.000109444025, -0.00453657258,
                                         0.573090623, -0.127574577, -0.0286125589])

    def initial_recognition():
        # global low, high, poly_init, cords_before_front

        # img = cv2.imread('Images/ntoshkapng.png', cv2.IMREAD_COLOR)
        img = rospy.wait_for_message('main_camera/image_raw', Image)

        telem = get_telemetry(frame_id="aruco_map")
        drone_x, drone_y, drone_h = telem.x, telem.y, telem.z

        img = CvBridge().imgmsg_to_cv2(img, 'bgr8')
        img = cv2.undistort(img, camera_matrix, distortion_coefficients)
        debug_img = img.copy()

        if img is None:
            print("Image not found!")
            return None

        height, width = img.shape[:2]
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(img_hsv, low, high)
        kernel = np.ones((5, 5), np.uint8)
        open_img = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(open_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = [i for i in contours if cv2.contourArea(i) > 200]

        if not contours:
            print("No contours found!")
            return None

        new = np.zeros_like(img)
        # rect = cv2.minAreaRect(contours[0])
        # box = cv2.boxPoints(rect)
        # print(box.tolist())
        # box = np.int32(box)
        # bo
        x, y, w, h = cv2.boundingRect(contours[0])
        cv2.rectangle(new, (x, y), (x + w, y + h), (0, 0, 255), 2)

        for cnt in contours:
            approx = cv2.approxPolyDP(cnt, 0.005 * cv2.arcLength(cnt, True), True)
            poly_init.append(approx)
            cv2.drawContours(new, [approx], -1, (0, 255, 0), thickness=2)

        for point in approx:
            px, py = point[0]
            if (width * 0.2) < px < (width * 0.8):
                if (abs(px - x) < 20 or abs(px - (x + w)) < 20 or
                        abs(py - y) < 20 or abs(py - (y + h)) < 20):
                    cords_before_front.append([int(px), int(py)])

        for point in cords_before_front:
            cv2.circle(new, tuple(point), 5, (255, 0, 0), 3)

        print("Edge points (excluding top and bottom):", cords_before_front)
        return new

    def px2cam(points):
        h = rospy.wait_for_message('/rangefinder/range', Range).range
        tel = get_telemetry(frame_id="aruco_map")
        drone_x, drone_y, drone_h = tel.x, tel.y, tel.z
        fx = camera_matrix[0][0]
        fy = camera_matrix[1][1]
        CX = camera_matrix[0][2]
        CY = camera_matrix[1][2]
        X = drone_x + (points[0] - CX) / fx * h
        Y = drone_y - (points[1] - CY) / fy * h

        return (X, Y)


    def distance_calculate(p1, p2):
        return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

    def distance_calculate_req(p1, p2):
        return math.sqrt((p1[0] - p2['x']) ** 2 + (p1[1] - p2['y']) ** 2)





    def recognition_stage_three(cords_before_tf, input_data, img):
        if img is None:
            print("No image provided!")
            return None

        if not cords_before_tf:
            print("No edge points found!")
            return img

        for p2 in yellow_mid:
            min_dist = float('inf')
            closest_point = None

            for p in cords_before_tf:
                dist = distance_calculate(p, p2)
                if dist < min_dist:
                    min_dist = dist
                    closest_point = p
                    points.append(closest_point)

            if closest_point is not None:
                cv2.line(img, (p2[0], p2[1]), tuple(closest_point), (255, 0, 0), 3)

        return img
    # navigation with code blocking
    xs, ys = 0, 0
    def navigate_wait(x=0, y=0, z=1, yaw=float('nan'), speed=0.5, frame_id='aruco_map', auto_arm=False, tolerance=0.2):
        if not button_flags['killswitch'] and not ((button_flags['stop'] or button_flags['home']) and x==xs and y==ys) and not button_flags['land']:
            navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

        while not rospy.is_shutdown():
            if button_flags['killswitch']:
                send_command(command=mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, param1=0, param2=21196)
                break
            if button_flags['stop'] and (x!=xs or x!=ys):
                break
            if button_flags['home'] and (x!=xs or x!=ys):
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
        # marks.append([0, 0, 5, 3])
        # Takeoff and flight to the central point
        navigate_wait(frame_id='body', auto_arm=True)
        telemetry = get_telemetry(frame_id='aruco_map')
        xs, ys = telemetry.x, telemetry.y
        # rospy.Subscriber('/main_camera/image_raw_throttled', Image, image_callback)

        # # Flight
        number_of_line = 0
        for X, Y in zip(x, y):
            if button_flags['stop'] or button_flags['killswitch'] or button_flags['home'] or button_flags['land']:
                break
            navigate_wait(x=X, y=Y, z=1, frame_id='aruco_map')

            img = bridge.imgmsg_to_cv2(rospy.wait_for_message('thermal_camera/image_raw', Image), 'bgr8')
            img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            _, thresh = cv2.threshold(img,125,255,cv2.THRESH_BINARY)
            thresh_to_publish = cv2.cvtColor(thresh, cv2.COLOR_GRAY2BGR)
            debug_pub.publish(CvBridge().cv2_to_imgmsg(thresh_to_publish, 'bgr8'))

            contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            contours = [i for i in contours if cv2.contourArea(i) > 30]

            if contours != []:
                moments = cv2.moments(thresh)
                x = int(moments["m10"] / moments["m00"])
                y = int(moments["m01"] / moments["m00"])
                dis = distance_calculate([x, y], [159, 119])
                if dis < 100:
                    cords.append((X, Y))
                    marks.append(thermal_lines[number_of_line])

        print(cords)
        #
        #
        navigate_wait(x=4, y=0, z=1, frame_id='aruco_map')
        navigate_wait(yellow_mid[0], yellow_mid[1], z=1.7, frame_id='aruco_map')
        navigate_wait(0, 0, 0, yaw=k, frame_id='body')

        in_im = initial_recognition()
        if in_im is not None and cords_before_front:
            res = recognition_stage_three(cords_before_front, yellow_mid, in_im.copy())
            # cv2.imshow("Result", res)
            debug_pub.publish(CvBridge().cv2_to_imgmsg(res, 'bgr8'))
            for i in points:
                cords_in = px2cam(i)
                cords_points.append(cords_in)
            for count_p2 in yellow_mid:
                for j in cords_points:
                    dis2 = distance_calculate_req(j, count_p2)
                    min_dist = float('inf')
                    if dis2 < min_dist:
                        min_dist = dis2
                        res = j
                        fin.append([res, (count_p2['x'], count_p2['y'])])

            for i in fin:
                marks.append(i)






        elif in_im is not None:
            # cv2.imshow("Initial Detection", in_im)
            debug_pub.publish(CvBridge().cv2_to_imgmsg(in_im, 'bgr8'))
        else:
            print("No results to display")
        # Wait for 5 seconds


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
