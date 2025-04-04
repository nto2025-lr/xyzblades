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

import socket

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
    headers = {"Authorization": "Bearer e7dd0527-df2e-44ac-aa01-c436d76daf13"}
    response = rq.get(url="http://157.180.22.113:8080/coords_1.json", headers=headers)
    response = response.json()
    yellow = [[response['nodes'][0]['x'], response['nodes'][0]['y']],
              [response['nodes'][1]['x'], response['nodes'][1]['y']]]
    yellow_mid = [(yellow[1][0] - yellow[0][0]) / 2 + yellow[0][0], (yellow[1][1] - yellow[0][1]) / 2 + yellow[0][1]]
    yellow_points = response['tappings']
    yellow_illegal = [i for i in response['tappings'] if i['legal'] == 0]
    yellow_illegal_coords = []

    print(f'response: {response}')
    print(f'yellow: {yellow}')
    print(f'yellow_mid: {yellow_mid}')
    print(f'yellow_illegal: {yellow_illegal}')

    for i in yellow_illegal:
        yellow_illegal_coords.append([i['x'], i['y']])
    # определение углового коэффицента нефтепровода для поворота дрона
    K = (yellow[0][1] - yellow[0][0]) / (yellow[1][1] - yellow[1][0])
    k = (math.pi / 2 - math.atan(K))

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
                     [x2 + (x1 - x2) / 4 * 2, y2 + (y1 - y2) / 4 * 2, x2 + (x1 - x2) / 4 * 3, y2 + (y1 - y2) / 4 * 3,
                      'red'],
                     [x2 + (x1 - x2) / 4 * 3, y2 + (y1 - y2) / 4 * 3, x1, y1, 'red']]

    print(f'thermal_lines: {thermal_lines}')

    for i in range(4):
        x.append((x1 - x2) / 8 + x2 + 2 * i * (x1 - x2) / 8)
        y.append((y1 - y2) / 8 + y2 + 2 * i * (y1 - y2) / 8)

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
    low = (0, 66, 134)
    high = (39, 255, 225)

    # low = (0, 47, 98)
    # high = (51, 255, 255)

    # параметры камеры
    camera_matrix = np.array([[332.47884746146343 / 2, 0, 320.0 / 2],
                              [0, 333.1761847948052 / 2, 240.0 / 2],
                              [0, 0, 1]])

    distortion_coefficients = np.array([0.215356885, -0.117472846, -0.000306197672,
                                        -0.000109444025, -0.00453657258,
                                        0.573090623, -0.127574577, -0.0286125589])

    # def px2cam(points):
    #     h = rospy.wait_for_message('/rangefinder/range', Range).range
    #     tel = get_telemetry(frame_id="aruco_map")
    #     drone_x, drone_y, drone_h = tel.x, tel.y, tel.z
    #     fx = camera_matrix[0][0]
    #     fy = camera_matrix[1][1]
    #     CX = camera_matrix[0][2]
    #     CY = camera_matrix[1][2]
    #     X = drone_x + (points[0] - CX) / fx * h
    #     Y = drone_y - (points[1] - CY) / fy * h
    #
    #     return (X, Y)

    # расчет евклидова расстояния
    def distance_calculate(p1, p2):
        return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

    # navigation with code blocking
    xs, ys = 0, 0

    def navigate_wait(x=0, y=0, z=1, yaw=float('nan'), speed=0.5, frame_id='aruco_map', auto_arm=False, tolerance=0.2):
        if button_flags['home']:
            navigate(x=xs, y=ys, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)
        elif not button_flags['killswitch'] and not button_flags['land']:
            navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

        while not rospy.is_shutdown():
            if button_flags['killswitch']:
                send_command(command=mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, param1=0, param2=21196)
                break
            if button_flags['home']:
                pass
            else:
                if button_flags['stop'] and (x != xs or x != ys):
                    break

            telem = get_telemetry(frame_id='navigate_target')
            if telem.x ** 2 + telem.y ** 2 + telem.z ** 2 < tolerance ** 2:
                break
            rospy.sleep(0.2)

    flag_video_camera = True

    # запись видео с камеры
    def record_video_camera():
        fourcc = cv2.VideoWriter_fourcc(*'MJPG')  # Codec (e.g., XVID, MJPG, MP4V)
        out = cv2.VideoWriter('video_camera.avi', fourcc, 20.0, (320, 240))
        while flag_video_camera:
            img = bridge.imgmsg_to_cv2(rospy.wait_for_message('main_camera/image_raw', Image), 'bgr8')
            out.write(img)
        out.release()

    flag_video_thermal = True

    # запись видео с тепловизора
    def record_video_thermal():
        fourcc = cv2.VideoWriter_fourcc(*'MJPG')  # Codec (e.g., XVID, MJPG, MP4V)
        out2 = cv2.VideoWriter('video_thermal.avi', fourcc, 20.0, (256, 192))
        while flag_video_thermal:
            img = bridge.imgmsg_to_cv2(rospy.wait_for_message('thermal_camera/image_raw', Image), 'bgr8')
            out2.write(img)
        out2.release()

    # отправка сообщения на есп32 (на дронопорт)
    def send_message_to_esp32(message, esp32_ip, esp32_port):
        try:

            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(5)
            sock.connect((esp32_ip, esp32_port))
            sock.sendall(message.encode())
            print(f"message sent: {message}")
            response = sock.recv(1024)
            if response:
                print(f"response ESP32: {response.decode()}")

        except Exception as e:
            print(f"Fail: {e}")
        finally:
            sock.close()

    # основная часть полета
    while True:
        print('while started')
        while not button_flags['start']:
            pass
        for i in range(len(marks)):
            marks.pop()
        print('arming')
        send_message_to_esp32('open', "192.168.50.211", 33300)
        rospy.sleep(10)
        print('droneport open')
        # Takeoff and flight to the central point
        navigate_wait(frame_id='body', auto_arm=True)
        send_message_to_esp32('close', "192.168.50.211", 33300)
        telemetry = get_telemetry(frame_id='aruco_map')
        xs, ys = telemetry.x, telemetry.y
        # rospy.Subscriber('/main_camera/image_raw_throttled', Image, image_callback)

        # # Flight
        number_of_line = 0
        navigate_wait(x=3, y=0, z=1, frame_id='aruco_map')
        rospy.sleep(3)
        navigate_wait(x=x[0], y=y[0], z=1, frame_id='aruco_map')
        rospy.sleep(5)
        threading.Thread(target=record_video_camera, daemon=True).start()
        threading.Thread(target=record_video_thermal, daemon=True).start()

        # полет по линии с тепловизором
        for X, Y in zip(x, y):
            if button_flags['stop'] or button_flags['killswitch'] or button_flags['home'] or button_flags['land']:
                break
            navigate_wait(x=X, y=Y, z=1, frame_id='aruco_map')

            # обработка изображения для алгоритма определения теплопотерь
            img = bridge.imgmsg_to_cv2(rospy.wait_for_message('thermal_camera/image_raw', Image), 'bgr8')
            img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            _, thresh = cv2.threshold(img, 125, 255, cv2.THRESH_BINARY)
            thresh_to_publish = cv2.cvtColor(thresh, cv2.COLOR_GRAY2BGR)
            debug_pub.publish(CvBridge().cv2_to_imgmsg(thresh_to_publish, 'bgr8'))

            contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            contours = [i for i in contours if cv2.contourArea(i) > 10]

            if contours != []:
                moments = cv2.moments(thresh)
                x = int(moments["m10"] / moments["m00"])
                y = int(moments["m01"] / moments["m00"])
                dis = distance_calculate([x, y], [159, 119])
                if dis < 100:
                    cords.append((X, Y))
                    marks.append(thermal_lines[number_of_line])
                    print(f'Thermal 1: {thermal_lines[number_of_line]}')
            number_of_line += 1

        print(cords)
        flag_video_camera = False
        flag_video_thermal = False

        navigate_wait(x=4, y=0, z=1.5, speed=0.3, frame_id='aruco_map')
        rospy.sleep(2)
        # navigate_wait(yellow_mid[0], yellow_mid[1], z=1.7, frame_id='aruco_map')
        
        # поворот параллельно нефтепроводу
        navigate_wait(0, 0, 0, yaw=k, frame_id='body')

        rospy.sleep(4)

        #полет по нефтепроводу
        for i in yellow_illegal_coords:
            if button_flags['stop'] or button_flags['killswitch'] or button_flags['home'] or button_flags['land']:
                break
            rospy.sleep(1)
            navigate_wait(x=i[0], y=i[1], z=1, speed=0.3, frame_id='aruco_map')
            rospy.sleep(1)
            #обработка изображения для отсечения центральной линии нефтепровода, а также для определения врезок
            img = bridge.imgmsg_to_cv2(rospy.wait_for_message('main_camera/image_raw', Image), 'bgr8')
            img = cv2.undistort(img, camera_matrix, distortion_coefficients)

            img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(img_hsv, low, high)
            kernel = np.ones((5, 5), np.uint8)
            open_img = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

            # new = np.zeros_like(img)

            # new = cv2.cvtColor(new, cv2.COLOR_GRAY2BGR)

            # отсечение центральной линии нефтепровода
            x = 40
            cv2.rectangle(open_img, (159 - x, 0), (159 + x, 239), (0, 0, 0), thickness=cv2.FILLED)
            contours, _ = cv2.findContours(open_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            contours = [i for i in contours if cv2.contourArea(i) > 50]

            points = []
            # поиск ближайшего контура к центру камеры, если такие есть
            if len(contours) >= 2:
                min_dist = float('inf')
                closest_cnt = None
                for j in range(len(contours) - 1):
                    for h in contours[j]:
                        dist = distance_calculate(h, [158, 119])
                        if dist < min_dist:
                            closest_cnt = j

                points.append(contours[closest_cnt])

            elif len(contours) == 1:
                points.append(contours[0])

            # mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

            # сохранение изобраение в директорию дрона и дебаг изображения в топик /debug
            if contours != []:
                cv2.rectangle(img, (159 - x, 239), (159 + x, 0), (255, 255, 255), thickness=cv2.FILLED)
                cv2.drawContours(img, points, -1, (0, 0, 255), thickness=cv2.FILLED)

                debug_pub.publish(CvBridge().cv2_to_imgmsg(img, 'bgr8'))
                count_vrezka = 0
                cv2.imwrite(f'vrezka{count_vrezka}.png', img)
                print("wrote image")
                count_vrezka += 1

        if button_flags['stop']:
            while True:
                if button_flags['home'] or button_flags['land']:
                    break

        if button_flags['home']:
            navigate_wait(xs, ys, 1, frame_id='aruco_map')
            send_message_to_esp32('open', "192.168.50.211", 33300)
            rospy.sleep(10)
            land()
            send_message_to_esp32('close', "192.168.50.211", 33300)

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
