#импорты библиотек
from flask import Flask, render_template, jsonify
import rospy
from sensor_msgs.msg import BatteryState
from mavros_msgs.msg import State

import threading

import logging
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)

#класс веб-приложения
class Web:
    def __init__(self):
        self.app = Flask(__name__)
        self.button_flags = None
        self.marks = None
        self.voltage = None
        self.state = None
    #инициализация рос-ноды
    def init_ros_flask(self):
        rospy.init_node('web_interface', anonymous=True, disable_signals=True)
        rospy.spin()

    def init_batt(self, volt, sta):
        threading.Thread(target=self.init_ros_flask, daemon=True).start()
        self.voltage = volt
        self.state = sta
        self.batt_sub = rospy.Subscriber('/mavros/battery', BatteryState, self.batterycallback)
        self.state_sub = rospy.Subscriber('mavros/state', State, self.statecallback)
    #функция для считывания батареи
    def batterycallback(self, msg):
        self.voltage.value = round(msg.voltage, 2)

    def statecallback(self, msg):
        self.state.value = msg.system_status
    #маршруты Flask
    def create_routes(self):
        @self.app.route('/')
        def onstart():
            print('app started')
            return render_template('index.html')

        @self.app.route('/start_handle')
        def start_pressed():
            self.button_flags["start"] = True
            self.button_flags["stop"] = False
            return jsonify({'message': 'start handled'})

        @self.app.route('/stop_handle')
        def stop_pressed():
            self.button_flags["start"] = False
            self.button_flags["stop"] = True
            return jsonify({'message': 'stop handled'})

        @self.app.route('/kill_handle')
        def kill_pressed():
            self.button_flags["killswitch"] = not self.button_flags["killswitch"]
            return jsonify({'message': 'kill handled'})

        @self.app.route('/home_handle')
        def home_pressed():
            self.button_flags["home"] = True
            return jsonify({'message': 'home handled'})

        @self.app.route('/land_handle')
        def land_pressed():
            self.button_flags["land"] = True
            return jsonify({'message': 'land handled'})

        @self.app.route('/get_marks')
        def send_marks():
            # print('received request, sent' + str(self.marks))
            return jsonify({'marks': list(self.marks)})

        @self.app.route('/get_voltage')
        def send_voltage():
            # print(f"Voltage: {self.voltage.value} V")
            # print(self.state.value)
            if self.state.value == 3:
                stateZ = 'Ready'
            elif self.state.value == 4:
                stateZ = 'Flying'
            else:
                stateZ = 'Not ready'
            return jsonify({'voltage': str(self.voltage.value), 'state': stateZ})


web = Web()
web.create_routes()
