#!/home/pi/spotmicroai/venv/bin/python3 -u

#import busio
# from board import SCL, SDA
# from adafruit_pca9685 import PCA9685
# from adafruit_motor import servo
from pick import pick
import time
import os
import sys
#import RPi.GPIO as GPIO

from spotmicroai.utilities.log import Logger
from spotmicroai.utilities.config import Config
from STservo_sdk import *

log = Logger().setup_logger('CALIBRATE SERVOS')

log.info('Calibrate rest position...')

packet_handler = None
port_handler = None
STS_MOVING_SPEED = 2400
STS_MOVING_ACC = 50




while True:
    options = {
        0: 'rear_shoulder_left   - BUS_SERVO_ADAPTER[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_LEFT_BUS_SERVO_ADAPTER)) + '] ID[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_LEFT_ID)) + ']  - ANGLE[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_LEFT_REST_ANGLE)) + ']',
        1: 'rear_leg_left        - BUS_SERVO_ADAPTER[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_LEG_LEFT_BUS_SERVO_ADAPTER)) + '] ID[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_LEG_LEFT_ID)) + ']  - ANGLE[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_LEG_LEFT_REST_ANGLE)) + ']',
        2: 'rear_feet_left       - BUS_SERVO_ADAPTER[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_FEET_LEFT_BUS_SERVO_ADAPTER)) + '] ID[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_FEET_LEFT_ID)) + ']  - ANGLE[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_FEET_LEFT_REST_ANGLE)) + ']',
        3: 'rear_shoulder_right  - BUS_SERVO_ADAPTER[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_RIGHT_BUS_SERVO_ADAPTER)) + '] ID[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_RIGHT_ID)) + ']  - ANGLE[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_RIGHT_REST_ANGLE)) + ']',
        4: 'rear_leg_right       - BUS_SERVO_ADAPTER[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_LEG_RIGHT_BUS_SERVO_ADAPTER)) + '] ID[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_LEG_RIGHT_ID)) + ']  - ANGLE[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_LEG_RIGHT_REST_ANGLE)) + ']',
        5: 'rear_feet_right      - BUS_SERVO_ADAPTER[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_FEET_RIGHT_BUS_SERVO_ADAPTER)) + '] ID[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_FEET_RIGHT_ID)) + ']  - ANGLE[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_FEET_RIGHT_REST_ANGLE)) + ']',
        6: 'front_shoulder_left  - BUS_SERVO_ADAPTER[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_LEFT_BUS_SERVO_ADAPTER)) + '] ID[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_LEFT_ID)) + ']  - ANGLE[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_LEFT_REST_ANGLE)) + ']',
        7: 'front_leg_left       - BUS_SERVO_ADAPTER[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_LEFT_BUS_SERVO_ADAPTER)) + '] ID[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_LEFT_ID)) + ']  - ANGLE[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_LEFT_REST_ANGLE)) + ']',
        8: 'front_feet_left      - BUS_SERVO_ADAPTER[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_FEET_LEFT_BUS_SERVO_ADAPTER)) + '] ID[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_FEET_LEFT_ID)) + ']  - ANGLE[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_FEET_LEFT_REST_ANGLE)) + ']',
        9: 'front_shoulder_right - BUS_SERVO_ADAPTER[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_RIGHT_BUS_SERVO_ADAPTER)) + '] ID[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_RIGHT_ID)) + ']  - ANGLE[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_RIGHT_REST_ANGLE)) + ']',
        10: 'front_leg_right      - BUS_SERVO_ADAPTER[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_RIGHT_BUS_SERVO_ADAPTER)) + '] ID[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_RIGHT_ID)) + '] - ANGLE[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_RIGHT_REST_ANGLE)) + ']',
        11: 'front_feet_right     - BUS_SERVO_ADAPTER[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_FEET_RIGHT_BUS_SERVO_ADAPTER)) + '] ID[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_FEET_RIGHT_ID)) + '] - ANGLE[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_FEET_RIGHT_REST_ANGLE)) + ']'}

    title = 'The folder integration_tests for more tests' + os.linesep + \
            '1. Follow the Sevo_Controller_Setup.md' + os.linesep + \
            '2. Write your devicenames and settings in your configuration file ~/spotmicroai.json' + os.linesep + \
            '3. if no angle is specified 90 will be the default position, for example if you just press Enter' + os.linesep + \
            '' + os.linesep + \
            'Write "menu" or "m" and press Enter to return to the list of servos' + os.linesep + \
            'Write "exit" or "e" and press Enter to exit' + os.linesep + \
            '' + os.linesep + \
            'Please choose the servo to calibrate its rest position: '

    screen_options = list(options.values())

    selected_option, selected_index = pick(screen_options, title)

    BUS_SERVP_ADAPTER_SERIAL_PORT, BUS_SERVP_ADAPTER_BAUDRATE , ID, MIN_ANGLE, MAX_ANGLE, REST_ANGLE = Config().get_by_section_name(selected_option.split()[0])

    while True:

        try:
            user_input = input("Write the angle and press Enter, or press Enter for 90: ")

            port_handler = PortHandler(BUS_SERVP_ADAPTER_SERIAL_PORT)
            packet_handler = sts(port_handler)

            if port_handler.openPort():
               print("Succeeded to open the port")
            else:
                print("Failed to open the port")

            if port_handler.setBaudRate(BUS_SERVP_ADAPTER_BAUDRATE):
                print("Succeeded to change the baudrate")
            else:    
                print("Failed to change the baudrate")


            if user_input == 'menu' or user_input == 'm':
                break
            if user_input == 'exit' or user_input == 'e':
                sys.exit(0)
            elif user_input == '':
                user_input = 90

            sts_addparam_result = packet_handler.SyncWritePosEx(ID, int(user_input), STS_MOVING_SPEED, STS_MOVING_ACC)
            if sts_addparam_result != True:
                print("[ID:%03d] groupSyncWrite addparam failed" % ID)

            sts_comm_result = packet_handler.groupSyncWrite.txPacket()
            if sts_comm_result != COMM_SUCCESS:
                print("%s" % packet_handler.getTxRxResult(sts_comm_result))
            time.sleep(0.1)
        finally:
            port_handler.closePort()
