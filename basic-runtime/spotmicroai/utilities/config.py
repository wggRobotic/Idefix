import json
import sys
import os
from spotmicroai.utilities.log import Logger
import jmespath  # http://jmespath.org/tutorial.html
import shutil
from pathlib import Path

log = Logger().setup_logger('Configuration')


class Singleton(type):
    _instances = {}

    def __call__(cls, *args, **kwargs):
        if cls not in cls._instances:
            cls._instances[cls] = super(Singleton, cls).__call__(*args, **kwargs)
        return cls._instances[cls]


class Config(metaclass=Singleton):
    ABORT_CONTROLLER_GPIO_PORT = 'abort_controller[0].gpio_port'
    LCD_SCREEN_CONTROLLER_I2C_ADDRESS = 'lcd_screen_controller[0].lcd_screen[0].address'
    REMOTE_CONTROLLER_CONTROLLER_DEVICE = 'remote_controller_controller[0].remote_controller[0].device'

    MOTION_CONTROLLER_BOARDS_bus_servo_adapter_1_SERIAL_PORT = 'motion_controller[*].boards[*].bus_servo_adapter_1[*].serial_port | [0] | [0] | [0]'
    MOTION_CONTROLLER_BOARDS_bus_servo_adapter_1_BAUDRATE = 'motion_controller[*].boards[*].bus_servo_adapter_1[*].baudrate | [0] | [0] | [0]'
    MOTION_CONTROLLER_BOARDS_bus_servo_adapter_2_SERIAL_PORT = 'motion_controller[*].boards[*].bus_servo_adapter_2[*].serial_port | [0] | [0] | [0]'
    MOTION_CONTROLLER_BOARDS_bus_servo_adapter_2_BAUDRATE = 'motion_controller[*].boards[*].bus_servo_adapter_2[*].baudrate | [0] | [0] | [0]'

    MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_LEFT_BUS_SERVO_ADAPTER = 'motion_controller[*].servos[*].rear_shoulder_left[*].bus_servo_adapter | [0] | [0] | [0]'
    MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_LEFT_ID = 'motion_controller[*].servos[*].rear_shoulder_left[*].id | [0] | [0] | [0]'
    MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_LEFT_MIN_ANGLE = 'motion_controller[*].servos[*].rear_shoulder_left[*].min_angle | [0] | [0] | [0]'
    MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_LEFT_MAX_ANGLE = 'motion_controller[*].servos[*].rear_shoulder_left[*].max_angle | [0] | [0] | [0]'
    MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_LEFT_REST_ANGLE = 'motion_controller[*].servos[*].rear_shoulder_left[*].rest_angle | [0] | [0] | [0]'

    MOTION_CONTROLLER_SERVOS_REAR_LEG_LEFT_BUS_SERVO_ADAPTER = 'motion_controller[*].servos[*].rear_leg_left[*].bus_servo_adapter | [0] | [0] | [0]'
    MOTION_CONTROLLER_SERVOS_REAR_LEG_LEFT_ID = 'motion_controller[*].servos[*].rear_leg_left[*].id | [0] | [0] | [0]'
    MOTION_CONTROLLER_SERVOS_REAR_LEG_LEFT_MIN_ANGLE = 'motion_controller[*].servos[*].rear_leg_left[*].min_angle | [0] | [0] | [0]'
    MOTION_CONTROLLER_SERVOS_REAR_LEG_LEFT_MAX_ANGLE = 'motion_controller[*].servos[*].rear_leg_left[*].max_angle | [0] | [0] | [0]'
    MOTION_CONTROLLER_SERVOS_REAR_LEG_LEFT_REST_ANGLE = 'motion_controller[*].servos[*].rear_leg_left[*].rest_angle | [0] | [0] | [0]'

    MOTION_CONTROLLER_SERVOS_REAR_FEET_LEFT_BUS_SERVO_ADAPTER = 'motion_controller[*].servos[*].rear_feet_left[*].bus_servo_adapter | [0] | [0] | [0]'
    MOTION_CONTROLLER_SERVOS_REAR_FEET_LEFT_ID = 'motion_controller[*].servos[*].rear_feet_left[*].id | [0] | [0] | [0]'
    MOTION_CONTROLLER_SERVOS_REAR_FEET_LEFT_MIN_ANGLE = 'motion_controller[*].servos[*].rear_feet_left[*].min_angle | [0] | [0] | [0]'
    MOTION_CONTROLLER_SERVOS_REAR_FEET_LEFT_MAX_ANGLE = 'motion_controller[*].servos[*].rear_feet_left[*].max_angle | [0] | [0] | [0]'
    MOTION_CONTROLLER_SERVOS_REAR_FEET_LEFT_REST_ANGLE = 'motion_controller[*].servos[*].rear_feet_left[*].rest_angle | [0] | [0] | [0]'

    MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_RIGHT_BUS_SERVO_ADAPTER = 'motion_controller[*].servos[*].rear_shoulder_right[*].bus_servo_adapter | [0] | [0] | [0]'
    MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_RIGHT_ID = 'motion_controller[*].servos[*].rear_shoulder_right[*].id | [0] | [0] | [0]'
    MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_RIGHT_MIN_ANGLE = 'motion_controller[*].servos[*].rear_shoulder_right[*].min_angle | [0] | [0] | [0]'
    MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_RIGHT_MAX_ANGLE = 'motion_controller[*].servos[*].rear_shoulder_right[*].max_angle | [0] | [0] | [0]'
    MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_RIGHT_REST_ANGLE = 'motion_controller[*].servos[*].rear_shoulder_right[*].rest_angle | [0] | [0] | [0]'

    MOTION_CONTROLLER_SERVOS_REAR_LEG_RIGHT_BUS_SERVO_ADAPTER = 'motion_controller[*].servos[*].rear_leg_right[*].bus_servo_adapter | [0] | [0] | [0]'
    MOTION_CONTROLLER_SERVOS_REAR_LEG_RIGHT_ID = 'motion_controller[*].servos[*].rear_leg_right[*].id | [0] | [0] | [0]'
    MOTION_CONTROLLER_SERVOS_REAR_LEG_RIGHT_MIN_ANGLE = 'motion_controller[*].servos[*].rear_leg_right[*].min_angle | [0] | [0] | [0]'
    MOTION_CONTROLLER_SERVOS_REAR_LEG_RIGHT_MAX_ANGLE = 'motion_controller[*].servos[*].rear_leg_right[*].max_angle | [0] | [0] | [0]'
    MOTION_CONTROLLER_SERVOS_REAR_LEG_RIGHT_REST_ANGLE = 'motion_controller[*].servos[*].rear_leg_right[*].rest_angle | [0] | [0] | [0]'

    MOTION_CONTROLLER_SERVOS_REAR_FEET_RIGHT_BUS_SERVO_ADAPTER = 'motion_controller[*].servos[*].rear_feet_right[*].bus_servo_adapter | [0] | [0] | [0]'
    MOTION_CONTROLLER_SERVOS_REAR_FEET_RIGHT_ID = 'motion_controller[*].servos[*].rear_feet_right[*].id | [0] | [0] | [0]'
    MOTION_CONTROLLER_SERVOS_REAR_FEET_RIGHT_MIN_ANGLE = 'motion_controller[*].servos[*].rear_feet_right[*].min_angle | [0] | [0] | [0]'
    MOTION_CONTROLLER_SERVOS_REAR_FEET_RIGHT_MAX_ANGLE = 'motion_controller[*].servos[*].rear_feet_right[*].max_angle | [0] | [0] | [0]'
    MOTION_CONTROLLER_SERVOS_REAR_FEET_RIGHT_REST_ANGLE = 'motion_controller[*].servos[*].rear_feet_right[*].rest_angle | [0] | [0] | [0]'

    MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_LEFT_BUS_SERVO_ADAPTER = 'motion_controller[*].servos[*].front_shoulder_left[*].bus_servo_adapter | [0] | [0] | [0]'
    MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_LEFT_ID = 'motion_controller[*].servos[*].front_shoulder_left[*].id | [0] | [0] | [0]'
    MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_LEFT_MIN_ANGLE = 'motion_controller[*].servos[*].front_shoulder_left[*].min_angle | [0] | [0] | [0]'
    MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_LEFT_MAX_ANGLE = 'motion_controller[*].servos[*].front_shoulder_left[*].max_angle | [0] | [0] | [0]'
    MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_LEFT_REST_ANGLE = 'motion_controller[*].servos[*].front_shoulder_left[*].rest_angle | [0] | [0] | [0]'

    MOTION_CONTROLLER_SERVOS_FRONT_LEG_LEFT_BUS_SERVO_ADAPTER = 'motion_controller[*].servos[*].front_leg_left[*].bus_servo_adapter | [0] | [0] | [0]'
    MOTION_CONTROLLER_SERVOS_FRONT_LEG_LEFT_ID = 'motion_controller[*].servos[*].front_leg_left[*].id | [0] | [0] | [0]'
    MOTION_CONTROLLER_SERVOS_FRONT_LEG_LEFT_MIN_ANGLE = 'motion_controller[*].servos[*].front_leg_left[*].min_angle | [0] | [0] | [0]'
    MOTION_CONTROLLER_SERVOS_FRONT_LEG_LEFT_MAX_ANGLE = 'motion_controller[*].servos[*].front_leg_left[*].max_angle | [0] | [0] | [0]'
    MOTION_CONTROLLER_SERVOS_FRONT_LEG_LEFT_REST_ANGLE = 'motion_controller[*].servos[*].front_leg_left[*].rest_angle | [0] | [0] | [0]'

    MOTION_CONTROLLER_SERVOS_FRONT_FEET_LEFT_BUS_SERVO_ADAPTER = 'motion_controller[*].servos[*].front_feet_left[*].bus_servo_adapter | [0] | [0] | [0]'
    MOTION_CONTROLLER_SERVOS_FRONT_FEET_LEFT_ID = 'motion_controller[*].servos[*].front_feet_left[*].id | [0] | [0] | [0]'
    MOTION_CONTROLLER_SERVOS_FRONT_FEET_LEFT_MIN_ANGLE = 'motion_controller[*].servos[*].front_feet_left[*].min_angle | [0] | [0] | [0]'
    MOTION_CONTROLLER_SERVOS_FRONT_FEET_LEFT_MAX_ANGLE = 'motion_controller[*].servos[*].front_feet_left[*].max_angle | [0] | [0] | [0]'
    MOTION_CONTROLLER_SERVOS_FRONT_FEET_LEFT_REST_ANGLE = 'motion_controller[*].servos[*].front_feet_left[*].rest_angle | [0] | [0] | [0]'

    MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_RIGHT_BUS_SERVO_ADAPTER = 'motion_controller[*].servos[*].front_shoulder_right[*].bus_servo_adapter | [0] | [0] | [0]'
    MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_RIGHT_ID = 'motion_controller[*].servos[*].front_shoulder_right[*].id | [0] | [0] | [0]'
    MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_RIGHT_MIN_ANGLE = 'motion_controller[*].servos[*].front_shoulder_right[*].min_angle | [0] | [0] | [0]'
    MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_RIGHT_MAX_ANGLE = 'motion_controller[*].servos[*].front_shoulder_right[*].max_angle | [0] | [0] | [0]'
    MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_RIGHT_REST_ANGLE = 'motion_controller[*].servos[*].front_shoulder_right[*].rest_angle | [0] | [0] | [0]'

    MOTION_CONTROLLER_SERVOS_FRONT_LEG_RIGHT_BUS_SERVO_ADAPTER = 'motion_controller[*].servos[*].front_leg_right[*].bus_servo_adapter | [0] | [0] | [0]'
    MOTION_CONTROLLER_SERVOS_FRONT_LEG_RIGHT_ID = 'motion_controller[*].servos[*].front_leg_right[*].id | [0] | [0] | [0]'
    MOTION_CONTROLLER_SERVOS_FRONT_LEG_RIGHT_MIN_ANGLE = 'motion_controller[*].servos[*].front_leg_right[*].min_angle | [0] | [0] | [0]'
    MOTION_CONTROLLER_SERVOS_FRONT_LEG_RIGHT_MAX_ANGLE = 'motion_controller[*].servos[*].front_leg_right[*].max_angle | [0] | [0] | [0]'
    MOTION_CONTROLLER_SERVOS_FRONT_LEG_RIGHT_REST_ANGLE = 'motion_controller[*].servos[*].front_leg_right[*].rest_angle | [0] | [0] | [0]'

    MOTION_CONTROLLER_SERVOS_FRONT_FEET_RIGHT_BUS_SERVO_ADAPTER = 'motion_controller[*].servos[*].front_feet_right[*].bus_servo_adapter | [0] | [0] | [0]'
    MOTION_CONTROLLER_SERVOS_FRONT_FEET_RIGHT_ID = 'motion_controller[*].servos[*].front_feet_right[*].id | [0] | [0] | [0]'
    MOTION_CONTROLLER_SERVOS_FRONT_FEET_RIGHT_MIN_ANGLE = 'motion_controller[*].servos[*].front_feet_right[*].min_angle | [0] | [0] | [0]'
    MOTION_CONTROLLER_SERVOS_FRONT_FEET_RIGHT_MAX_ANGLE = 'motion_controller[*].servos[*].front_feet_right[*].max_angle | [0] | [0] | [0]'
    MOTION_CONTROLLER_SERVOS_FRONT_FEET_RIGHT_REST_ANGLE = 'motion_controller[*].servos[*].front_feet_right[*].rest_angle | [0] | [0] | [0]'

    values = {}

    def __init__(self):

        try:
            log.debug('Loading configuration...')

            self.load_config()
            self.list_modules()

        except Exception as e:
            log.error('Problem while loading the configuration file', e)

    def load_config(self):
        try:
            if not os.path.exists(str(Path.home()) + '/spotmicroai.json'):
                shutil.copyfile(str(Path.home()) + '/spotmicroai/spotmicroai.default',
                                str(Path.home()) + '/spotmicroai.json')

            with open(str(Path.home()) + '/spotmicroai.json') as json_file:
                self.values = json.load(json_file)
                # log.debug(json.dumps(self.values, indent=4, sort_keys=True))

        except Exception as e:
            log.error("Configuration file don't exist or is not a valid json, aborting.")
            sys.exit(1)

    def list_modules(self):
        log.info('Detected configuration for the modules: ' + ', '.join(self.values.keys()))

    def save_config(self):
        try:
            with open('~/spotmicroai.json', 'w') as outfile:
                json.dump(self.values, outfile)
        except Exception as e:
            log.error("Problem saving the configuration file", e)

    def get(self, search_pattern):
        log.debug(search_pattern + ': ' + str(jmespath.search(search_pattern, self.values)))
        return jmespath.search(search_pattern, self.values)

    def get_by_section_name(self, search_pattern):

        BUS_SERVO_ADAPTER = 'motion_controller[*].servos[*].' + str(search_pattern) + '[*].bus_servo_adapter | [0] | [0] | [0]'
        ID = 'motion_controller[*].servos[*].' + str(search_pattern) + '[*].id | [0] | [0] | [0]'
        MIN_ANGLE = 'motion_controller[*].servos[*].' + str(search_pattern) + '[*].min_angle | [0] | [0] | [0]'
        MAX_ANGLE = 'motion_controller[*].servos[*].' + str(search_pattern) + '[*].max_angle | [0] | [0] | [0]'
        REST_ANGLE = 'motion_controller[*].servos[*].' + str(search_pattern) + '[*].rest_angle | [0] | [0] | [0]'

        BUS_SERVO_ADAPTER = jmespath.search(BUS_SERVO_ADAPTER, self.values)

        BUS_SERVO_ADAPTER_SERIAL_PORT = 'motion_controller[*].boards[*].bus_servo_adapter_' + str(BUS_SERVO_ADAPTER) + '[*].serial_port | [0] | [0] | [0]'
        BUS_SERVO_ADAPTER_BAUDRATE = 'motion_controller[*].boards[*].bus_servo_adapter_' + str(BUS_SERVO_ADAPTER) + '[*].baudrate| [0] | [0] | [0]'
        
        log.info('BUS_SERVO_ADAPTER_SERIAL_PORT: ' + str(jmespath.search(BUS_SERVO_ADAPTER_SERIAL_PORT, self.values)))
        log.info('BUS_SERVO_ADAPTER_SERIAL_BAUDRATE: ' + str(jmespath.search(BUS_SERVO_ADAPTER_BAUDRATE, self.values)))
        log.info('ID: ' + str(jmespath.search(ID, self.values)))
        log.info('MIN_ANGLE: ' + str(jmespath.search(MIN_ANGLE, self.values)))
        log.info('MAX_ANGLE: ' + str(jmespath.search(MAX_ANGLE, self.values)))
        log.info('REST_ANGLE: ' + str(jmespath.search(REST_ANGLE, self.values)))

        return jmespath.search(BUS_SERVO_ADAPTER_SERIAL_PORT, self.values), jmespath.search(BUS_SERVO_ADAPTER_BAUDRATE, self.values), jmespath.search(ID, self.values), jmespath.search(MIN_ANGLE, self.values), jmespath.search(MAX_ANGLE, self.values), jmespath.search(REST_ANGLE, self.values)
