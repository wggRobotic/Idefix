import signal
import sys

import queue

# import busio
# from board import SCL, SDA
import serial
import time

from spotmicroai.utilities.log import Logger
from spotmicroai.utilities.config import Config
import spotmicroai.utilities.queues as queues
from spotmicroai.utilities.general import General
from STservo_sdk import *

log = Logger().setup_logger("Motion controller")


class MotionController:

    boards = 1
    sts_moving_speed = 2400
    sts_moving_acc = 50

    is_activated = False

    bus_servo_adapter_1_port_handler = None
    bus_servo_adapter_1_packet_handler = None
    bus_servo_adapter_1_serial_port = None
    bus_servo_adapter_1_baudrate = None

    bus_servo_adapter_2_port_handler = None
    bus_servo_adapter_2_packet_handler = None
    bus_servo_adapter_2_serial_port = None
    bus_servo_adapter_2_baudrate = None

    servo_rear_shoulder_left_bus_servo_adapter = None
    servo_rear_shoulder_left_id = None
    servo_rear_shoulder_left_min_angle = None
    servo_rear_shoulder_left_max_angle = None
    servo_rear_shoulder_left_rest_angle = None

    servo_rear_leg_left_bus_servo_adapter = None
    servo_rear_leg_left_id = None
    servo_rear_leg_left_min_angle = None
    servo_rear_leg_left_max_angle = None
    servo_rear_leg_left_rest_angle = None

    servo_rear_feet_left_bus_servo_adapter = None
    servo_rear_feet_left_id = None
    servo_rear_feet_left_min_angle = None
    servo_rear_feet_left_max_angle = None
    servo_rear_feet_left_rest_angle = None

    servo_rear_shoulder_right_bus_servo_adapter = None
    servo_rear_shoulder_right_id = None
    servo_rear_shoulder_right_min_angle = None
    servo_rear_shoulder_right_max_angle = None
    servo_rear_shoulder_right_rest_angle = None

    servo_rear_leg_right_bus_servo_adapter = None
    servo_rear_leg_right_id = None
    servo_rear_leg_right_min_angle = None
    servo_rear_leg_right_max_angle = None
    servo_rear_leg_right_rest_angle = None

    servo_rear_feet_right_bus_servo_adapter = None
    servo_rear_feet_right_id = None
    servo_rear_feet_right_min_angle = None
    servo_rear_feet_right_max_angle = None
    servo_rear_feet_right_rest_angle = None

    servo_front_shoulder_left_bus_servo_adapter = None
    servo_front_shoulder_left_id = None
    servo_front_shoulder_left_min_angle = None
    servo_front_shoulder_left_max_angle = None
    servo_front_shoulder_left_rest_angle = None

    servo_front_leg_left_bus_servo_adapter = None
    servo_front_leg_left_id = None
    servo_front_leg_left_min_angle = None
    servo_front_leg_left_max_angle = None
    servo_front_leg_left_rest_angle = None

    servo_front_feet_left_bus_servo_adapter = None
    servo_front_feet_left_id = None
    servo_front_feet_left_min_angle = None
    servo_front_feet_left_max_angle = None
    servo_front_feet_left_rest_angle = None

    servo_front_shoulder_right_bus_servo_adapter = None
    servo_front_shoulder_right_id = None
    servo_front_shoulder_right_min_angle = None
    servo_front_shoulder_right_max_angle = None
    servo_front_shoulder_right_rest_angle = None

    servo_front_leg_right_bus_servo_adapter = None
    servo_front_leg_right_id = None
    servo_front_leg_right_min_angle = None
    servo_front_leg_right_max_angle = None
    servo_front_leg_right_rest_angle = None

    servo_front_feet_right_bus_servo_adapter = None
    servo_front_feet_right_id = None
    servo_front_feet_right_min_angle = None
    servo_front_feet_right_max_angle = None
    servo_front_feet_right_rest_angle = None

    def __init__(self, communication_queues):

        try:

            log.debug("Starting controller...")

            signal.signal(signal.SIGINT, self.exit_gracefully)
            signal.signal(signal.SIGTERM, self.exit_gracefully)

            # self.i2c = busio.I2C(SCL, SDA)
            self.load_bus_servo_adapter_boards_configuration()
            self.load_servos_configuration()

            self._abort_queue = communication_queues[queues.ABORT_CONTROLLER]
            self._motion_queue = communication_queues[queues.MOTION_CONTROLLER]

            # self._lcd_screen_queue = communication_queues[queues.LCD_SCREEN_CONTROLLER]
            # if self.bus_servo_adapter_2_serial_port:
            #     self._lcd_screen_queue.put("motion_controller_1 OK")
            #     self._lcd_screen_queue.put("motion_controller_2 OK")
            # else:
            #     self._lcd_screen_queue.put("motion_controller_1 OK")
            #     self._lcd_screen_queue.put("motion_controller_2 NOK")

            self._previous_event = {}

        except Exception as e:
            log.error("Motion controller initialization problem", e)
            # self._lcd_screen_queue.put("motion_controller_1 NOK")
            # self._lcd_screen_queue.put("motion_controller_2 NOK")
            try:
                self.bus_servo_adapter_1_port_handler.closePort()
            finally:
                try:
                    if self.boards == 2:
                        self.bus_servo_adapter_2_port_handler.closePort()
                finally:
                    sys.exit(1)

    def exit_gracefully(self, signum, frame):
        try:
            self.bus_servo_adapter_1_port_handler.closePort()
        finally:
            try:
                if self.boards == 2:
                    self.bus_servo_adapter_2_port_handler.closePort()
            finally:
                log.info("Terminated")
                sys.exit(0)

    def do_process_events_from_queues(self):

        while True:

            try:

                event = self._motion_queue.get(block=True, timeout=60)
                event_abort = self._abort_queue.get()

                # log.debug(event)

                if event_abort == queues.ABORT_CONTROLLER_ACTION_ACTIVATE:
                    self.activate_servos()

                if event_abort == queues.ABORT_CONTROLLER_ACTION_ABORT:
                    self.abort()

                if event["start"]:
                    if self.is_activated:
                        self.rest_position()
                        time.sleep(0.5)
                        self.deactivate_bus_servo_adapter_boards()
                        self._abort_queue.put(queues.ABORT_CONTROLLER_ACTION_ABORT)
                    else:
                        self._abort_queue.put(queues.ABORT_CONTROLLER_ACTION_ACTIVATE)
                        self.activate_servo_adapters()
                        # self.activate_servos()
                        self.rest_position()

                if not self.is_activated:
                    log.info("Press START/OPTIONS to enable the servos")
                    continue

                if event["a"]:
                    self.rest_position()

                if event["hat0y"]:
                    self.body_move_body_up_and_down(event["hat0y"])

                if event["hat0x"]:
                    self.body_move_body_left_right(event["hat0x"])

                if event["ry"]:
                    self.body_move_body_up_and_down_analog(event["ry"])

                if event["rx"]:
                    self.body_move_body_left_right_analog(event["rx"])

                if event["hat0x"] and event["tl2"]:
                    # 2 buttons example
                    pass

                if event["y"]:
                    self.standing_position()

                if event["b"]:
                    self.body_move_position_right()

                if event["x"]:
                    self.body_move_position_left()

                self.move()

            except queue.Empty as e:
                log.info(
                    "Inactivity lasted 60 seconds, shutting down the servos, "
                    "press start to reactivate"
                )
                if self.is_activated:
                    self.rest_position()
                    time.sleep(0.5)
                    self.deactivate_bus_servo_adapter_boards()

            except Exception as e:
                log.error(
                    "Unknown problem while processing the queue of the motion controller"
                )
                log.error(
                    " - Most likely a servo is not able to get to the assigned position"
                )

    def load_bus_servo_adapter_boards_configuration(self):
        self.bus_servo_adapter_1_serial_port = int(
            Config().get(
                Config.MOTION_CONTROLLER_BOARDS_BUS_SERVO_ADAPTER_1_SERIAL_PORT
            ),
            0,
        )
        self.bus_servo_adapter_1_baudrate = int(
            Config().get(Config.MOTION_CONTROLLER_BOARDS_BUS_SERVO_ADAPTER_1_BAUDRATE)
        )

        self.bus_servo_adapter_2_serial_port = False
        try:
            self.bus_servo_adapter_2_serial_port = int(
                Config().get(
                    Config.MOTION_CONTROLLER_BOARDS_BUS_SERVO_ADAPTER_2_SERIAL_PORT
                ),
                0,
            )

            if self.bus_servo_adapter_2_serial_port:
                self.bus_servo_adapter_2_serial_port = int(
                    Config().get(
                        Config.MOTION_CONTROLLER_BOARDS_BUS_SERVO_ADAPTER_2_SERIAL_PORT
                    ),
                    0,
                )
                self.bus_servo_adapter_2_baudrate = int(
                    Config().get(
                        Config.MOTION_CONTROLLER_BOARDS_BUS_SERVO_ADAPTER_2_BAUDRATE
                    )
                )

        except Exception as e:
            log.debug("Only 1 Bus servo adapter is present in the configuration")

    def activate_servo_adapters(self):

        self.bus_servo_adapter_1_port_handler = PortHandler(
            self.bus_servo_adapter_1_serial_port
        )
        self.bus_servo_adapter_1_packet_handler = sts(
            self.bus_servo_adapter_1_port_handler
        )

        if self.bus_servo_adapter_1_port_handler.openPort():
            log.debug("Bus servo adapter 1 port opened")
        else:
            log.error("Bus servo adapter 1 port open error")
            self._abort_queue.put(queues.ABORT_CONTROLLER_ACTION_ABORT)
            return

        if self.bus_servo_adapter_1_port_handler.setBaudRate(
            self.bus_servo_adapter_1_baudrate
        ):
            log.debug("Bus servo adapter 1 baudrate set")
        else:
            log.error("Bus servo adapter 1 baudrate set error")
            self._abort_queue.put(queues.ABORT_CONTROLLER_ACTION_ABORT)
            return

        if self.bus_servo_adapter_2_serial_port:
            self.bus_servo_adapter_2_port_handler = PortHandler(
                self.bus_servo_adapter_2_serial_port
            )
            self.bus_servo_adapter_2_packet_handler = sts(
                self.bus_servo_adapter_2_port_handler
            )

            if self.bus_servo_adapter_2_port_handler.openPort():
                log.debug("Bus servo adapter 2 port opened")
            else:
                log.error("Bus servo adapter 2 port open error")
                self._abort_queue.put(queues.ABORT_CONTROLLER_ACTION_ABORT)
                return

            if self.bus_servo_adapter_2_port_handler.setBaudRate(
                self.bus_servo_adapter_2_baudrate
            ):
                log.debug("Bus servo adapter 2 baudrate set")
            else:
                log.error("Bus servo adapter 2 baudrate set error")
                self._abort_queue.put(queues.ABORT_CONTROLLER_ACTION_ABORT)
                return

        self.is_activated = True
        log.debug(str(self.boards) + " Bus servo adapter board(s) activated")

    def deactivate_bus_servo_adapter_boards(self):

        try:
            if self.bus_servo_adapter_1_port_handler:
                self.bus_servo_adapter_1_port_handler.closePort()
        finally:
            try:
                if self.boards == 2 and self.bus_servo_adapter_2_port_handler:
                    self.bus_servo_adapter_2_port_handler.closePort()
            finally:
                # self._abort_queue.put(queues.ABORT_CONTROLLER_ACTION_ABORT)
                self.is_activated = False

        log.debug(str(self.boards) + "Bus servo adapter board(s) deactivated")

    def load_servos_configuration(self):

        self.servo_rear_shoulder_left_bus_servo_adapter = Config().get(
            Config.MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_LEFT_BUS_SERVO_ADAPTER
        )
        self.servo_rear_shoulder_left_id = Config().get(
            Config.MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_LEFT_ID
        )
        self.servo_rear_shoulder_left_min_pulse = Config().get(
            Config.MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_LEFT_MIN_ANGLE
        )
        self.servo_rear_shoulder_left_max_pulse = Config().get(
            Config.MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_LEFT_MAX_ANGLE
        )
        self.servo_rear_shoulder_left_rest_angle = Config().get(
            Config.MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_LEFT_REST_ANGLE
        )

        self.servo_rear_leg_left_bus_servo_adapter = Config().get(
            Config.MOTION_CONTROLLER_SERVOS_REAR_LEG_LEFT_BUS_SERVO_ADAPTER
        )
        self.servo_rear_leg_left_id = Config().get(
            Config.MOTION_CONTROLLER_SERVOS_REAR_LEG_LEFT_ID
        )
        self.servo_rear_leg_left_min_pulse = Config().get(
            Config.MOTION_CONTROLLER_SERVOS_REAR_LEG_LEFT_MIN_ANGLE
        )
        self.servo_rear_leg_left_max_pulse = Config().get(
            Config.MOTION_CONTROLLER_SERVOS_REAR_LEG_LEFT_MAX_ANGLE
        )
        self.servo_rear_leg_left_rest_angle = Config().get(
            Config.MOTION_CONTROLLER_SERVOS_REAR_LEG_LEFT_REST_ANGLE
        )

        self.servo_rear_feet_left_bus_servo_adapter = Config().get(
            Config.MOTION_CONTROLLER_SERVOS_REAR_FEET_LEFT_BUS_SERVO_ADAPTER
        )
        self.servo_rear_feet_left_id = Config().get(
            Config.MOTION_CONTROLLER_SERVOS_REAR_FEET_LEFT_ID
        )
        self.servo_rear_feet_left_min_pulse = Config().get(
            Config.MOTION_CONTROLLER_SERVOS_REAR_FEET_LEFT_MIN_ANGLE
        )
        self.servo_rear_feet_left_max_pulse = Config().get(
            Config.MOTION_CONTROLLER_SERVOS_REAR_FEET_LEFT_MAX_ANGLE
        )
        self.servo_rear_feet_left_rest_angle = Config().get(
            Config.MOTION_CONTROLLER_SERVOS_REAR_FEET_LEFT_REST_ANGLE
        )

        self.servo_rear_shoulder_right_bus_servo_adapter = Config().get(
            Config.MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_RIGHT_BUS_SERVO_ADAPTER
        )
        self.servo_rear_shoulder_right_id = Config().get(
            Config.MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_RIGHT_ID
        )
        self.servo_rear_shoulder_right_min_pulse = Config().get(
            Config.MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_RIGHT_MIN_ANGLE
        )
        self.servo_rear_shoulder_right_max_pulse = Config().get(
            Config.MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_RIGHT_MAX_ANGLE
        )
        self.servo_rear_shoulder_right_rest_angle = Config().get(
            Config.MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_RIGHT_REST_ANGLE
        )

        self.servo_rear_leg_right_bus_servo_adapter = Config().get(
            Config.MOTION_CONTROLLER_SERVOS_REAR_LEG_RIGHT_BUS_SERVO_ADAPTER
        )
        self.servo_rear_leg_right_id = Config().get(
            Config.MOTION_CONTROLLER_SERVOS_REAR_LEG_RIGHT_ID
        )
        self.servo_rear_leg_right_min_pulse = Config().get(
            Config.MOTION_CONTROLLER_SERVOS_REAR_LEG_RIGHT_MIN_ANGLE
        )
        self.servo_rear_leg_right_max_pulse = Config().get(
            Config.MOTION_CONTROLLER_SERVOS_REAR_LEG_RIGHT_MAX_ANGLE
        )
        self.servo_rear_leg_right_rest_angle = Config().get(
            Config.MOTION_CONTROLLER_SERVOS_REAR_LEG_RIGHT_REST_ANGLE
        )

        self.servo_rear_feet_right_bus_servo_adapter = Config().get(
            Config.MOTION_CONTROLLER_SERVOS_REAR_FEET_RIGHT_BUS_SERVO_ADAPTER
        )
        self.servo_rear_feet_right_id = Config().get(
            Config.MOTION_CONTROLLER_SERVOS_REAR_FEET_RIGHT_ID
        )
        self.servo_rear_feet_right_min_pulse = Config().get(
            Config.MOTION_CONTROLLER_SERVOS_REAR_FEET_RIGHT_MIN_ANGLE
        )
        self.servo_rear_feet_right_max_pulse = Config().get(
            Config.MOTION_CONTROLLER_SERVOS_REAR_FEET_RIGHT_MAX_ANGLE
        )
        self.servo_rear_feet_right_rest_angle = Config().get(
            Config.MOTION_CONTROLLER_SERVOS_REAR_FEET_RIGHT_REST_ANGLE
        )

        self.servo_front_shoulder_left_bus_servo_adapter = Config().get(
            Config.MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_LEFT_BUS_SERVO_ADAPTER
        )
        self.servo_front_shoulder_left_id = Config().get(
            Config.MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_LEFT_ID
        )
        self.servo_front_shoulder_left_min_pulse = Config().get(
            Config.MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_LEFT_MIN_ANGLE
        )
        self.servo_front_shoulder_left_max_pulse = Config().get(
            Config.MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_LEFT_MAX_ANGLE
        )
        self.servo_front_shoulder_left_rest_angle = Config().get(
            Config.MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_LEFT_REST_ANGLE
        )

        self.servo_front_leg_left_bus_servo_adapter = Config().get(
            Config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_LEFT_BUS_SERVO_ADAPTER
        )
        self.servo_front_leg_left_id = Config().get(
            Config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_LEFT_ID
        )
        self.servo_front_leg_left_min_pulse = Config().get(
            Config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_LEFT_MIN_ANGLE
        )
        self.servo_front_leg_left_max_pulse = Config().get(
            Config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_LEFT_MAX_ANGLE
        )
        self.servo_front_leg_left_rest_angle = Config().get(
            Config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_LEFT_REST_ANGLE
        )

        self.servo_front_feet_left_bus_servo_adapter = Config().get(
            Config.MOTION_CONTROLLER_SERVOS_FRONT_FEET_LEFT_BUS_SERVO_ADAPTER
        )
        self.servo_front_feet_left_id = Config().get(
            Config.MOTION_CONTROLLER_SERVOS_FRONT_FEET_LEFT_ID
        )
        self.servo_front_feet_left_min_pulse = Config().get(
            Config.MOTION_CONTROLLER_SERVOS_FRONT_FEET_LEFT_MIN_ANGLE
        )
        self.servo_front_feet_left_max_pulse = Config().get(
            Config.MOTION_CONTROLLER_SERVOS_FRONT_FEET_LEFT_MAX_ANGLE
        )
        self.servo_front_feet_left_rest_angle = Config().get(
            Config.MOTION_CONTROLLER_SERVOS_FRONT_FEET_LEFT_REST_ANGLE
        )

        self.servo_front_shoulder_right_bus_servo_adapter = Config().get(
            Config.MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_RIGHT_BUS_SERVO_ADAPTER
        )
        self.servo_front_shoulder_right_id = Config().get(
            Config.MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_RIGHT_ID
        )
        self.servo_front_shoulder_right_min_pulse = Config().get(
            Config.MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_RIGHT_MIN_ANGLE
        )
        self.servo_front_shoulder_right_max_pulse = Config().get(
            Config.MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_RIGHT_MAX_ANGLE
        )
        self.servo_front_shoulder_right_rest_angle = Config().get(
            Config.MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_RIGHT_REST_ANGLE
        )

        self.servo_front_leg_right_bus_servo_adapter = Config().get(
            Config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_RIGHT_BUS_SERVO_ADAPTER
        )
        self.servo_front_leg_right_id = Config().get(
            Config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_RIGHT_ID
        )
        self.servo_front_leg_right_min_pulse = Config().get(
            Config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_RIGHT_MIN_ANGLE
        )
        self.servo_front_leg_right_max_pulse = Config().get(
            Config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_RIGHT_MAX_ANGLE
        )
        self.servo_front_leg_right_rest_angle = Config().get(
            Config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_RIGHT_REST_ANGLE
        )

        self.servo_front_feet_right_bus_servo_adapter = Config().get(
            Config.MOTION_CONTROLLER_SERVOS_FRONT_FEET_RIGHT_BUS_SERVO_ADAPTER
        )
        self.servo_front_feet_right_id = Config().get(
            Config.MOTION_CONTROLLER_SERVOS_FRONT_FEET_RIGHT_ID
        )
        self.servo_front_feet_right_min_pulse = Config().get(
            Config.MOTION_CONTROLLER_SERVOS_FRONT_FEET_RIGHT_MIN_ANGLE
        )
        self.servo_front_feet_right_max_pulse = Config().get(
            Config.MOTION_CONTROLLER_SERVOS_FRONT_FEET_RIGHT_MAX_ANGLE
        )
        self.servo_front_feet_right_rest_angle = Config().get(
            Config.MOTION_CONTROLLER_SERVOS_FRONT_FEET_RIGHT_REST_ANGLE
        )

    def move_servo(self, bus_servo_adapter, servo_id, rest_angle):
        result = bus_servo_adapter.SyncWritePosEx(
            servo_id, rest_angle, self.sts_moving_speed, self.sts_moving_acc
        )

        if result != True:
            log.error(f"Impossible angle requested for servo {servo_id}")

    def move(self):
        servos = [
            (
                self.servo_rear_shoulder_left_bus_servo_adapter,
                self.servo_rear_shoulder_left_id,
                self.servo_rear_shoulder_left_rest_angle,
            ),
            (
                self.servo_rear_leg_left_bus_servo_adapter,
                self.servo_rear_leg_left_id,
                self.servo_rear_leg_left_rest_angle,
            ),
            (
                self.servo_rear_feet_left_bus_servo_adapter,
                self.servo_rear_feet_left_id,
                self.servo_rear_feet_left_rest_angle,
            ),
            (
                self.servo_rear_shoulder_right_bus_servo_adapter,
                self.servo_rear_shoulder_right_id,
                self.servo_rear_shoulder_right_rest_angle,
            ),
            (
                self.servo_rear_leg_right_bus_servo_adapter,
                self.servo_rear_leg_right_id,
                self.servo_rear_leg_right_rest_angle,
            ),
            (
                self.servo_rear_feet_right_bus_servo_adapter,
                self.servo_rear_feet_right_id,
                self.servo_rear_feet_right_rest_angle,
            ),
            (
                self.servo_front_shoulder_left_bus_servo_adapter,
                self.servo_front_shoulder_left_id,
                self.servo_front_shoulder_left_rest_angle,
            ),
            (
                self.servo_front_leg_left_bus_servo_adapter,
                self.servo_front_leg_left_id,
                self.servo_front_leg_left_rest_angle,
            ),
            (
                self.servo_front_feet_left_bus_servo_adapter,
                self.servo_front_feet_left_id,
                self.servo_front_feet_left_rest_angle,
            ),
            (
                self.servo_front_shoulder_right_bus_servo_adapter,
                self.servo_front_shoulder_right_id,
                self.servo_front_shoulder_right_rest_angle,
            ),
            (
                self.servo_front_leg_right_bus_servo_adapter,
                self.servo_front_leg_right_id,
                self.servo_front_leg_right_rest_angle,
            ),
            (
                self.servo_front_feet_right_bus_servo_adapter,
                self.servo_front_feet_right_id,
                self.servo_front_feet_right_rest_angle,
            ),
        ]

        for bus_servo_adapter_num, servo_id, rest_angle in servos:
            bus_servo_adapter = (
                self.bus_servo_adapter_1_packet_handler
                if bus_servo_adapter_num == 1
                else self.bus_servo_adapter_2_packet_handler
            )
            self.move_servo(bus_servo_adapter, servo_id, rest_angle)

        sts_comm_result_1 = (
            self.bus_servo_adapter_1_packet_handler.groupSyncWrite.txPacket()
        )
        if sts_comm_result_1 != COMM_SUCCESS:
            print(
                "%s"
                % self.bus_servo_adapter_1_packet_handler.getTxRxResult(
                    sts_comm_result_1
                )
            )

        sts_comm_result_2 = (
            self.bus_servo_adapter_2_packet_handler.groupSyncWrite.txPacket()
        )
        if sts_comm_result_2 != COMM_SUCCESS:
            print(
                "%s"
                % self.bus_servo_adapter_2_packet_handler.getTxRxResult(
                    sts_comm_result_2
                )
            )

        self.bus_servo_adapter_1_packet_handler.groupSyncWrite.clearParam()
        self.bus_servo_adapter_2_packet_handler.groupSyncWrite.clearParam()
        time.sleep(0.002)

        # TODO: check if the servos are in the requested position

    def rest_position(self):

        self.servo_rear_shoulder_left_rest_angle = Config().get(
            Config.MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_LEFT_REST_ANGLE
        )
        self.servo_rear_leg_left_rest_angle = Config().get(
            Config.MOTION_CONTROLLER_SERVOS_REAR_LEG_LEFT_REST_ANGLE
        )
        self.servo_rear_feet_left_rest_angle = Config().get(
            Config.MOTION_CONTROLLER_SERVOS_REAR_FEET_LEFT_REST_ANGLE
        )
        self.servo_rear_shoulder_right_rest_angle = Config().get(
            Config.MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_RIGHT_REST_ANGLE
        )
        self.servo_rear_leg_right_rest_angle = Config().get(
            Config.MOTION_CONTROLLER_SERVOS_REAR_LEG_RIGHT_REST_ANGLE
        )
        self.servo_rear_feet_right_rest_angle = Config().get(
            Config.MOTION_CONTROLLER_SERVOS_REAR_FEET_RIGHT_REST_ANGLE
        )
        self.servo_front_shoulder_left_rest_angle = Config().get(
            Config.MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_LEFT_REST_ANGLE
        )
        self.servo_front_leg_left_rest_angle = Config().get(
            Config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_LEFT_REST_ANGLE
        )
        self.servo_front_feet_left_rest_angle = Config().get(
            Config.MOTION_CONTROLLER_SERVOS_FRONT_FEET_LEFT_REST_ANGLE
        )
        self.servo_front_shoulder_right_rest_angle = Config().get(
            Config.MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_RIGHT_REST_ANGLE
        )
        self.servo_front_leg_right_rest_angle = Config().get(
            Config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_RIGHT_REST_ANGLE
        )
        self.servo_front_feet_right_rest_angle = Config().get(
            Config.MOTION_CONTROLLER_SERVOS_FRONT_FEET_RIGHT_REST_ANGLE
        )

    def body_move_body_up_and_down(self, raw_value):

        range = 10
        range2 = 15

        if raw_value < 0:
            self.servo_rear_leg_left_rest_angle -= range
            self.servo_rear_feet_left_rest_angle += range2
            self.servo_rear_leg_right_rest_angle += range
            self.servo_rear_feet_right_rest_angle -= range2
            self.servo_front_leg_left_rest_angle -= range
            self.servo_front_feet_left_rest_angle += range2
            self.servo_front_leg_right_rest_angle += range
            self.servo_front_feet_right_rest_angle -= range2

        elif raw_value > 0:
            self.servo_rear_leg_left_rest_angle += range
            self.servo_rear_feet_left_rest_angle -= range2
            self.servo_rear_leg_right_rest_angle -= range
            self.servo_rear_feet_right_rest_angle += range2
            self.servo_front_leg_left_rest_angle += range
            self.servo_front_feet_left_rest_angle -= range2
            self.servo_front_leg_right_rest_angle -= range
            self.servo_front_feet_right_rest_angle += range2

        else:
            self.rest_position()

        print(
            str(self.servo_rear_leg_left_rest_angle)
            + ", "
            + str(self.servo_rear_feet_left_rest_angle)
            + ", "
            + str(self.servo_rear_leg_right_rest_angle)
            + ", "
            + str(self.servo_rear_feet_right_rest_angle)
            + ", "
            + str(self.servo_front_leg_left_rest_angle)
            + ", "
            + str(self.servo_front_feet_left_rest_angle)
            + ", "
            + str(self.servo_front_leg_right_rest_angle)
            + ", "
            + str(self.servo_front_feet_right_rest_angle)
        )

    def body_move_body_up_and_down_analog(self, raw_value):

        servo_rear_leg_left_max_angle = 38
        servo_rear_feet_left_max_angle = 70
        servo_rear_leg_right_max_angle = 126
        servo_rear_feet_right_max_angle = 102
        servo_front_leg_left_max_angle = 57
        servo_front_feet_left_max_angle = 85
        servo_front_leg_right_max_angle = 130
        servo_front_feet_right_max_angle = 120

        delta_rear_leg_left = int(
            General().maprange(
                (1, -1),
                (
                    Config().get(
                        Config.MOTION_CONTROLLER_SERVOS_REAR_LEG_LEFT_REST_ANGLE
                    ),
                    servo_rear_leg_left_max_angle,
                ),
                raw_value,
            )
        )
        delta_rear_feet_left = int(
            General().maprange(
                (1, -1),
                (
                    Config().get(
                        Config.MOTION_CONTROLLER_SERVOS_REAR_FEET_LEFT_REST_ANGLE
                    ),
                    servo_rear_feet_left_max_angle,
                ),
                raw_value,
            )
        )
        delta_rear_leg_right = int(
            General().maprange(
                (1, -1),
                (
                    Config().get(
                        Config.MOTION_CONTROLLER_SERVOS_REAR_LEG_RIGHT_REST_ANGLE
                    ),
                    servo_rear_leg_right_max_angle,
                ),
                raw_value,
            )
        )
        delta_rear_feet_right = int(
            General().maprange(
                (1, -1),
                (
                    Config().get(
                        Config.MOTION_CONTROLLER_SERVOS_REAR_FEET_RIGHT_REST_ANGLE
                    ),
                    servo_rear_feet_right_max_angle,
                ),
                raw_value,
            )
        )
        delta_front_leg_left = int(
            General().maprange(
                (1, -1),
                (
                    Config().get(
                        Config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_LEFT_REST_ANGLE
                    ),
                    servo_front_leg_left_max_angle,
                ),
                raw_value,
            )
        )
        delta_front_feet_left = int(
            General().maprange(
                (1, -1),
                (
                    Config().get(
                        Config.MOTION_CONTROLLER_SERVOS_FRONT_FEET_LEFT_REST_ANGLE
                    ),
                    servo_front_feet_left_max_angle,
                ),
                raw_value,
            )
        )
        delta_front_leg_right = int(
            General().maprange(
                (1, -1),
                (
                    Config().get(
                        Config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_RIGHT_REST_ANGLE
                    ),
                    servo_front_leg_right_max_angle,
                ),
                raw_value,
            )
        )
        delta_front_feet_right = int(
            General().maprange(
                (1, -1),
                (
                    Config().get(
                        Config.MOTION_CONTROLLER_SERVOS_FRONT_FEET_RIGHT_REST_ANGLE
                    ),
                    servo_front_feet_right_max_angle,
                ),
                raw_value,
            )
        )

        self.servo_rear_leg_left_rest_angle = delta_rear_leg_left
        self.servo_rear_feet_left_rest_angle = delta_rear_feet_left
        self.servo_rear_leg_right_rest_angle = delta_rear_leg_right
        self.servo_rear_feet_right_rest_angle = delta_rear_feet_right
        self.servo_front_leg_left_rest_angle = delta_front_leg_left
        self.servo_front_feet_left_rest_angle = delta_front_feet_left
        self.servo_front_leg_right_rest_angle = delta_front_leg_right
        self.servo_front_feet_right_rest_angle = delta_front_feet_right

    def body_move_body_left_right(self, raw_value):

        range = 5

        if raw_value < 0:
            self.servo_rear_shoulder_left_rest_angle -= range
            self.servo_rear_shoulder_right_rest_angle -= range
            self.servo_front_shoulder_left_rest_angle += range
            self.servo_front_shoulder_right_rest_angle += range

        elif raw_value > 0:
            self.servo_rear_shoulder_left_rest_angle += range
            self.servo_rear_shoulder_right_rest_angle += range
            self.servo_front_shoulder_left_rest_angle -= range
            self.servo_front_shoulder_right_rest_angle -= range

        else:
            self.rest_position()

    def body_move_body_left_right_analog(self, raw_value):

        delta_a = int(General().maprange((-1, 1), (30, 150), raw_value))
        delta_b = int(General().maprange((-1, 1), (150, 30), raw_value))

        self.servo_rear_shoulder_left_rest_angle = delta_a
        self.servo_rear_shoulder_right_rest_angle = delta_a
        self.servo_front_shoulder_left_rest_angle = delta_b
        self.servo_front_shoulder_right_rest_angle = delta_b

    def standing_position(self):

        variation_leg = 50
        variation_feet = 70

        self.move_servo(
            (
                self.bus_servo_adapter_1_packet_handler
                if self.servo_rear_shoulder_left_bus_servo_adapter == 1
                else self.bus_servo_adapter_2_packet_handler
            ),
            self.servo_rear_shoulder_left_id,
            (self.servo_rear_shoulder_left_rest_angle + 10),
        )
        
        self.move_servo(
            (
                self.bus_servo_adapter_1_packet_handler
                if self.servo_rear_leg_left_bus_servo_adapter == 1
                else self.bus_servo_adapter_2_packet_handler
            ),
            self.servo_rear_leg_left_id,
            (self.servo_rear_leg_left_rest_angle - variation_leg),
        )

        self.move_servo(
            (
                self.bus_servo_adapter_1_packet_handler
                if self.servo_rear_feet_left_bus_servo_adapter == 1
                else self.bus_servo_adapter_2_packet_handler
            ),
            self.servo_rear_feet_left_id,
            (self.servo_rear_feet_left_rest_angle + variation_feet),
        )
        
        
        self.move_servo(
            (
                self.bus_servo_adapter_1_packet_handler
                if self.servo_rear_shoulder_right_bus_servo_adapter == 1
                else self.bus_servo_adapter_2_packet_handler
            ),
            self.servo_rear_shoulder_right_id,
            (self.servo_rear_shoulder_right_rest_angle - 10),
        )
        
        self.move_servo(
            (
                self.bus_servo_adapter_1_packet_handler
                if self.servo_rear_leg_right_bus_servo_adapter == 1
                else self.bus_servo_adapter_2_packet_handler
            ),
            self.servo_rear_leg_right_id,
            (self.servo_rear_leg_right_rest_angle + variation_leg),
        )

        self.move_servo(
            (
                self.bus_servo_adapter_1_packet_handler
                if self.servo_rear_feet_right_bus_servo_adapter == 1
                else self.bus_servo_adapter_2_packet_handler
            ),
            self.servo_rear_feet_right_id,
            (self.servo_rear_feet_right_rest_angle - variation_feet),
        )

        sts_comm_result_1 = (
            self.bus_servo_adapter_1_packet_handler.groupSyncWrite.txPacket()
        )
        if sts_comm_result_1 != COMM_SUCCESS:
            print(
                "%s"
                % self.bus_servo_adapter_1_packet_handler.getTxRxResult(
                    sts_comm_result_1
                )
            )

        sts_comm_result_2 = (
            self.bus_servo_adapter_2_packet_handler.groupSyncWrite.txPacket()
        )
        if sts_comm_result_2 != COMM_SUCCESS:
            print(
                "%s"
                % self.bus_servo_adapter_2_packet_handler.getTxRxResult(
                    sts_comm_result_2
                )
            )

        self.bus_servo_adapter_1_packet_handler.groupSyncWrite.clearParam()
        self.bus_servo_adapter_2_packet_handler.groupSyncWrite.clearParam()
        time.sleep(0.002)
        
        self.move_servo(
            (
                self.bus_servo_adapter_1_packet_handler
                if self.servo_front_shoulder_left_bus_servo_adapter == 1
                else self.bus_servo_adapter_2_packet_handler
            ),
            self.servo_front_shoulder_left_id,
            (self.servo_front_shoulder_left_rest_angle - 10),
        )
        
        self.move_servo(
            (
                self.bus_servo_adapter_1_packet_handler
                if self.servo_front_leg_left_bus_servo_adapter == 1
                else self.bus_servo_adapter_2_packet_handler
            ),
            self.servo_front_leg_left_id,
            (self.servo_front_leg_left_rest_angle - variation_leg + 5),
        )

        self.move_servo(
            (
                self.bus_servo_adapter_1_packet_handler
                if self.servo_front_feet_left_bus_servo_adapter == 1
                else self.bus_servo_adapter_2_packet_handler
            ),
            self.servo_front_feet_left_id,
            (self.servo_front_feet_left_rest_angle + variation_feet - 5),
        )
        
        
        self.move_servo(
            (
                self.bus_servo_adapter_1_packet_handler
                if self.servo_front_shoulder_right_bus_servo_adapter == 1
                else self.bus_servo_adapter_2_packet_handler
            ),
            self.servo_front_shoulder_right_id,
            (self.servo_front_shoulder_right_rest_angle + 10),
        )
        
        self.move_servo(
            (
                self.bus_servo_adapter_1_packet_handler
                if self.servo_front_leg_right_bus_servo_adapter == 1
                else self.bus_servo_adapter_2_packet_handler
            ),
            self.servo_front_leg_right_id,
            (self.servo_front_leg_right_rest_angle + variation_leg - 5),
        )

        self.move_servo(
            (
                self.bus_servo_adapter_1_packet_handler
                if self.servo_front_feet_right_bus_servo_adapter == 1
                else self.bus_servo_adapter_2_packet_handler
            ),
            self.servo_front_feet_right_id,
            (self.servo_front_feet_right_rest_angle - variation_feet + 5),
        )

        sts_comm_result_1 = (
            self.bus_servo_adapter_1_packet_handler.groupSyncWrite.txPacket()
        )
        if sts_comm_result_1 != COMM_SUCCESS:
            print(
                "%s"
                % self.bus_servo_adapter_1_packet_handler.getTxRxResult(
                    sts_comm_result_1
                )
            )

        sts_comm_result_2 = (
            self.bus_servo_adapter_2_packet_handler.groupSyncWrite.txPacket()
        )
        if sts_comm_result_2 != COMM_SUCCESS:
            print(
                "%s"
                % self.bus_servo_adapter_2_packet_handler.getTxRxResult(
                    sts_comm_result_2
                )
            )

        self.bus_servo_adapter_1_packet_handler.groupSyncWrite.clearParam()
        self.bus_servo_adapter_2_packet_handler.groupSyncWrite.clearParam()
        time.sleep(0.002)


    def body_move_position_right(self):

        move = 20

        variation_leg = 50
        variation_feet = 70
        
        
        self.move_servo(
            (
                self.bus_servo_adapter_1_packet_handler
                if self.servo_rear_shoulder_left_bus_servo_adapter == 1
                else self.bus_servo_adapter_2_packet_handler
            ),
            self.servo_rear_shoulder_left_id,
            (self.servo_rear_shoulder_left_rest_angle + 10 + move),
        )
        
        self.move_servo(
            (
                self.bus_servo_adapter_1_packet_handler
                if self.servo_rear_leg_left_bus_servo_adapter == 1
                else self.bus_servo_adapter_2_packet_handler
            ),
            self.servo_rear_leg_left_id,
            (self.servo_rear_leg_left_rest_angle - variation_leg),
        )

        self.move_servo(
            (
                self.bus_servo_adapter_1_packet_handler
                if self.servo_rear_feet_left_bus_servo_adapter == 1
                else self.bus_servo_adapter_2_packet_handler
            ),
            self.servo_rear_feet_left_id,
            (self.servo_rear_feet_left_rest_angle + variation_feet),
        )
        
        
        self.move_servo(
            (
                self.bus_servo_adapter_1_packet_handler
                if self.servo_rear_shoulder_right_bus_servo_adapter == 1
                else self.bus_servo_adapter_2_packet_handler
            ),
            self.servo_rear_shoulder_right_id,
            (self.servo_rear_shoulder_right_rest_angle - 10 + move),
        )
        
        self.move_servo(
            (
                self.bus_servo_adapter_1_packet_handler
                if self.servo_rear_leg_right_bus_servo_adapter == 1
                else self.bus_servo_adapter_2_packet_handler
            ),
            self.servo_rear_leg_right_id,
            (self.servo_rear_leg_right_rest_angle + variation_leg),
        )

        self.move_servo(
            (
                self.bus_servo_adapter_1_packet_handler
                if self.servo_rear_feet_right_bus_servo_adapter == 1
                else self.bus_servo_adapter_2_packet_handler
            ),
            self.servo_rear_feet_right_id,
            (self.servo_rear_feet_right_rest_angle - variation_feet),
        )

        sts_comm_result_1 = (
            self.bus_servo_adapter_1_packet_handler.groupSyncWrite.txPacket()
        )
        if sts_comm_result_1 != COMM_SUCCESS:
            print(
                "%s"
                % self.bus_servo_adapter_1_packet_handler.getTxRxResult(
                    sts_comm_result_1
                )
            )

        sts_comm_result_2 = (
            self.bus_servo_adapter_2_packet_handler.groupSyncWrite.txPacket()
        )
        if sts_comm_result_2 != COMM_SUCCESS:
            print(
                "%s"
                % self.bus_servo_adapter_2_packet_handler.getTxRxResult(
                    sts_comm_result_2
                )
            )

        self.bus_servo_adapter_1_packet_handler.groupSyncWrite.clearParam()
        self.bus_servo_adapter_2_packet_handler.groupSyncWrite.clearParam()
        time.sleep(0.002)
        
        self.move_servo(
            (
                self.bus_servo_adapter_1_packet_handler
                if self.servo_front_shoulder_left_bus_servo_adapter == 1
                else self.bus_servo_adapter_2_packet_handler
            ),
            self.servo_front_shoulder_left_id,
            (self.servo_front_shoulder_left_rest_angle - 10 - move),
        )
        
        self.move_servo(
            (
                self.bus_servo_adapter_1_packet_handler
                if self.servo_front_leg_left_bus_servo_adapter == 1
                else self.bus_servo_adapter_2_packet_handler
            ),
            self.servo_front_leg_left_id,
            (self.servo_front_leg_left_rest_angle - variation_leg + 5),
        )

        self.move_servo(
            (
                self.bus_servo_adapter_1_packet_handler
                if self.servo_front_feet_left_bus_servo_adapter == 1
                else self.bus_servo_adapter_2_packet_handler
            ),
            self.servo_front_feet_left_id,
            (self.servo_front_feet_left_rest_angle + variation_feet - 5),
        )
        
        
        self.move_servo(
            (
                self.bus_servo_adapter_1_packet_handler
                if self.servo_front_shoulder_right_bus_servo_adapter == 1
                else self.bus_servo_adapter_2_packet_handler
            ),
            self.servo_front_shoulder_right_id,
            (self.servo_front_shoulder_right_rest_angle + 10 - move),
        )
        
        self.move_servo(
            (
                self.bus_servo_adapter_1_packet_handler
                if self.servo_front_leg_right_bus_servo_adapter == 1
                else self.bus_servo_adapter_2_packet_handler
            ),
            self.servo_front_leg_right_id,
            (self.servo_front_leg_right_rest_angle + variation_leg - 5),
        )

        self.move_servo(
            (
                self.bus_servo_adapter_1_packet_handler
                if self.servo_front_feet_right_bus_servo_adapter == 1
                else self.bus_servo_adapter_2_packet_handler
            ),
            self.servo_front_feet_right_id,
            (self.servo_front_feet_right_rest_angle - variation_feet + 5),
        )

        sts_comm_result_1 = (
            self.bus_servo_adapter_1_packet_handler.groupSyncWrite.txPacket()
        )
        if sts_comm_result_1 != COMM_SUCCESS:
            print(
                "%s"
                % self.bus_servo_adapter_1_packet_handler.getTxRxResult(
                    sts_comm_result_1
                )
            )

        sts_comm_result_2 = (
            self.bus_servo_adapter_2_packet_handler.groupSyncWrite.txPacket()
        )
        if sts_comm_result_2 != COMM_SUCCESS:
            print(
                "%s"
                % self.bus_servo_adapter_2_packet_handler.getTxRxResult(
                    sts_comm_result_2
                )
            )

        self.bus_servo_adapter_1_packet_handler.groupSyncWrite.clearParam()
        self.bus_servo_adapter_2_packet_handler.groupSyncWrite.clearParam()
        time.sleep(0.002)

        

    def body_move_position_left(self):

        move = 20

        variation_leg = 50
        variation_feet = 70
        
        
        
        self.move_servo(
            (
                self.bus_servo_adapter_1_packet_handler
                if self.servo_rear_shoulder_left_bus_servo_adapter == 1
                else self.bus_servo_adapter_2_packet_handler
            ),
            self.servo_rear_shoulder_left_id,
            (self.servo_rear_shoulder_left_rest_angle + 10 - move),
        )
        
        self.move_servo(
            (
                self.bus_servo_adapter_1_packet_handler
                if self.servo_rear_leg_left_bus_servo_adapter == 1
                else self.bus_servo_adapter_2_packet_handler
            ),
            self.servo_rear_leg_left_id,
            (self.servo_rear_leg_left_rest_angle - variation_leg),
        )

        self.move_servo(
            (
                self.bus_servo_adapter_1_packet_handler
                if self.servo_rear_feet_left_bus_servo_adapter == 1
                else self.bus_servo_adapter_2_packet_handler
            ),
            self.servo_rear_feet_left_id,
            (self.servo_rear_feet_left_rest_angle + variation_feet),
        )
        
        
        self.move_servo(
            (
                self.bus_servo_adapter_1_packet_handler
                if self.servo_rear_shoulder_right_bus_servo_adapter == 1
                else self.bus_servo_adapter_2_packet_handler
            ),
            self.servo_rear_shoulder_right_id,
            (self.servo_rear_shoulder_right_rest_angle - 10 - move),
        )
        
        self.move_servo(
            (
                self.bus_servo_adapter_1_packet_handler
                if self.servo_rear_leg_right_bus_servo_adapter == 1
                else self.bus_servo_adapter_2_packet_handler
            ),
            self.servo_rear_leg_right_id,
            (self.servo_rear_leg_right_rest_angle + variation_leg),
        )

        self.move_servo(
            (
                self.bus_servo_adapter_1_packet_handler
                if self.servo_rear_feet_right_bus_servo_adapter == 1
                else self.bus_servo_adapter_2_packet_handler
            ),
            self.servo_rear_feet_right_id,
            (self.servo_rear_feet_right_rest_angle - variation_feet),
        )

        sts_comm_result_1 = (
            self.bus_servo_adapter_1_packet_handler.groupSyncWrite.txPacket()
        )
        if sts_comm_result_1 != COMM_SUCCESS:
            print(
                "%s"
                % self.bus_servo_adapter_1_packet_handler.getTxRxResult(
                    sts_comm_result_1
                )
            )

        sts_comm_result_2 = (
            self.bus_servo_adapter_2_packet_handler.groupSyncWrite.txPacket()
        )
        if sts_comm_result_2 != COMM_SUCCESS:
            print(
                "%s"
                % self.bus_servo_adapter_2_packet_handler.getTxRxResult(
                    sts_comm_result_2
                )
            )

        self.bus_servo_adapter_1_packet_handler.groupSyncWrite.clearParam()
        self.bus_servo_adapter_2_packet_handler.groupSyncWrite.clearParam()
        time.sleep(0.002)
        
        self.move_servo(
            (
                self.bus_servo_adapter_1_packet_handler
                if self.servo_front_shoulder_left_bus_servo_adapter == 1
                else self.bus_servo_adapter_2_packet_handler
            ),
            self.servo_front_shoulder_left_id,
            (self.servo_front_shoulder_left_rest_angle - 10 + move),
        )
        
        self.move_servo(
            (
                self.bus_servo_adapter_1_packet_handler
                if self.servo_front_leg_left_bus_servo_adapter == 1
                else self.bus_servo_adapter_2_packet_handler
            ),
            self.servo_front_leg_left_id,
            (self.servo_front_leg_left_rest_angle - variation_leg + 5),
        )

        self.move_servo(
            (
                self.bus_servo_adapter_1_packet_handler
                if self.servo_front_feet_left_bus_servo_adapter == 1
                else self.bus_servo_adapter_2_packet_handler
            ),
            self.servo_front_feet_left_id,
            (self.servo_front_feet_left_rest_angle + variation_feet - 5),
        )
        
        
        self.move_servo(
            (
                self.bus_servo_adapter_1_packet_handler
                if self.servo_front_shoulder_right_bus_servo_adapter == 1
                else self.bus_servo_adapter_2_packet_handler
            ),
            self.servo_front_shoulder_right_id,
            (self.servo_front_shoulder_right_rest_angle + 10 + move),
        )
        
        self.move_servo(
            (
                self.bus_servo_adapter_1_packet_handler
                if self.servo_front_leg_right_bus_servo_adapter == 1
                else self.bus_servo_adapter_2_packet_handler
            ),
            self.servo_front_leg_right_id,
            (self.servo_front_leg_right_rest_angle + variation_leg - 5),
        )

        self.move_servo(
            (
                self.bus_servo_adapter_1_packet_handler
                if self.servo_front_feet_right_bus_servo_adapter == 1
                else self.bus_servo_adapter_2_packet_handler
            ),
            self.servo_front_feet_right_id,
            (self.servo_front_feet_right_rest_angle - variation_feet + 5),
        )

        sts_comm_result_1 = (
            self.bus_servo_adapter_1_packet_handler.groupSyncWrite.txPacket()
        )
        if sts_comm_result_1 != COMM_SUCCESS:
            print(
                "%s"
                % self.bus_servo_adapter_1_packet_handler.getTxRxResult(
                    sts_comm_result_1
                )
            )

        sts_comm_result_2 = (
            self.bus_servo_adapter_2_packet_handler.groupSyncWrite.txPacket()
        )
        if sts_comm_result_2 != COMM_SUCCESS:
            print(
                "%s"
                % self.bus_servo_adapter_2_packet_handler.getTxRxResult(
                    sts_comm_result_2
                )
            )

        self.bus_servo_adapter_1_packet_handler.groupSyncWrite.clearParam()
        self.bus_servo_adapter_2_packet_handler.groupSyncWrite.clearParam()
        time.sleep(0.002)
    
    def set_torque_for_all_servos(packetHandler,portHandler,board_ids, enable_torque):
        torque_value = 1 if enable_torque else 0
        action = "enabled" if enable_torque else "disabled"
        
        for board_id in board_ids:
            result, error = packetHandler.write1ByteTxRx(portHandler, board_id, STS_TORQUE_ENABLE, torque_value)
            if result != COMM_SUCCESS:
                print(f"[ID:{board_id:03d}] Error setting torque: {packetHandler.getTxRxResult(result)}")
            elif error:
                print(f"[ID:{board_id:03d}] Servo returned an error: {packetHandler.getRxPacketError(error)}")
            else:
                print(f"[ID:{board_id:03d}] Torque {action} successfully")

    def abort(self):
        servo_ids_board_1 = [0, 1, 2, 3, 4, 5]
        servo_ids_board_2 = [0, 1, 2, 3, 4, 5]
        self.set_torque_for_all_servos(self.bus_servo_adapter_1_packet_handler, self.bus_servo_adapter_1_port_handler, servo_ids_board_1, False)
        self.set_torque_for_all_servos(self.bus_servo_adapter_2_packet_handler, self.bus_servo_adapter_2_port_handler, servo_ids_board_2, False)
        

    def activate_servos(self):
        servo_ids_board_1 = [0, 1, 2, 3, 4, 5]
        servo_ids_board_2 = [0, 1, 2, 3, 4, 5]
        self.set_torque_for_all_servos(self.bus_servo_adapter_1_packet_handler, self.bus_servo_adapter_1_port_handler, servo_ids_board_1, True)
        self.set_torque_for_all_servos(self.bus_servo_adapter_2_packet_handler, self.bus_servo_adapter_2_port_handler, servo_ids_board_2, True)
        

    
   

   
