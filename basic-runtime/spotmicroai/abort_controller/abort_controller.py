import signal
import RPi.GPIO as GPIO
import sys
from spotmicroai.utilities.log import Logger
from spotmicroai.utilities.config import Config
import spotmicroai.utilities.queues as queues

log = Logger().setup_logger('Abort controller')


class AbortController:
    gpio_port = None

    def __init__(self, communication_queues):

        try:

            log.debug('Starting controller...')

            signal.signal(signal.SIGINT, self.exit_gracefully)
            signal.signal(signal.SIGTERM, self.exit_gracefully)

            self.gpio_port = Config().get(Config.ABORT_CONTROLLER_GPIO_PORT)

            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.gpio_port, GPIO.OUT)

            self._abort_queue = communication_queues[queues.ABORT_CONTROLLER]
            #self._lcd_screen_queue = communication_queues[queues.LCD_SCREEN_CONTROLLER]

            self.abort()

            #self._lcd_screen_queue.put(queues.LCD_SCREEN_SHOW_ABORT_CONTROLLER_OK_ON)

        except Exception as e:
            log.error('Abort controller initialization problem', e)
            #self._lcd_screen_queue.put(queues.LCD_SCREEN_SHOW_ABORT_CONTROLLER_NOK)
            try:
                self.abort()
            finally:
                sys.exit(1)

    def exit_gracefully(self, signum, frame):
        try:
            self.abort()
        finally:
            log.info('Terminated')
            sys.exit(0)

    def do_process_events_from_queue(self):

        try:
            while True:
                event = self._abort_queue.get()

                if event == queues.ABORT_CONTROLLER_ACTION_ACTIVATE:
                    self.activate_servos()

                if event == queues.ABORT_CONTROLLER_ACTION_ABORT:
                    self.abort()

        except Exception as e:
            log.error('Unknown problem while processing the queue of the abort controller', e)
            sys.exit(1)

    def activate_servos(self):
        #self._lcd_screen_queue.put(queues.LCD_SCREEN_SHOW_ABORT_CONTROLLER_OK_ON)
        GPIO.output(self.gpio_port, GPIO.LOW)

    def abort(self):
        #self._lcd_screen_queue.put(queues.LCD_SCREEN_SHOW_ABORT_CONTROLLER_OK_OFF)
        GPIO.output(self.gpio_port, GPIO.HIGH)
