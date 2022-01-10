import rospy
from rospy import ROSException

from numpy import clip, polyfit, poly1d

from rvl_robotiq_controller.msg import Robotiq2FCommand, Robotiq2FStatus
from rvl_utilities.CustomLogger import ColorLogger

class Robotiq2FController:
    def __init__(self, stroke, default_force = 100, default_speed = 100, finger_thickness = 7.6, initialize = False, calibrate = False):
        # check definition
        if stroke != 85 and stroke != 140:
            raise ValueError('2F Gripper stroke must be 85 or 140 mm')
        else:
            self.stroke = stroke
            self.logger = ColorLogger(label = 'R2F' + str(self.stroke) + ' Controller')

        # factory preset (page 25 of user manual)
        self.force = int(clip(default_force, 0, 255))
        self.speed = int(clip(default_speed, 0, 255))

        # NBR Finger (page 121 of user manual)
        self.finger_thickness = finger_thickness # mm

        # mechanical specification (page 123 of user manual)
        if self.stroke == 85:
            self.force_range = (20.0, 235.0) # Newton
            self.speed_range = (20.0, 150.0) # mm/s
        else:
            self.force_range = (10.0, 125.0) # Newton
            self.speed_range = (30.0, 250.0) # mm/s

        # conversion ratio for force, speed, and distance (metrics to binary)
        self.f_ratio = poly1d(polyfit(self.force_range, (0, 255), 1))
        self.s_ratio = poly1d(polyfit(self.speed_range, (0, 255), 1))

        # theoretical
        # self.d_ratio = poly1d(polyfit((self.stroke, 0), (0, 255), 1))

        # measured
        self.d_ratio = poly1d(polyfit(x = [50.3, 42.05, 33.76, 25.5, 17.23],
                                      y = [100, 120, 140, 160, 180],
                                      deg = 1))

        # status keeping
        self.status = None

        if initialize:
            self.register()
            # self.reset()

        if initialize and calibrate:
            self.binary_range = self.calibrate()
        else:
            self.binary_range = (0, 255)

    def register(self, timeout = 10):
        try:
            self.logger.log_warn(f'Registering publisher and subscriber')
            rospy.wait_for_message('/Robotiq2F/gripper_status', Robotiq2FStatus, timeout = timeout)
            self.status_monitor = rospy.Subscriber('/Robotiq2F/gripper_status', Robotiq2FStatus, callback = self.status_monitor_callback)
            self.command_publisher = rospy.Publisher('/Robotiq2F/command_interface', Robotiq2FCommand, queue_size = 1)
            self.logger.log_warn(f'Waiting for {5} seconds so publishers and subscribers register correctly')
            rospy.sleep(5)
            self.logger.log_success(f'Controller registered successfully and ready to send commands')
        except ROSException as error:
            self.logger.log_error('Unable to assign controller to gripper. Did you start Robotiq Node?')
            self.logger.log_error(error)

    def calibrate(self):
        self.logger.log_warn('Calibrating binary limits')
        self.auto_open()
        rospy.sleep(5)
        upper = self.status.current_position
        rospy.sleep(1)
        self.auto_close()
        rospy.sleep(5)
        lower = self.status.current_position
        rospy.sleep(1)
        self.reset()
        self.logger.log_success('Calibration completed')
        return tuple(lower, upper)

    def auto_close(self, f = None, s = None, blocking = True):
        command = Robotiq2FCommand()
        command.activate = 1
        command.goto = 1
        command.position = 255
        command.speed = self.speed if s is None else s
        command.force = self.force if f is None else f
        self.compensated_publish(command)
        if blocking:
            self.block()

    def auto_open(self, f = None, s = None, blocking = True):
        command = Robotiq2FCommand()
        command.activate = 1
        command.goto = 1
        command.position = 0
        command.speed = self.speed if s is None else s
        command.force = self.force if f is None else f
        self.compensated_publish(command)
        if blocking:
            self.block()

    def grasp_soft(self, s = None, blocking = True):
        command = Robotiq2FCommand()
        command.activate = 1
        command.goto = 1
        command.position = 255
        command.speed = self.speed if s is None else s
        command.force = 0
        self.compensated_publish(command)
        if blocking:
            self.block()

    def grasp_soft_regrasp(self, s = None, blocking = True):
        command = Robotiq2FCommand()
        command.activate = 1
        command.goto = 1
        command.position = 255
        command.speed = self.speed if s is None else s
        command.force = 1
        self.compensated_publish(command)
        if blocking:
            self.block()

    def grasp_medium(self, s = None, blocking = True):
        command = Robotiq2FCommand()
        command.activate = 1
        command.goto = 1
        command.position = 255
        command.speed = self.speed if s is None else s
        command.force = 128
        self.compensated_publish(command)
        if blocking:
            self.block()

    def grasp_hard(self, s = None, blocking = True):
        command = Robotiq2FCommand()
        command.activate = 1
        command.goto = 1
        command.position = 255
        command.speed = self.speed if s is None else s
        command.force = 255
        self.compensated_publish(command)
        if blocking:
            self.block()

    def open_gripper(self, value, unit = 'mm', s = None, f = None, blocking = True):
        acceptable_units = ['raw', 'mm', 'in']
        if unit not in acceptable_units:
            self.logger.log_error(f'{unit} is invalid unit')
            self.logger.log_error(f'Expecting {acceptable_units}')
            return False
        else:
            command = Robotiq2FCommand()
            command.activate = 1
            command.goto = 1
            command.speed = self.speed if s is None else s
            command.force = self.force if f is None else f

            if unit == 'raw':
                command.position = int(clip(value, *self.binary_range))
            elif unit == 'mm':
                command.position = round(self.d_ratio(clip(value, 0, self.stroke)))
            else:
                command.position = round(self.d_ratio(clip(value * 25.4, 0, self.stroke)))

            print(command.position)

            # self.compensated_publish(command)
            # if blocking:
            #     self.block()
            return True

    def set_gripper_speed(self, value, unit = 'mm/s'):
        acceptable_units = ['raw', 'mm/s', 'in/s']
        if unit not in acceptable_units:
            self.logger.log_error(f'{unit} is invalid unit')
            self.logger.log_error(f'Expecting {acceptable_units}')
            return False
        else:
            if unit == 'raw':
                self.speed = int(clip(value, 0, 255))
            elif unit == 'mm/s':
                self.speed = round(self.s_ratio(clip(value, *self.speed_range)))
            else:
                self.speed = round(self.s_ratio(clip(value * 25.4, *self.speed_range)))
            return True

    def set_gripper_force(self, value, unit = 'newton'):
        acceptable_units = ['raw', 'newton', 'poundf']
        if unit not in acceptable_units:
            self.logger.log_error(f'{unit} is invalid unit')
            self.logger.log_error(f'Expecting {acceptable_units}')
            return False
        else:
            if unit == 'raw':
                self.force = int(clip(value, 0, 255))
            elif unit == 'newton':
                self.force = round(self.f_ratio(clip(value, *self.force_range)))
            else:
                self.force = round(self.f_ratio(clip(value * 0.224809, *self.force_range)))
            return True

    def set_gripper_joint_angle(self, value, unit = 'rad'):
        raise NotImplementedError

    def is_moving(self):
        return (self.status.object_status == 0)

    def is_holding(self):
        return (self.status.object_status == 1 or self.status.object_status == 2)

    def reset(self):
        self.logger.log_warn(f'Gripper reset')
        self.deactivate()
        self.activate()
        rospy.sleep(3)

    def deactivate(self):
        command = Robotiq2FCommand()
        command.activate = 0
        self.compensated_publish(command)
        rospy.sleep(1)

    def activate(self):
        command = Robotiq2FCommand()
        command.activate = 1
        self.compensated_publish(command)
        rospy.sleep(1)

    def compensated_publish(self, command, lag = 0.1):
        self.command_publisher.publish(command)
        rospy.sleep(lag)

    def status_monitor_callback(self, msg):
        self.status = msg

    def block(self):
        while self.is_moving():
            rospy.sleep(0.1)

class Robotiq3FController:
    def __init__(self):
        raise NotImplementedError