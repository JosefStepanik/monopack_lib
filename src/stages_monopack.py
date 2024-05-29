#!/usr/bin/python3

# stages_monopack.py
#
# ~~~~~~~~~~~~
#
# Module with StagePI class for controlling X, Y stages with two Monopack drivers.
#
# ~~~~~~~~~~~~
#
# ------------------------------------------------------------------
# Author : Josef Stepanik
# Language: Python3
# ------------------------------------------------------------------
#
# Copyright (C) 2022-2024 IQS Group  
# Info at http://www.iqsgroup.cz
# 

# Import built-in modules.
import os
import sys
import time

# find paths to the modules
file_path   = os.path.dirname(__file__)
depend_path = os.path.join('..', 'abstract_classes')

# add path to the modules
abspath = os.path.abspath(os.path.join(file_path, depend_path))
sys.path.append(abspath)

# import own modules
from loguru import logger
from monopack_v2 import MonoPack
from stagesAbstract import StagesAbstract

class StagesPI(StagesAbstract):
    """
    Class for managing XY stage from PI based on model 410.2S linear stages.
    """

    x_min = 0
    x_max = 400
    y_min = 0
    y_max = 400

    x_min_limit = x_min
    x_max_limit = x_max
    y_min_limit = y_min
    y_max_limit = y_max
    


    def __init__(self, master, can, verbose: bool = True, idx = 7, idy = 1):
        """
        Initialize parameters necessary for functions inside class. Nicknames for stages,
        switches, directions, regimes, etc. are defined. Runs initializing procedures for
        serial connection and stages.
        """
        self.m_can = can  # instance of can object
        self.IDX = idx
        self.IDY = idy
        self.master = master
        self.stages = {'X': '1', 'Y': '2'}  # nicknames for stages

        self.x_center = 200
        self.y_center = 200

        self.is_connected = False
        self.is_enabled = True
        self.is_referenced = False
        
        self.init_msg = None

        # # positions of stages for resist alignment - position is tuned by hex tools
        # self.resist_alignment_positions = {'F1': [8.0, 52.0],
        #                                    'F2': [73.0, 16.0],
        #                                    'F3': [73.0, 86.0],
        #                                    'CENTER': [40.5, 52.0],
        #                                    'HOME': [0, 0]}

        self.verbose = verbose  # if detailed information should be printed

        self.is_referenced = None
        self.error_prefix = 'STAGES_________DUMMY: '

        self.init_msg = self.connect()  # try to establish connection to the controller
        #self.init_stages()  # initialize stages

    def connect(self):
        """
        Try to establish connection to the controller machine. If successful, set is_connected to
        """
        # Initializing of two axis.
        try:
            self.axis_x = MonoPack(can_object=self.m_can, address=self.IDX)
            self.axis_y = MonoPack(can_object=self.m_can, address=self.IDY)
            x = self.axis_x.get_version_number()
            y = self.axis_y.get_version_number()
            msg_x = 'Axis X controller: FW version = {} and temperature {} \u00B0C\n'.format(x[0], x[2])
            msg_y = 'Axis Y controller: FW version = {} and temperature {} \u00B0C\n'.format(y[0], y[2])
            logger.info(msg_x)
            logger.info(msg_y)
            self.is_connected = True
            return msg_x, msg_y
        except:
            self.is_connected = False
            return None
            logger.error(self.error_prefix + 'Stage can\'t be connected.')

    def disconnect(self):

        self.is_connected = False

    def init_stages(self):
        """
        Initialize stages X and Y. Set initial values of important parameters and set
        both motors on.
        """

        if self.is_connected:
            self.set_default_parameters()

            if self.is_referenced:
                self.update_current_pos()
            else:
                logger.debug('Now we can send command to reference stages')
                self.reference_stages()

    def set_default_parameters(self):
        '''
        Set default parameters for stages. These comands 
        '''

        self.axis_x.reset_alarm()
        self.axis_x.set_can_send_id(ID=7)
        self.axis_x.pid_6F(P0=0, P1=0)
        self.axis_x.reset_position()
        self.axis_x.set_alarm_mode(P1=1, P2=1)
        self.axis_x.set_current_control(P1=0, P2=0x80, P3=0xC8)
        self.axis_x.set_deviation_alarm(P1=0, P2=0, correction_start_after=1)
        self.axis_x.conf_auto_position_correction(P1=5, tolerance=10)
        self.axis_x.set_frequency_range(P1=self.axis_x.PREDIVIDER)
        self.axis_x.set_microstep_resolution(P1=50, P2=0, P5=1)
        self.axis_x.encoder_configuration(P1=64, P2=3, P5=2, deviation=16)
        self.axis_x.set_switch_mode(P2=1, P3=0, P4=1, P5=0, P6=0)
        self.axis_x.set_velocity(velocity=4915, acceleration=245)
        self.axis_x.set_current_control(P1=0x47, P2=0x99, P3=0xE0)
        self.axis_x.conf_auto_position_correction(P1=5, tolerance=50)
        self.axis_x.set_deceleration_at_stop_switches(deceleration=0)
        self.axis_x.set_current_control(P1=0x47, P2=0xAD, P3=0xFF)
        self.axis_x.set_reference_search_velocity(velocity=1228)
        

        self.axis_y.reset_alarm()
        self.axis_y.set_can_send_id(ID=1)
        self.axis_y.pid_6F(P0=0, P1=0)
        self.axis_y.reset_position()
        self.axis_y.set_alarm_mode(P1=1, P2=1)
        self.axis_y.set_current_control(P1=0, P2=0x80, P3=0xC8)
        self.axis_y.set_deviation_alarm(P1=0, P2=0, correction_start_after=1)
        self.axis_y.conf_auto_position_correction(P1=5, tolerance=10)
        self.axis_y.set_frequency_range(P1=self.axis_y.PREDIVIDER)
        self.axis_y.set_microstep_resolution(P1=50, P2=0, P5=1)
        self.axis_y.encoder_configuration(P1=64, P2=3, P5=2, deviation=16)
        self.axis_y.set_switch_mode(P2=1, P3=0, P4=1, P5=0, P6=0)
        self.axis_y.set_velocity(velocity=4915, acceleration=245)
        self.axis_y.set_current_control(P1=0x47, P2=0x99, P3=0xE0)
        self.axis_y.conf_auto_position_correction(P1=5, tolerance=50)
        self.axis_y.set_deceleration_at_stop_switches(deceleration=0)
        self.axis_y.set_current_control(P1=0x47, P2=0xAD, P3=0xAD)
        self.axis_y.set_reference_search_velocity(velocity=1228)


        if self.verbose:
            x = self.axis_x.get_acceleration_velocity_settings()
            y = self.axis_y.get_acceleration_velocity_settings()
            logger.info('Velocity X check: {}'.format(x[2]))
            logger.info('Velocity Y check: {}'.format(y[2]))
            logger.info('Acceleration X check: {}'.format(x[0]))
            logger.info('Acceleration Y check: {}'.format(y[0]))
            logger.info('Reference search velocity X check: {}'.format(x[1]))
            logger.info('Reference search velocity Y check: {}'.format(y[1]))

    def enable_stages(self, new_state: bool = True):

        self.is_enabled = new_state
        message = 'enabled' if new_state else 'DISABLED'
        logger.info(f'MonoPack Stages {message}')

    # Motor status functions ----------------------------------------------------------------

    # UNDONE: Unimplemented get_error function
    def get_error(self, stage: str):
        """
        Get error number for specified stage and reset error status.
        """
        if stage=='X':
            answer = self.axis_x.reset_alarm()
        if stage=='Y':
            answer = self.axis_y.reset_alarm()
        answer = self.write_serial(self.stages[stage] + ' ERR?')
        return answer

    def reboot_controller(self, stage: str):
        """
        Reboot controller for current axis
        """
        if stage=='X':
            self.axis_x.hardware_reset()
        if stage=='Y':
            self.axis_y.hardware_reset()
    

    def __get_real_position(self, stage: str,
                            recursion: int = 0,
                            verbose: bool = True):
        """
        Get real position of specified stage in mm.
        stage = 'X' or 'Y'
        """

        if self.is_connected:
            try:
                if recursion and verbose:
                    logger.info(f'Recursive call {recursion} started: __get_real_position')

                answer = lambda stage: self.axis_x.get_encoder_counter() if stage == 'X' else self.axis_y.get_encoder_counter()
                answer = answer(stage) * self.axis_x.STEP # encoder counter to mm
                # if recursion and verbose:
                #     logger.info(f'Recursive call ({recursion}) of __get_real_position, answer : "{answer}"')

                # if not MIN_X < answer < MAX_X :
                #     if verbose:
                #         logger.info('__get_real_position ERROR! Got answer: "{}"'.format(answer))
                #         logger.info('Answer value', len(answer))
                #         logger.info('Will start recursive call of "__get_real_position"')
                #     return self.__get_real_position(stage, recursion=recursion + 1)
                # else:
                #     return answer 
                return answer
            except Exception as err:
                logger.warning('Something went wrong: {}'.format(err))
                #return self.__get_real_position(stage, recursion=recursion + 1)
        else:
            return 0

    def get_on_target_state(self,
                            stage: str,
                            recursive: bool = False):
        """
        Check if the specified stage is on target position.

        Answer is like this: 0 1 1=0
        =0 - NOT on target
        =1 - is on target position

        """

        command = self.stages[stage] + ' ONT?'
        return 0


    # Motor action functions ----------------------------------------------------------------
    def check_position_limit(self, x: float = None,
                             y: float = None):
        '''
        Check position of the stages with respect to the maximum and minimum values.
        '''
        if x is not None:
            if self.x_min <= x <= self.x_max:
                return True
            else:
                return False

        if y is not None:
            if self.y_min <= y <= self.y_max:
                return True
            else:
                return False

    def limit_positions(self, x: float = None,
                        y: float = None):
        '''
        Check if the position is within the limits. If not, return the limit value.
        '''
        if x is not None:
            if self.check_position_limit(x=x):
                return x
            else:
                if x <= self.x_min:
                    logger.info(f'limit_positions x {x}  replaced by {self.x_min}')
                    return self.x_min
                elif x >= self.x_max:
                    logger.info(f'limit_positions x {x} replaced by {self.x_max}')
                    return self.x_max

        if y is not None:
            if self.check_position_limit(y=y):
                return y
            else:
                if y <= self.y_min:
                    logger.info('limit_positions y ', y, ' replaced by ', {self.y_min})
                    return self.y_min
                elif y >= self.y_max:
                    logger.info('limit_positions y ', y, ' replaced by ', self.y_max)
                    return self.y_max

    @staticmethod
    def limit_positions_static(x: float = None,
                               y: float = None):

        x_min = StagesPI.x_min
        x_max = StagesPI.x_max

        y_min = StagesPI.y_min
        y_max = StagesPI.y_max

        if x is not None:
            if x <= x_min:
                logger.info(f'limit_positions x {x}  replaced by {x_min}')
                return x_min
            elif x >= x_max:
                logger.info('limit_positions x ', x, ' replaced by ', x_max)
                return x_max
            else:
                return x

        if y is not None:
            if y <= y_min:
                logger.info('limit_positions y ', y, ' replaced by ', {y_min})
                return y_min
            elif y >= y_max:
                logger.info('limit_positions y ', y, ' replaced by ', y_max)
                return y_max
            else:
                return y



    def stop(self):
        '''
        Call this function to stop the motors.
        '''
        super().stop()
        self.stop_motors()

    def stop_motors(self):
        """
        Stop motor smoothly.
        """
        self.axis_x.soft_stop()
        self.axis_y.soft_stop()
        self.update_current_pos()

    # UNDONE: Unimplemented move_x_relative function
    def move_x_relative(self, shift: float):
        """
        Move stage X about the distance specified in shift in millimeters. Positive shift
        means moving to the left, negative to the right.
        """
        if self.is_referenced and self.is_connected and self.is_enabled:
            x_new = self.limit_positions(x=(self.x_mm + shift))
            command = '1 MOV 1 ' + str(x_new)
            answer = self.write_serial(command)
            # answer = self.write_serial('1 MVR 1 ' + str(shift))
            self.set_new_pos(x=x_new)

    # UNDONE: Unimplemented move_y_relative function
    def move_y_relative(self, shift: float):
        """
        Move stage Y about the distance specified in shift in millimeters. Positive shift
        means moving to the left, negative to the right.
        """
        if self.is_referenced and self.is_connected and self.is_enabled:
            y_new = self.limit_positions(y=self.y_mm + shift)
            # answer = self.write_serial('2 MVR 1 ' + str(shift))
            answer = self.write_serial('2 MOV 1 ' + str(y_new))
            self.set_new_pos(y=y_new)

    def go_home(self, stage: str = 'XY'):
        """
        Go to home position - 0.
        By default both stages are homed.
        """
        if self.is_referenced and self.is_connected and self.is_enabled:
            if 'X' in stage:
                self.axis_x.drive_a_ramp(position=0)
            if 'Y' in stage:
                self.axis_y.drive_a_ramp(position=0)
    
    def move_to_center(self):
        '''
        Move the stages to the center of the stage.
        '''
        self.move_to_x(self.x_center)
        self.move_to_y(self.y_center)

    def move_to_x(self, position: float):
        """
        Move stage X to a specified absolute position in millimeters.
        """
        if self.is_referenced and self.is_connected and self.is_enabled:
            x_new = self.limit_positions(x=position)
            self.axis_x.drive_a_ramp(position=round(x_new/self.axis_x.STEP))
            self.set_new_pos(x=x_new)

    def move_to_y(self, position: float):
        """
        Move stage Y to a specified absolute position in millimeters.
        """
        if self.is_referenced and self.is_connected and self.is_enabled:
            y_new = self.limit_positions(y=position)
            self.axis_y.drive_a_ramp(position=round(y_new/self.axis_y.STEP))
            self.set_new_pos(y=y_new)

    def move_to_xy(self, position_x: float, position_y: float):
        '''
        Move the stages to the specified position.
        '''
        self.move_to_x(position_x)
        self.move_to_y(position_y)

    def set_new_pos(self, x: float = None,
                    y: float = None):

        if x is not None:
            x_new = self.limit_positions(x=x)
            self.x_mm = x_new
        if y is not None:
            y_new = self.limit_positions(y=y)
            self.y_mm = y_new
        return

    def update_current_pos(self):
        '''
        Update the current position of the stages.
        '''
        self.set_new_pos(x=self.get_x(),
                         y=self.get_y())
        return

    def reference_stages(self,
                         stage: str = 'XY',
                         force_reference: bool = False):
        '''
        Reference the stages. If force_reference is True, the stages will be referenced even if they are already referenced.
        '''

        if self.verbose:
            logger.info('Referencing stages')
        # if force_reference:
        #     self.set_new_pos(x=0.0, y=0.0)

        # try:
        if 'X' in stage:
            self.axis_x.reference_search()
        if 'Y' in stage:
            self.axis_y.reference_search()

        if 'X' in stage:
            self.is_ready('X')
        if 'Y' in stage:
            self.is_ready('Y')

        self.axis_x.pid_6F(P0=0, P1=0)
        self.axis_y.pid_6F(P0=0, P1=0)
        time.sleep(0.5)
        self.axis_x.reset_position()
        self.axis_y.reset_position()
        time.sleep(1.5)
        logger.debug('Time sleep 1.5 s')
        
        if 'X' in stage:
            self.axis_x.drive_a_ramp(P0=0x01, position=2500)
        if 'Y' in stage:
            self.axis_y.drive_a_ramp(P0=0x01, position=2500)
    
        if 'X' in stage:
            self.is_ready('X')
        if 'Y' in stage:
            self.is_ready('Y')
            
        logger.info('Stages are referenced')
        self.is_referenced = True
        self.print_current_positions()
        self.master.runx_button.configure(state='normal')
        self.master.runy_button.configure(state='normal')


    def is_referenced(self,
                      run_reference_procedure: bool = True) -> bool:
        '''
        Check if the motors are referenced. If not, run the reference procedure.
        '''

        if self.is_referenced:
            return True

        else:
            if run_reference_procedure:
                logger.info('Getting reference status of stages.')
                self.reference_stages()
                if self.is_referenced:
                    return True

        raise Exception('Unable to reference motors')

    def is_ready(self, stage):
        '''
        Check if the stage is ready after move by checking the actual velocity.
        '''
        check = lambda stage: (self.axis_y.get_actual_acceleration_velocity() if stage == 'Y' else self.axis_x.get_actual_acceleration_velocity())
        is_finished = None  # self.get_on_target_state(stage)
        
        while True:
            #  TODO interruption by user!!! User should be able to kill the motors.
            #   Before user could kill the engine by pressing F1
            # events = pygame.event.get()
            # mods = pygame.key.get_mods()
            # for event in events:
            #     if event.type == KEYDOWN:
            #         # interrupt
            #         if event.key == K_F1:
            #             is_finished = '1'
            #             self.stop_motors()

            if is_finished == '1':
                break
            else:
                try:
                    attempt = 0
                    while attempt < 2 :
                        check_it = check(stage)     # Three times check the stage for movement. If actual velocity is 0 and reference search is not active , then the stage is ready.
                        if not check_it[0] and not check_it[2]:
                            attempt += 1
                        else:
                            attempt = 0
                            logger.debug('Resetting attempt to 0')
                        time.sleep(0.3)
                    is_finished = '1'
                    logger.debug('Stage ' + stage + ' is ready.')
                    return is_finished
                except Exception as err:
                    raise
                    logger.debug('Error by checking if is ready: {}'.format(err))
                    return '0'

    def get_x(self) -> float:
        # raise Exception('Do not use this!')
        # return self.x_mm
        return self.__get_real_position('X')

    def get_y(self) -> float:
        # raise Exception('Do not use this!')
        # return self.y_mm
        return self.__get_real_position('Y')

    def print_current_positions(self):
        logger.info(f"Current position: X {self.__get_real_position('X')}, Y: {self.__get_real_position('Y')}")
