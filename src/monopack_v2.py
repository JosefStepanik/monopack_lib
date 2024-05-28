#!/usr/bin/python3

# monopack_v2.py
#
# ~~~~~~~~~~~~
#
# Module with fuctions for control stepper motor through MONOpack2 V2 driver from Trinamic manufacturer.
#
# ~~~~~~~~~~~~
#
# ------------------------------------------------------------------
# Author : Josef Stepanik
# Language: Python 3
# ------------------------------------------------------------------
#
# Copyright (C) 2022-2024 IQS Group  
# Info at http://www.iqsgroup.cz
# 
'''
Stepper motors Vexta PK266-E2.0B for X,Y axis with basic step angle 1.8 degree (200 steps per revolution).
X and Y axis have trapezodial shafts with pitch 2mm (measured). One rotation is 10000 steps
These axis have drivers Monopack 2 from Trinamic. 
This driver has parameters: microstep resolution 32(2 mm / (200*32) = 0.0003125 mm). 
Monopack driver parameters are discussed in manual V1.04 section 4.3.1. About the Parameters paragraf. 

##########################################################################################
Format of the data telegrams. See Monopack 2 Manual V1.04.
## First byte | Second byte | Third byte | Fourth byte | Fifth byte | Sixth byte | Seventh byte | Eigth byte | Ninth byte ##
##  Adress ID |   Command   |     P0     |     P1      |     P2     |     P3     |      P4      |     P5     |     P6     ##
For right value of P0 byte see manual s.4.1 . P0 byte = 0: set value and store the new value in the EEPROM
                                                        1: set value, but do not alter the EEPROM
                                                        2: read the value from the EEPROM
                                                        3: read the actual value
Some commands are followed with their answer ($43, $74, $21... etc)
##########################################################################################

'''

# Import built-in modules.
import struct
import os
import sys

# find paths to the modules
file_path   = os.path.dirname(__file__)
depend_path = os.path.join('..', 'Peak_PCAN', 'src')

# add path to the modules
abspath = os.path.abspath(os.path.join(file_path, depend_path))
sys.path.append(abspath)

# import own modules
from loguru import logger
from tool_can import *


# Exception class.
class InvalidValue(ValueError): pass


class MonoPack():
    '''
    Class defines the driver with ID, list of instructions(commands) and operations.
    '''
    def __init__(self, can_object= NewPCANBasic(), address=None, step = 0.0002, fclk = 16000000, predivider = 5):
        '''
        >>> driver_x = MonoPack(address = 0x07)
        >>> driver_x.id
        7
        '''
        self.can_object             = can_object
        self.id                     = address
        self.STEP                   = step
        self.FCLK                   = fclk
        self.PREDIVIDER             = predivider
        

    def speed_mms1_to_steps(self, speed):
        '''
        Convert speed from mm/s to velocity value.
        '''
        return round((speed/(self.FCLK * self.STEP)) * 2**(15 + self.PREDIVIDER))
    
    def acceleration_mms2_to_steps(self, acceleration):
        '''
        Convert acceleration from mm/s**2 to steps.
        '''
        return round((acceleration/(self.FCLK * self.STEP)) * 2**(15 + self.PREDIVIDER))
        
        
    #***********************************************************************************#
    #                              MOTOR PARAMETERS METHODS                             #
    #***********************************************************************************#    
    def set_current_limit(self, P0=0x01, P1=None):
        '''
        Info in manual section 4.2.1 Current Limit ($10).
        Set the absolute maximum current.
        
        CMD:    $10
        P0:     Parameter storage control (s. 4.1 in manual)
        P1:     0: Selected by MC0 and MC1 inputs
                1: 0.8A when DIP switch #8 is OFF and 4.15A when switch #8 is ON
                2: 1.6A when DIP switch #8 is OFF and 5.0A when switch #8 is ON
                3: 2.5A when DIP switch #8 is OFF and 5.7A when switch #8 if ON
        '''
        if P1 not in [0, 1, 2, 3]:
            raise InvalidValue("P1 value must be 0, 1, 2 or 3.")
        command = [0x10, P0, P1, 0x0, 0x0 , 0x0, 0x0, 0x0]
        return self.can_object.write_message(self.id, command)
        
    def set_current_control(self, P0=0x01, P1=None, P2=None, P3=None):
        '''
        Info in manual section 4.2.2 Current Control ($11).
        Set up the motor current control. The current is separately controlled for the standby phase (v=0), the acceleration phase and the constant velocity phase. All three values with this command have a range of 0..255, where 255 means 100% (5A or 2.5A, demand on DIP 8.) of the selected absolute maximum current.
        
        CMD:    $11
        P0:     Parameter storage control (s. 4.1 in manual)
        P1:     Standby current (motor standing still) (0..255)
        P3:     Acceleration phase current (0..255)
        P2:     Active current (motor rotating with constant velocity) (0..255)
        '''
        if P1 not in range(256) or P2 not in range(256) or P3 not in range(256):
            raise InvalidValue("P1, P2 and P3 values must be in range 0..255.")
        command = [0x11, P0, P1, P2, P3 , 0x0, 0x0, 0x0]
        return self.can_object.write_message(self.id, command)
        
        
    def get_current_control_settings(self):
        '''
        Info in manual section 4.2.3 Get Current Control Settings ($53).
        Get the current control settings from the internal EEPROM (as set up using command $11).

        CMD:    $53
        P0:     0
        
        Answer
        CMD:    $53
        P0:     0
        P1:     Standby current
        P2:     Active current
        P3:     Acceleration current
        '''
        command = [0x53, 0x0, 0x0, 0x0, 0x0 , 0x0, 0x0, 0x0]
        response_data = None
        try:
            response_data = bytearray(self.can_object.write_read(self.id, command)[1].DATA)
            standby_current, active_current, accelaration_current = struct.unpack('B', response_data[2:5])
        except Exception as err:
            logger.debug("Get current control settings function was done with error: {}!".format(err))
            raise
        else:
            logger.debug("Get current control settings function was done without errors.")
        return standby_current, active_current, accelaration_current
            
    def set_frequency_range(self, P0=0x01, P1=None):
        '''
        Info in manual section 4.2.4 Frequency Range ($12).
        Set the frequency range for all ramp operations. The following formula defines the microstep frequency. Fclk is the clock frequency of the device, which is 16MHz.  Fstep = Fclk*v/2^(15+F)
        
        CMD:    $12
        P0:     Parameter storage control (s. 4.1 in manual)
        P1:     Pre-divider value (f) (0..15)
        '''
        if P1 not in range(16):
            raise InvalidValue("P1 value must be in range 0..15.")
        command = [0x12, P0, P1, 0x0, 0x0 , 0x0, 0x0, 0x0]
        return self.can_object.write_message(self.id, command)
        
    def set_microstep_resolution(self, P0=0x01, P1=None, P2=None, P5=None):
        '''
        Info in manual section 4.2.5 Microstep Resolution, Waveform and Mixed Decay ($17).
        This command allows setting the microstep resolution and the waveform which is used to generate the microsteps.
        The number of microsteps can either be set between 1 and 64 or to 65(resolution 100.8), 66(resolution 202.125) or 67(resolution 406.5).
        The second parameter controls the waveform. It is a sine wave when set to zero, a triangular wave when set to -1.0 and a trapezoid wave when set to +1.0. Please note that the waveform can only besetwhen the microstep resolution is set to a value between 1 and 64. If other microstep resolutions are used, the waveform parameter will be ignored and a sine wave will be used.
        The mixed decay setting especially at rotation velocities in the range of a few 10 steps per seconds to several 100 steps per second improves motor behavior (less resonance). However, theactualperformance depends on the motor and mechanics. For supply voltages above 24V and for low inductivity motors, best microstep behavior is reached when mixed decay setting is continuously on.
        
        CMD:    $17
        P0:     Parameter storage control (s. 4.1 in manual)
        P1:     Microstep resolution (1..64 or 65, 66, 67)
        P2:     Waveform (-1..+1)*127 (signed byte). (-127 (0x81) means -1.0, +127 (0x7F) means +1.0)
        P5:     0: Mixed Decay disabled
                1: Mixed Decay enabled
        '''
        if P1 not in range(1, 68) or P2 not in [0x81,0x00, 0x7F] or P5 not in [0,1]:
            raise InvalidValue("P1, P2 or P5 values are not in correct range.")
        command = [0x17, P0, P1, P2, 0x0 , 0x0, P5, 0x0]
        return self.can_object.write_message(self.id, command)
        
    def set_velocity(self, P0= 0x01, velocity=None, acceleration=None):
        '''
        Info in manual section 4.3.2 Velocity and Maximum Acceleration ($14).
        Set the acceleration and the maximum velocity.
        Inputs
        velocity:       mm/s .
        acceleration:   mm/s**2 
        
        CMD:    $14
        P0:     Parameter storage control (s. 4.1 in manual)
        P1,P2:  Maximum Acceleration. Decimal 1..8191 means in little indian 0x0100..xFF1F.
        P3,P4:  Maximum Velocity. Decimal 1..8191 means in little indian 0x0100..xFF1F.
        '''
        if velocity not in range(1, 8192) or acceleration not in range(1, 8192):
            raise InvalidValue("Velocity and acceleration values must be in range 1..8191.")
        P1, P2, P3, P4 = struct.pack('<2h', acceleration, velocity)
        command = [0x14, P0, P1, P2, P3, P4, 0x0, 0x0]
        return self.can_object.write_message(self.id, command)
        
    def set_bow_value(self, P0= 0x01, bow=None):
        '''
        Info in manual section 4.3.3 Bow Value ($63).
        Set the bow value for the ramp generation. This value must not be set to zero.
        Command $23 (s. 4.3.7) uses S-shaped ramps. With these ramps, the bow parameter is used: a high bow value increases the positioning speed, while a low bow value smoothens the acceleration ramp.
        
        CMD:    $63
        P0:     Parameter storage control (s. 4.1 in manual)
        P1,P2:  Bow. Decimal 1..8191 means in little indian 0x0100..0xFF1F.
        '''
        if bow not in range(1, 8192):
            raise InvalidValue("Bow value must be in range 1..8191.")
        P1, P2 = struct.pack('<H', bow)
        command = [0x63, P0, P1, P2, 0x0 , 0x0, 0x0, 0x0]
        return self.can_object.write_message(self.id, command)
        
    def get_acceleration_velocity_settings(self):
        '''
        Info in manual section 4.3.4 Get Acceleration and Velocity Settings ($52).
        Get the acceleration and velocity settings from the internal EEPROM.
        
        CMD:    $52
        P0:     0
        
        Answer
        CMD:    $52
        P0:     0
        P1,P2:  Maximum Acceleration little indian
        P3,P4:  Reference Search Velocity little indian
        P5,P6:  Maximum Velocity little indian
        '''
        response_data = None
        command = [0x52, 0x0, 0x0, 0x0, 0x0 , 0x0, 0x0, 0x0]
        try:
            response_data = bytearray(self.can_object.write_read(self.id, command)[1].DATA)
            maximum_acceleration, reference_search_velocity, maximum_velocity = struct.unpack('<3h', response_data[2:])
        except Exception as err:
            logger.debug("Get the acceleration and velocity settings was done with error: {}!".format(err))
            raise
        else:
            logger.debug("Get the acceleration and velocity settings function was done without errors.")
        return maximum_acceleration, reference_search_velocity, maximum_velocity
            
    def get_actual_position(self):
        '''
        Info in manual section 4.3.5 Get actual Position ($20).
        CMD:    $20
        Get the actual position of the motor.
        
        P0:     0
        
        Answer
        CMD:    $20
        P0:     0
        P1-P4:  Actual position (signed 24 bit). 24 bits is in decimal range 0 to 16777215(0xFFFFFF00 little indian). 
                The range of signed integers that can be represented in 24 bits is -8388608..8388607(0x00008000..0xFFFF7F00 little indian).
        '''
        response_data = None
        command = [0x20, 0x0, 0x0, 0x0, 0x0 , 0x0, 0x0, 0x0]
        try:
            response_data = bytearray(self.can_object.write_read(self.id, command)[1].DATA)
            actual_motor_position = struct.unpack('<i', response_data[2:6])[0]
            logger.info("Position of motor with ID {} is {}.".format(self.id, actual_motor_position))
        except Exception as err:
            logger.debug("Checking motor position function was done with error: {}!".format(err))
            raise
        return actual_motor_position
        
    def get_actual_acceleration_velocity(self):
        '''
        Info in manual section 4.3.6 Get actual Acceleration and Velocity ($21).
        Get the actual acceleration and velocity of the motor and if a reference search is just being executed. Please note that during a reference search or an automatic position correction, the returned acceleration and velocity values are invalid.
        CMD:    $21
        P0:     0
        
        Answer
        CMD:    $21
        P0:     0
        P1,P2:  Velocity (signed 12 bit). 12 bits is in decimal range 0 to 4095(0xFF0F little indian). The range of signed integers that can be represented in 12 bits is -2048..2047(0xF800..0xFF07 little indian).
        P3,P4:  Acceleration (signed 12 bit) little indian
        P5:     0: reference search not active
                1: reference search active (acc./vel. values are invalid!)
        P6:     0: automatic correction not active
                1: automatic correction active (acc./vel. values are invalid!)
        '''
        response_data = None
        command = [0x21, 0x0, 0x0, 0x0, 0x0 , 0x0, 0x0, 0x0]
        try:
            response_data = bytearray(self.can_object.write_read(self.id, command)[1].DATA)
            actual_velocity, actual_accelaration =  struct.unpack('<2h', response_data[2:6])
            reference_search_on = bool(response_data[6])
            auto_correction_active = bool(response_data[7])
        except Exception as err:
            logger.debug("Check velocity function was done with error: {}!".format(err))
            raise
        return actual_velocity, actual_accelaration, reference_search_on, auto_correction_active
        
    def drive_a_ramp(self, P0=0x00, position=None):
        '''
        Info in manual section 4.3.7 Drive a Ramp ($23).
        Drive an S-shaped ramp to the given position. The maximum acceleration, maximum velocity and bow settings (s. 4.3.1, 4.3.3) are used. If this command is given while a ramp is active, it will be queued and executed after the currently active ramp has terminated.
        
        CMD:    $23
        P0:     0
        P1-P4:  Position (32 bit signed long) # (-8388608..+16777215 -> 0x000080FF..0xFFFFFF00) little indian. 32 bit signed has range -2147483648 to 2147483647(0z00000080..0xFFFFFF7F little indian)
        '''
        if position not in range(-8388608, 16777216):
            raise InvalidValue("Position value must be in range -8388608..16777215.")
        logger.debug("Count for position {0} is: {1}\n".format(position*self.STEP, position))
        b_position = struct.pack('<l', position)
        logger.debug('Send position {} to Monopack.'.format(b_position))
        P1, P2, P3, P4 = b_position
        command = [0x23, P0, P1, P2, P3, P4, 0x0, 0x0]
        return self.can_object.write_message(self.id, command)
        
        
    def constant_rotation(self, P0=0x00, velocity=None):
        '''
        Info in manual section 4.3.8 Constant Rotation ($25).
        Constant rotation of the motor using the given velocity. The maximum acceleration setting (s. 4.3.1) is used to accelerate or to decelerate the motor.
        
        CMD:    $25
        P0:     0
        P1,P2:  Velocity (-8191..8191) little indian
        '''
        if velocity not in range(-8192, 8193):
            raise InvalidValue("Velocity value must be in range -8191..8191.")
        P1, P2 = struct.pack('<h', velocity)
        command = [0x25, P0, P1, P2, 0x0 , 0x0, 0x0, 0x0]
        return self.can_object.write_message(self.id, command)
        
    def reset_position(self):
        '''
        Info in manual section 4.3.9 Reset Position ($27).
        Set the position counter and the encoder counter to zero. If the PID controller is switched on you will have to turn it off before using this command (s. 4.5.7, command $6F).
        
        CMD:    $27
        P0:     0
        '''
        command = [0x27, 0x00, 0x00, 0x00, 0x0 , 0x0, 0x0, 0x0]
        return self.can_object.write_message(self.id, command)
        
    def soft_stop(self):
        '''
        Info in manual section 4.3.10 Soft Stop ($2A).
        Terminate a ramp and stop the motor softly. The maximum acceleration (s. 4.3.1) parameter is used to decelerate the motor.
        
        CMD:    $2A
        P0:     0
        '''
        command = [0x2A, 0x00, 0x00, 0x00, 0x0 , 0x0, 0x0, 0x0]
        return self.can_object.write_message(self.id, command)
        
    def emergency_stop(self):
        '''
        Info in manual section 4.3.11 Emergency Stop ($2B).
        Stop the motor immediately. This command has the same functionality as setting the external alarm input high.
        
        CMD:    $2B
        P0:     0
        '''
        command = [0x2B, 0x00, 0x00, 0x00, 0x0 , 0x0, 0x0, 0x0]
        return self.can_object.write_message(self.id, command)
        
    #***********************************************************************************#
    #                           STOP AND REFERENCE SWITCHES                             #
    #***********************************************************************************#

    def set_switch_mode(self, P0=0x01, P1=0x00, P2=None, P3=None, P4=None, P5=None, P6=None):
        '''
        Info in manual section 4.4.1 Switch Mode ($54).
        
        CMD:    $54
        P0:     Parameter storage control (s. 4.1)
        P1:     0: Reserved; always set to zero
        P2:     0: separate reference and stop switches
                1: stop switch also used as reference switch
        P3:     0: use left stop switch for calibration
                1: use right stop switch for calibration
        P4:     0: circular movement
                1: linear movement
        P5:     0 Reserved; always set to zero
        P6:     0:travel check disabled
                1: use the reference switch also as a travel check switch (see also 4.4.5)
        '''
        if P2 not in [0, 1] or P3 not in [0, 1] or P4 not in [0, 1] or P6 not in [0, 1]:
            raise InvalidValue("P2, P3, P4 and P6 values must be 0 or 1.")
        command = [0x54, P0, P1, P2, P3 , P4, P5, P6]
        return self.can_object.write_message(self.id, command)
        
    def set_deceleration_at_stop_switches(self, P0=0x01, deceleration=None):
        '''
        Info in manual section 4.4.2 Deceleration at Stop Switches ($57).
        Set the deceleration which is used to decelerate the velocity to zero when a stop switch is reached. If this value is set to zero, a hard stop will be used (the motor stops abruptly when a stop switch is reached). If set to a value other than zero, a soft stop will be used.
        
        CMD:    $54
        P0:     Parameter storage control (s. 4.1)
        P1,P2:  Deceleration (0..8191) little indian
        '''
        if deceleration not in range(8192):
            raise InvalidValue("Deceleration value must be in range 0..8191.")
        P1, P2 = struct.pack('<h', deceleration)
        command = [0x57, P0, P1, P2, 0x0 , 0x0, 0x0, 0x0]
        return self.can_object.write_message(self.id, command)
        
    def reference_search(self):
        '''
        Info in manual section 4.4.3 Reference Search ($22).
        This command starts a reference search. During the reference search also other commands can be given and they are processed. Especially use the command $21 to see if the reference search has already finished. Any driving command will abort the reference search.
        The behavior of the reference search depends on the switch mode setting (command $54, s. 4.4.1) and is as follows:
        * Circular mode: The reference switch (connected to the reference switch input) is searched at first from one and then form the other side. The zero position is then set to the middle of the reference switch.
        * Linear mode:  Reference switch is also end switch:    A move into the reference switch and then out of the reference switch is executed. The zero position is then set to the beginning of the switch.
                        Separate reference switch and end switch:   First, the left or the right stop switch (as configured) is searched. After that, the reference switch is searched at first from one, then from the other side. The zero position is then set to the middle of the reference switch.
        
        CMD:    $22
        P0:     0
        '''
        command = [0x22, 0x00, 0x00, 0x00, 0x0 , 0x0, 0x0, 0x0]
        return self.can_object.write_message(self.id, command)
        
    def set_reference_search_velocity(self, P0=0x01, velocity=None):
        '''
        Info in manual section 4.4.4 Reference Search Velocity ($16).
        Set the velocity which is used for reference searches.
        
        CMD:    $16
        P0:     Parameter storage control (s. 4.1)
        P1,P2:  Velocity (-8191..8191) little indian
        '''
        if velocity not in range(-8192, 8193):
            raise InvalidValue("Velocity value must be in range -8191..8191.")
        P1, P2 = struct.pack('<h', velocity)
        command = [0x16, P0, P1, P2, 0x0 , 0x0, 0x0, 0x0]
        return self.can_object.write_message(self.id, command)
        
    def set_travel_check_tolerance(self, P0=0x01, tolerance=None):
        '''
        Info in manual section 4.4.5 Travel Check Tolerance ($59).
        Set the tolerance of the travel check (see also command $54, section 4.4.1). The travel checking can be enabled or disabled using command $54. If the travel check (reference switch) is found out of the travel check tolerance the alarm output will be set high.
        
        CMD:    $59
        P0:     Parameter storage control (s. 4.1)
        P1:     Tolerance
        '''
        if tolerance not in range(256):
            raise InvalidValue("Tolerance value must be in range 0..255.")
        P1 = tolerance
        command = [0x59, P0, P1, 0x0, 0x0 , 0x0, 0x0, 0x0]
        return self.can_object.write_message(self.id, command)
        
    def set_microsteps_per_revolution(self, P0=0x01, microsteps=None):
        '''
        Info in manual section 4.4.6 Number of Microsteps per Revolution ($15).
        Set the number of microsteps per revolution. This is needed for travel checking in the circular movement mode. The number of microsteps per revolution can be calculated from the number of full steps per revolution multiplied by the number of microsteps set using command $17 (s. 4.2.5).
        
        CMD:    $15
        P0:     Parameter storage control (s. 4.1)
        P1,P2,P3,P4:  Microsteps per revolution (max. 16777215) little indian
        '''
        if microsteps not in range(16777216):
            raise InvalidValue("Microsteps value must be in range 0..16777215.")
        P1, P2, P3, P4 = struct.pack('<i', microsteps)
        command = [0x15, P0, P1, P2, P3 , P4, 0x0, 0x0]
        return self.can_object.write_message(self.id, command)
        
    def get_stop_switches_state(self):
        '''
        Info in manual section 4.4.7 Get the State of the Stop Switches ($30).
        This command reads out the actual state of the stop and reference switch inputs of the Monopack.
        
        CMD:    $30
        P0:     0
        
        Answer
        CMD:    $30
        P0:     0
        P1:     State of left stop switch input (0 or 1)
        P2:     State of right stop switch input (0 or 1)
        P3:     State of reference switch input (0 or 1)
        '''
        response_data = None
        command = [0x30, 0x0, 0x0, 0x0, 0x0 , 0x0, 0x0, 0x0]
        try:
            response_data = bytearray(self.can_object.write_read(self.id, command)[1].DATA)
            stop_switch_left_state = bool(response_data[2])[0]
            stop_switch_right_state = bool(response_data[3])[0]
            reference_switch_state = bool(response_data[4])[0]
        except Exception as err:
            logger.debug("Get switches state function was done with error: {}!".format(err))
        return stop_switch_left_state, stop_switch_right_state, reference_switch_state
        
    #***********************************************************************************#
    #                               INCREMENTAL ENCODER                                 #
    #***********************************************************************************#

    def encoder_configuration(self, P0=0x01, P1=None, P2=None, P5=None, deviation=None):
        '''
        Info in manual section 4.5.2 Encoder Configuration ($70).
        Configure the behavior of the incremental encoder input.
        
        CMD:    $70
        P0:     Parameter storage control (s. 4.1)
        P1:     Encoder configuration. Explanation of the configuration bits:
                0:  Polarity of the encoder N input (1=positive, 0=negative)
                1:  The next N signal clears the encoder counter register when set
                2:  Reserved. Always set to zero.
                3:  Reserved. Always set to zero.
                4:  Set this bit to copy the actual ramp position to the encoder counter register. The bit will be reset automatically.
                5:  Reserved. Always set to zero.
                6:  Direction of the encoder signals. 1: A->B, 0: B->A
        P2:     Encoder pre-divider (0..255)
        P3,P4:  Maximum deviation between ramp position and encoder position counter (11 Bit unsigned #)
        P5:     Encoder multiplier (0..255).
        '''
        if P1 not in range(128) or P2 not in range(256) or deviation not in range(2048) or P5 not in range(256):
            raise InvalidValue("P1, P2, P5 values must be in range 0..255 and deviation value must be in range 0..2047.")
        P3, P4 = struct.pack('<H', deviation)
        command = [0x15, P0, P1, P2, P3, P4, P5, 0x0]
        return self.can_object.write_message(self.id, command)
        
    def get_encoder_counter(self):
        '''
        Info in manual section 4.5.3 Encoder Counter ($71).
        Get the value of the incremental encoder counter register.
        
        CMD:    $71
        P0:     0
        
        Answer
        CMD:    $71
        P0:     0
        P1,P2,P3:   Encoder counter value (signed 24 bit) little indian
        '''
        response_data = None
        command = [0x71, 0x0, 0x0, 0x0, 0x0 , 0x0, 0x0, 0x0]
        try:
            response_data = bytearray(self.can_object.write_read(self.id, command)[1].DATA)
            encoder_counter = struct.unpack('<L', response_data[2:6])[0]
            actual_position = encoder_counter * self.STEP
            logger.info("Position with ID {} in mm: {}.".format(self.id, actual_position))
        except Exception as err:
            logger.debug("Get the encoder counter value function was done with error: {}!".format(err))
            raise
        return encoder_counter
            
    def set_deviation_alarm(self, P0=0x01, P1=None, P2=None, correction_start_after=None):
        '''
        Info in manual section 4.5.5 Deviation Alarm ($73).
        Enables or disables the deviation alarm. This alarm occurs when the maximum deviation is exceeded. The alarm output will then be set high. Set the maximum deviation using command $70. The motor can also be stopped immediately or softly when the alarm occurs. Furthermore, the automatic position correction (see 4.5.6) can be started n/200 sec (n=1..65535) after a deviation has been detected. In this case, the alarm output will only be set when the maximum number of retries for automatic position correction has been exceeded. The maximum deviation has to be set up using command $70 (s. 4.5.1) first. The automatic position correction has to be set up using command $58 (s. 4.5.6).
        
        CMD:    $73
        P0:     Parameter storage control (s. 4.1)
        P1:     0: disable deviation alarm
                1: enable deviation alarm
        P2:     0: do not stop when deviation alarm occurs
                1: soft stop when deviation alarm occurs
                2: hard stop when deviation alarm occurs
        P3,P4:  0: no automatic position correction after deviation
                1-65535: start automatic position correction n/200 sec after a deviation has been detected (available since firmware V2.09)
        '''
        if P1 not in [0, 1] or P2 not in [0, 1, 2] or correction_start_after not in range(65536):
            raise InvalidValue("P1 values must be 0 or 1, P2 values must be 0, 1 or 2, P3 value must be in range 0..65535.")
        P3, P4 = struct.pack('<H', correction_start_after)
        command = [0x15, P0, P1, P2, P3 , P4, 0x0, 0x0]
        return self.can_object.write_message(self.id, command)
        
    def conf_auto_position_correction(self, P0=0x01, P1=None, tolerance=None):
        '''
        Info in manual section 4.5.6 Configure Automatic Position Correction ($58).
        Configure if an automatic position correction shall be done at the end of each ramp or when a deviation has been detected. Automatic position correction can only be used in conjunction with an incremental encoder which has to be configured correctly first.
        When this function is turned on, the Monopack checks if the position counter of the incremental encoder matches the desired end position at the end of every ramp (since firmware V2.09, the tolerance value defines a tolerance window around the end position). If this is not the case, the Monopack will try to correct the position of the motor using the reference search velocity. The maximum number of retries after each ramp can also be configured. The alarm output will be set high and the position correction will be aborted if this number is exceeded.
        This function is an easy to use alternative to the PID controller.
        
        CMD:    $58
        P0:     Parameter storage control (s. 4.1)
        P1:     0: automatic correction turned off
                1-255: maximum number of retries for position correction after each ramp.
        P3,P4:  end position tolerance (16 bit unsigned) little indian, (available since firmware V2.09)
        '''
        if P0 not in range(256) or tolerance not in range(65536):
            raise InvalidValue("P0 values must be in range 0..255, tolerance value must be in range 0..65535.")
        P3, P4 = struct.pack('<H', tolerance)
        command = [0x58, P0, P1, 0x0, P3 , P4, 0x0, 0x0]
        return self.can_object.write_message(self.id, command)
        

    #***********************************************************************************#
    #                         PID CONTROLLER CONFIGURATION                              #
    #***********************************************************************************#
    
    '''
    Info in manual section 4.5.7 PID Controller Configuration ($6A, $6B, $6C, $6D, $6F).
    The following commands ($6A, $6B, $6C, $6D, $6F) are to be used for setting up the PID controller configuration registers of the TMC453. 
    Please see the TMC453 manual for further explanation of the PID controller registers.
    '''
    def pid_6A(self, P0=0x01, P1=None, P2=None, P3=None, P4=None):
        command = [0x6A, P0, P1, P2, P3, P4, 0x0, 0x0]
        return self.can_object.write_message(self.id, command)
    
    def pid_6B(self, P0=0x01, P1=None, P2=None, P3=None, P4=None, P5=None, P6=None):
        command = [0x6B, P0, P1, P2, P3, P4, P5, P6]
        return self.can_object.write_message(self.id, command)
    
    def pid_6C(self, P0=0x01, P1=None, P2=None, P3=None):
        command = [0x6C, P0, P1, P2, P3, 0x0, 0x0, 0x0]
        return self.can_object.write_message(self.id, command)
    
    def pid_6D(self, P0=0x01, P1=None, P2=None, P3=None, P4=None):
        command = [0x6D, P0, P1, P2, P3, P4, 0x0, 0x0]
        return self.can_object.write_message(self.id, command)
    
    def pid_6F(self, P0=0x01, P1=None):
        command = [0x6F, P0, P1, 0x0, 0x0, 0x0, 0x0, 0x0]
        return self.can_object.write_message(self.id, command)
    

    #***********************************************************************************#
    #                              ALARM AND ERRORS                                     #
    #***********************************************************************************#

    def set_alarm_mode(self, P0=0x01, P1=None, P2=None):
        '''
        Info in manual section 4.6.1 Alarm Mode ($51).
        Set up the alarm mode. The alarm mode determines if the motor is to be powered off when an external alarm (alarm input set high) or a driver error occurs. The motor can be powered on again by resetting the alarm using command $74 (s. 4.6.2).
        CMD:    $51
        P0:     Parameter storage control (s. 4.1)
        P1:     0: only stop motor in case of external alarm
                1: stop and power off in case of external alarm
        P2:     0: do not stop motor in case of driver error
                1: stop and power off in case of driver error
        '''
        if P1 not in [0, 1] or P2 not in [0, 1]:
            raise InvalidValue("P1 and P2 values must be 0 or 1.")
        command = [0x51, P0, P1, P2, 0x0 , 0x0, 0x0, 0x0]
        return self.can_object.write_message(self.id, command)
        
    def reset_alarm(self):
        '''
        Info in manual section 4.6.2 Reset Alarm Output ($74)
        Reset the alarm output and return the alarm reason as answer.
        CMD:    $74
        P0:     0
        
        Answer
        CMD:    $74
        P0:     0
        P1:     Driver error (0=no/1=yes) (short circuit, overload, etc.)
        P2:     Deviation error (0=no/1=yes) (s. 4.5.5)
        P3:     External alarm input (0=no/1=yes)
        P4:     Travel check tolerance error (0=no/1=yes) (s. 4.4.5)
        P5:     Position correction error (0=no/1=yes) (s. 4.5.6)
        '''
        response_data = None
        command = [0x74, 0x0, 0x0, 0x0, 0x0 , 0x0, 0x0, 0x0]
        try:
            response_data = bytearray(self.can_object.write_read(self.id, command)[1].DATA)
            driver_error           = bool(response_data[2])
            deviation_error        = bool(response_data[3])
            external_alarm_input   = bool(response_data[4])
            check_tolerance_error  = bool(response_data[5])
            correction_error       = bool(response_data[6])
        except Exception as err:
            logger.debug("Get switches function state was done with error: {}!".format(err))
        return driver_error, deviation_error, external_alarm_input, check_tolerance_error, correction_error


    #***********************************************************************************#
    #                                   OTHER SETTINGS                                  #
    #***********************************************************************************#
            
    def enter_step_direction_mode(self, P1=None):
        '''
        Info in manual section 4.7.1 Enter Step/Direction Mode ($50).
        This command can be used to switch the Monopack back to step-/direction mode (e.g. after changing parameters or resetting an alarm).
        
        CMD:    $50
        P0:     0
        P1:     0: step/direction mode
                1: command mode (any other command will also switch to command mode)
        '''
        if P1 not in [0, 1]:
            raise InvalidValue("P1 value must be 0 or 1.")
        command = [0x50, 0x0, P1, 0x0, 0x0 , 0x0, 0x0, 0x0]
        return self.can_object.write_message(self.id, command)
        
    def set_can_receive_rs485_ID(self, P0=0x01, ID=None):
        '''
        Info in manual section 4.7.2 Set CAN receive ID and RS485 ID ($55).
        Set the CAN identifier which will be used for receiving data from the CAN bus and the RS485 address.
        
        CMD:    $55
        P0:     Parameter storage control (s. 4.1)
        P1-P4:  32 bit unsigned long ID # (only 11 bits are used)
        '''
        P1, P2, P3, P4 = struct.pack('<I', ID)
        command = [0x55, P0, P1, P2, P3 , P4, 0x0, 0x0]
        return self.can_object.write_message(self.id, command)
        
    def set_can_send_id(self, P0=0x01, ID=None):
        '''
        Info in manual section 4.7.3 Set CAN send ID ($56).
        Set the CAN identifier which will be used for sending data on the CAN bus. Please see also section 3.2.2 and 2.8.
        
        CMD:    $56
        P0:     Parameter storage control (s. 4.1)
        P1-P4:  32 bit unsigned long ID # (only 11 bits are used)
        '''
        P1, P2, P3, P4 = struct.pack('<I', ID)
        command = [0x56, P0, P1, P2, P3 , P4, 0x0, 0x0]
        return self.can_object.write_message(self.id, command)
        
    def set_can_baud_rate(self, P1=None):
        '''
        Info in manual section 4.7.4 Set CAN Baud Rate ($C0).
        Change the baud rate of the CAN interface. After executing this command, a hardware reset is required to make the change take effect.
        
        CMD:    $C0
        P0:     0
        P1:     1: 125 kBit/s
                2: 250 kBit/s
                3: 500 kBit/s
                4: 1 Mbit/s
        '''
        if P1 not in [1, 2, 3, 4]:
            raise InvalidValue("P1 value must be 1, 2, 3 or 4.")
        command = [0xC0, 0x0, P1, 0x0, 0x0 , 0x0, 0x0, 0x0]
        return self.can_object.write_message(self.id, command)
        
    def get_version_number(self):
        '''
        Info in manual section 4.7.5 Get Version Number ($43).
        Get the firmware version number and the reset flag. The reset flag is 1 if a reset occurred before the last $43 command was given.
        
        CMD:    $43
        P0:     0
        
        Answer:
        CMD:    $43
        P0:     Firmware revision (decimal 203 means V2.03)
        P1:     Reset Flag: 1 after a reset
        P2:     Not used
        P3, P4: Temperature (16 bit #) (units: TBD)
        
        Output:
        firmware_revision      = 203
        reset_flag             = 1
        temperature            = 0
        '''
        response_data = None
        command = [0x43, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0]
        try:
            response_data = bytearray(self.can_object.write_read(self.id, command)[1].DATA)
            firmware_revision      = struct.unpack('B', response_data[1:2])[0]/100
            reset_flag             = bool(response_data[2])
            temperature            = struct.unpack('<h', response_data[4:6])[0]/10
        except Exception as err:
            logger.debug("Get version function was done with error: {}!".format(err))
            raise
        return firmware_revision, reset_flag, temperature
            
    def hardware_reset(self):
        '''
        Info in manual section 4.7.6 Hardware Reset ($CC).
        Reset the microcontroller of the MONOPACK so that all parameters are re-read from the EEPROM. This command can be used to make parameter changes which need a hardware reset take effect.
        
        CMD:    $CC
        P0:     0
        '''
        command = [0xCC, 0x0, 0x0, 0x0, 0x0 , 0x0, 0x0, 0x0]
        return self.can_object.write_message(self.id, command)
        
    def factory_default_settings(self, P1=None, P2=None):
        '''
        Info in manual section 4.8 Factory Default Settings ($DD).
        Set all parameters which are stored in the EEPROM (including the CAN and RS485 addresses and the CAN baud rate) to their factory default settings. After executing this command, a hardware reset has to be issued to make all the changes take effect.
        
        CMD:    $DD
        P0:     0
        P1:     Must be $31
        P2:     Must be $41
        '''
        if P1 != 0x31 or P2 != 0x41:
            raise InvalidValue("P1 and P2 values must be 0x31 and 0x41 for correct execution.")
        command = [0xDD, 0x0, P1, P2, 0x0 , 0x0, 0x0, 0x0]
        return self.can_object.write_message(self.id, command)

if __name__ == "__main__":
    import doctest
    doctest.testmod()