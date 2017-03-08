#!/usr/bin/env python

import rospy
import smach
import smach_ros
import time
import threading
import traceback
from classes.OrientationController import *
from classes.PositionController import *
from robdos_sim.msg import StateCommand

current_target = [0.0, 0.0, 0.0]  # x, y, z


# define state InitPoint
class InitPoint(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Press Reposo', 'Press Safety'])

    def execute(self, userdata):
        # time.sleep(2)
        return 'Press Reposo'


# define state Reset
class Reset(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Press Init', 'Press Safety'])

    def execute(self, userdata):
        # rospy.loginfo('Executing state StopRobot')
        time.sleep(2)
        return 'Press Init'


# define state Safety
class Safety(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Press Reposo'])

    def execute(self, userdata):
        # rospy.loginfo('Executing state StopRobot')
        time.sleep(2)
        return 'Press Reposo'


# # define state Reposo
# class Reposo(smach.State):
#     def __init__(self):
#         smach.State.__init__(self, outcomes=['Press Teleoperation', 'Press Reposo', 'Press Safety'])

#     def execute(self, userdata):
#         #rospy.loginfo('Executing state StopRobot')
#         time.sleep(2)
#         return 'Press Teleoperation'

##########################################TELEOPERATION############################################
# define state ControlHoover
class ControlHoover(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Press Change', 'Press Safety', 'Press Reposo', 'Press Autonomous',
                                             'Press Semiautonomous'])

    def execute(self, userdata):
        # rospy.loginfo('Executing state FixOrientation')
        time.sleep(2)
        return 'Press Semiautonomous'


# define state ControlSubmarine
class ControlSubmarine(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Press Change', 'Press Safety', 'Press Reposo', 'Press Autonomous',
                                             'Press Semiautonomous'])

    def execute(self, userdata):
        # rospy.loginfo('Executing state FixOrientation')
        time.sleep(2)
        return 'Press Change'


##########################################SEMIAUTONOMOUS###########################################
# define state FixOrientation
class FixOrientation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Succeeded', 'Press Safety', 'Press Reposo', 'Press Teleoperation'])

    def execute(self, userdata):
        # rospy.loginfo('Executing state FixOrientation')
        OrientationController(current_target)
        # time.sleep(1)
        return 'Succeeded'


# define state FixPosition
class FixPosition(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Succeeded', 'Fail_orientation', 'Press Safety', 'Press Reposo',
                                             'Press Teleoperation'])

    def execute(self, userdata):
        controller = PositionController(current_target)
        [is_oriented, is_positioned] = controller.init_controller()
        if is_oriented and is_positioned:
            return 'Succeeded'
        else:
            # rospy.loginfo('fail_orientation')
            return 'Fail_orientation'
        #time.sleep(1)
        #return 'Succeeded'



# define state ReachWaypoint
class ReachWaypoint(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Succeeded', 'Press Safety', 'Press Reposo', 'Press Teleoperation'])

    def execute(self, userdata):
        # rospy.loginfo('Executing state ReachWaypoint')
        # time.sleep(5)
        return 'Succeeded'


############################################AUTONOMOUS#############################################
# define state mision1
class mision1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Press Safety', 'Press Reposo', 'Press Teleoperation'])

    def execute(self, userdata):
        # rospy.loginfo('Executing state ReachWaypoint')
        time.sleep(2)
        return 'Press Reposo'


#########################################MACHINE STATE#############################################
value = 0


def reposo_state_function(ud, msg):
    global value

    # use same constant definition
    value = msg.cmd

    # Review?
    # if msg.cmd == msg.TELEOPERATION:
    #    value = 1
    # elif msg.cmd == msg.SAFETY:
    #    value = 2
    # elif msg.cmd == msg.RESET:
    #    value = 3

    # if twist.angular.z == 1.0:
    # 	return False


__all__ = ['MonitorState']


class MonitorState(smach.State):
    def __init__(self, topic, msg_type, cond_cb, max_checks=-1, input_keys=[], output_keys=[]):
        smach.State.__init__(
            self,
            outcomes=['Press Reset', 'Press Teleoperation', 'Press Safety', 'Reposo'],
            input_keys=input_keys,
            output_keys=output_keys)
        global value
        self._topic = topic
        self._msg_type = msg_type
        self._cond_cb = cond_cb
        self._max_checks = max_checks
        self._n_checks = 0

        self._trigger_event = threading.Event()

    def execute(self, ud):
        self._n_checks = 0
        self._trigger_event.clear()

        self._sub = rospy.Subscriber(self._topic, self._msg_type, self._cb, callback_args=ud)

        self._trigger_event.wait()
        self._sub.unregister()

        msg_sc = StateCommand()

        # default case
        return_value = 'Reposo'

        if value == msg_sc.TELEOPERATION:
            return_value = 'Press Teleoperation'
        elif value == msg_sc.SAFETY:
            return_value = 'Press Teleoperation'
        elif value == msg_sc.RESET:
            return_value = 'Press Reset'
        elif value == msg_sc.SEMIAUTONOMOUS:
            return_value = 'Press Reset'

        return return_value

    def _cb(self, msg, ud):
        try:
            if self._cond_cb(ud, msg):
                self._n_checks += 1
            else:
                self._trigger_event.set()
        except Exception as e:
            rospy.logerr("Error thrown while executing condition callback %s: %s" % (str(self._cond_cb), e))
        self._trigger_event.set()

        if (self._max_checks > 0 and self._n_checks >= self._max_checks):
            self._trigger_event.set()

###################################################################################################
############################################### MAIN ##############################################
###################################################################################################

def main():
    rospy.init_node('robdos_state_machine')

    # Create the top level SMACH state machine
    sm_top = smach.StateMachine(outcomes=['succeeded'])

    # Open the container
    with sm_top:
        smach.StateMachine.add('InitPoint', InitPoint(),
                               transitions={'Press Reposo': 'Reposo',
                                            'Press Safety': 'Safety'})

        smach.StateMachine.add('Reset', Reset(),
                               transitions={'Press Init': 'InitPoint',
                                            'Press Safety': 'Safety'})

        smach.StateMachine.add('Safety', Safety(),
                               transitions={'Press Reposo': 'Reposo'})

        # smach.StateMachine.add('Reposo', MonitorState("/turtle1/cmd_vel", Twist, reposo_state_function), 
        smach.StateMachine.add('Reposo', MonitorState("teleop/stateMachine", StateCommand, reposo_state_function),
                               transitions={'Press Teleoperation': 'Teleoperation',
                                            'Press Reset': 'Reset',
                                            'Press Safety': 'Safety',
                                            'Reposo': 'Reposo'})

        # smach.StateMachine.add('Reposo', Reposo(),
        #                        transitions={'Press Teleoperation':'Teleoperation',
        #                                     'Press Reposo':'Reposo',
        #                                     'Press Safety':'Safety'})

        ##########################################TELEOPERATION############################################
        # Create the sub SMACH state machine
        robdos_teleoperation = smach.StateMachine(
            outcomes=['SafetyTeleoperation', 'ReposoTeleoperation', 'AutonomousTeleoperation',
                      'SemiautonomousTeleoperation'])

        # Open the container
        with robdos_teleoperation:
            # Add states to the container
            smach.StateMachine.add('ControlHoover', ControlHoover(),
                                   transitions={'Press Change': 'ControlSubmarine',
                                                'Press Safety': 'SafetyTeleoperation',
                                                'Press Reposo': 'ReposoTeleoperation',
                                                'Press Autonomous': 'AutonomousTeleoperation',
                                                'Press Semiautonomous': 'SemiautonomousTeleoperation'})

            smach.StateMachine.add('ControlSubmarine', ControlSubmarine(),
                                   transitions={'Press Change': 'ControlHoover',
                                                'Press Safety': 'SafetyTeleoperation',
                                                'Press Reposo': 'ReposoTeleoperation',
                                                'Press Autonomous': 'AutonomousTeleoperation',
                                                'Press Semiautonomous': 'SemiautonomousTeleoperation'})

        smach.StateMachine.add('Teleoperation', robdos_teleoperation,
                               transitions={'SafetyTeleoperation': 'Safety',
                                            'ReposoTeleoperation': 'Reposo',
                                            'AutonomousTeleoperation': 'Autonomous',
                                            'SemiautonomousTeleoperation': 'SemiAutonomous'})

        ##########################################SEMIAUTONOMOUS###########################################
        # Create the sub SMACH state machine
        semiautonomous = smach.StateMachine(
            outcomes=['SafetySemiAutonomous', 'ReposoSemiAutonomous', 'TeleoperationSemiAutonomous'])

        # Open the container
        with semiautonomous:
            # Add states to the container
            smach.StateMachine.add('FixOrientation', FixOrientation(),
                                   transitions={'Succeeded': 'FixPosition',
                                                'Press Safety': 'SafetySemiAutonomous',
                                                'Press Reposo': 'ReposoSemiAutonomous',
                                                'Press Teleoperation': 'TeleoperationSemiAutonomous'})

            smach.StateMachine.add('FixPosition', FixPosition(),
                                   transitions={'Succeeded': 'ReachWaypoint',
                                                'Fail_orientation': 'FixOrientation',
                                                'Press Safety': 'SafetySemiAutonomous',
                                                'Press Reposo': 'ReposoSemiAutonomous',
                                                'Press Teleoperation': 'TeleoperationSemiAutonomous'})

            smach.StateMachine.add('ReachWaypoint', ReachWaypoint(),
                                   transitions={'Succeeded': 'FixOrientation',
                                                'Press Safety': 'SafetySemiAutonomous',
                                                'Press Reposo': 'ReposoSemiAutonomous',
                                                'Press Teleoperation': 'TeleoperationSemiAutonomous', })

        smach.StateMachine.add('SemiAutonomous', semiautonomous,
                               transitions={'SafetySemiAutonomous': 'Safety',
                                            'ReposoSemiAutonomous': 'Reposo',
                                            'TeleoperationSemiAutonomous': 'Teleoperation'})

        ############################################AUTONOMOUS#############################################
        robdos_autonomous = smach.StateMachine(
            outcomes=['SafetyAutonomous', 'ReposoAutonomous', 'TeleoperationAutonomous'])

        # Open the container
        with robdos_autonomous:
            smach.StateMachine.add('Mision 1', mision1(),
                                   transitions={'Press Safety': 'SafetyAutonomous',
                                                'Press Reposo': 'ReposoAutonomous',
                                                'Press Teleoperation': 'TeleoperationAutonomous', })

        smach.StateMachine.add('Autonomous', robdos_autonomous,
                               transitions={'SafetyAutonomous': 'Safety',
                                            'ReposoAutonomous': 'Reposo',
                                            'TeleoperationAutonomous': 'Teleoperation'})

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm_top, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    outcome = sm_top.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    # wait before system is ready
    #time.sleep(3)
    main()
