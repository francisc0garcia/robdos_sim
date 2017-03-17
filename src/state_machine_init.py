#!/usr/bin/env python

import rospy
import smach
import smach_ros
import time
import threading
import traceback
from classes.OrientationController import *
from classes.PositionController import *
from classes.TeleopController import *
from classes.GazeboMapper import *
from robdos_sim.msg import StateEvent

current_target = [0.0, 0.0, 0.0]  # x, y, z
current_event = 0
trigger_event = threading.Event()

OrientationController__ = None

PositionController__ = None

TeleopController__ = None

def mapEventToAction(event, state):

    global OrientationController__
    global PositionController__
    global TeleopController__

    msg = StateEvent()

    if state == "Reposo":
        if event == msg.TELEOPERATION:
            return 'Press Teleoperation'
        elif event == msg.SAFETY:
            return 'Press Teleoperation'
        elif event == msg.RESET:
            return 'Press Reset'
        elif event == msg.SEMIAUTONOMOUS:
            return 'Press Reset'
        else:
            return False

    if state == "Teleoperation":
        if event == msg.CHANGE:
            TeleopController__.joy_subscriber.unregister()
            TeleopController__.RCOR_msg = OverrideRCIn()
            TeleopController__.RCOR_msg.channels = [0, 0, 0, 1500, 0, 1500, 0, 0]
            TeleopController__.mavros_vel_pub.publish(TeleopController__.RCOR_msg)
            return 'Press Change'
        elif event == msg.SAFETY:
            TeleopController__.joy_subscriber.unregister()
            TeleopController__.RCOR_msg = OverrideRCIn()
            TeleopController__.RCOR_msg.channels = [0, 0, 0, 1500, 0, 1500, 0, 0]
            TeleopController__.mavros_vel_pub.publish(TeleopController__.RCOR_msg)
            return 'Press Safety'
        elif event == msg.REPOSO:
            TeleopController__.joy_subscriber.unregister()
            TeleopController__.RCOR_msg = OverrideRCIn()
            TeleopController__.RCOR_msg.channels = [0, 0, 0, 1500, 0, 1500, 0, 0]
            TeleopController__.mavros_vel_pub.publish(TeleopController__.RCOR_msg)
            return 'Press Reposo'
        elif event == msg.SEMIAUTONOMOUS:
            TeleopController__.joy_subscriber.unregister()
            TeleopController__.RCOR_msg = OverrideRCIn()
            TeleopController__.RCOR_msg.channels = [0, 0, 0, 1500, 0, 1500, 0, 0]
            TeleopController__.mavros_vel_pub.publish(TeleopController__.RCOR_msg)
            return 'Press Semiautonomous'
        elif event == msg.AUTONOMOUS:
            TeleopController__.joy_subscriber.unregister()
            TeleopController__.RCOR_msg = OverrideRCIn()
            TeleopController__.RCOR_msg.channels = [0, 0, 0, 1500, 0, 1500, 0, 0]
            TeleopController__.mavros_vel_pub.publish(TeleopController__.RCOR_msg)
            return 'Press Autonomous'
        else:
            return False

    elif state == "FixOrientation":
        if event == msg.ORIENTED:
            return 'Succeeded'
        elif event == msg.SAFETY:
            OrientationController__.sub_localization.unregister()
            OrientationController__.RCOR_msg = OverrideRCIn()
            OrientationController__.RCOR_msg.channels = [0, 0, 0, 1500, 0, 1500, 0, 0]
            OrientationController__.mavros_vel_pub.publish(OrientationController__.RCOR_msg)
            OrientationController__.srv.set_service.shutdown("close process")
            return 'Press Safety'
        elif event == msg.REPOSO:
            OrientationController__.sub_localization.unregister()
            OrientationController__.RCOR_msg = OverrideRCIn()
            OrientationController__.RCOR_msg.channels = [0, 0, 0, 1500, 0, 1500, 0, 0]
            OrientationController__.mavros_vel_pub.publish(OrientationController__.RCOR_msg)
            OrientationController__.srv.set_service.shutdown("close process")
            return 'Press Reposo'
        elif event == msg.TELEOPERATION:
            OrientationController__.sub_localization.unregister()
            OrientationController__.RCOR_msg = OverrideRCIn()
            OrientationController__.RCOR_msg.channels = [0, 0, 0, 1500, 0, 1500, 0, 0]
            OrientationController__.mavros_vel_pub.publish(OrientationController__.RCOR_msg)
            OrientationController__.srv.set_service.shutdown("close process")
            return 'Press Teleoperation'
        else:
            return False

    elif state == "FixPosition":
        if event == msg.POSITIONED:
            return 'Succeeded'
        elif event == msg.NOT_ORIENTED:
            return 'Fail_orientation'
        elif event == msg.SAFETY:
            PositionController__.sub_localization.unregister()
            PositionController__.RCOR_msg = OverrideRCIn()
            PositionController__.RCOR_msg.channels = [0, 0, 0, 1500, 0, 1500, 0, 0]
            PositionController__.mavros_vel_pub.publish(OrientationController__.RCOR_msg)
            return 'Press Safety'
        elif event == msg.REPOSO:
            PositionController__.sub_localization.unregister()
            PositionController__.RCOR_msg = OverrideRCIn()
            PositionController__.RCOR_msg.channels = [0, 0, 0, 1500, 0, 1500, 0, 0]
            PositionController__.mavros_vel_pub.publish(OrientationController__.RCOR_msg)
            return 'Press Reposo'
        elif event == msg.TELEOPERATION:
            PositionController__.sub_localization.unregister()
            PositionController__.RCOR_msg = OverrideRCIn()
            PositionController__.RCOR_msg.channels = [0, 0, 0, 1500, 0, 1500, 0, 0]
            PositionController__.mavros_vel_pub.publish(OrientationController__.RCOR_msg)
            return 'Press Teleoperation'
        else:
            return False
    else:
        return False


def handleEvent(eventMsg):
    global current_event
    global trigger_event
    current_event = eventMsg.cmd
    trigger_event.set()



def waitForEvent(state):
    global trigger_event
    trigger_event.clear()
    sub = rospy.Subscriber("/robdos/stateEvents", StateEvent, handleEvent)
    while not rospy.is_shutdown():
        trigger_event.wait()
        sub.unregister()
        action = mapEventToAction(current_event, state)
        if action:
            return action

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
        rospy.logerr("Safety")
        # rospy.loginfo('Executing state StopRobot')
        time.sleep(2)
        return 'Press Reposo'


##########################################TELEOPERATION############################################
# define state ControlHoover
class ControlHoover(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Press Change', 'Press Safety', 'Press Reposo', 'Press Autonomous',
                                             'Press Semiautonomous'])

    def execute(self, userdata):
        global TeleopController__
        time.sleep(0.1)
        TeleopController__.register()
        result = waitForEvent("Teleoperation")       
        return result
        #time.sleep(0.1)
        #return 'Press Semiautonomous'


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
        global OrientationController__
        time.sleep(0.2)
        OrientationController__.register()
        result = waitForEvent("FixOrientation")
        return result


# define state FixPosition
class FixPosition(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Succeeded', 'Fail_orientation', 'Press Safety', 'Press Reposo',
                                             'Press Teleoperation'])

    def execute(self, userdata):
        global PositionController__
        time.sleep(0.2)
        PositionController__.register()
        result = waitForEvent("FixPosition")
        return result        



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
#value = 0

class Reposo(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Press Reset', 'Press Teleoperation', 'Press Safety', 'Reposo'],)
        
    def execute(self, ud):
        rospy.logerr("espera")
        result = waitForEvent("Reposo")
        return result


###################################################################################################
############################################### MAIN ##############################################
###################################################################################################

def main():
    rospy.init_node('robdos_state_machine')

    global  OrientationController__
    global PositionController__
    global TeleopController__

    GazeboMapper__ = GazeboMapper()

    OrientationController__ = OrientationController(current_target)
    OrientationController__.sub_localization.unregister()

    PositionController__ = PositionController(current_target)
    PositionController__.sub_localization.unregister()

    TeleopController__ = TeleopController()
    TeleopController__.joy_subscriber.unregister()

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
        smach.StateMachine.add('Reposo', Reposo(),
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
