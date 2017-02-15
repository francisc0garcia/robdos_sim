#!/usr/bin/env python

import rospy
import smach_ros
import smach
import time

# define state InitPoint
class InitPoint(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        time.sleep(2)
        return 'succeeded'

# define state CheckSystemIntegrity
class CheckSystemIntegrity(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        #rospy.loginfo('Executing state CheckSystemIntegrity')
        time.sleep(2)
        return 'succeeded'

# define state ManualControl
class ManualControl(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        #rospy.loginfo('Executing state ManualControl')
        time.sleep(2)
        return 'succeeded'

# define state FixOrientation
class FixOrientation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        #rospy.loginfo('Executing state FixOrientation')
        time.sleep(2)
        return 'succeeded'

# define state FixPosition
class FixPosition(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        #rospy.loginfo('Executing state FixPosition')
        time.sleep(2)
        return 'succeeded'

# define state ReachWaypoint
class ReachWaypoint(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        #rospy.loginfo('Executing state ReachWaypoint')
        time.sleep(2)
        return 'succeeded'

# define state StopRobot
class StopRobot(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        #rospy.loginfo('Executing state StopRobot')
        time.sleep(2)
        return 'succeeded'

def main():
    rospy.init_node('robdos_state_machine')

    # Create the top level SMACH state machine
    sm_top = smach.StateMachine(outcomes=['succeeded'])

    # Open the container
    with sm_top:

        smach.StateMachine.add('InitPoint', InitPoint(),
                               transitions={'succeeded':'CheckSystemIntegrity'})

        smach.StateMachine.add('CheckSystemIntegrity', CheckSystemIntegrity(),
                               transitions={'succeeded':'ManualControl'})

        smach.StateMachine.add('ManualControl', ManualControl(),
                               transitions={'succeeded':'WayPointFollower'})

        smach.StateMachine.add('StopRobot', StopRobot(),
                               transitions={'succeeded':'InitPoint'})

        # Create the sub SMACH state machine
        sm_sub = smach.StateMachine(outcomes=['succeeded'])

        # Open the container
        with sm_sub:

            # Add states to the container
            smach.StateMachine.add('FixOrientation', FixOrientation(),
                                   transitions={'succeeded':'FixPosition'})

            smach.StateMachine.add('FixPosition', FixPosition(),
                                   transitions={'succeeded':'ReachWaypoint'})

            smach.StateMachine.add('ReachWaypoint', ReachWaypoint(),
                                   transitions={'succeeded':'FixOrientation'})

        smach.StateMachine.add('WayPointFollower', sm_sub,
                               transitions={'succeeded':'succeeded'})


    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm_top, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    outcome = sm_top.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()