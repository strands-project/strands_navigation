import rospy
import smach




class RecoverStateMachine(smach.StateMachine):
    def __init__(self, input_keys=[], output_keys=[]):
            smach.StateMachine.__init__(self, 
                                    outcomes=['succeeded',
                                              'preempted',
                                              'recovered_with_help',
                                              'recovered_without_help',
                                              'not_recovered_with_help',
                                              'not_recovered_without_help'],
                                    input_keys=input_keys,
                                    output_keys=output_keys)
            


