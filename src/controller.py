#!/usr/bin/env python
##!/usr/bin/python

"""Created by Thomas Rostrup Andersen on 11/11/2016.
Copyright (C) 2016 Thomas Rostrup Andersen. All rights reserved."""

import sys
import os
import threading
import rospy
import smach
from module import Module
from statemachinemonitor import StateMachineMonitor
from emotionsystem import EmotionSystem
from motivator import Motivator
from databasehandler import DatabaseHandler

__author__ = "Thomas Rostrup Andersen"
__copyright__ = "Copyright (C) 2016 Thomas Rostrup Andersen"
#__license__ = ""
__version__ = "0.0.3"
__all__ = []

def main():
    rospy.init_node("cyborg_controller")

    # Create emotions
    emotion_system = EmotionSystem()
    emotion_system.add_emotion(name="angry", pleasure=-0.51, arousal=0.59, dominance=0.25)
    emotion_system.add_emotion(name="bored", pleasure=-0.65, arousal=-0.62, dominance=-0.33)
    emotion_system.add_emotion(name="curious", pleasure=0.22, arousal=0.62, dominance=-0.10)
    emotion_system.add_emotion(name="dignified", pleasure=0.55, arousal=0.22, dominance=0.61)
    emotion_system.add_emotion(name="elated", pleasure=0.50, arousal=0.42, dominance=0.23) # Happy
    emotion_system.add_emotion(name="inhibited", pleasure=-0.54, arousal=-0.04, dominance=-0.41) # Sadness
    emotion_system.add_emotion(name="puzzled", pleasure=-0.41, arousal=0.48, dominance=-0.33) # Suprized
    emotion_system.add_emotion(name="loved", pleasure=0.89, arousal=0.54, dominance=-0.18)
    emotion_system.add_emotion(name="unconcerned", pleasure=-0.13, arousal=-0.41, dominance=0.08)

    homedir = os.path.expanduser("~")
    path = homedir + "/controller.db"

    # Fill database with default values
    if (os.path.exists(path) == False):
        event_cost = 0.45
        database_handler = DatabaseHandler(filename=path)
        database_handler.create()
        database_handler.add_event(state="idle", event="music_play", reward_pleasure=0.08, reward_arousal=0.01, reward_dominance=-0.02, event_cost=event_cost)
        database_handler.add_event(state="idle", event="navigation_emotional", reward_pleasure=0.00, reward_arousal=0.05, reward_dominance=-0.01, event_cost=event_cost)

        database_handler.add_event(state="conversation", event="weather_tell", reward_pleasure=0.05, reward_arousal=0.00, reward_dominance=0.00, event_cost=event_cost)
        database_handler.add_event(state="conversation", event="joke_tell", reward_pleasure=0.05, reward_arousal=0.02, reward_dominance=0.00, event_cost=event_cost)
        database_handler.add_event(state="conversation", event="selfie_take", reward_pleasure=0.05, reward_arousal=-0.02, reward_dominance=0.01, event_cost=event_cost*1.5)
        database_handler.add_event(state="conversation", event="simon_says_play", reward_pleasure=0.00, reward_arousal=0.00, reward_dominance=0.10, event_cost=event_cost)
        database_handler.add_event(state="conversation", event="follower_follow", reward_pleasure=0.00, reward_arousal=0.05, reward_dominance=-0.05, event_cost=event_cost)

    # Create motivator
    motivator = Motivator(database_file=path)
    motivator.start()

    # Create a SMACH state machine
    state_machine = smach.StateMachine(outcomes=["error"])
    state_machine.userdata.state_machine_events = []
    state_machine.userdata.last_state = "initializing"
    state_machine.userdata.last_event = "start_up"

    # Open the container
    with state_machine:
        # Remapp outputs, so userdata can be moved between states
        state_machine_remapping = {"input_events":"state_machine_events", "output_events":"state_machine_events", "previous_state":"last_state", "current_state":"last_state", "previous_event":"last_event", "current_event":"last_event"}

        # Add states to the container
        idle_transitions = {"conversation_interest":"conversation", "navigation_schedualer":"navigation_planing", "navigation_emotional":"navigation_planing", "aborted":"idle", "navigation_command":"navigation_planing", "music_play":"music"}
        idle_resources = {} # Idle does not require any resources
        smach.StateMachine.add(label="idle", state=Module(state_name="idle", actionlib_name="cyborg_idle/idle", transitions=idle_transitions, resources=idle_resources), transitions=idle_transitions, remapping=state_machine_remapping)

        conversation_transitions = {"aborted":"idle", "succeded":"idle", "navigation_feedback":"navigation_talking", "navigation_command":"navigation_talking", "navigation_information":"navigation_talking", "simon_says_play":"simon_says", "selfie_take":"selfie", "follower_follow":"follower", "weather_tell":"weather", "joke_tell":"joke"} 
        conversation_resources = {"trollface":"cyborg_conversation"}
        smach.StateMachine.add(label="conversation", state=Module(state_name="conversation", actionlib_name="cyborg_conversation/conversation", transitions=conversation_transitions, resources=conversation_resources), transitions=conversation_transitions, remapping=state_machine_remapping)

        navigation_transitions = {"aborted":"navigation_talking", "navigation_start_wandering":"navigation_moving", "navigation_start_moving":"navigation_moving", "succeded":"navigation_talking"} 
        navigation_resources = {"base":"cyborg_navigation"}
        smach.StateMachine.add(label="navigation_planing", state=Module(state_name="navigation_planing", actionlib_name="cyborg_navigation/planing", transitions=navigation_transitions, resources=navigation_resources), transitions=navigation_transitions, remapping=state_machine_remapping)

        navigation_transitions = {"aborted":"navigation_talking", "succeded":"navigation_talking", "navigation_wandering_completed":"idle"} 
        navigation_resources = {"base":"cyborg_navigation"}
        smach.StateMachine.add(label="navigation_moving", state=Module(state_name="navigation_moving", actionlib_name="cyborg_navigation/moving", transitions=navigation_transitions, resources=navigation_resources), transitions=navigation_transitions, remapping=state_machine_remapping)

        navigation_transitions = {"aborted":"idle", "succeded":"idle", "navigation_feedback_completed":"conversation", "navigation_command":"navigation_planing"} 
        navigation_resources = {"base":"cyborg_navigation", "trollface":"cyborg_navigation"}
        smach.StateMachine.add(label="navigation_talking", state=Module(state_name="navigation_talking", actionlib_name="cyborg_navigation/talking", transitions=navigation_transitions, resources=navigation_resources), transitions=navigation_transitions, remapping=state_machine_remapping)

        music_transitions = {"aborted":"idle", "succeded":"idle", "conversation_interest":"conversation"} 
        music_resources = {}
        smach.StateMachine.add(label="music", state=Module(state_name="music", actionlib_name="cyborg_music/music", transitions=music_transitions, resources=music_resources), transitions=music_transitions, remapping=state_machine_remapping)


        smach.StateMachine.add(label="simon_says", 
                                state=Module(state_name="simon_says", 
                                            actionlib_name="cyborg_conversation/simon_says", 
                                            transitions={"aborted":"conversation", "succeded":"conversation"}, 
                                            resources={"trollface":"cyborg_simon_says"}), 
                                transitions={"aborted":"conversation", "succeded":"conversation"}, 
                                remapping=state_machine_remapping)

        smach.StateMachine.add(label="selfie", 
                                state=Module(state_name="selfie", 
                                            actionlib_name="cyborg_conversation/selfie", 
                                            transitions={"aborted":"conversation", "succeded":"conversation"}, 
                                            resources={"trollface":"cyborg_selfie"}), 
                                transitions={"aborted":"conversation", "succeded":"conversation"}, 
                                remapping=state_machine_remapping)

        smach.StateMachine.add(label="follower", 
                                state=Module(state_name="follower", 
                                            actionlib_name="cyborg_conversation/follower", 
                                            transitions={"aborted":"conversation", "succeded":"conversation"}, 
                                            resources={"trollface":"cyborg_follower"}), 
                                transitions={"aborted":"conversation", "succeded":"conversation"}, 
                                remapping=state_machine_remapping)

        smach.StateMachine.add(label="weather", 
                                state=Module(state_name="weather", 
                                            actionlib_name="cyborg_conversation/weather", 
                                            transitions={"aborted":"conversation", "succeded":"conversation"}, 
                                            resources={"trollface":"cyborg_weather"}), 
                                transitions={"aborted":"conversation", "succeded":"conversation"}, 
                                remapping=state_machine_remapping)

        smach.StateMachine.add(label="joke", # name on state
                                state=Module(state_name="joke", # name on state
                                            actionlib_name="cyborg_conversation/joke", # actionlib name
                                            transitions={"aborted":"conversation", "succeded":"conversation"}, #event name:state - events that leads away from state
                                            resources={"trollface":"cyborg_joke"}), # for gatekeepers: resource_name:node_name
                                transitions={"aborted":"conversation", "succeded":"conversation"}, 
                                remapping=state_machine_remapping)


        ################################################### ADD MORE STATES BELOW ################################################### 




        ################################################### STOP ADDING YOUR STATES ################################################### 

    # Create a state machine monitorer
    smm = StateMachineMonitor(state_machine, display_all=True)
    rospy.loginfo("Controller: State Machine Monitor Activated...")

    import smach_ros
    sis = smach_ros.IntrospectionServer('controler_viewer', state_machine, '/controler_viewer')
    sis.start()

    # Create a thread to execute the smach
    smach_thread = threading.Thread(target=state_machine.execute)
    smach_thread.daemon = True
    smach_thread.start()
    rospy.loginfo("Controller: SMACH activated...")

    # Start ROS main looping
    rospy.loginfo("Controller: Activated...")
    rospy.spin()
    sis.stop()
    rospy.loginfo("Controller: Terminated...")


if __name__ == "__main__":
    print("Cyborg Controller: Starting Program...")

    if sys.version_info < (2,5):
        print("Cyborg Controller: Running Python version " + str(sys.version_info.major) + "." + str(sys.version_info.minor) + "." + str(sys.version_info.micro) + " (Python version 2.5 or grater is required)...")
        exit()

    main()

    print("Cyborg Controller: End of Program...")

