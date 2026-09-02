#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from Planner.path_planner_node import run_planner
import threading

current_state = State()

def state_cb(msg):
    global current_state
    current_state = msg


if __name__ == "__main__":
    rospy.init_node("offboard_setpoints", log_level=rospy.INFO)

    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)

    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)


    # Setpoint publishing faster than 2Hz
    rate = rospy.Rate(20)

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()
        
    init_pose = PoseStamped()

    init_pose.pose.position.x = 0
    init_pose.pose.position.y = 0
    init_pose.pose.position.z = 1.8

    # Send a few setpoints before starting
    rospy.loginfo("Sending initial setpoints ...")
    for i in range(100):
        if(rospy.is_shutdown()):
            break

        local_pos_pub.publish(init_pose)
        rate.sleep()
    rospy.loginfo("Finised sending initial setpoints")
    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True
    last_req = rospy.Time.now()

    planner_started = False

    def delayed_planner_start():
        global planner_started
        rospy.loginfo("Waiting 15 seconds before starting the planner...")
        rospy.sleep(15)
        planner_started = True
        rospy.loginfo("Starting planner thread now.")
        run_planner()

    while not rospy.is_shutdown():
        if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")
            last_req = rospy.Time.now()

        elif(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(arming_client.call(arm_cmd).success == True):
                rospy.loginfo("Vehicle armed")
            else:
                rospy.loginfo("Not armed")

            # Start planner with 15-second delay
            planner_thread = threading.Thread(target=delayed_planner_start)
            planner_thread.daemon = True 
            planner_thread.start()
            last_req = rospy.Time.now()

        # Publish init_pose only until planner starts publishing
        if not planner_started:
            local_pos_pub.publish(init_pose)

        rate.sleep()
