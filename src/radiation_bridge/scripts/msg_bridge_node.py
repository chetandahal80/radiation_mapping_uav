#!/usr/bin/env python3

import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import Pose
from gazebo_radiation_plugins.msg import Simulated_Radiation_Msg
from hector_radiation_mapping_msgs.msg import Sample, Samples

class RadiationBridge:
    def __init__(self):
        self.pub = rospy.Publisher('/radiation_bridge/output', Sample, queue_size=10)
        rospy.Subscriber('/radiation_sensor_plugin/sensor_0', Simulated_Radiation_Msg, self.callback)

    def callback(self, msg):
        sample = Sample()
        sample.header = msg.header
        sample.position = [
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ]
        sample.doseRate = msg.value*(pow(10,6))
        sample.cps = sample.doseRate * 231.91
        sample.id = 0
        sample.for2d = True
        sample.for3d = True

        # Add flat field: store the frame_id
        sample.frame_id = msg.header.frame_id  # <------ NEW

        self.pub.publish(sample)

if __name__ == '__main__':
    rospy.init_node('radiation_bridge')
    bridge = RadiationBridge()
    rospy.spin()

