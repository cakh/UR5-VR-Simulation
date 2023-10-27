#!/usr/bin/env python

import rospy
import time
import rosgraph
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from math import pi

TOPIC_NAME = 'joint_pos'
NODE_NAME = 'pospub'

angle = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
def publish_pos():
    pub = rospy.Publisher(TOPIC_NAME, RobotTrajectory, queue_size=10)
    rospy.init_node(NODE_NAME, anonymous=True)
    wait_for_connections(pub, TOPIC_NAME)
    while not rospy.is_shutdown():
        jointval = RobotTrajectory()
        jointval.joint_trajectory.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]        
        point = JointTrajectoryPoint()
        rospy.Subscriber("joint_states", JointState, callback)
        
        point.positions = [30.0, -90.0, -120.0, 20.0, 80.0, 0.0]  # Start positions for joint1 and joint2
        for i in range(6):
            point.positions[i] = angle[i]*180/pi
        point.time_from_start = rospy.Duration(0.0)
        jointval.joint_trajectory.points.append(point)
        pub.publish(jointval)
        time.sleep(0.1)

def callback(data):
    global angle
    for i, joint_name in enumerate(data.name):
        angle[i] = data.position[i]
        
def wait_for_connections(pub, topic):
    ros_master = rosgraph.Master('/rostopic')
    topic = rosgraph.names.script_resolve_name('rostopic', topic)
    num_subs = 0
    for sub in ros_master.getSystemState()[1]:
        if sub[0] == topic:
            num_subs+=1

    for i in range(10):
        if pub.get_num_connections() == num_subs:
            return
        time.sleep(0.1)
    raise RuntimeError("failed to get publisher")


if __name__ == '__main__':
    try:
        publish_pos()
    except rospy.ROSInterruptException:
        pass
