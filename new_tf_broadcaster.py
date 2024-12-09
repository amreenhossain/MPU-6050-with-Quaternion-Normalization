#!/usr/bin/env python3  
import rospy
import tf2_ros
import geometry_msgs.msg
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler, euler_from_quaternion

def handle_imu_pose(msg):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    # Extract and convert the orientation quaternion to roll, pitch, yaw
    q = msg.orientation
    rpy = euler_from_quaternion([q.x, q.y, q.z, q.w])

    # Adjust the RPY values if necessary
    # Apply any necessary transformations to correct the axes
    corrected_roll = rpy[0]
    corrected_pitch = rpy[1]
    corrected_yaw = rpy[2]

    # Convert back to quaternion after adjusting axes
    corrected_q = quaternion_from_euler(corrected_roll, corrected_pitch, corrected_yaw)

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "plane"
    t.child_frame_id = "imu_link"
    t.transform.translation.x = 0
    t.transform.translation.y = 0
    t.transform.translation.z = 0
    t.transform.rotation.x = corrected_q[0]
    t.transform.rotation.y = corrected_q[1]
    t.transform.rotation.z = corrected_q[2]
    t.transform.rotation.w = corrected_q[3]

    br.sendTransform(t)

if __name__ == '__main__':
    rospy.init_node('tf_broadcaster_imu')
    rospy.Subscriber('/imu/data', Imu, handle_imu_pose)
    rospy.spin()

