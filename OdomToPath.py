#!/usr/bin/python
import rospy
import numpy
import csv
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped



pathMsg = Path()

def callback(msg):
    '''Listens to msg, records latest path, publishes full path

    this approach only works for Path style messages,

    :param msg: path msg received from ros
    '''
    global pathMsg
    global pub
    pathMsg.header.stamp    = msg.header.stamp
    pathMsg.header.frame_id = msg.header.frame_id

    poseS = PoseStamped()
    poseS.header = msg.header
    poseS.pose.position.x = msg.pose.pose.position.x
    poseS.pose.position.y = msg.pose.pose.position.y
    poseS.pose.position.z = msg.pose.pose.position.z
    
    poseS.pose.orientation.w = msg.pose.pose.orientation.w
    poseS.pose.orientation.x = msg.pose.pose.orientation.x
    poseS.pose.orientation.y = msg.pose.pose.orientation.y
    poseS.pose.orientation.z = msg.pose.pose.orientation.z


    pathMsg.poses.append(poseS)

    pub.publish(pathMsg)



    # print(msg)
def writePath():
    '''Uses latest message, writes all appropriate data from each pose into a csv

    '''
    global pathMsg
    with open('~/Documents/msckf_poses.csv', 'w') as csvFile:
        Wrt = csv.writer(csvFile, delimiter=',')
        Wrt.writerow(['timestamp (nS)', 'pos_x', 'pos_y', 'pos_z', 'quat_w', 'quat_x', 'quat_y', 'quat_z' ])
        # print(pathMsg)
        initPose = pathMsg.poses[0]
        for pose in pathMsg.poses:
            # print(pose)
            Wrt.writerow([  str(pose.header.stamp.secs) + '' + str(pose.header.stamp.nsecs),
                            "%.6f" % (pose.pose.position.x - initPose.pose.position.x),
                            "%.6f" % (pose.pose.position.y - initPose.pose.position.y),
                            "%.6f" % (pose.pose.position.z - initPose.pose.position.z),
                            "%.6f" %  pose.pose.orientation.w,
                            "%.6f" %  pose.pose.orientation.x,
                            "%.6f" %  pose.pose.orientation.y,
                            "%.6f" %  pose.pose.orientation.z
                        ])
        print("done")






if __name__ =="__main__":
    global pub
    rospy.init_node("pathConverter")
    rospy.Subscriber("/msckf/odom", Odometry, callback)
    pub = rospy.Publisher("/msckf/imu_path", Path, queue_size=5)
    rospy.on_shutdown(writePath)
    rospy.spin()
