#!/home/kevin/.pyenv/versions/pyenv_py385/bin/python

import rospy
from std_msgs.msg import Bool, Pose


def main():
    rospy.Subscriber('aruco_xyzw', Pose, find_aruco_pose_callback)
    rospy.Subscriber('check_aruco', Bool, check_aruco_pose_callback)
    rospy.spin()


def find_aruco_pose_callback(data):
    print(data)


def check_aruco_pose_callback(check):
    print(check)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
