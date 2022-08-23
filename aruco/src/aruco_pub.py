#!/home/kevin/.pyenv/versions/pyenv_py385/bin/python

import cv2
import glob
import numpy as np
from cv2 import aruco
import os
import rospy
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool

aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
board = aruco.CharucoBoard_create(7, 5, 0.38, 0.3, aruco_dict)
cam = cv2.VideoCapture(0)         # usb cam
cam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)


def calibrate_charuco():
    allCorners = []
    allIds = []
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.00001)
    images = glob.glob('/home/kevin/workspace/2022AutonomousShippingRobot/ArUco/camera_cal_img/video*.png')
    for im in images:
        print("=> Processing image {0}".format(im))
        frame = cv2.imread(im)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict)

        if len(corners) > 0:
            for corner in corners:
                cv2.cornerSubPix(gray, corner,
                                 winSize=(3, 3),
                                 zeroZone=(-1, -1),
                                 criteria=criteria)
            res2 = cv2.aruco.interpolateCornersCharuco(corners, ids, gray, board)
            if res2[1] is not None and res2[2] is not None and len(res2[1]) > 3:
                allCorners.append(res2[1])
                allIds.append(res2[2])

        imsize = gray.shape

    return allCorners, allIds, imsize


def calibrate_camera(allCorners,allIds,imsize):
    print("CAMERA CALIBRATION")

    cameraMatrixInit = np.array([[654.67156, 0., 311.80561],
                                 [0., 651.85143, 242.57995],
                                 [0.,        0.,        1.]])

    distCoeffsInit = np.zeros((5,1))
    flags = (cv2.CALIB_USE_INTRINSIC_GUESS + cv2.CALIB_RATIONAL_MODEL + cv2.CALIB_FIX_ASPECT_RATIO)
    #flags = (cv2.CALIB_RATIONAL_MODEL)
    (ret, camera_matrix, distortion_coefficients0,
     rotation_vectors, translation_vectors,
     stdDeviationsIntrinsics, stdDeviationsExtrinsics,
     perViewErrors) = cv2.aruco.calibrateCameraCharucoExtended(
                      charucoCorners=allCorners,
                      charucoIds=allIds,
                      board=board,
                      imageSize=imsize,
                      cameraMatrix=cameraMatrixInit,
                      distCoeffs=distCoeffsInit,
                      flags=flags,
                      criteria=(cv2.TERM_CRITERIA_EPS & cv2.TERM_CRITERIA_COUNT, 10000, 1e-9))
    #print("ret")
    #print(ret)
    #print("camera matrix")
    #print(camera_matrix)
    #print("distortion")
    #print(distortion_coefficients0)

    return ret, camera_matrix, distortion_coefficients0, rotation_vectors, translation_vectors


def detect_marker(matrix, distortion):
    parameters = cv2.aruco.DetectorParameters_create()
    aruco_pub = rospy.Publisher('aruco_xyzw', Pose, queue_size=1)
    check_pub = rospy.Publisher('check_aruco', Bool, queue_size=1)
    rate = rospy.Rate(10)
    aruco = Pose()
    check = Bool()
    if cam.isOpened():
        while True:
            _, frame = cam.read()
            frame2 = frame.copy()
            gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            coners, ids, point = cv2.aruco.detectMarkers(gray_frame, aruco_dict, parameters=parameters)
            if np.all(ids != None):
                check.data = True
                # rvecs, tvecs, _objPoints = cv2.aruco.estimatePoseSingleMarkers(coners, 0.04, mtx, dist)
                # frame = cv2.aruco.drawAxis(frame, mtx, dist, rvecs[0], tvecs[0], 0.04)
                rvecs, tvecs, _objPoints = cv2.aruco.estimatePoseSingleMarkers(coners, 0.03, matrix, distortion)
                frame = cv2.aruco.drawAxis(frame, matrix, distortion, rvecs[0], tvecs[0], 0.03)
                rvecs_msg = rvecs.tolist()
                tvecs_msg = tvecs.tolist()
                aruco.orientation.x = rvecs_msg[0][0][0]
                aruco.orientation.y = rvecs_msg[0][0][1]
                aruco.orientation.z = rvecs_msg[0][0][2]
                aruco.position.x = tvecs_msg[0][0][0]
                aruco.position.y = tvecs_msg[0][0][1]
                aruco.position.z = tvecs_msg[0][0][2]
                rospy.loginfo("{}".format(aruco))
                aruco_pub.publish(aruco)
            else:
                check.data = False
            check_pub.publish(check)
            rate.sleep()

            #cv2.putText(frame, "%5.2f/  %5.2f/  %5.2f" % ((tvecs[0][0][0]), (tvecs[0][0][1]), (tvecs[0][0][2])),
            #            (0, 450), cv2.FONT_HERSHEY_PLAIN, 2.0, (0, 0, 255), 2)

            cv2.putText(frame, "%.1f cm / %.0f deg" % ((tvecs[0][0][2] * 100), (rvecs[0][0][2] / math.pi * 180)),
                         (0, 450), cv2.FONT_HERSHEY_PLAIN, 2.0, (0, 0, 255), 2)

            frame = cv2.aruco.drawDetectedMarkers(frame, coners, ids)
            cv2.imshow('video', frame)
            k = cv2.waitKey(1) & 0xff                           #press 'esc' to kill
            if k == 27:
                #cv2.imread('video', cv2.IMREAD_UNCHANGED)
                #cv2.imwrite('video14.png', frame2)
                break
        cam.release()


def main():
    rospy.init_node("aruco_pub")
    allCorners, allIds, imsize = calibrate_charuco()
    ret, mtx, dist, rvec, tvec = calibrate_camera(allCorners, allIds, imsize)
    detect_marker(mtx, dist)

if __name__ == "__main__":
    main()
