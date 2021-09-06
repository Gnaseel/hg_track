#!/usr/bin/env python

import rospy
from sensor_msgs.msg import *
from geometry_msgs.msg import *
import numpy as np
from cv_bridge import CvBridge 
import cv2

bridge = CvBridge()



warp_img_w = 340
warp_img_h = 300

warpx_margin = 20
warpy_margin = 3

nwindows = 20
margin = 50
minpix = 3
lane_bin_th=120
#[x,y]
#13
#24
warp_src  = np.array([
    [280, 130],  
    [280,  450],
    [620, 130],
    [620, 450]
], dtype=np.float32)

warp_dist = np.array([
    [0,0],
    [0,warp_img_h],
    [warp_img_w,0],
    [warp_img_w, warp_img_h]
], dtype=np.float32)

def warp_image(img, src, dst, size):
    M = cv2.getPerspectiveTransform(src, dst)
    Minv = cv2.getPerspectiveTransform(dst, src)
    warp_img = cv2.warpPerspective(img, M, size, flags=cv2.INTER_LINEAR)
    return warp_img, M, Minv

def warp_process_image(img):
    global nwindows
    global margin
    global minpix
    global lane_bin_th

    blur = cv2.GaussianBlur(img,(5, 5), 0)
    _, L, _ = cv2.split(cv2.cvtColor(blur, cv2.COLOR_BGR2HLS))
    _, lane = cv2.threshold(L, lane_bin_th, 255, cv2.THRESH_BINARY)

    histogram = np.sum(lane[lane.shape[0]//2:,:], axis=0)      
    midpoint = np.int(histogram.shape[0]/2)
    # leftx_current = np.argmax(histogram[:midpoint])
    leftx_current = np.argmax(histogram[:])
    rightx_current = np.argmax(histogram[midpoint:]) + midpoint

    window_height = np.int(lane.shape[0]/nwindows)
    nz = lane.nonzero()

    left_lane_inds = []

    lx, ly = [], []

    out_img = np.dstack((lane, lane, lane))*255


    poseArr = PoseArray()
    poses_pub = rospy.Publisher("/hg_lane/lane_poses", PoseArray)

    for window in range(nwindows):

        win_yl = lane.shape[0] - (window+1)*window_height
        win_yh = lane.shape[0] - window*window_height

        win_xll = leftx_current - margin
        win_xlh = leftx_current + margin

        cv2.rectangle(out_img,(win_xll,win_yl),(win_xlh,win_yh),(0,255,0), 2)

        good_left_inds = ((nz[0] >= win_yl)&(nz[0] < win_yh)&(nz[1] >= win_xll)&(nz[1] < win_xlh)).nonzero()[0]

        left_lane_inds.append(good_left_inds)

        if len(good_left_inds) > minpix:
            leftx_current = np.int(np.mean(nz[1][good_left_inds]))
            pose = Pose()
            pose.position.x = (window+0.5)*window_height
            pose.position.y = leftx_current
            poseArr.poses.append(pose)
            
        # print(leftx_current)

        lx.append(leftx_current)
        ly.append((win_yl + win_yh)/2)

    poses_pub.publish(poseArr)
    left_lane_inds = np.concatenate(left_lane_inds)
    
    lfit = np.polyfit(np.array(ly),np.array(lx),2)

    out_img[nz[0][left_lane_inds], nz[1][left_lane_inds]] = [0, 125, 255]

  
    # print("--------")
    cv2.imshow("hg_lane_viewer", out_img)
    
    # return left_fit, rig
    return lfit

def image_callback(msg):
    global image_np
    np_arr = np.fromstring(msg.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    image_np = bridge.imgmsg_to_cv2(msg, "bgr8")
    
    warp_img, M, Minv = warp_image(image_np, warp_src, warp_dist, (warp_img_w, warp_img_h))
    left_fit = warp_process_image(warp_img)
    # lane_img = draw_lane(image, warp_img, Minv, left_fit, right_fit)
    cv2.imshow("img", image_np)
    cv2.imshow("img2", warp_img)
    
    cv2.waitKey(3)
    return



if __name__ == '__main__':

    rospy.init_node("lane_detection")
    imu_sub = rospy.Subscriber("/cv_camera/image_raw", Image, image_callback)

    print("ON")
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        rate.sleep()