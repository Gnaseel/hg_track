#!/usr/bin/env python

import rospy
from nav_msgs.msg import *
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from std_msgs.msg import Float32, UInt16, UInt8
import matplotlib.pyplot as plt
import math
import os
import con as cl
import threading
import time

firstRun = True
odomX=[]
odomY=[]

odomCon=[]

global_cons = []
global_convex = []
lanes = []
targetCon = cl.Con()
new1 = cl.Con()
new2 = cl.Con()
nowX    = 0
nowY    = 0
nowYaw  = 0

ground_truth_steer=0
cur_speed=0
# ----------------------------- NEWNEWNEW ------------------------------


def norDeg(deg):
    while deg < -180:
        deg +=360
    while 180 < deg:
        deg -=360 
    return deg
def poseBycon(con):
    pose = Pose()
    pose.position.x=con.x
    pose.position.y=con.y
    return pose
def getCons(cons, target, t_degree, t_dist):
    re = []
    for con in cons:
        deg = math.atan2(con.y, con.x)*180/math.pi
        dist = getDist(target.x,target.y  ,  con.x, con.y)
        if con.x<1.5:
            continue
        if abs(con.y) > 3.5:
            continue
        if deg < -t_degree or t_degree < deg:
            continue
        if t_dist < dist:
            continue
        re.append(con)
    return re
# def sen(cons, start_deg, end_deg, start_dist, end_dist):

#     me = cl.Con()
#     first_con = []
#     all_con=[]
#     poses = PoseArray()
#     pub_poses = PoseArray()
    
#     for i in range(int(start_dist), end_dist):
#         first_con = getCons (cons, me, 70, i)

#         if (len(first_con)==1 and i > 7) or len(first_con)==2:
#             break

#     for f_con in first_con:
#         for s_con in cons:
#             deg = math.atan2(s_con.y-f_con.y, s_con.x-f_con.x)*180/math.pi
#             dist = getDist(s_con.x,s_con.y  ,  f_con.x, f_con.y)

#             if deg < start_deg or end_deg < deg:
#                 continue
#             if dist < start_dist or end_dist < dist:
#                 continue
#             all_con.append(s_con)
#     # if len(first_con)==0:
#     #     return
#     mean_x=0
#     mean_y=0
#     for con in first_con:
#         poses.poses.append(poseBycon(con))    
#         mean_x += con.x
#         mean_y += con.y
#     for con in all_con:
#         poses.poses.append(poseBycon(con))
#         mean_x += con.x
#         mean_y += con.y
#     mean_x /= len(poses.poses)+0.0001
#     mean_y /= len(poses.poses)+0.0001

#     print("SIZE {}".format(len(poses.poses)))
#     print("{} / {}".format(mean_x, mean_y))

#     for pose in poses.poses:
#         dist = getDist(mean_x, mean_y, pose.position.x, pose.position.y)
#         print("{} / {}   -------- {}".format(pose.position.x, pose.position.y, dist))
#         if  dist < 5:
#             pub_poses.poses.append(pose)
#     os.system('clear')
#     con_pub.publish(pub_poses)


def getFinalTarget(can1, can2, target):

    finalTarget = cl.Con()
    val1 = ccw(0,0  ,  can1.x,can1.y  ,  target.x,target.y)
    val2 = ccw(0,0  ,  can2.x,can2.y  ,  target.x,target.y)

    dist = getDist(can1.x, can1.y, target.x, target.y)

    if dist < 1.8:
        finalTarget = target
        return finalTarget
    dist1 = getDist(can1.x, can1.y, 0, 0)
    dist2 = getDist(can2.x, can2.y, 0, 0)
    if dist1<dist2:
        return can1
    else:
        return can2

def getNearTarget(con1, con2): 
    interval = 1.6
    dx = con2.x - con1.x
    dy = con2.y - con1.y
    dist = math.sqrt(dx*dx+dy*dy)

    newTarget=cl.Con()
    newTarget.x = con1.x + dx*(interval/(dist+0.001))
    newTarget.y = con1.y + dy*(interval/(dist+0.001))

    newTarget2=cl.Con()
    newTarget2.x = con2.x - dx*(interval/(dist+0.001))
    newTarget2.y = con2.y - dy*(interval/(dist+0.001))
    return newTarget, newTarget2
import copy

def getFirstContact(global_cons):
    cons = []
    for con in global_cons:
        cons.append(copy.copy(con))
    min_dist = 9999
    con1 = cl.Con()
    con2 = cl.Con()
    x1=0
    y1=0
    x2=0
    y2=0
    # print("CONVEX SIZE = {}".format(len(cons)))
    for idx, con in enumerate(cons):
        if idx == len(cons)-1:
            break
        if cons[idx].y * cons[idx+1].y < 0:
            dx = cons[idx+1].x - cons[idx].x
            dy = cons[idx+1].y - cons[idx].y
            # firstX = abs(cons[idx].y)/dy + cons[idx].x
            firstX = -1*dx*cons[idx].y/dy+cons[idx].x

            if firstX < min_dist and (firstX > cons[idx].x or firstX > cons[idx+1].x):
                min_dist=firstX
                x1 = cons[idx].x
                x2 = cons[idx+1].x
                y1 = cons[idx].y
                y2 = cons[idx+1].y
                con1 = cons[idx]
                con2 = cons[idx+1]

            # print("DIST X = {0:0.2f} --- {1:0.2f} {2:0.2f} //// {3:0.2f} {4:0.2f}".format(firstX, cons[idx].x, cons[idx].y,  cons[idx+1].x, cons[idx+1].y))

    if min_dist != 9999 and (con1.x+con2.x)/2 < 5.0:
        return con1, con2
    for con in cons:
        con.y +=1
    for idx, con in enumerate(cons):
        if idx == len(cons)-1:
            break
        if cons[idx].y * cons[idx+1].y < 0:
            dx = cons[idx+1].x - cons[idx].x
            dy = cons[idx+1].y - cons[idx].y
            # firstX = abs(cons[idx].y)/dy + cons[idx].x
            firstX = -1*dx*cons[idx].y/dy+cons[idx].x

            if firstX < min_dist and (firstX > cons[idx].x or firstX > cons[idx+1].x):
                min_dist=firstX
                x1 = cons[idx].x
                x2 = cons[idx+1].x
                y1 = cons[idx].y
                y2 = cons[idx+1].y
                
                added = cl.Con()
                added.x = cons[idx].x
                added.y = cons[idx].y-1
                added2 = cl.Con()
                added2.x = cons[idx+1].x
                added2.y = cons[idx+1].y-1
                con1 = added
                con2 = added2
    
    for con in cons:
        con.y -=2
    for idx, con in enumerate(cons):
        if idx == len(cons)-1:
            break
        if cons[idx].y * cons[idx+1].y < 0:
            dx = cons[idx+1].x - cons[idx].x
            dy = cons[idx+1].y - cons[idx].y
            # firstX = abs(cons[idx].y)/dy + cons[idx].x
            firstX = -1*dx*cons[idx].y/dy+cons[idx].x

            if firstX < min_dist and (firstX > cons[idx].x or firstX > cons[idx+1].x):
                min_dist=firstX
                x1 = cons[idx].x
                x2 = cons[idx+1].x
                y1 = cons[idx].y
                y2 = cons[idx+1].y
                
                added = cl.Con()
                added.x = cons[idx].x
                added.y = cons[idx].y+1
                added2 = cl.Con()
                added2.x = cons[idx+1].x
                added2.y = cons[idx+1].y+1
                con1 = added
                con2 = added2

    if con1.x == 0:
        print("ZERO!!!")
        for con in cons:
            con.y +=3
        for idx, con in enumerate(cons):
            if idx == len(cons)-1:
                break
            if cons[idx].y * cons[idx+1].y < 0:
                dx = cons[idx+1].x - cons[idx].x
                dy = cons[idx+1].y - cons[idx].y
                # firstX = abs(cons[idx].y)/dy + cons[idx].x
                firstX = -1*dx*cons[idx].y/dy+cons[idx].x
                if firstX < min_dist and (firstX > cons[idx].x or firstX > cons[idx+1].x):
                    min_dist=firstX
                    x1 = cons[idx].x
                    x2 = cons[idx+1].x
                    y1 = cons[idx].y
                    y2 = cons[idx+1].y
                    added = cl.Con()
                    added.x = cons[idx].x
                    added.y = cons[idx].y-2
                    added2 = cl.Con()
                    added2.x = cons[idx+1].x
                    added2.y = cons[idx+1].y-2
                    con1 = added
                    con2 = added2
                    # print("{} {} // {} {}".format(con1.x, con1.y, con2.x, con2.y))

        for con in cons:
            con.y -=4
        for idx, con in enumerate(cons):
            if idx == len(cons)-1:
                break
            if cons[idx].y * cons[idx+1].y < 0:
                dx = cons[idx+1].x - cons[idx].x
                dy = cons[idx+1].y - cons[idx].y
                # firstX = abs(cons[idx].y)/dy + cons[idx].x
                firstX = -1*dx*cons[idx].y/dy+cons[idx].x
                if firstX < min_dist and (firstX > cons[idx].x or firstX > cons[idx+1].x):
                    min_dist=firstX
                    x1 = cons[idx].x
                    x2 = cons[idx+1].x
                    y1 = cons[idx].y
                    y2 = cons[idx+1].y
                    added = cl.Con()
                    added.x = cons[idx].x
                    added.y = cons[idx].y+2
                    added2 = cl.Con()
                    added2.x = cons[idx+1].x
                    added2.y = cons[idx+1].y+2
                    con1 = added
                    con2 = added2
    print("{} {} // {} {}".format(con1.x, con1.y, con2.x, con2.y))
    # print("     FIRST X = {0:0.2f} --- {1:0.2f} {2:0.2f} //// {3:0.2f} {4:0.2f}".format(min_dist, x1, y1, x2, y2))
    # os.system('clear')
    return con1, con2
#-------------------------- CALLBACK -----------------------------------
def convex_callback(msg):
    global global_convex, targetCon, new1, new2
    # print("CONVEX SIXE {} ".format(len(msg.poses)))
    to_global_convex=[]
    if len(msg.poses) <=2:
        return
    for pose in msg.poses:
        con = cl.Con()
        con.x = pose.position.x
        con.y = pose.position.y
        to_global_convex.append(con)
    con = cl.Con()
    con.x = msg.poses[0].position.x
    con.y = msg.poses[0].position.y
    to_global_convex.append(con)
    
    con1, con2 = getFirstContact(to_global_convex)
    new1, new2 = getNearTarget(con1, con2)
    if con1.x != 0:
        targetCon.x = (con1.x+con2.x)/2
        targetCon.y = (con1.y+con2.y)/2
    targetCon = getFinalTarget(new1, new2, targetCon)
    global_convex=to_global_convex

def cons_callback(msg):
    global global_cons
    pre_con=global_cons

    new_cons =[]

    off_x=0
    off_y=0
    off_num=0
    new_num=0
     
    for con in msg.poses:
        dist = math.sqrt(con.position.x*con.position.x +  con.position.y*con.position.y)
        # if con.position.x < -0.5:
        #     continue
        if dist<0.8 and con.position.x < -0.5 and abs(con.position.y) < 0.5:
            continue
        if dist>20:
            continue
        temp = cl.Con()
        temp.x=con.position.x
        temp.y=con.position.y
        new_cons.append(temp)

    global_cons=[]
    global_cons=new_cons

def steer_callback(msg):
    global ground_truth_steer
    ground_truth_steer = msg

def speed_callback(msg):
    global cur_speed
    cur_speed=msg.data
    return
#-------------------------- COMMON -----------------------------------
def getDist(posX, posY, posX2, posY2):
    return math.sqrt((posX-posX2)*(posX-posX2) + (posY-posY2)*(posY-posY2))


def ccw(x1, y1, x2, y2, x3, y3):
    arw = 0
    arw = (x2-x1) * (y3-y1) - (y2-y1) * (x3-x1)
    return arw

if __name__ == '__main__':

    rospy.init_node("pppy")
    # imu_sub = rospy.Subscriber("/imu/data", Imu, imu_callback)
    convex_con_sub = rospy.Subscriber("/convex/cons", PoseArray, convex_callback)
    con_sub = rospy.Subscriber("/lane/cons", PoseArray, cons_callback)
    steer_sub = rospy.Subscriber("/erp42/steer_r", Float32, steer_callback)
    # steer_sub = rospy.Subscriber("/erp42/steer_r", Float32, steer_callback)

    speed_sub = rospy.Subscriber("/erp42/speed_r", UInt16, speed_callback)


    con_pub = rospy.Publisher("/hg_lane/cons", PoseArray)
    steer_pub = rospy.Publisher("/control/angle", Float32)
    speed_pub = rospy.Publisher("/control/accel", UInt16)
    brake_pub = rospy.Publisher("/control/brake", UInt8)

    print("ON")
    rate = rospy.Rate(100)
    lock = threading.Lock() 


    targetCon.x = 2
    targetCon.y = 0
    
    while not rospy.is_shutdown():
        now_cons = global_cons
        now_convex = global_convex

        # max_line=None

        # sen(now_cons, -50, 50, 0.0, 4)
        # sen(now_cons, -70, 70, 0.0, 4)
        # print("CONVEX SIZE ====================== {}".format(len(global_convex)))
        plt.cla()
        plt.xlim([-13,13])
        plt.ylim([-13,13])
        plt.scatter(0,0 ,c='black',s=400)   
        size = len(now_cons)
        convex_size = len(now_convex)
        plt.plot([now_cons[idx].x for idx in range(size)],[now_cons[idx].y for idx in range(size)], "b*")
        plt.plot([now_convex[idx].x for idx in range(convex_size)],[now_convex[idx].y for idx in range(convex_size)], "r-")

        target=cl.Con()
        target.x =3
        target.y =-0.5
        target_deg = -math.atan2( targetCon.y, targetCon.x)*180/math.pi
        
        plt.scatter(new1.x  ,new1.y  ,c='gray',s=200)
        plt.scatter(new2.x  ,new2.y  ,c='gray',s=200)
        plt.scatter(targetCon.x  ,targetCon.y  ,c='yellow',s=150)

        plt.plot([0, targetCon.x], [0, targetCon.y], linewidth=5, linestyle='--', color="red" )

        if target_deg > 90:
            target_deg -=180
        if target_deg < -90:
            target_deg +=180

        speed=0
        brake=0
        if abs(target_deg) > 10:
            speed = 6
            if cur_speed > 70:
                print("2222222222Cur speed {}".format(cur_speed))
                print("2222222222Cur speed {}".format(cur_speed*2))
                brake=50
        else:
            speed = 12
        # os.system('clear')
        if cur_speed < 70:
            target_deg*=1.5
        steer_pub.publish(target_deg)
        speed_pub.publish(speed)  

        print("CURR -----------------------------")
        print("Cur speed {}\n".format(cur_speed))
        print("TARG -----------------------------")
        print("Tar speed {}".format(speed))
        print("Tar steer {}".format(target_deg))
        print("Tar brake {}\n".format(brake))
        plt.pause(0.001)
        rate.sleep()
