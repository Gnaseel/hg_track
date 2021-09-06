#!/usr/bin/env python

import rospy
from nav_msgs.msg import *
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from std_msgs.msg import Float32
from std_msgs.msg import UInt16
import matplotlib.pyplot as plt
import math
import os
import con as cl
import threading

firstRun = True
odomX=[]
odomY=[]

odomCon=[]

global_cons = []
lanes = []

nowX    = 0
nowY    = 0
nowYaw  = 0

ground_truth_steer=0

#-------------------------- CALLBACK -----------------------------------
def cons_callback(msg):
    global global_cons

    pre_con=global_cons

    new_cons =[]
    update_cons=[]

    off_x=0
    off_y=0
    off_num=0
    new_num=0
    for con in msg.poses:
        dist = math.sqrt(con.position.x*con.position.x +  con.position.y*con.position.y)
        # if con.position.x < -0.5:
        #     continue
        if dist<1.5 and con.position.x < -0.5 and abs(con.position.y) < 0.5:
            continue
        if dist>9:
            continue
        near_con = getNearCons(pre_con, con, 0.6)
        temp = cl.Con()
        temp.x=con.position.x
        temp.y=con.position.y
        if near_con.x == -99:
            new_cons.append(temp)
            new_num +=1
        else:
            off_x += con.position.x - near_con.x
            off_y += con.position.y - near_con.y
            new_cons.append(temp)
            update_cons.append(temp)
            off_num +=1
        

    if off_num != 0:
        off_x = off_x/off_num
        off_y = off_y/off_num

        for con in pre_con:
            update_con = cl.Con()
            update_con.x = con.x + off_x
            update_con.y = con.y + off_y
            dist = getDist(update_con.x, update_con.y, 0, 0)
            # if 2<dist and dist < 6 :
            if dist < 1.9  and update_con.x > 0:
                new_cons.append(update_con)
        off_dist = getDist(off_x, off_y, 0, 0)
        # print("OFFX = "+str(off_x))
        # print("OFFY = "+str(off_y))
        # print("OFFD = "+str(off_dist))
        # print("OFFD_RAW = "+str(math.sqrt(off_x*off_x + off_y*off_y)))
        if off_dist > 0.5:
        # if abs(off_x) > 0.3 or abs(off_y) > 0.3:
            print("WHY????????????????")
            rospy.sleep(100)
    # print("SIZE - "+str(len(new_cons)))
    # print(str(new_num)+" / "+str(off_num))

    global_cons=new_cons
    # os.system('clear')

def lane_callback(msg):
    global lanes
    lanes=msg.poses
    # print("LANE NUM {} ".format(len(lanes)))
    # for lane in lanes:
    #     print("     LANE NUM {} {}".format(lane.position.x, lane.position.y))

   

def steer_callback(msg):
    global ground_truth_steer
    ground_truth_steer = msg

# def imu_callback(msg):
#     print("im")


#-------------------------- COMMON -----------------------------------

# rel to abs
def rel2abs(posX, posY):
    abs_X = posX*math.cos(nowYaw) - posY*math.sin(nowYaw) + nowX
    abs_Y = posX*math.sin(nowYaw) + posY*math.cos(nowYaw) + nowY
    return abs_X, abs_Y

def abs2rel(posX, posY):
    re_X = (posX-nowX)*math.cos(-nowYaw) - (posY-nowY)*math.sin(-nowYaw)
    re_Y = (posX-nowX)*math.sin(-nowYaw) + (posY-nowY)*math.cos(-nowYaw)
    return re_X, re_Y

def getDist(posX, posY, posX2, posY2):
    return math.sqrt((posX-posX2)*(posX-posX2) + (posY-posY2)*(posY-posY2))

def printConInfo(instance):
    print("X = "+str(instance.x)+"  Y = "+str(instance.y))

def abstoDist(posX, posY):
    rel_x, rel_y = abs2rel(posX, posY)
    return getDist(rel_x, rel_y, 0, 0)

def ccw(x1, y1, x2, y2, x3, y3):
    arw = 0
    arw = (x2-x1) * (y3-y1) - (y2-y1) * (x3-x1)
    return arw

#-------------------------- ALGORITHM -----------------------------------

# map update
def getNearCons(cons, pose, interval):
    min_dist = 9999
    re_con = cl.Con()
    for con in cons:
        dist = getDist(con.x, con.y, pose.position.x, pose.position.y)
        if dist < min_dist:
            min_dist = dist
            re_con = con


    if min_dist < interval:
        return re_con
    temp = cl.Con()
    temp.x=-99
    return temp

def getLcon(cons, last_con):
    min_dist = 900
    max_point = -300
    l_con = cl.Con()
    for con in cons:
        
        #----------------------- OUT ---------------------
        if con.y < -0.2 or 3.2 < con.y:
            # print("[OUT] Y condition")
            continue
        if con.x < 0.2 or 5.0 < con.x:
            # print("[OUT] X condition")
            continue
        if con.y - last_con.y > 2.0 and con.x < 3.0:
            continue
        #----------------------- POINT ---------------------
        # printConInfo(con)
        con.point=0
        # DEGREE
        con.deg = math.atan2(con.y, con.x)*180/math.pi
        if con.deg < -10 and con.x<4:
            continue
        if con.deg < -0 and con.x<3.5:
            continue
        if con.deg < 10 and con.x<3:
            continue
        if -20.0 < con.deg and con.deg < 60:# and 2.0 < con.x:
            # print("[POINT] deg")
            con.point +=100
            # print("DEG = "+str(con.deg))
        
        # DIST
        distpoint = con.y*120
        con.dist = getDist(con.x, con.y, 0, 0)
        distpoint = distpoint - con.dist*100
        # print("[POINT] dist = "+str(distpoint))
        con.point += distpoint

        # PART
        # print("     CDCDCDCD")

        for cd in cons:
            dist = getDist(con.x, con.y, cd.x, cd.y)
            if cd.x < 0.0:
                continue
            if dist < 0.3 or 4.0 < dist:
                continue
            cd.deg = math.atan2(cd.y, cd.x)*180/math.pi
            # printConInfo(cd)
            # print("DIST = "+str(dist))

            path_deg = math.atan2(cd.y-con.y, cd.x-con.x)*180/math.pi
            # print("PATH DEG  = "+str(path_deg))
            # print("[POINT] PART = ")

            if path_deg > 30:
                continue
            
            con.point += 150
        # print("--------------------------------------")
        if con.point > max_point:
            l_con = con
            max_point = con.point
        # print("\n")

    return l_con

def getLcon2(cons, rcon):
    lcon=cl.Con()
    min_dist = 100
    for con in cons:
        r_dist = getDist(con.x, con.y, rcon.x, rcon.y)  # between lr cons
        dist = getDist(con.x, con.y, 0,0)               # between car, lcon
        if r_dist>3.5 or r_dist < 0.5:
            continue
        if ccw(0,0,  rcon.x,rcon.y,  con.x,con.y) < -1:
            continue

        if con.y <-1.0:
            continue
        if dist < 0.5:
            continue


        if dist < min_dist:
            min_dist=dist
            lcon = con
    if lcon.x==0 and lcon.y==0:
        lcon.x = rcon.x
        lcon.y = -rcon.y

    return lcon
def getRcon(cons,  lane_x, lane_y):
    r_con = cl.Con()

    max_point = -10000
    for cd in cons:
        lane_dist = 0
        lane_dist_count = 0
        for idx, lane in enumerate(lane_x):
            if lane_x[idx]==0:
                continue
            else:
                added_dist = getDist(lane_x[idx],  lane_y[idx], cd.x, cd.y)
                if added_dist < cd.dist_lane_min:
                    cd.dist_lane_min = added_dist
                lane_dist += added_dist
                lane_dist_count +=1

        if lane_dist_count!=0:
            cd.dist_lane = lane_dist/lane_dist_count

        # print("COORD {} / {}    laneDist = {}  {} / {}".format(cd.x, cd.y, cd.dist_lane, lane_dist, lane_dist_count))
        cd.cdpoint=0
        # dist = getDist(lcon.x, lcon.y, cd.x, cd.y)
        if cd.x < -0.5:
            continue
        if cd.y < -2 or cd.y > 0.5:
            continue
        # if dist < 0.3 or 6.0 < dist:
        #     continue
        cd.deg = math.atan2(cd.y, cd.x)*180/math.pi

        # if lcon.deg -15 < cd.deg:
        #     continue
        # path_deg = math.atan2(cd.y-lcon.y, cd.x-lcon.x)*180/math.pi
        # if path_deg > 30:
        #     continue

        
        # cd.cdpoint -= path_deg*10
        # cd.cdpoint += path_deg*10

        if cd.cdpoint > max_point:
            max_point = cd.cdpoint
            r_con = cd

    min_dist = 1000
    print("LANE Num {}".format(len(lane_x)))
    if len(lane_x)>3:
        for cd in cons:
            if cd.dist_lane < min_dist:
                if cd.dist_lane < 3 and cd.x > -0.5 and cd.dist_lane_min < 2:
                    min_dist=cd.dist_lane
                    r_con = cd
    return r_con
    
def getVisionLane(lanes):
    lane_x=[]
    lane_y=[]
    for lane in lanes:
        x = lane.position.x/95
        y = (-lane.position.y+160)/120
        nx = x*math.cos(tilt) - y*math.sin(tilt)+0.6
        ny = x*math.sin(tilt) + y*math.cos(tilt)-0.7
        # print("     LANE NUM {} {}".format(nx, ny))
        lane_x.append(nx)
        lane_y.append(ny)
    return lane_x, lane_y

if __name__ == '__main__':

    rospy.init_node("pppy")
    # imu_sub = rospy.Subscriber("/imu/data", Imu, imu_callback)
    con_sub = rospy.Subscriber("/lane/cons", PoseArray, cons_callback)
    steer_sub = rospy.Subscriber("/erp42/steer_r", Float32, steer_callback)
    steer_sub = rospy.Subscriber("/erp42/steer_r", Float32, steer_callback)
    lane_sub = rospy.Subscriber("/hg_lane/lane_poses", PoseArray, lane_callback)

    steer_pub = rospy.Publisher("/control/angle", Float32)
    speed_pub = rospy.Publisher("/control/accel", UInt16)
    print("ON")
    rate = rospy.Rate(100)
    lock = threading.Lock() 


    left_con=cl.Con()
    right_con=cl.Con()

    tilt = -42*math.pi/180
    while not rospy.is_shutdown():
        now_cons = global_cons

        plt.cla()
        plt.xlim([-3,8])
        plt.ylim([-5,5])
        # print("LANE NUM {} ".format(len(lanes)))
        lane_x, lane_y=getVisionLane(lanes)
        # for lane in lanes:
        #     x = lane.position.x/95
        #     y = (-lane.position.y+160)/120

        #     nx = x*math.cos(tilt) - y*math.sin(tilt)+0.6
        #     ny = x*math.sin(tilt) + y*math.cos(tilt)-0.7
        #     # print("     LANE NUM {} {}".format(nx, ny))
        #     lane_x.append(nx)
        #     lane_y.append(ny)

        right_con = getRcon(now_cons, lane_x, lane_y)
        left_con = getLcon2(now_cons, right_con)
        
        plt.scatter(lane_x  , lane_y  ,c='green',s=200)
        size = len(now_cons)
        # print("SIZE - "+str(size))
        plt.plot([now_cons[idx].x for idx in range(size)],[now_cons[idx].y for idx in range(size)], "b*")
        # left_con = getLcon(now_cons, left_con)
        # ----------- NEARPOINT PRINT ----------------------

        # if len(cocons) != 0:
        #     plt.scatter([item.x for item in cocons ],[item.y for item in cocons], c='black',s=500)

        # print("LEFT "+str(left_con.x)+"  / "+str(left_con.y))

        left_b=False
        right_b=False


        if left_con.x !=0:
            plt.scatter(left_con.x,left_con.y , c='red',s=200)
            left_b=True
        if right_con.x !=0:
            plt.scatter(right_con.x  ,right_con.y  ,c='black',s=200)
            right_b=True



        target=cl.Con()
        
        if left_b and right_b:
            target.x = 0.5*(right_con.x+left_con.x)
            target.y = 0.5*(right_con.y+left_con.y)
            
            dist = getDist(right_con.x, right_con.y, left_con.x, left_con.y)
            dx = left_con.x-right_con.x
            dy = left_con.y-right_con.y

            target.x = right_con.x + 1.3*dx/dist
            target.y = right_con.y + 1.3*dy/dist

        elif left_b:
            target.x = (left_con.x)
            target.y = 0.5*(left_con.y)

        else:
            target.x =3
            target.y =0
        
        target_deg = -math.atan2( target.y, target.x)*180/math.pi
        path_deg = math.atan2(right_con.y-left_con.y, right_con.x-left_con.x)*180/math.pi

        plt.scatter(target.x  ,target.y  ,c='yellow',s=200)

        plt.scatter(0,0 ,c='black',s=400)
        if target_deg > 90:
            target_deg -=180
        if target_deg < -90:
            target_deg +=180
        print("------------------- TTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT "+str(Float32(target_deg)))
        steer_pub.publish(target_deg)
        speed_pub.publish(5)
        # plt.scatter([odomCon[idx].x for idx in range(sizeo) ],[odomCon[idx].y for idx in range(sizeo)], c='yellow',s=50)
        plt.pause(0.001)
        
        os.system('clear')
        rate.sleep()


