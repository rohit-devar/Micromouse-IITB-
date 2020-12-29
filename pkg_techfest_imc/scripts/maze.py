#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
from tf.transformations import euler_from_quaternion
import time
side=0
# Global Variables
sensor_l, sensor_c, sensor_r = 0, 0, 0
sensor_goal =0
goal_1 =0
pose=[1.35, 1.23,0]
pub=None
count=1
count_=0
max_vel =.4

l= []
reverse_list = []            # reverse path from goal to start

s = 0

def clbk_laser(msg):
    global sensor_l, sensor_c, sensor_r,sensor_goal,goal_1
    regions=[0,0,0,0,0]
    #print(side)
    # 360/ 3 = 120
    ons = [
        round(100*min(min(msg.ranges[0:210]), 100))                    # to detect Goal  for righ is wall
        ]
    
    on = [
        round(100*min(min(msg.ranges[180:360]), 100))                    # to detect Goal  for left is wall
        ]

    regions = [
        round(100*min(min(msg.ranges[0:10]), 100),3),                # first 0 to 30
        round(100*min(min(msg.ranges[170:210]), 100),3),
        round(100*min(min(msg.ranges[350:360]), 100),3)              # first 330 to 360 
        
    ]

    sensor_l = regions[2]
    sensor_c = regions[1]
    sensor_r = regions[0]
    sensor_goal = ons[0]
    goal_1 = on[0]






def odom_callback(data):
    global pose
    #print("pose call back")
    x  = data.pose.pose.orientation.x
    y  = data.pose.pose.orientation.y
    z = data.pose.pose.orientation.z
    w = data.pose.pose.orientation.w
    pose = [data.pose.pose.position.x, data.pose.pose.position.y, euler_from_quaternion([x,y,z,w])[2]]
    
    #print(pose)
    
def get_angle():
    global pose
    angle=pose[2] * 57.29
    if angle < 0:
        angle = angle + 360
    
    return angle


def follow():                                 #normal ge 0 u turn ge 1
    global pub,side,max_vel
    v=max_vel
    m=12
    #v=.2
    w=0
    e=0
    msg = Twist()
    p= 0.08   # 0.06
    
    if sensor_l < m and sensor_r < m and side < 3 :
        msg.linear.x = v
        msg.angular.z=  0.03
        pub.publish(msg)
        side=side+1
        #print("both snsor madidhu adra straigh with side< 1")
       
        

    if sensor_l < m and sensor_r < m and side > 2 :
        e=sensor_l - sensor_r
        
        w=e * 0.08
        msg.linear.x = v
        msg.angular.z=  w
        pub.publish(msg)
        #print("both snsor madidhu adra following right wiht anglular vel", w)
       

    if sensor_l > m  and sensor_r < m:
        e=8.00 - sensor_r
        
        w=e * p
        msg.linear.x = v
        msg.angular.z=  w
        pub.publish(msg)
        #print("right snsor madidhu :",w)
    if sensor_l < m and sensor_r > m:
        e=sensor_l - 8.00
        
        w= e  * p
        msg.linear.x = v
        msg.angular.z=  w
        pub.publish(msg)
        #print("left snsor madidhu :", w )
    
    if sensor_l > m and sensor_r > m:
        #print("nan both greater than 14 snsor madidhu ")
        msg.linear.x = v
        msg.angular.z=  0
        pub.publish(msg)

def recover350_to_zero():
    global pub
    msg = Twist()



    if get_angle() > 270 :
        print("greater that 345")
        while 1 :
            if abs(get_angle() - 359) < 3 :
                stop()
                break
            if get_angle() > 0 and get_angle() < 90:
                break
            print("greater that 345")
            msg.linear.x = 0
            msg.angular.z= 1
            pub.publish(msg)
            #print("recovered")
    else:
        pass
            
def recover0_to_359():
    global pub
    msg = Twist()
    
    if get_angle() > 0 and get_angle() < 90:

        while 1 :
            if  abs (get_angle() - 0 ) < 3 :
                stop()
                break
            if get_angle() > 270:
                break
            msg.linear.x = 0
            msg.angular.z= -1
            pub.publish(msg)
            print("recovered to to 360")

    else:
        pass
    
def recovery(angle):
    global pub
    msg = Twist()
    while 1:


        a=angle - get_angle()
        if abs(a) < 3 :
            break

        else:
            #print("recovery sste")
            if a > 0:
                b= 1
            else:
                b= -1
            msg.linear.x = 0
            msg.angular.z = b * 1.5
            pub.publish(msg)
    #print("recovery over")


def special_recovery():
    global pub
    msg = Twist()
    if get_angle() > 0  and get_angle() < 90 :
        while 1:

            a = 0 - get_angle()
            if abs(a) < 1.5 or get_angle() > 270 :
                break

            else:
                print("recovery sste in zero ")
                if a > 0:
                    b = 1
                else:
                    b = -1
                msg.linear.x = 0
                msg.angular.z = b * .5
                pub.publish(msg)
        print("recovery over   in zero ")

    else:
        #recovery(359)
        while 1:

            a = 359 - get_angle()
            if abs(a) < 1.5 or  get_angle() < 90 :
                break

            else:
                print("recovery sste  in 359")
                if a > 0:
                    b = 1
                else:
                    b = -1
                msg.linear.x = 0
                msg.angular.z = b * .5
                pub.publish(msg)
        print("recovery over in 359 ")






def ninty(l):         # not using using during lef or right
    global pub,s,count
    angular_vel= .4
    print("angle slow")
    if count == 1:
        count = 0
        if l == 1:
            recover350_to_zero()
        if l == 0 :
            recover0_to_359()
        
    else:
        count = 1
        #print("count 1")
    
    msg = Twist()
    start=abs(get_angle())
    if l == 1:
        w=angular_vel
    else:
        w= -angular_vel
    #print("start angle",start)
    while ( abs (   abs(get_angle()) - start  )) <=  90 :
        #print("taking turn:",abs (   abs(get_angle()) - start  ))
        #print(start,get_angle())
        #print("end angle",abs(get_angle()))
        #print("inn")
        msg.linear.x = 0.0
        msg.angular.z=  w
        pub.publish(msg)
    #print(" ninty turn taken")
    msg.linear.x = 0
    msg.angular.z=  0
    pub.publish(msg)                           # # #n


def turn(degree,direction):
    global pub
    msg = Twist()
    while 1:
        if get_angle() == degree:
            break
        msg.linear.x = 0
        msg.angular.z=  direction * 0.4
        pub.publish(msg)


def ninty_test_2(l):  # l= 1  left  0 for right

    global pub, s, count, count_
    msg = Twist()
    w = 4
    if count == 1:

        if get_angle() > 270 or get_angle() < 90:
            count_ = 0

        if get_angle() < 270 and get_angle() > 90:
            count_ = 1


    else:

        if get_angle() > 0 and get_angle() < 180:
            count_ = 0

        if get_angle() > 180:
            count_ = 1

    if count == 1 and count_ == 1:  # robot is in south dirn # IN 180
        count = 0
        if l == 1:  # left shold go
            # turn(270,1)
            while 1:
                if get_angle() > 250:         # > 270
                    break
                msg.linear.x = 0
                msg.angular.z = 1 * w
                pub.publish(msg)

            recovery(270)

        else:
            # turn(90,-1)                        # right
            while 1:
                if get_angle() < 110:        # 91
                    break
                msg.linear.x = 0
                msg.angular.z = -1 * w
                pub.publish(msg)

            recovery(90)

    elif count == 1 and count_ == 0:                           # robot is in north dirn   # in zero degree
        count = 0
        print("robot is in north dirn")
        if l == 1:  # left shod go
            # turn(90,1)

            while 1:
                if get_angle() > 70 and get_angle() < 180:          # BEFORE > 90 AND < 180
                    break
                msg.linear.x = 0
                msg.angular.z = 1 * w
                pub.publish(msg)

            recovery(90)
        else:
            # turn(270-1)
            # right
            while 1:
                if get_angle() > 180 and get_angle() < 292:           # >180 AND < 292
                    break
                msg.linear.x = 0
                msg.angular.z = -1 * w
                pub.publish(msg)
            recovery(270)


    elif count == 0 and count_ == 1:  # robot is in esst  dirn
        count = 1
        if l == 1:  # left shod go
            # turn(0,1)
            while 1:
                if get_angle()  > 0 and get_angle() < 90 :             #> 0 and get_angle() < 90:
                    break
                msg.linear.x = 0
                msg.angular.z = 1 * 1.9
                pub.publish(msg)
            #special_recovery()


        else:
            # turn(180,-1)                        # right
            while 1:
                if get_angle() < 200:          # 180
                    break
                msg.linear.x = 0
                msg.angular.z = -1 * w
                pub.publish(msg)

            recovery(180)

    elif count == 0 and count_ == 0:  # robot is in west dirn
        count = 1
        if l == 1:  # left shod go
            # turn(180,1)
            while 1:
                if get_angle() > 160:         # 180
                    break
                msg.linear.x = 0
                msg.angular.z = 1 * w
                pub.publish(msg)

            recovery(180)

        else:
            # turn(0,-1)                        # right
            while 1:
                if get_angle() > 300  :            # 300
                    break
                msg.linear.x = 0
                msg.angular.z = -1 * 1.9
                pub.publish(msg)


        pass

    msg.linear.x = 0
    msg.angular.z = 0
    pub.publish(msg)


def ninty_test(l):                     # l= -1 for right  0 for left
    
    global pub,s,count,count_
    msg=Twist()
    w = 2
    if count == 1:
        
        
        
        if get_angle() > 270 or get_angle() < 90:
            
            count_= 0

        if get_angle() < 270 and  get_angle() > 90:
            count_= 1

        
    else:

       
        if get_angle() > 0 and get_angle() <180:
            count_ = 0

        if get_angle() > 180 :
            count_ = 1

    



    if count == 1 and count_ == 1 :          # robot is in south dirn # IN 180
        count =0
        if l == 1:                    # left shold go
            #turn(270,1)
            while 1:
                if get_angle() > 270  :
                    break
                msg.linear.x = 0
                msg.angular.z=  1 * w
                pub.publish(msg)


            
        else: 
            #turn(90,-1)                        # right 
            while 1:
                if get_angle() < 91 :
                    break
                msg.linear.x = 0
                msg.angular.z=  -1 * w
                pub.publish(msg)
            
    
    elif count == 1 and count_ == 0 :          # robot is in north dirn   # in zero degree
        count =0
        print("robot is in north dirn")
        if l == 1:                    # left shod go
            #turn(90,1)

            while 1:
                if get_angle() > 90 and get_angle() < 180 :
                    break
                msg.linear.x = 0
                msg.angular.z=  1 * w
                pub.publish(msg)

            
        else: 
            #turn(270-1)  
                                 # right
            while 1:
                if get_angle() > 180 and get_angle() < 272 :
                    break
                msg.linear.x = 0
                msg.angular.z=  -1 * w
                pub.publish(msg)
            


    elif count == 0 and count_ == 1 :          # robot is in esst  dirn
        count =1
        if l == 1:                    # left shod go
            #turn(0,1)
            while 1:
                if get_angle() > 0 and get_angle() < 90 :
                    break
                msg.linear.x = 0
                msg.angular.z=  1 * w
                pub.publish(msg)
 

            
        else: 
            #turn(180,-1)                        # right
            while 1:
                if get_angle() < 180  :
                    break
                msg.linear.x = 0
                msg.angular.z=  -1 * w
                pub.publish(msg)


    elif count == 0 and count_ == 0 :          # robot is in west dirn
        count=1
        if l == 1:                             # left shod go
            #turn(180,1)
            while 1:
                if get_angle() > 180 :
                    break
                msg.linear.x = 0
                msg.angular.z=  1 * w
                pub.publish(msg)

            
        else: 
            #turn(0,-1)                        # right
             while 1:
                if get_angle() > 300 :
                    break
                msg.linear.x = 0
                msg.angular.z=  -1 * w
                pub.publish(msg)


    else:
        pass

    msg.linear.x = 0
    msg.angular.z=  0
    pub.publish(msg)


def move_half_forward(l):
    #print("count")
    global pub
    msg = Twist()
    #print("moving half forward")
    
    if sensor_l > 14 and sensor_r > 14 :
        ove_half_forward(l)
        

    else :
        if count == 1 :
            #print("robot go in y direction ")
            start= (pose[1])
        
            while  (  math.sqrt(    math.pow(  (pose[1] - start)   ,2  )     )    )   < l:
                if sensor_c < 6.0 and sensor_c > 2.5:
                    break
                #print("difference:",abs ( abs(pose[1])  - start))
                #print("ff")
                follow()
            #print(".08 completed")

        else :
            #print("robot go in x direction ")
            start= (pose[0])
         
            while  (  math.sqrt(    math.pow(  (pose[0] - start)   ,2  )     )    )  < l:
                #print(abs(abs(pose[0]) - start))
                if sensor_c < 6.0 and sensor_c > 2.5 :
                    break
                follow()
            
            

    
    msg.linear.x = 0
    msg.angular.z=  0
    pub.publish(msg) 

def ove_half_forward(l):
    #print("count")
    global pub,max_vel
    msg = Twist()
    v=max_vel
    #print("moving half forward")
    a=get_angle()
    
    if count == 1 :
        #print("robot go in y direction ")
        start= (pose[1])
        
        while  (  math.sqrt(    math.pow(  (pose[1] - start)   ,2  )     )    )  < l:
            #print("difference  y :",abs ( abs(pose[1])  - start))
            if sensor_c < 6.0 and sensor_c > 4.0:
                    break
            msg.linear.x = v
            msg.angular.z=  0
            pub.publish(msg)
    
    if count == 0 :
        #print("robot go in x direction ")
        start= (pose[0])
        #print(start)
        while  (  math.sqrt(    math.pow(  (pose[0] - start)   ,2  )     )    )   < l:
            #print("difference  x,",abs(abs(pose[0]) - start))
            if sensor_c < 6.0 and sensor_c > 4.0:
                break
            msg.linear.x = v
            msg.angular.z=  0
            pub.publish(msg)
           
    msg.linear.x = 0
    msg.angular.z=  0
    pub.publish(msg) 


def stop():
    global pub
    
    msg = Twist()
    
    msg.linear.x = 0
    msg.angular.z=0
    pub.publish(msg)

def left():
    global side
    print("gnggg left ")
    ninty_test_2(1)
    side=0
    move_half_forward(.08)
    side=0
    move_half_forward(.1)
    
    #print("gon left ")
    stop()


def right():
    global side
    print("gng right")
    ninty_test_2(0)
    side=0
    move_half_forward(.08)
    side=0
    move_half_forward(.1)
    
    #print("gon tight")
    stop()

def straight():
    global side
    print("gngg straight")
    side=0
    move_half_forward(.08)            # .1


    side=0
    move_half_forward(.1)             #.08

    side = 0
    print("completed")
    stop()

def U_turn():
    global side
    print("taking U turn ")
    ninty_test_2(1)
    ninty_test_2(1)
    side = 2  
    move_half_forward(.08)
    side = 0
    move_half_forward(.1)
    stop()
    
def maze_left_hand_rule():
    
    m= 14
    #if sensor_l_g < 12 and ( sensor_c_g > 20 and sensor_c_g < 33 ) and  ( sensor_rl_g < 33 and  sensor_rl_g > 20 )  and ( sensor_rc_g < 33 and sensor_rc_g > 20 ) and (sensor_rm_g < 33 and sensor_rm_g ) > 20 :
        #print("goal reached")
        #return 
    
    
    
    if sensor_l > m and sensor_c < m and sensor_r < m :                               #only left
        left()
        l.append('L')
        print("=================Its     left shape       so ROBOT should go LEFT =======================")
        
        
    elif sensor_l < m and sensor_c < m and sensor_r > m:                             #only right
        right()
        l.append('R')
        print("=================Its     Right shape      so ROBOT should go RIGHT =======================")
     
    elif sensor_l > m and sensor_c < m and sensor_r > m:                              # T shape
        left()
        l.append('L')
        print("=================Its      T shape         so ROBOT should go LEFT =========================")
    
    elif sensor_l > m and sensor_c > m and sensor_r < m:                              # --|  shape
        left()
        l.append('L')
        print("=================Its    --| shape         so ROBOT should go LEFT ===========================")
     
    elif sensor_l < m and sensor_c > m and sensor_r > m:                              # |--   shape
        straight()
        l.append('S')
        print("=================Its     |-- shape          so ROBOT should go STRAIGHT ====================")
    
    elif sensor_l > m and sensor_c > m and sensor_r > m:                              # + shape
        left()
        l.append('L')
        print("=================Its     + shape          so ROBOT should go LEFT ===========================")
    
    elif sensor_l < m and sensor_c < m and sensor_r < m and sensor_goal < 10  :                             #  U all sids wall  shape
        U_turn()
        l.append('U')
        print("=================Its     U shape          so ROBOT should go 180 ============================")
     
    elif sensor_l < m and sensor_c > m and sensor_r < m:                               #  | |  shape
        straight()
        l.append('S')
        print("=================Its      hape      | |    so  ROBOT should go  STRAIGHT ===========================")
    
    
    #elif  sensor_goal  < 30 and sensor_goal > 18:
        
        
        #print("      GOAL      ########################         LAZER   *INSIDE MAZE DETECTED          ++++==========================       66666666666666      ####")
    
    else:
        print("strange ting ")
    
def maze_right_hand_rule():
    
    m= 14
    #if sensor_l_g < 12 and ( sensor_c_g > 20 and sensor_c_g < 33 ) and  ( sensor_rl_g < 33 and  sensor_rl_g > 20 )  and ( sensor_rc_g < 33 and sensor_rc_g > 20 ) and (sensor_rm_g < 33 and sensor_rm_g ) > 20 :
        #print("goal reached")
        #return 
    
    
    
    if sensor_l > m and sensor_c < m and sensor_r < m :                               #only left
        left()
        print("=================Its     left shape       so ROBOT should go LEFT =======================")
        
        
    elif sensor_l < m and sensor_c < m and sensor_r > m:                             #only right
        right()
        print("=================Its     Right shape      so ROBOT should go RIGHT =======================")
     
    elif sensor_l > m and sensor_c < m and sensor_r > m:                              # T shape
        #left()

        right()

        print("=================Its      T shape         so ROBOT should go LEFT =========================")
    
    elif sensor_l > m and sensor_c > m and sensor_r < m:                              # --|  shape
        #left()

        straight()
        print("=================Its    --| shape         so ROBOT should go LEFT ===========================")
     
    elif sensor_l < m and sensor_c > m and sensor_r > m:                              # |--   shape
        #straight()

        right()
        print("=================Its     |-- shape          so ROBOT should go STRAIGHT ====================")
    
    elif sensor_l > m and sensor_c > m and sensor_r > m:                              # + shape
        #left()

        right()
        print("=================Its     + shape          so ROBOT should go LEFT ===========================")
    
    elif sensor_l < m and sensor_c < m and sensor_r < m and sensor_goal < 10  :                             #  U all sids wall  shape
        U_turn()
        print("=================Its     U shape          so ROBOT should go 180 ============================")
     
    elif sensor_l < m and sensor_c > m and sensor_r < m:                               #  | |  shape
        straight()
        print("=================Its     in straight Tunnel           so ROBOT should go STRAIGHT ===========================")
    
    
    #elif  sensor_goal  < 30 and sensor_goal > 18:
        
        
        #print("      GOAL      ########################         LAZER   *INSIDE MAZE DETECTED          ++++==========================       66666666666666      ####")
    
    else:
        print("strange ting ")
    
def testing():
    
    ninty_test_2(1)
    move_half_forward(.1)
    side=0
    move_half_forward(.08)
    side=0

    ninty_test_2(0)
    #move_half_forward(.17)
    move_half_forward(.1)
    side=0
    move_half_forward(.08)
    side=0

    move_half_forward(.1)
    side=0
    move_half_forward(.08)
    ninty(1)
    side=0
    

    move_half_forward(.1)
    side=0
    move_half_forward(.08)
    side=0

    move_half_forward(.1)
    side=0
    move_half_forward(.08)
    side=0

    ninty_test_2(0)
    move_half_forward(.1)
    side=0
    move_half_forward(.08)
    side=0

    #ninty(1)
    #move_half_forward(.17)
    stop()

def test_angles():

    ninty_test(0)
    print("over")
    ninty_test(0)
    print("over")
    ninty_test(0)
    print("over")
    ninty_test(0)
    print("over")
    ninty_test(1)
    print("over1")
    ninty_test(1)
    print("over1")
    ninty_test(1)
    print("over1")
    ninty_test(1)
    print("over1")

def test_angles_2():
    ninty_test_2(0)
    time.sleep(1)
    print("over")
    ninty_test_2(0)
    time.sleep(1)
    print("over")
    ninty_test_2(0)
    print("over")
    time.sleep(1)
    ninty_test_2(0)
    time.sleep(1)
    print("over")
    ninty_test_2(1)
    time.sleep(1)
    print("over1")
    ninty_test_2(1)
    time.sleep(1)
    print("over1")
    ninty_test_2(1)
    time.sleep(1)
    print("over1")
    ninty_test_2(1)
    time.sleep(1)
    print("over1")

def path_planner():
    global l
    count=0
    while 1 :
        if count == len(l) :
            break
        i=count
        if l[i] == 'U':




            if l[i-1] == 'L':

                if l[i+1] == 'R':             #LUR
                    l[i-1] = 'U'
                    l.pop(i+1)
                    l.pop(i)
                    count = i - 1


                elif l[i+1] == 'S':           #LUS
                    l[i-1] = 'R'
                    l.pop(i+1)
                    l.pop(i)
                    count = count

                elif  l[i+1] == 'L':          # LUL
                    l[i-1] = 'S'
                    l.pop(i+1)
                    l.pop(i)
                    count = count


            elif l[i-1] == 'S':

                if l[i+1] == 'L':             #SUL
                    l[i-1] = 'R'
                    l.pop(i+1)
                    l.pop(i)
                    count = count


                elif l[i+1] == 'S':           #SUS
                    l[i-1] = 'U'
                    l.pop(i+1)
                    l.pop(i)
                    count = i-1

            elif l[i - 1] == 'R':

                if l[i + 1] == 'L':            # RUL
                    l[i - 1] = 'U'
                    l.pop(i + 1)
                    l.pop(i)
                    count = i - 1

        else :
            count=count+1
    print(l)

def reverse_path():

    global reverse_list 
    reverse_list= l

    for i in range(len(reverse_list)):
        if reverse_list[i] == 'L':
            reverse_list[i] = 'R'
        elif reverse_list[i] == 'R':
            reverse_list[i] = 'L'
    reverse_list.reverse()
    print(reverse_list)

def navagation():
    global count
    print("count ",count)
    global l
    print(l)
    U_turn()
    for i in l:

        if i == 'L':
            left()
            print("navagating left")
        elif i == 'R' :
            right()
        elif i == 'S':
            straight()
    stop()

def reverse_navigation():
    global reverse_list
    
    U_turn()
    
    for i in reverse_list:

        if i == 'L':
            left()
        elif i == 'R' :
            right()
        elif i == 'S':
            straight()
    stop()       
        
def main():
    
    global sensor_l, sensor_c, sensor_r,sensor_goal ,sensor_goal_
    global pub

    msg = Twist()

    rospy.init_node('node_maze_runner')
    rospy.Subscriber('/odom', Odometry, odom_callback)
    rospy.Subscriber('/my_mm_robot/laser/scan', LaserScan, clbk_laser)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    # pub.publish(msg)
    
    # rospy.spin()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():

        #test_angles_2()
        move_half_forward(0.07)
        #test_angles_2()
        #test_angles_2()

        
        #time.sleep(1000)

        
        
        while 1:
            if (sensor_goal > 18 and  sensor_goal < 30) or  (goal_1 > 18 and  goal_1 < 30):        # first if left sensor is near object in goal and second if right one
                stop()
                print("---------Goal reached  -------------")
                break
            
            maze_left_hand_rule()

        path_planner()
        print("PATH PLANNINHHHHHHHGGGGG OVER")

        reverse_path()
        print("going back to goal")
        reverse_navigation()


        reverse_path()
        print("navigation beins GET ---- SET ------GOOOOOOO---")
        navagation()
        
        
        
        
        

            
        
        print("outside")
        time.sleep(1000)
        rate.sleep()

if __name__ == '__main__':
    
    
    main()
