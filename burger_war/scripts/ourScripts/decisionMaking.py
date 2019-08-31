#!/usr/bin/env python
# -*- coding: utf-8 -*-
# license removed for brevity
import rospy
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from std_msgs.msg import Int8
from nav_msgs.msg import Odometry
import math
import time
from std_msgs.msg import Int32MultiArray,MultiArrayLayout,MultiArrayDimension
from move_base_msgs.msg import MoveBaseActionResult
from geometry_msgs.msg import Twist
import is_field_out as fo #katano

status=10
enemy_x=0.0
enemy_y=0.0
enemy_th=0.0
pose_=[-1.3,0,0]
OLDpose_=[0,0,0]
Tx=0.0
Ty=0.0
th=0.0
my_position_num=0
my_position_num2=0
my_position_num_to=0
enemy_position_num=2 #my start:0 right:1 enemy start:2 left:3
position=[[-1.3,0,0],[0.0,-1.3,math.pi/2],[1.3,0.0,math.pi],[0.0,1.3,-1*math.pi/2],
[-0.72051407,-0.74551407,0.0],[0.72051407,-0.74551407,0.0],[0.72051407,0.74551407,0.0],[-0.72051407,0.74551407,0.0]]
#sniper_pos = [[-1.3,0,0],[0.0,-1.3,math.pi/2],[1.3,0.0,math.pi],[0.0,1.3,-math.pi/2]]
sniper_pos = [[-1.0,0.0,0.0],[0.0,-1.0,math.pi/2],[1.0,0.0,math.pi],[0.0,1.0,-math.pi/2]]
#way_pos = [[0.0,-1.0,-math.pi/2],[1.0,0.0,0.0],[0.0,1.0,math.pi/2],[-1.0,0,math.pi]]
##way_pos = [[-0.4,0.9,-math.pi * 3.0 / 4.0],[-0.9,-0.4,-math.pi / 4.0],[0.4,-0.9,math.pi / 4.0],[0.9,0.4,math.pi * 3.0 / 4.0]]
way_pos = [[-0.4,0.9,-math.pi * 3.0 / 4.0 -math.pi/12.0],[-0.9,-0.4,-math.pi / 4.0 -math.pi/12.0],[0.4,-0.9,math.pi / 4.0 -math.pi/12.0],[0.9,0.4,math.pi * 3.0 / 4.0 -math.pi/12.0]]
###way_pos = [[-0.72051407,0.74551407,-math.pi * 3.0 / 4.0],[-0.72051407,-0.74551407,-math.pi / 4.0],[0.72051407,-0.74551407,math.pi / 4.0],[0.72051407,0.74551407,math.pi * 3.0 / 4.0]]
my_point=0
enemy_point=0
goal_reached_flg = False    # ゴールに到達したかどうかのフラグ
back_direction=0.0

def movebaseStatusCallback(movebase_status):
    global status
    status=movebase_status.data

def enemyPoseCallback(pose):
    global enemy_x
    global enemy_y
    global enemy_th
    global status
    global enemy_position_num
    enemy_x=pose.x
    enemy_y=pose.y
    enemy_th=pose.theta
    #print 'enemy_pose'
    #print pose
    #print status
    enemy_position_num=checkPosition(pose.x,pose.y)[0]
    print enemy_position_num

def pointsArrayPoseCallback(point):
    global enemy_point
    global my_point
    my_point=point.data[18]
    enemy_point=point.data[19]
    #print point.data

def odomCallback(my_pose_msg):
    global Tx
    global Ty
    global th
    global my_position_num
    global my_position_num2
    Ty=-1*my_pose_msg.pose.pose.position.x
    Tx=my_pose_msg.pose.pose.position.y
    th=2*math.acos(my_pose_msg.pose.pose.orientation.w)*(math.asin(my_pose_msg.pose.pose.orientation.z)/abs(math.asin(my_pose_msg.pose.pose.orientation.z))) - math.pi / 2.0
    #print 'my_pose'
    #print Tx
    #print Ty
    my_position_num=checkPosition(Tx,Ty)[0]
    my_position_num2=checkPosition(Tx,Ty)[1]
    #print my_position_num
    #print my_position_num2

def checkMVresultCallback(move_result_msg):
    global goal_reached_flg
    print "------------------move_result_msg.status.text--------------------------"
    print move_result_msg.status.text
    if move_result_msg.status.text == "Goal reached.":
        goal_reached_flg = True
    #else:
    #    goal_reached_flg = False

def InputPose(x,y,z):
    pose=Pose2D()
    pose.x=x
    pose.y=y
    pose.theta=z
    #print 'move pose:'
    #print pose
    point_pub.publish(pose)
    time.sleep(0.5)

def checkPosition(x,y):
    #print "x,y"
    #print x
    #print y
    position_num=0
    position_num2=0
    if x+y > 0 and x-y > 0: #C opposite
        position_num=2
    elif x+y < 0 and x-y > 0: #B right
        position_num=1
    elif x+y > 0 and x-y < 0: #D left
        position_num=3
    elif x+y < 0 and x-y < 0: #A start
        position_num=0

    if x-(0.455/0.63)*y < 0 and x-(0.605/0.43)*y > 0:
        position_num2=4 #E
    elif x-(0.455/0.63)*y > 0 and x-(0.605/0.43)*y < 0:
        position_num2=6 #G
    elif x+(0.455/0.63)*y < 0 and x+(0.605/0.43)*y > 0:
        position_num2=7 #H
    elif x+(0.455/0.63)*y > 0 and x+(0.605/0.43)*y < 0:
        position_num2=5 #F
    else:
        position_num2=0
    #print position_num
    #print position_num2
    return [position_num,position_num2]

def obstacleDirectionCallback(direction_msg):
    global back_direction
    print "direction_msg"
    print direction_msg.data
    back_direction=direction_msg.data

def recoveryBehavior():
    twist=Twist()
    global back_direction
    global th
    print "back_direction"
    print back_direction
    print "th"
    print th
    turn_speed=1.35
    th_tmp=th
    back_direction_tmp=back_direction
    while abs(th_tmp-th)<abs(back_direction_tmp):
        twist.angular.z=turn_speed*back_direction_tmp/abs(back_direction_tmp)
        cmd_vel_pub.publish(twist)
        #print th
        r.sleep()
    '''
        if back_direction<-math.pi/10:
            twist.angular.z=-0.1
        elif back_direction>math.pi/10:
            twist.angular.z=0.1
        elif -math.pi/10<=back_direction<=math.pi/10:
            twist.angular.z=0.0
        #print twist
    '''
    twist.angular.z=0.0
    cmd_vel_pub.publish(twist)
    r.sleep()
    twist.linear.x=0.2
    cmd_vel_pub.publish(twist)
    Tx_old=Tx
    Ty_old=Ty
    while 0.2>math.sqrt((Tx-Tx_old)*(Tx-Tx_old)+(Ty-Ty_old)*(Ty-Ty_old)):
        r.sleep()
    twist.linear.x=0.0
    cmd_vel_pub.publish(twist)
    #print th_tmp
    #print th
    #print th_tmp-th
    #print back_direction_tmp
    #print "ssssssssssssssssssstoppppppppppppppppppppppp"

# 敵との距離を計算
def getEnemyDistance(my_pos, enemy_pos):
    distance = math.sqrt(math.pow(enemy_pos[0] - my_pos[0],2) + math.pow(enemy_pos[1] - my_pos[1],2))
    return distance

# その場で首振り
def swingBurger(my_pos, pivot_pos, pos_pub, swing_ang, swing_timer, swing_time, swing_num):
    next_ang = 0                # 次に向く角度
    #ang_error = math.pi / 36.0

    if int(swing_timer / (swing_time / swing_num)) % 2 == 0:
        next_ang = pivot_pos[2] + swing_ang
    else:
        next_ang = pivot_pos[2] - swing_ang
    InputPose(pivot_pos[0], pivot_pos[1], next_ang)


# 敵を見続ける速度
def lookaEnemytTurnVel(my_pos, enemy_pos):
    kp = 5.0
    if enemy_pos[0] - my_pos[0] != 0:
        theta_diff = math.atan( (enemy_pos[1] - my_pos[1]) / (enemy_pos[0] - my_pos[0]) ) - my_pos[2]
        turn_vel = kp * theta_diff

    # 自機-敵機 直線と自機の姿勢θとの差分をゼロにすることで的に顔を向ける
    return turn_vel

# 敵のいる角度
def lookatEnemyAng(my_pos, enemy_pos):
    if enemy_pos[0] - my_pos[0] != 0:
        #enemy_ang = math.atan( (enemy_pos[1] - my_pos[1]) / (enemy_pos[0] - my_pos[0]) )
        enemy_ang = math.atan2( enemy_pos[1] - my_pos[1], enemy_pos[0] - my_pos[0] )

    return enemy_ang

# 敵が居るかチェック
def checkEnemy(enemy_pos):
    if enemy_pos[0] == 0.0 and enemy_pos[1] == 0.0 and enemy_pos[2] == 0.0:
        return False
    else:
        return True

def goBack():
    global Tx, Ty, th
    L=0.15
    while fo.isFieldOut(Tx-L*math.cos(th), Ty-L*math.sin(th), 0.12) :
        #print "L"
        #print L
        x=Tx-L*math.cos(th)
        y=Ty-L*math.sin(th)
        #print x
        #print y
        InputPose(x,y,0.0)
        L=L-0.01

if __name__ == '__main__':
    rospy.init_node('decision_making', anonymous=True)
    point_pub = rospy.Publisher('my_pose', Pose2D, queue_size=10)
    movebase_status_sub = rospy.Subscriber('move_base_state', Int8, movebaseStatusCallback)
    enemy_pose_sub = rospy.Subscriber('enemy_pose', Pose2D, enemyPoseCallback)
    points_array_sub = rospy.Subscriber('points_array', Int32MultiArray, pointsArrayPoseCallback)
    odom_sub = rospy.Subscriber('odom', Odometry, odomCallback)
    check_move_result_sub = rospy.Subscriber('move_base/result', MoveBaseActionResult, checkMVresultCallback)
    cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    obstacle_direction_sub = rospy.Subscriber('obstacle_direction', Float32, obstacleDirectionCallback)

    
    #r = rospy.Rate(10) # 10hz
    Time=0
    OLDTime=time.time()
    #global enemy_x
    #global enemy_y
    global my_position_num_to
    global my_position_num
    global enemy_position_num
    global enemy_point
    global my_point
    #get field points
    #whether get points or not
    global Tx, Ty, th
    global enemy_x, enemy_y, enemy_th
    global position
    global sniper_pos
    global way_pos
    global status

    global goal_reached_flg

    operation_sequence = "Initial Move"
    #operation_sequence = "Okazu Sniper"
    start_time = time.time()
    swing_time = 30.0
    enemy_distance_threshold = 2.0  #0.6      # 敵からの距離のしきい値
    swing_ang = math.pi/4.0             # 首振り角度　±θ
    swing_num = 4
    zone_num = 4    # フィールドを4つに分けてます　スタート位置が0で左回り
    my_zone_no = 0
    next_zone_no = 0
    initial_motion_flg = False  # 最初の首振り動作が終わっているかのフラグ
    onway_flg = False   # ウェイポイントに向かっているかのフラグ
    topoint_flg = False # 狙撃ポイントへ向かっているかのフラグ
    enemy_twist = Twist()   # 敵を見続ける用の速度

    enemy_twist.linear.x = 0
    enemy_twist.linear.y = 0
    enemy_twist.linear.z = 0
    enemy_twist.angular.x = 0
    enemy_twist.angular.y = 0

    loop_timer = rospy.Rate(5)     # ループの時間調整用 [Hz]



    while not rospy.is_shutdown():

        print operation_sequence

        if operation_sequence == "Initial Move":    # 最初の移動
            if not goal_reached_flg:
                my_zone_no = (checkPosition(Tx,Ty))[0]
                InputPose(sniper_pos[my_zone_no][0], sniper_pos[my_zone_no][1], sniper_pos[my_zone_no][2])
            else:
                goal_reached_flg = False
                operation_sequence = "Okazu Sniper"
                start_time = time.time()
        elif initial_motion_flg and checkEnemy([enemy_x, enemy_y, enemy_th]) and getEnemyDistance([Tx, Ty, th], [enemy_x, enemy_y, enemy_th]) < enemy_distance_threshold:  #敵が居て、かつ近かったら
            print "!!!!!!!! Enemy Sniper !!!!!!!!"
            my_zone_no = (checkPosition(Tx,Ty))[0]
            enemy_look_ang = lookatEnemyAng([Tx, Ty, th], [enemy_x, enemy_y, enemy_th])
            InputPose(Tx, Ty, enemy_look_ang)
        else:   # 敵が居ない or 距離が遠い
            if operation_sequence == "Moving to Next Point":      # 次のポイントへ移動
                if onway_flg:
                    if not goal_reached_flg:
                        my_zone_no = (checkPosition(Tx,Ty))[0]
                        InputPose(way_pos[next_zone_no][0], way_pos[next_zone_no][1], way_pos[next_zone_no][2])
                    else:
                        goal_reached_flg = False
                        onway_flg = False
                        topoint_flg = True
                elif topoint_flg:               # 狙撃ポイントへ
                    #print "My Zone: " + str(my_zone_no)
                    #print "Next Zone: " + str(next_zone_no)
                    if not goal_reached_flg:
                        my_zone_no = (checkPosition(Tx,Ty))[0]
                        InputPose(sniper_pos[next_zone_no][0], sniper_pos[next_zone_no][1], sniper_pos[next_zone_no][2])
                    else:       # ポイントに到着したら
                        goal_reached_flg = False
                        onway_flg = False
                        topoint_flg = False
                        operation_sequence = "Okazu Sniper"
                        start_time = time.time()
                else:
                    goal_reached_flg = False
                    onway_flg = False
                    topoint_flg = False
                    operation_sequence = "Okazu Sniper"
                    start_time = time.time()
                """
                if checkEnemy([enemy_x, enemy_y, enemy_th]):  #敵が居たら
                    goal_reached_flg = False
                    onway_flg = False
                    topoint_flg = False
                    operation_sequence = "Enemy Sniper"
                    start_time = time.time()
                elif onway_flg:       # ウェイポイント中継
                    if not goal_reached_flg:
                        if next_zone_no - my_zone_no == 1 or next_zone_no - my_zone_no == -3:       # 右回りなら
                            InputPose(way_pos[my_zone_no][0], way_pos[my_zone_no][1], way_pos[my_zone_no][2])
                        else:                                                                       # 左回りなら
                            InputPose(way_pos[my_zone_no][0], way_pos[my_zone_no][1], math.pi + way_pos[my_zone_no][2])
                    else:
                        goal_reached_flg = False
                        onway_flg = False
                        topoint_flg = True
                elif topoint_flg:               # 狙撃ポイントへ
                    if not goal_reached_flg:
                        InputPose(sniper_pos[next_zone_no][0], sniper_pos[next_zone_no][1], sniper_pos[next_zone_no][2])
                    else:       # ポイントに到着したら
                        goal_reached_flg = False
                        topoint_flg = False
                        operation_sequence = "Okazu Sniper"
                        start_time = time.time()
                """

            elif operation_sequence == "Okazu Sniper":  # 首を振っておかずをShoot!
                now_time = time.time()
                elapsed_time = now_time - start_time
                if elapsed_time > swing_time:  # 一定時間首振り
                    start_time = now_time
                    my_zone_no = (checkPosition(Tx,Ty))[0]
                    #print "My Zone: " + str(my_zone_no)
                    if my_zone_no < zone_num - 1:   # とりあえず右回りにしてます
                        next_zone_no = my_zone_no + 1
                    else:
                        next_zone_no = 0
                    initial_motion_flg = True
                    onway_flg = True
                    #topoint_flg = True
                    goal_reached_flg = False
                    operation_sequence = "Moving to Next Point"
                else:
                    my_zone_no = (checkPosition(Tx,Ty))[0]
                    swingBurger([Tx, Ty, th], sniper_pos[my_zone_no], point_pub, swing_ang, elapsed_time, swing_time, swing_num)
                    
            """
            else:
                goal_reached_flg = False
                #onway_flg = False
                topoint_flg = False
                operation_sequence = "Okazu Sniper"
                start_time = time.time()
            """

        loop_timer.sleep()



"""
    InputPose(-1.3,0.0,10.0*math.pi/180) # 5deg
    r.sleep()
    r.sleep()
    r.sleep()
    r.sleep()
    r.sleep()
    '''
    while True:
        if status==2:
            print "arrive"
            break
        print "moving"      
        r.sleep()
    '''
    InputPose(-1.3,0.0,-10.0*math.pi/180) # -5deg
    r.sleep()
    r.sleep()
    r.sleep()
    r.sleep()
    r.sleep()
    '''
    while True:
        if status==2:
            print "arrive"
            break
        print "moving"      
        r.sleep()
    '''
    while True:
      print "point"
      print my_point
      print enemy_point
      if (Time > 10 and my_point<enemy_point):
        #move to enemy's back
        #get enemy's point
        #whether get points or not
        print "active mode"

      else:
        print "sniper mode"
        #print "move space"
        #search enemy's position and enemy change position
        #if pose_[0]!=OLDpose_[0] or pose_[1]!=OLDpose_[1] or pose_[2]!=OLDpose_[2]:

        if enemy_position_num==3:
            if my_position_num==0 and my_position_num2!=4:
                my_position_num_to=4
            elif my_position_num==0 and my_position_num2==4:
                my_position_num_to=1
            if my_position_num==2 and my_position_num2!=5:
                my_position_num_to=5
            elif my_position_num==2 and my_position_num2==5:
                my_position_num_to=1
            #if my_position_num==3:
                #turn to enemy direction
            if my_position_num==1:
                #stay
                my_position_num_to=1

        if enemy_position_num==2:
            if my_position_num==1 and my_position_num2!=4:
                my_position_num_to=4
            elif my_position_num==1 and my_position_num2==4:
                my_position_num_to=0
            if my_position_num==3 and my_position_num2!=7:
                my_position_num_to=7
            elif my_position_num==3 and my_position_num2==7:
                my_position_num_to=0
            #if my_position_num==2:
                #turn to enemy direction
            if my_position_num==0:
                #stay
                my_position_num_to=0

        if enemy_position_num==1:
            if my_position_num==2 and my_position_num2!=6:
                my_position_num_to=6
            elif my_position_num==2 and my_position_num2==6:
                my_position_num_to=3
            if my_position_num==0 and my_position_num2!=7:
                my_position_num_to=7
            elif my_position_num==0 and my_position_num2==7:
                my_position_num_to=3
            #if my_position_num==1:
                #turn to enemy direction
            if my_position_num==3:
                #stay
                my_position_num_to=3

        if enemy_position_num==0:
            if my_position_num==1 and my_position_num2!=5:
                my_position_num_to=5
            elif my_position_num==1 and my_position_num2==5:
                my_position_num_to=3
            if my_position_num==3 and my_position_num2!=6:
                my_position_num_to=6
            elif my_position_num==3 and my_position_num2==6:
                my_position_num_to=3
            #if my_position_num==0:
                #turn to enemy direction
            if my_position_num==2:
                #stay
                my_position_num_to=2
        
        #OLDpose_=pose_
        #OLDpose_=position[my_position_num_to]
        Time =time.time()-OLDTime
        print Time
        print "my_position_num"
        print my_position_num
        print "my_position_num2"
        print my_position_num2
        print "enemy_position_num"
        print enemy_position_num
        print "my_position_num_to"
        print my_position_num_to
        print " "
        #InputPose(position[my_position_num_to][0],position[my_position_num_to][1],position[my_position_num_to][2])
    r.sleep()

"""
