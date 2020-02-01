#!/usr/bin/env python
# license removed for brevity
import re
import rospy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Int32MultiArray,MultiArrayLayout,MultiArrayDimension
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3
import math
import numpy as np

class GetEnemyPose(object):
	def __init__(self):
		self.pose_p=Pose2D()
		self.pose=Pose2D()
		self.pose_r=Pose2D()
		self.pose_g=Pose2D()
		self.pose_ar=Pose2D()
		self.NEWtarget=[0]*18
		self.OLDtarget=[0]*18
		self.flag=[0]*18
		self.pose_red_ball=[0.0]*3
		self.pose_green=[0.0]*3
		self.poseAR=[0.0]*3
		self.obstacle=[0.0]*2
		self.makerPose=[[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0],
		[0.9,0.5,math.pi],[0.0,0.5,0.0],[0.9,-0.5,math.pi],[0.0,-0.5,0.0],
		[0.0,0.5,math.pi],[-0.9,0.5,0.0],[0.0,-0.5,math.pi],[-0.9,-0.5,0.0],
		[0.5,0.0,math.pi],[0.0,-0.5,math.pi/2],[0.0,0.5,-1*math.pi/2],[-0.5,0.0,0.0]]

		"""
		self.bmakerPose=[[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0],
		[-0.9,-0.5,0.0],[0.0,-0.5,math.pi],[-0.9,0.5,0.0],[0.0,0.5,math.pi],
		[0.0,-0.5,0.0],[0.9,-0.5,math.pi],[0.0,0.5,0.0],[0.9,0.5,math.pi],
		[-0.5,0.0,0.0],[0.0,0.5,-1*math.pi/2],[0.0,-0.5,math.pi/2],[0.5,0.0,math.pi]]
		"""
		self.enemyPose=[0.0]*3
		#self.my_bot_id = ""
		self.myColor=0
		self.enemyTime=0.0
		self.OLDenemyTime=0.0
		self.Tx=0.0
		self.Ty=0.0
		self.th=0.0
		self.enemyPose_p=[0.0]*2
		self.myPoint=0
		self.enemyPoint=0
		#print "side!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
		#print rospy.get_param("send_id_to_judge/side")
		self.my_side = rospy.get_param("send_id_to_judge/side")
		self.myColor=self.checkMySide()
		#print "self.myColor"
		#print self.myColor
		self.warState_sub = rospy.Subscriber('war_state', String, self.warStateCallback) 
		#self.map_sub = rospy.Subscriber('map', OccupancyGrid, self.mapCallback)
		self.odom_sub = rospy.Subscriber('odom', Odometry, self.odomCallback)
		self.enemy_pose_green_marker_sub = rospy.Subscriber('green_position', Pose2D, self.enemyPoseGreenCallback)
		self.enemy_pose_red_ball_sub = rospy.Subscriber('red_ball_position', Pose2D, self.enemyPoseRedBallCallback)
		self.enemy_pose_ar_sub = rospy.Subscriber('ar_enemybot_pose', Vector3, self.enemyPoseARCallback)
		self.pose_pub = rospy.Publisher('enemy_pose', Pose2D, queue_size=10)
		self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scanCallback)
		self.points_array_pub = rospy.Publisher('points_array', Int32MultiArray, queue_size=10)
		self.obstacle_direction_pub = rospy.Publisher('obstacle_direction', Float32, queue_size=10)

	def enemyPoseARCallback(self,pose):
		self.poseAR[0]=pose.x
		self.poseAR[1]=pose.y
		self.poseAR[2]=pose.z
		#print 'enemy_pose_AR'
		#print pose
		self.pose_ar.x=self.poseAR[0]
		self.pose_ar.y=self.poseAR[1]
		self.pose_ar.theta=self.poseAR[2]

	def enemyPoseGreenCallback(self,pose):
		self.pose_green[0]=pose.x
		self.pose_green[1]=pose.y
		self.pose_green[2]=pose.theta
		#print 'enemy_pose_green'
		#print pose
		self.pose_g.x=self.pose_green[0]
		self.pose_g.y=self.pose_green[1]
		self.pose_g.theta=0.0

	def enemyPoseRedBallCallback(self,pose):
		self.pose_red_ball[0]=pose.x
		self.pose_red_ball[1]=pose.y
		self.pose_red_ball[2]=pose.theta
		#print 'enemy_pose_red_ball'
		#print pose
		self.pose_r.x=self.pose_red_ball[0]
		self.pose_r.y=self.pose_red_ball[1]
		self.pose_r.theta=0.0
	'''
	def mapCallback(self,map_info):
		self.my_bot_id = map_info.header.frame_id
		#print my_bot_id
		if 'red_bot' in self.my_bot_id:
			self.myColor=1
		elif 'blue_bot' in self.my_bot_id:
			self.myColor=-1
		else:
			self.myColor=-1
		#print myColor
	'''
	def checkMySide(self):
		if rospy.get_param("send_id_to_judge/side") == 'r':
			Color=1
		elif rospy.get_param("send_id_to_judge/side") == 'b':
			Color=-1
		return Color

	def warStateCallback(self,warStates):
		r_point=0
		b_point=0
		flags=0
		self.warState_str = warStates.data
		self.warState = []
		self.target=[]
		self.warState_data_str = self.warState_str.split('\n')
		self.OLDtarget=self.NEWtarget
		for self.warState_data in self.warState_data_str:
			if self.warState_data == '':
				continue
			self.warState_list = []
			self.warState_num_list = re.findall("(?<=\").*?(?=\")",self.warState_data)
			for self.warState_num in self.warState_num_list:
				self.warState_list.append(self.warState_num)
			self.warState.append(self.warState_list)

		for i in range(104):
			if (i>=16 and i<46 and i%5==3):
				if self.warState[i-1][2]=='r':
					r_point+=5
					self.target.append(1*int(self.warState[i][2])*self.myColor)
				elif self.warState[i-1][2]=='b':
					b_point+=5
					self.target.append(-1*int(self.warState[i][2])*self.myColor)
				elif self.warState[i-1][2]=='n':
					self.target.append(0*int(self.warState[i][2])*self.myColor)

			if (i>=46 and i%5==3):
				if self.warState[i-1][2]=='r':
					r_point+=1
					self.target.append(1*int(self.warState[i][2])*self.myColor)
				elif self.warState[i-1][2]=='b':
					b_point+=1
					self.target.append(-1*int(self.warState[i][2])*self.myColor)
				elif self.warState[i-1][2]=='n':
					self.target.append(0*int(self.warState[i][2])*self.myColor) 
		self.NEWtarget=self.target
		for i in range(18):
			if self.NEWtarget[i]!=self.OLDtarget[i]:
				self.flag[i]=1
				flags=1
				self.OLDenemyTime=rospy.get_time()
				if self.NEWtarget[i]==-1:
					self.enemyPose=self.makerPose[i]
					#print "enemyPose: "
					#print self.enemyPose
					#pose=Pose2D()
					self.pose.x=self.enemyPose[0]
					self.pose.y=self.enemyPose[1]
					self.pose.theta=self.enemyPose[2]
					#self.pose_pub.publish(self.pose)
			elif self.NEWtarget[i]==self.OLDtarget[i]:
				self.flag[i]=0
		if flags==0:
			self.pose.x=0.0
			self.pose.y=0.0
			self.pose.theta=0.0
		#print "pose_est"
		#print self.pose
		if self.myColor==1:
			self.myPoint=r_point
			self.enemyPoint=b_point
		elif self.myColor==-1:
			self.myPoint=b_point
			self.enemyPoint=r_point
		self.enemyTime=rospy.get_time()-self.OLDenemyTime
		#print self.NEWtarget
		#print self.myPoint
		#print self.enemyPoint
		self.point_data=self.NEWtarget+[self.myPoint,self.enemyPoint]
		pointsArray=Int32MultiArray()
		pointsArray.data=[]
		pointsArray.data=self.point_data
		#points_array_pub.publish(pointsArray)
		#print point_data
		#print OLDtarget
		#print enemyTime
		#print flag
		#print r_point
		#print b_point

	def odomCallback(self,my_pose_msg): #r:-1.3,0,0 b:1.3,0,-pi
		self.Ty=-1*my_pose_msg.pose.pose.position.x
		self.Tx=my_pose_msg.pose.pose.position.y
		#z=my_pose_msg.pose.pose.orientation.z
		#w=my_pose_msg.pose.pose.orientation.w
		#th=2*math.acos(w)*(math.asin(z)/abs(math.asin(z)))
		self.th=2*math.acos(my_pose_msg.pose.pose.orientation.w)*(math.asin(my_pose_msg.pose.pose.orientation.z)/abs(math.asin(my_pose_msg.pose.pose.orientation.z)))-math.pi/2
		#print self.th

	def scanCallback(self,scan):
		back_direction=0.0
		tmp_direction=100.0
		ave_range=[0.0]*10
		for i in range(360):
			ave_range[int(i/36)]+=scan.ranges[i]
		for i in range(10):
			ave_range[i]=ave_range[i]/36
		#print "obstacle--------------------------"
		#print ave_range
		for i in range(10):
			#print (i*36+18)*math.pi/180
			if (i<5 and ave_range[i]>0.5 and ave_range[i+5]<0.20) or (i>=5 and ave_range[i]>0.5 and ave_range[i-5]<0.20):
				if math.pi<(i*36+18)*math.pi/180 and abs(back_direction)<tmp_direction:
					back_direction=(i*36+18)*math.pi/180-2*math.pi
					tmp_direction=abs(back_direction)
					#print "excape"
					#print i
					#print back_direction
				elif -math.pi>(i*36+18)*math.pi/180 and abs(back_direction)<tmp_direction:
					back_direction=(i*36+18)*math.pi/180+2*math.pi
					tmp_direction=abs(back_direction)
					#print "excape"
					#print i
					#print back_direction
				elif abs(back_direction)<tmp_direction:
					back_direction=(i*36+18)*math.pi/180
					tmp_direction=abs(back_direction)
					#print "excape"
					#print i
					#print back_direction
		#print "back_direction"
		#print back_direction
		obstacle_direction=Float32()
		obstacle_direction= back_direction
		self.obstacle_direction_pub.publish(obstacle_direction)
		#print "obstacle_direction"
		#print obstacle_direction
		#print "obstacle--------------------------"
		points=np.array([[0.0]*360,[0.0]*360])
		rot=np.array([[math.cos(math.pi/4),math.sin(math.pi/4)],[-1*math.sin(math.pi/4),math.cos(math.pi/4)]])
		rot2=np.array([[math.cos(-1*math.pi/4),math.sin(-1*math.pi/4)],[-1*math.sin(-1*math.pi/4),math.cos(-1*math.pi/4)]])
		rot3=np.array([[math.cos(-self.th),math.sin(-self.th)],[-1*math.sin(-self.th),math.cos(-self.th)]])
		rot4=np.array([[math.cos(math.pi),math.sin(math.pi)],[-1*math.sin(math.pi),math.cos(math.pi)]])
		#print "pointcloud------------------------------------------------------"
		for i in range(360):
			points[1][i]=scan.ranges[i]*math.sin(2*math.pi*i/360)
			points[0][i]=scan.ranges[i]*math.cos(2*math.pi*i/360)
		points=np.dot(rot3,points)
		for i in range(360):
			points[0][i]+=self.Tx
			points[1][i]+=self.Ty
			'''
			if ((0.235<points[0][i]<0.735 and 0.305<points[1][i]<0.805) or
				(-0.735<points[0][i]<-0.235 and -0.755<points[1][i]<-0.305) or
				(0.23<points[0][i]<0.735 and -0.755<points[1][i]<-0.305) or
				(-0.735<points[0][i]<-0.235 and 0.305<points[1][i]<0.755) or
				(-0.325<points[0][i]<0.325 and -0.325<points[1][i]<0.325)):
			if ((0.23<points[0][i]<0.83 and 0.255<points[1][i]<0.805) or
				(-0.83<points[0][i]<-0.23 and -0.805<points[1][i]<-0.255) or
				(0.23<points[0][i]<0.83 and -0.805<points[1][i]<-0.255) or
				(-0.83<points[0][i]<-0.23 and 0.255<points[1][i]<0.805) or
				(-0.375<points[0][i]<0.375 and -0.375<points[1][i]<0.375)):
			if ((0.33<points[0][i]<0.73 and 0.355<points[1][i]<0.705) or
				(-0.73<points[0][i]<-0.33 and -0.705<points[1][i]<-0.355) or
				(0.33<points[0][i]<0.73 and -0.705<points[1][i]<-0.355) or
				(-0.73<points[0][i]<-0.33 and 0.355<points[1][i]<0.705) or
				(-0.275<points[0][i]<0.275 and -0.275<points[1][i]<0.275)):
			'''
			if ((0.33<points[1][i]<0.73 and 0.355<points[0][i]<0.705) or
				(-0.73<points[1][i]<-0.33 and -0.705<points[0][i]<-0.355) or
				(0.33<points[1][i]<0.73 and -0.705<points[0][i]<-0.355) or
				(-0.73<points[1][i]<-0.33 and 0.355<points[0][i]<0.705) or
				(-0.275<points[0][i]<0.275 and -0.275<points[1][i]<0.275)):
					points[0][i]=0.0
					points[1][i]=0.0
		points=np.dot(rot,points)
		for i in range(360):
			#if (1.0<points[0][i] or points[0][i]<-1.0 or 1.0<points[1][i] or points[1][i]<-1.0):
			#if (1.1<points[0][i] or points[0][i]<-1.1 or 1.1<points[1][i] or points[1][i]<-1.1):
			#if (1.05<points[0][i] or points[0][i]<-1.05 or 1.05<points[1][i] or points[1][i]<-1.05):
			if (1.0<points[0][i] or points[0][i]<-1.0 or 1.0<points[1][i] or points[1][i]<-1.0):
				points[0][i]=0.0
				points[1][i]=0.0
		points=np.dot(rot2,points)
		count=0
		ave=[0.0]*2
		for i in range(360):
			if (0.001<points[0][i] or points[0][i]<-0.001 or 0.001<points[1][i] or points[1][i]<-0.001):
				count+=1
				ave[0]+=points[0][i]
				ave[1]+=points[1][i]
				#print points[0][i]
				#print points[1][i]
				#print count
		if count !=0:
			ave[0]=ave[0]/count
			ave[1]=ave[1]/count
		'''
		if self.myColor==-1:
			points=np.dot(rot4,points)
		'''
		while True:
			maxL=0.0
			maxi=-1
			for i in range(360):
				if ((0.001<points[0][i] or points[0][i]<-0.001 or 0.001<points[1][i] or points[1][i]<-0.001)
				and maxL < math.sqrt((ave[0]-points[0][i])*(ave[0]-points[0][i])+(ave[1]-points[1][i])*(ave[1]-points[1][i])) ):
					maxL=math.sqrt((ave[0]-points[0][i])*(ave[0]-points[0][i])+(ave[1]-points[1][i])*(ave[1]-points[1][i]))
					maxi=i
			points[0][maxi]=0.0
			points[1][maxi]=0.0
			ave[0]=0.0
			ave[1]=0.0
			#print "points"
			#print points
			count=0
			for i in range(360):
				if (0.001<points[0][i] or points[0][i]<-0.001 or 0.001<points[1][i] or points[1][i]<-0.001):
					count+=1
					ave[0]+=points[0][i]
					ave[1]+=points[1][i]
			if count !=0:
				ave[0]=ave[0]/count
				ave[1]=ave[1]/count
			#print "maxL:"
			#print maxL
			#print "ave:"
			#print ave
			#print "count:"
			#print count
			#print "points"
			#print points
			if maxL < 0.15:
				if maxL>0.001 and count>3 and not (-0.001<ave[0] <0.001 and -0.001<ave[1]<0.001):
					#print "maxL"
					#print maxL
					#print "ave"
					#print ave
					#print "enemy!!!!!!!!!!!!!!!!!!!!!!"
					self.pose_p.x=ave[0]
					self.pose_p.y=ave[1]
					self.pose_p.theta=1000.0
				else:
					self.pose_p.x=0.0
					self.pose_p.y=0.0
					self.pose_p.theta=0.0
				#self.pose_pub.publish(self.pose_p)
				break
			
		#print "pointcloud------------------------------------------------------"
		self.integratePoses()

	def integratePoses(self):
		'''
		if not (self.pose_p.x==0 and self.pose_p.y==0 and self.pose_p.theta==0):
			self.pose_pub.publish(self.pose_p)
		elif not (self.pose.x==0 and self.pose.y==0 and self.pose.theta==0):
			self.pose_pub.publish(self.pose)
		else:
			self.pose_pub.publish(self.pose)
		'''
		if not (self.pose_ar.x==0 and self.pose_ar.y==0 and self.pose_ar.theta==0):
			self.pose_pub.publish(self.pose_ar)
			print "AR"
		elif not (self.pose_g.x==0 and self.pose_g.y==0 and self.pose_g.theta==0):
			self.pose_pub.publish(self.pose_g)
			print "green"
		elif not (self.pose_r.x==0 and self.pose_r.y==0 and self.pose_r.theta==0):
			self.pose_pub.publish(self.pose_r)
			print "red"
		elif not (self.pose_p.x==0 and self.pose_p.y==0 and self.pose_p.theta==0):
			self.pose_pub.publish(self.pose_p)
			print "pointcloud"
		elif not (self.pose.x==0 and self.pose.y==0 and self.pose.theta==0):
			self.pose_pub.publish(self.pose)
			print "maker"
		else:
			self.pose_pub.publish(self.pose)

	def lookatEnemyAng(self,my_pos, enemy_pos):
		if enemy_pos[0] - my_pos[0] != 0:
			enemy_ang = math.atan( (enemy_pos[1] - my_pos[1]) / (enemy_pos[0] - my_pos[0]) )
		return enemy_ang

if __name__ == '__main__':
	rospy.init_node('get_enemy_pose', anonymous=True)
	GetEnemyPose()
	rospy.spin()
