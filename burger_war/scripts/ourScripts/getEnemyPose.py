#!/usr/bin/env python
# license removed for brevity
import re
import rospy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray,MultiArrayLayout,MultiArrayDimension
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import LaserScan
import math
import numpy as np

class GetEnemyPose(object):
	def __init__(self):
		self.warState_sub = rospy.Subscriber('war_state', String, self.warStateCallback) 
		self.map_sub = rospy.Subscriber('map', OccupancyGrid, self.mapCallback)
		self.odom_sub = rospy.Subscriber('odom', Odometry, self.odomCallback)
		self.enemy_pose_red_ball_sub = rospy.Subscriber('red_ball_position', Pose2D, self.enemyPoseRedBallCallback)
		self.pose_pub = rospy.Publisher('enemy_pose', Pose2D, queue_size=10)
		#self.pose_p_pub = rospy.Publisher('enemy_pose_p', Pose2D, queue_size=10)
		self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scanCallback)
		self.points_array_pub = rospy.Publisher('points_array', Int32MultiArray, queue_size=10)
		self.pose_p=Pose2D()
		self.pose=Pose2D()
		self.pose_r=Pose2D()

		self.NEWtarget=[0]*18
		self.OLDtarget=[0]*18
		self.flag=[0]*18
		self.pose_red_ball=[0.0]*3

		self.rmakerPose=[[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0],
		[0.9,0.5,math.pi],[0.0,0.5,0.0],[0.9,-0.5,math.pi],[0.0,-0.5,0.0],
		[0.0,0.5,math.pi],[-0.9,0.5,0.0],[0.0,-0.5,math.pi],[-0.9,-0.5,0.0],
		[0.5,0.0,math.pi],[0.0,-0.5,math.pi/2],[0.0,0.5,-1*math.pi/2],[-0.5,0.0,0.0]]

		self.bmakerPose=[[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0],
		[-0.9,-0.5,0.0],[0.0,-0.5,math.pi],[-0.9,0.5,0.0],[0.0,0.5,math.pi],
		[0.0,-0.5,0.0],[0.9,-0.5,math.pi],[0.0,0.5,0.0],[0.9,0.5,math.pi],
		[-0.5,0.0,0.0],[0.0,0.5,-1*math.pi/2],[0.0,-0.5,math.pi/2],[0.5,0.0,math.pi]]

		self.enemyPose=[0.0]*3
		self.my_bot_id = ""
		self.myColor=0
		self.enemyTime=0.0
		self.OLDenemyTime=0.0
		self.Tx=0.0
		self.Ty=0.0
		self.th=0.0
		self.enemyPose_p=[0.0]*2
		self.myPoint=0
		self.enemyPoint=0

	def enemyPoseRedBallCallback(self,pose):
		self.pose_red_ball[0]=pose.x
		self.pose_red_ball[1]=pose.y
		self.pose_red_ball[2]=pose.theta
		print 'enemy_pose_red_ball'
		print pose
		self.pose_r.x=self.pose_red_ball[0]
		self.pose_r.y=self.pose_red_ball[1]
		self.pose_r.theta=0.0

	def mapCallback(self,map_info):
		self.my_bot_id = map_info.header.frame_id
		#print my_bot_id
		if 'red_bot' in self.my_bot_id:
			self.myColor=1
		elif 'blue_bot' in self.my_bot_id:
			self.myColor=-1
		#print myColor

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
					if self.myColor==1:
						self.enemyPose=self.rmakerPose[i]
					elif self.myColor==-1:
						self.enemyPose=self.bmakerPose[i]
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

	def odomCallback(self,my_pose_msg):
		self.Tx=my_pose_msg.pose.pose.position.x
		self.Ty=my_pose_msg.pose.pose.position.y
		#z=my_pose_msg.pose.pose.orientation.z
		#w=my_pose_msg.pose.pose.orientation.w
		#th=2*math.acos(w)*(math.asin(z)/abs(math.asin(z)))
		self.th=2*math.acos(my_pose_msg.pose.pose.orientation.w)*(math.asin(my_pose_msg.pose.pose.orientation.z)/abs(math.asin(my_pose_msg.pose.pose.orientation.z)))
		#print self.th

	def scanCallback(self,scan):
		points=np.array([[0.0]*360,[0.0]*360])
		rot=np.array([[math.cos(math.pi/4),math.sin(math.pi/4)],[-1*math.sin(math.pi/4),math.cos(math.pi/4)]])
		rot2=np.array([[math.cos(-1*math.pi/4),math.sin(-1*math.pi/4)],[-1*math.sin(-1*math.pi/4),math.cos(-1*math.pi/4)]])
		rot3=np.array([[math.cos(self.th),math.sin(self.th)],[-1*math.sin(self.th),math.cos(self.th)]])
		for i in range(360):
			points[0][i]=scan.ranges[i]*math.sin(2*math.pi*i/360)
			points[1][i]=scan.ranges[i]*math.cos(2*math.pi*i/360)
		points=np.dot(rot3,points)
		for i in range(360):
			points[0][i]+=self.Ty
			points[1][i]+=self.Tx

			if ((0.33<points[0][i]<0.73 and 0.355<points[1][i]<0.705) or
				(-0.73<points[0][i]<-0.33 and -0.705<points[1][i]<-0.355) or
				(0.33<points[0][i]<0.73 and -0.705<points[1][i]<-0.355) or
				(-0.73<points[0][i]<-0.33 and 0.355<points[1][i]<0.705) or
				(-0.275<points[0][i]<0.275 and -0.275<points[1][i]<0.275)):
					points[0][i]=0.0
					points[1][i]=0.0
		points=np.dot(rot,points)
		for i in range(360):
			if (1.1<points[0][i] or points[0][i]<-1.1 or 1.1<points[1][i] or points[1][i]<-1.1):
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
			if maxL < 0.15:
				if maxL>0.001 and count!=0 and not (-0.001<ave[0] <0.001 and -0.001<ave[1]<0.001):
					#print maxL
					#print ave
					#print "Fin"
					self.pose_p.x=ave[0]
					self.pose_p.y=-1*ave[1]
					self.pose_p.theta=1000.0
				else:
					self.pose_p.x=0.0
					self.pose_p.y=0.0
					self.pose_p.theta=0.0
				#self.pose_pub.publish(self.pose_p)
				break
		self.integratePoses()

	def integratePoses(self):
		if not (self.pose_p.x==0 and self.pose_p.y==0 and self.pose_p.theta==0):
			self.pose_pub.publish(self.pose_p)
		elif not (self.pose.x==0 and self.pose.y==0 and self.pose.theta==0):
			self.pose_pub.publish(self.pose)
		elif not (self.pose_r.x==0 and self.pose_r.y==0 and self.pose_r.theta==0):
			self.pose_pub.publish(self.pose_r)
		else:
			self.pose_pub.publish(self.pose)

if __name__ == '__main__':
	rospy.init_node('get_enemy_pose', anonymous=True)
	#warState_sub = rospy.Subscriber('war_state', String, warStateCallback) 
	#map_sub = rospy.Subscriber('map', OccupancyGrid, mapCallback)
	#odom_sub = rospy.Subscriber('odom', Odometry, odomCallback)
	#pose_pub = rospy.Publisher('enemy_pose', Pose2D, queue_size=10)
	##pose_p_pub = rospy.Publisher('enemy_pose_p', Pose2D, queue_size=10)
	#scan_sub = rospy.Subscriber('scan', LaserScan, scanCallback)
	#points_array_pub = rospy.Publisher('points_array', Int32MultiArray, queue_size=10)
	#r = rospy.Rate(10) # 10hz
	#while not rospy.is_shutdown():
		#r.sleep()
	GetEnemyPose()
	rospy.spin()
