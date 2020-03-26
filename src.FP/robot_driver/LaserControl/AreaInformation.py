#!/usr/bin/env python
#-*- coding:utf-8 -*-

import rospy
from geometry_msgs.msg import Point
#from transform_utils import quat_to_angle, normalize_angle
#from numpy import sign
from math import  sqrt, pow, pi,sin,cos,asin
import time
import zmq
import MySQLdb
#import uuid
from std_msgs.msg import String,Float32,Int32
from AllParameter import SERVICEPUBIP, SERVICESUBIP, AllTopic, ORDER, SENDER, RECVER
from tf_pose import TfRelation
from PackageMessage_pb2 import PackageMessage

from dynamic_reconfigure.server import Server
import dynamic_reconfigure.client

#import ConfigParser
#import threading
		
class AreaInfor():
	def __init__(self):
		#rospy.init_node('AreaInformation', anonymous=False)
		self.con = zmq.Context()
		self.tf_relation = TfRelation()   #TF relation
		self.rows = None
		self.plan_lines()
		
# 		# Fire up the dynamic_reconfigure server
# 		dyn_server = Server(AreaInformationConfig, self.dynamic_reconfigure_callback)
# 		# Connect to the dynamic_reconfigure server
# 		dyn_client = dynamic_reconfigure.client.Client("LaserStatus", timeout=60)
		
#		try:
#			Sqlcon = MySQLdb.connect(host = '127.0.0.1', user = 'root', passwd = '1', db = 'xrs2015', port = 3306, charset = 'utf8' )
#			cur = Sqlcon.cursor()
#			cur.execute("select a.id line,b.fname fname1,b.lasermapX point1x,b.lasermapY point1y,c.fname fname2,c.lasermapX point2x,c.lasermapY point2y,a.distance distance_line,b.id posid1 ,b.lasermapTh point1Th,c.lasermapTh point2Th from rb_topologyline a,rb_topologypoint b,rb_topologypoint c where a.point1 = b.id and a.point2 = c.id and b.fname = 'cda' and c.fname = 'kua';")
#			row = cur.fetchall()
#			cur.close()
#			Sqlcon.close() 
			#print "AreaInformation:: row :",row						
#		except MySQLdb.Error,e:
#			print "AreaInformation::AreaInfor:__init__ MySQL Error %d: %s" %(e.args[0], e.args[1])
		
		
				
# 		rospy.set_param('/LaserStatus/CurLine',row[0][0])
# 		rospy.set_param('/LaserStatus/CurLinePass',row[0][7])
# 		rospy.set_param('/LaserStatus/CurLineDone',1.0)
# 		rospy.set_param('/LaserStatus/Curx',row[0][5])
# 		rospy.set_param('/LaserStatus/Cury',row[0][6])
# 		rospy.set_param('/LaserStatus/Fname',row[0][1])
# 		rospy.set_param('/LaserStatus/Fnameend',row[0][4])
# 		rospy.set_param('/LaserStatus/PosID',row[0][8])
		
#		self.ReAreaInfo = {}
#		self.ReAreaInfo['CurLine'] = rospy.get_param('/LaserStatus/CurLine',row[0][0])
# 		#print "AreaInformation::Area_line cur_line",line_id
#		self.ReAreaInfo['CurLinePass'] = rospy.get_param('/LaserStatus/CurLinePass',0.0)
#		self.ReAreaInfo['CurLineDone'] = rospy.get_param('/LaserStatus/CurLineDone',0.0)
#		self.ReAreaInfo['Curx'] = rospy.get_param('/LaserStatus/Curx',row[0][5])
#		self.ReAreaInfo['Cury'] = rospy.get_param('/LaserStatus/Cury',row[0][6])
#		self.ReAreaInfo['CurTh'] = rospy.get_param('/LaserStatus/CurTh',row[0][10])
#		self.ReAreaInfo['Fname'] = rospy.get_param('/LaserStatus/Fname',row[0][1])
#		self.ReAreaInfo['Fnameend'] = rospy.get_param('/LaserStatus/Fnameend',row[0][4])
#		self.ReAreaInfo['PosID'] = rospy.get_param('/LaserStatus/PosID',row[0][8])
		#print "AreaInformation::AreaInfor::__init__ ReareaInfo :" ,self.ReAreaInfo
# 		self.params = {'CurLine': self.row[0],
# 				'CurLinePass':self.row[7],
# 				'CurLineDone':1.0,
# 				'Curx':self.row[5],
# 				'Cury':self.row[6],
# 				'Fname':self.row[1],
# 				'Fnameend':self.row[4],
# 				'PosID':self.row[8]}
# 		self.params = {}
# 		self.params['CurLine'] = self.row[0]
# 		self.params['CurLinePass'] = self.row[7]
# 		print "self.params",self.params
#         #rospy.loginfo(self.params)
#         dyn_client.update_configuration(self.params)
#         rospy.loginfo(self.ReAreaInfo)
        
# 	def dynamic_reconfigure_callback(self, config, level):
# 		self.ReAreaInfo['CurLine'] = config['CurLine']
# 		self.ReAreaInfo['CurLinePass'] = config['CurLinePass']
# 		self.ReAreaInfo['Curx'] = config['Curx']
# 		self.ReAreaInfo['Cury'] = config['Cury']
# 		self.ReAreaInfo['Fname'] = config['Fname']
# 		self.ReAreaInfo['Fnameend'] = config['Fnameend']
# 		self.ReAreaInfo['PosID'] = config['PosID']
# 		return config
       
	def zmqpubcon(self):
		pusock = self.con.socket(zmq.PUB)
		pusock.connect(SERVICEPUBIP)
		return pusock
	
	def zmqsubcon(self):
		subsock = self.con.socket(zmq.SUB)
		subsock.connect(SERVICESUBIP)
		return subsock

#计算当前点与路径直线间的距离            
	def cal_distance(self,xcur, ycur,startpose_x,startpose_y,nextpose_x,nextpose_y):
		try:
			a_x = nextpose_x- startpose_x
			a_y = nextpose_y- startpose_y
			if a_x == 0:
				if a_y >= 0:
					ptl = xcur - nextpose_x
				else:
					ptl = nextpose_x- xcur
			else:
				K = (nextpose_y- startpose_y)/(nextpose_x- startpose_x)
				B = (nextpose_x*startpose_y - startpose_x*nextpose_y)/(nextpose_x- startpose_x)
				if a_x>0:
					if (-K*xcur + ycur - B) > 0:
						ptl = -abs(-K*xcur + ycur - B)/sqrt(pow(-K,2) + 1)    
					else:
						ptl = abs(-K*xcur + ycur - B)/sqrt(pow(-K,2) + 1)    
				else:
					if (-K*xcur + ycur - B) > 0:
						ptl = abs(-K*xcur + ycur - B)/sqrt(pow(-K,2) + 1)    
					else:
						ptl = -abs(-K*xcur + ycur - B)/sqrt(pow(-K,2) + 1)    
            #print "AreaInformation::cal_distance  ptl ==%f" %(ptl)
			return ptl
		except Exception as err:    
			print "AreaInformation::AreaInfor::cal_distance  Error: ",err
		#return
    
#计算路径与地图正方向间夹角
	def cal_path_rotation(self,xcur,ycur,nextpose_x,nextpose_y):
	#计算下一条路径直线和全局正方间的夹角 (规定正方向向量（xcur+1,ycur）)
		prodis = sqrt(pow(nextpose_x-xcur,2)+pow(nextpose_y-ycur,2))
		curdis = 1
		dot_product = (nextpose_x-xcur)
		dif_product = (nextpose_y-ycur)
		try:
			cosm = dot_product / (prodis*curdis)
			sinm = dif_product / (prodis*curdis)
	                
			if cosm >= 0:
				path_th = asin(sinm)
			else:
				if sinm >=0:
					path_th = pi - asin(sinm)
				else:
					path_th = -pi - asin(sinm)
			return path_th
		except Exception as err:    
			print "AreaInformation::AreaInfor::cal_path_rotation  Error:",err
			#return
	
	def plan_lines(self):
	
		try:
			Sqlcon = MySQLdb.connect(host = '127.0.0.1', user = 'root', passwd = '1', db = 'xrs2015', port = 3306, charset = 'utf8' )
			#Sqlcon.ping(True)
			
			cur = Sqlcon.cursor()
			cur.execute("select a.id line,b.fname fname1,b.lasermapX point1x,b.lasermapY point1y,c.fname fname2,c.lasermapX point2x,c.lasermapY point2y,a.distance distance_line,b.id posid1 from rb_topologyline a,rb_topologypoint b,rb_topologypoint c where a.point1 = b.id and a.point2 = c.id;")
			self.rows = cur.fetchall()
			cur.close()
			Sqlcon.close() 
		except MySQLdb.Error,e:
			print "AreaInformation::Area_line MySQL Error %d: %s" %(e.args[0], e.args[1])
			#return 
	
#def PubAreaInforLooper():
#	pub = zmqpubcon()		
#	AreaInforMsg = PackageMessage()
#	while not rospy.is_shutdown():
#		time.sleep(4)
#		try:
#			AreaInforMsg.Clear()
# 			AreaInforMsg.SessionId = "%s" %uuid.uuid1()
#	 	 	AreaInforMsg.Time = int(time.time()*100000000)
#			AreaInforMsg.From = "AreaInfo"
#			AreaInforMsg.To = RECVER[1]
#			AreaInforMsg.CallMessage.Function = ORDER[15]
#			
#			AreaInforParam = Area_line()
#			#print "AreaInforParam::PubAreaInforLooper 1: ", AreaInforParam
#				
#			if (AreaInforParam != None) and (AreaInforParam != AreaInfo()):
#				AreaInforMsg.CallMessage.Parameters.append(AreaInforParam.SerializeToString())
#				try:
#					pub.send_multipart([AllTopic[0], "\0", AreaInforMsg.SerializeToString()])
#					#print "AreaInforParam::PubAreaInforLooper 2: ", AreaInforMsg
#				except Exception as err:
#					print "AreaInforParam::PubAreaInforLooper exception the pub.send_multipart Error: ",err
#					continue
#			else:
#				continue
#		except Exception as err:
#			print "AreaInforParam::PubAreaInforLooper exception the Error: ",err
#			continue

	
	def Area_line(self):

		ReAreaInfo = {}
		
		start_point = Point()
		goal_point = Point()
		position = Point()
		
# 		try:
# 			Sqlcon = MySQLdb.connect(host = '127.0.0.1', user = 'root', passwd = '1', db = 'xrs2015', port = 3306, charset = 'utf8' )
# 			#Sqlcon.ping(True)
#  			
# 			if len(Roadlists) != 0:
# 				for Roadlist in Roadlists:
# 					if rospy.is_shutdown():
# 						break
#  			
# 					cur = Sqlcon.cursor()
# 					cur.execute("select a.id line,b.fname fname1,b.lasermapX point1x,b.lasermapY point1y,c.fname fname2,c.lasermapX point2x,c.lasermapY point2y,a.distance distance_line,b.id posid1 from rb_topologyline a,rb_topologypoint b,rb_topologypoint c where a.point1 = b.id and a.point2 = c.id and a.id = '%s';" %(Roadlist))
# 					row = cur.fetchall()
# 					rows.append(row)
#  				
# 				cur.close()
# 				Sqlcon.close() 
#  						
# 		except MySQLdb.Error,e:
# 			print "AreaInformation::Area_line MySQL Error %d: %s" %(e.args[0], e.args[1])
# 			return 
#		try:
#			Sqlcon = MySQLdb.connect(host = '127.0.0.1', user = 'root', passwd = '1', db = 'xrs2015', port = 3306, charset = 'utf8' )
			#Sqlcon.ping(True)
			
#			cur = Sqlcon.cursor()
#			cur.execute("select a.id line,b.fname fname1,b.lasermapX point1x,b.lasermapY point1y,c.fname fname2,c.lasermapX point2x,c.lasermapY point2y,a.distance distance_line,b.id posid1 from rb_topologyline a,rb_topologypoint b,rb_topologypoint c where a.point1 = b.id and a.point2 = c.id;")
#			rows = cur.fetchall()
#			cur.close()
#			Sqlcon.close() 
#		except MySQLdb.Error,e:
#			print "AreaInformation::Area_line MySQL Error %d: %s" %(e.args[0], e.args[1])
#			return 
		  
		try:
			(position, rotation) = self.tf_relation.get_odom_map()
			#print "AreaInformation::Area_line the position,rotation" ,position,rotation
		except Exception as err:
			print "AreaInformation::AreaInfor::Area_line  get odom Error"        
		               	
		try:
			if len(self.rows) != 0:
				print "AreaInformation::AreaInfor::Area_line len(self.rows)++++++",len(self.rows)
				for row in self.rows:
					if rospy.is_shutdown():
						break
					
					line_id = row[0]
					fname_start = row[1]
					start_point.x = row[2]
					start_point.y = row[3]
					fname_end = row[4]
					goal_point.x = row[5]
					goal_point.y = row[6]
					line_distance = sqrt(pow((goal_point.x - start_point.x), 2) + pow((goal_point.y - start_point.y), 2))#row[7]
					posid = row[8]
					
					if (line_distance > 1.5):
						distance_Th = 0.55 #0.5
					else:
						distance_Th = 0.4 #0.35				
					try:			
						distances = self.cal_distance(position.x, position.y, start_point.x, start_point.y, goal_point.x, goal_point.y)
						#print "AreaInformation::Area_line  distances %.2f" %distances				
						distance_done = sqrt(pow((position.x - start_point.x), 2) + pow((position.y - start_point.y), 2))
						distance_overplus = sqrt(pow(goal_point.x-position.x,2) + pow(goal_point.y-position.y,2))
						path_rot = self.cal_path_rotation(start_point.x,start_point.y,goal_point.x,goal_point.y)
			 		except Exception as err:
					 	print "AreaInformation::AreaInfor::Area_line The distance calculate exception :",err
					 	continue
					 					
					th_angle = rotation - path_rot
					if th_angle >= pi:
						th_angle -= pi*2
					elif th_angle < -pi:
						th_angle += pi*2
					else:
						th_angle = th_angle
					
					#print "AreaInformation::Area_line the distances and angle:%.2f,%.2f" ,distances,th_angle		
					if (abs(distances) <= distance_Th) and (-pi/3 <= th_angle <= pi/3):
						if (start_point.x <= goal_point.x):
							s_x = start_point.x
							g_x = goal_point.x
							x_range = goal_point.x - start_point.x
						else:
							s_x = goal_point.x
							g_x = start_point.x
							x_range = start_point.x - goal_point.x
							
						if (start_point.y <= goal_point.y):
							s_y = start_point.y
							g_y = goal_point.y
							y_range = goal_point.y - start_point.y
						else:
							s_y = goal_point.y
							g_y = start_point.y
							y_range = start_point.y - goal_point.y
						
						try:	
							if (y_range <= x_range):
								if ((s_x - distance_Th) <= position.x <= (g_x + distance_Th)):
									try:
										if (distance_done >= line_distance ) and (distance_overplus <= 0.5):
											distance_done = line_distance
											#print"AreaInformation::Area_line the distance_done and distance_overplus11 :%.2f,%.2f",distance_done ,distance_overplus
										elif (distance_overplus >= line_distance) and (distance_done <= 0.5):
											distance_done = 0
											#print"AreaInformation::Area_line the distance_done and distance_overplus12 :%.2f,%.2f",distance_done ,distance_overplus
										elif (distance_done >= line_distance) and (distance_overplus > 0.5):
											#print"AreaInformation::Area_line continue the distance_overplus11:%.2f",distance_overplus
											continue
										elif (distance_overplus >= line_distance) and (distance_done > 0.5):
											#print"AreaInformation::Area_line continue the distance_overplus12:%.2f",distance_overplus
											continue
										else:
											pass
																	
										ReAreaInfo['CurLine'] = line_id
										print "AreaInformation::AreaInfor::Area_line cur_line 1111",line_id
										ReAreaInfo['CurLinePass'] = distance_done
										ReAreaInfo['CurLineDone'] = distance_done/line_distance
										ReAreaInfo['Curx'] = position.x
										ReAreaInfo['Cury'] = position.y
										ReAreaInfo['CurTh'] = rotation
										ReAreaInfo['Fname'] = fname_start
										ReAreaInfo['Fnameend'] = fname_end
										ReAreaInfo['PosID'] = posid
										#print"AreaInformation::Area_line ReAreaInfo1: ",ReAreaInfo
										break
									except Exception as err:
										print "AreaInformation::AreaInfor::Area_line exception if-else judge1 the Error: ",err
										continue							
							else:
								if ((s_y - distance_Th) <= position.y <= (g_y + distance_Th)):
									try:
										if (distance_done >= line_distance ) and (distance_overplus <= 0.5):
											distance_done = line_distance
											#print"AreaInformation::Area_line the distance_done and distance_overplus21 :%.2f,%.2f",distance_done ,distance_overplus
										elif (distance_overplus >= line_distance) and (distance_done <= 0.5):
											distance_done = 0
											#print"AreaInformation::Area_line the distance_done and distance_overplus22 :%.2f,%.2f",distance_done ,distance_overplus
										elif (distance_done >= line_distance) and (distance_overplus > 0.5):
											#print"AreaInformation::Area_line continue the distance_overplus21:%.2f",distance_overplus
											continue
										elif (distance_overplus >= line_distance) and (distance_done > 0.5):
											#print"AreaInformation::Area_line continue the distance_overplus22:%.2f",distance_overplus
											continue
										else:
											pass
										
										ReAreaInfo['CurLine'] = line_id
										print "AreaInformation::AreaInfor::Area_line cur_line 2222",line_id
										ReAreaInfo['CurLinePass'] = distance_done
										ReAreaInfo['CurLineDone'] = distance_done/line_distance
										ReAreaInfo['Curx'] = position.x
										ReAreaInfo['Cury'] = position.y
										ReAreaInfo['CurTh'] = rotation
										ReAreaInfo['Fname'] = fname_start
										ReAreaInfo['Fnameend'] = fname_end
										ReAreaInfo['PosID'] = posid
										#print"AreaInformation::Area_line ReAreaInfo2: ",ReAreaInfo
										break
									except Exception as err:
										print "AreaInformation::AreaInfor::Area_line exception if-else judge2 the Error: ",err
										continue
						except Exception as err:
							print "AreaInformation::AreaInfor::Area_line exception  The if-else the Error: ",err
							continue
					else:
						continue			
			else:
#				ReAreaInfo['CurLine'] = self.ReAreaInfo['CurLine']
#				print "AreaInformation::AreaInfor::Area_line the length of rows is 0 and the cur_line 3333",ReAreaInfo['CurLine']
#				ReAreaInfo['CurLinePass'] = self.ReAreaInfo['CurLinePass'] 
#				ReAreaInfo['CurLineDone'] = self.ReAreaInfo['CurLineDone']
#				ReAreaInfo['Curx'] = self.ReAreaInfo['Curx']
#				ReAreaInfo['Cury'] = self.ReAreaInfo['Cury']
#				ReAreaInfo['CurTh'] = self.ReAreaInfo['CurTh']
#				ReAreaInfo['Fname'] = self.ReAreaInfo['Fname']
#				ReAreaInfo['Fnameend'] = self.ReAreaInfo['Fnameend']
#				ReAreaInfo['PosID'] = self.ReAreaInfo['PosID']
				print"AreaInformation::Area_line the length of self.rows is 0 "
		except Exception as err:
			print "AreaInformation::AreaInfor::Area_line The calculate exception error:",err
			#return None
		print "AreaInformation::AreaInfor::Area_line The Calculate Done "
		return ReAreaInfo		

	def AnswerRecv(self,recvssid,ReAreaInfo):
		pub = self.zmqpubcon()
		Result = PackageMessage()
		Result.SessionId = recvssid
		Result.Time = int(time.time()*100000000)
		Result.From = "AreaInfo"
		Result.To = RECVER[1]
		
		try:
			if (ReAreaInfo):
				Result.ResultMessage.ErrorCode = 0
			else:
				Result.ResultMessage.ErrorCode = 0xd003
			
			Result.ResultMessage.ResultData = ReAreaInfo.SerializeToString()
			
			try:
				pub.send_multipart([AllTopic[0],'\0',Result.SerializeToString()])
			except Exception as err:
				print "AreaInformation::AnswerRecv sock.send_multipart() exception error:",err
		
		except Exception as err:
			print "AreaInformation::AnswerRecv exception the error:",err
	
	def PubPos(self):   
		try:
			(trans, rot)  = self.tf_relation.get_odom_map()  
			pos = Point()
			pos.x = trans.x
			pos.y = trans.y
			pos.z = rot
			#print"Areainformation::PubPos  pos: ",pos
		except:
			print"Areainformation::AreaInfor::PubPos  Failed to get the Robotpose"
			return None
		return pos
         
	def Plan_line(self):
		#rospy.init_node("AreaInformation", anonymous=False)   
		#global Amcl_Status
		#rospy.Subscriber("AmclState", Bool, AmclState)
		#while Amcl_Status == False:
			#time.sleep(2)
		#print"AreaInformation:: AreaInfor main start working...."
		
		RecvMsg = PackageMessage()
		sock = self.zmqsubcon()
		sock.setsockopt(zmq.SUBSCRIBE,AllTopic[0])
				
		while not rospy.is_shutdown():
			try:
				topic,re,msg = sock.recv_multipart()
				if topic != AllTopic[4]:
					if (msg != None) and (msg != ""):
						RecvMsg.ParseFromString(msg)
						#print"AreaInformation::AreaInfor RecvMsg:",RecvMsg
						if (RecvMsg.HasField("CallMessage") == True) and (RecvMsg.CallMessage.HasField("Function") == True) and (RecvMsg.CallMessage.Function == "Roadlist"):
							try:
								#print"AreaInformation::AreaInfor while RecvMsg.CallMessage.Function: ",RecvMsg.CallMessage.Function
								if ((len(RecvMsg.CallMessage.Parameters)) != 0) and (RecvMsg.CallMessage.Parameters):
									Roadlists = RecvMsg.CallMessage.Parameters
									print "AreaInformation::AreaInfor::Plan_line The receive Roadlists ：",Roadlists
									try:
										Sqlcon = MySQLdb.connect(host = '127.0.0.1', user = 'root', passwd = '1', db = 'xrs2015', port = 3306, charset = 'utf8' )
										#Sqlcon.ping(True)
										
										if len(Roadlists) > 0:
											for Roadlist in Roadlists:
												#print "+++Roadlist+++",Roadlist
												if rospy.is_shutdown():
													break
										
												cur = Sqlcon.cursor()
												cur.execute("select a.id line,b.fname fname1,b.lasermapX point1x,b.lasermapY point1y,c.fname fname2,c.lasermapX point2x,c.lasermapY point2y,a.distance distance_line,b.id posid1 from rb_topologyline a,rb_topologypoint b,rb_topologypoint c where a.point1 = b.id and a.point2 = c.id and a.id = '%s';" %(Roadlist))
												row = cur.fetchall()
												#print "+++row+++",row
												self.rows.append(list(row[0]))
												print "self.rows",self.rows
											cur.close()
										Sqlcon.close() 
													
									except MySQLdb.Error,e:
										print "AreaInformation::AreaInfor::Plan_line MySQL Error %d: %s" %(e.args[0], e.args[1])
										continue
							except Exception as err:    
								print "AreaInformation::AreaInfor::Plan_line The if judge exception error :",err
								continue
					else:
						print "AreaInformation::AreaInfor::Plan_line msg is NULL or None"
						continue
			except Exception as err:    
				print "AreaInformation::AreaInfor::Plan_line except the error: ",err
				continue	 	

# if __name__ == "__main__":
#     try:
# 		AreaInfor()  
#     except Exception as err:
#         print "AreaInformation exception error:",err                 
