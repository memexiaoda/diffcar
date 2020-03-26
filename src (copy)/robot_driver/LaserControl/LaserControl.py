#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import zmq
from PackageMessage_pb2 import PackageMessage
from CarConfig_pb2 import CarConfig
from Setgoalwork_pb2 import Setgoalwork
import threading
from ControlThread  import ControlCar
from geometry_msgs.msg import Point, Vector3,Twist
import MySQLdb
from std_msgs.msg import Bool,String,Float32,Int32,String
import time
import uuid
from AllParameter import SERVICEPUBIP, SERVICESUBIP, AllTopic, ORDER, SENDER, AllWork,\
    RECVER, Manualorder, Configorder
import copy
from tf_pose import TfRelation
from initpose import initrobotpose
from CtrlCar import TkCtrl
#from curses.ascii import ctrl
#from transform_utils import quat_to_angle
import ConfigParser
#from ConfigIni import Db_Connector

ObstacleDistance = 6.0                                  #障碍物距离
ObstacleEnable_B = False     #激光雷达避障使能
carObstacleEnable_B = False
AmclStatus = False
ObstacleStop = 1.8
ObstacleSlow = 1.8

CON = zmq.Context()
TOKEN = ""                                              #令牌位置
tokenpub = rospy.Publisher("TokenPose", String, queue_size=5)
NowGoal = None  #PackageMessage()


#更新RFID卡在LASER地图上坐标
def InitialMapCallBack(recvmsg):
    #RfidMapPub = rospy.Publisher("RfidMapPose", Vector3, queue_size=5)
    Result = PackageMessage()
    ReturnMsg(Result,recvmsg.SessionId,recvmsg.From)
    Result.ResultMessage.ErrorCode = 0
    try:
        try:
            Sqlcon = MySQLdb.connect(host = '127.0.0.1', user = 'root', passwd = '1', db = 'xrs2015', port = 3306, charset = 'utf8' )
            cur = Sqlcon.cursor()
            if (recvmsg.CallMessage.Parameters[0] != None) and (recvmsg.CallMessage.Parameters[0] != ""):
                fname = recvmsg.CallMessage.Parameters[0]
                if fname == "tmpoint":
                    try:
                        (position, rotation) = tf_relation.get_odom_map()
                    except Exception as err:
                        print "initialpose get_odom_map:",err
                    initrobotpose(position.x, position.y, rotation)            
                    print "LaserControl::InitialMapCallBack fname is :%s"%fname
                else:
                    cur.execute("select lasermapX,lasermapY,lasermapTh from rb_topologypoint where fname ='%s';" %fname)
                    rfidmappose = Vector3()
                    result = cur.fetchall()
                    cur.close()
                    Sqlcon.close()
                    if len(result) != 0:
                        if (result[0][0] != None) and (result[0][1] != None) and (result[0][2] != None):
                            rfidmappose.x = result[0][0]
                            rfidmappose.y = result[0][1]
                            rfidmappose.z = result[0][2]
                            #RfidMapPub.publish(rfidmappose)
                            #if rfid == goal_rfid:
                            initrobotpose(rfidmappose.x, rfidmappose.y, rfidmappose.z)
                            print"LaserControl::InitialMapCallBack rfidmappose", rfidmappose
            else:
                Result.ResultMessage.ErrorCode = 0xd003
        except MySQLdb.Error,e:
            print "LaserControl::InitialMapCallBack MySQL Error %d: %s" %(e.args[0], e.args[1])
        pub = ZmqPubConnect()
        pub.send_multipart([AllTopic[0],'\0',Result.SerializeToString()])
    except Exception as err:
        print "LaserControl::InitialMapCallBack err : ", err
 
#发送障碍物距离   
def SendDisCallBack():   
    #RecvMsg = PackageMessage()
    rospy.Subscriber("obstacle", String, SubObstacleDis)
    #dissub = ZmqSubConnect()
    #dissub.setsockopt(zmq.SUBSCRIBE,AllTopic[1])
    #pub = ZmqPubConnect()
    global ObstacleEnable_B
    global ObstacleStop
    global ObstacleSlow
    global carObstacleEnable_B
    global controlcar
    #rospy.get_param('Lasercontrol/ku', 'kua')
    #rospy.get_param('Lasercontrol/cd', 'cda')
    kufname = 'kua'
    cdfname = 'cda'
    currentfname = cdfname
    nextfname = kufname

    while not rospy.is_shutdown():
        #time.sleep(0.3)
        #topic,re,msg = dissub.recv_multipart()
        #if msg != None:
        #    RecvMsg.ParseFromString(msg)
        #    if RecvMsg.To == SENDER and RecvMsg.CallMessage.Function == ORDER[8]:
        #        Result = PackageMessage()
        #        try:
        #            ReturnMsg(Result, RecvMsg.SessionId, RecvMsg.From)
        #            Result.ResultMessage.ErrorCode = 0
        #            Result.ResultMessage.ResultData = str(ObstacleDistance)
        #            pub.send_multipart([AllTopic[1], '\0', Result.SerializeToString()])
        #            #print Result
        #        except Exception as err:
        #            print "send fail%s" %err
        #else:
        #    continue
        ObstacleEnable_B = True#carObstacleEnable_B
        if currentfname == kufname or currentfname == cdfname or nextfname == kufname or nextfname == cdfname :
            ObstacleEnable_B = False       
        # 当到kua时关闭避障功能
        goalmsg =Setgoalwork()
        global NowGoal
        goal = NowGoal
        try:
            if goal != None:
                #print"LaserControl::SendDisCallBack goal: ",goal
                if ((len(goal.CallMessage.Parameters)) == 0) or (goal.CallMessage.Parameters[0] == ""):
                    controlcar.obstacle_flag = 0
                    continue
                goalmsg.ParseFromString(goal.CallMessage.Parameters[0])
                currentfname = goalmsg.fname
                nextfname = goalmsg.fnameend
        except Exception as err:
            print "LaserControl::SendDisCallBack except close obstacle error: ",err
            controlcar.obstacle_flag = 0
            continue
        #print "LaserControl::SendDisCallBack  ObstacleDistance: ",ObstacleDistance      
        try:
            if ObstacleEnable_B == False:
                controlcar.obstacle_flag = 0
                #print "ObstacleEnable_B-2",ObstacleEnable_B
                continue         
                #print "LaserControl::SendDisCallBack controlcar.slowdown()",ObstacleDistance
                #controlcar.slowdown()
            if ObstacleDistance > 0 and ObstacleDistance <= ObstacleStop:
                controlcar.obstacle_flag = 1
                #print "LaserControl::SendDisCallBack controlcar.carstop()",ObstacleDistance
                #controlcar.carstop()
            if ObstacleDistance > ObstacleStop or ObstacleDistance <= 0:
                controlcar.obstacle_flag = 0
                
            #print"LaserControl::SendDisCallBack controlcar.obstacle_flag: ",controlcar.obstacle_flag
#             ObstacleMsg = PackageMessage()
#             SendMsg(ObstacleMsg,RECVER[0],ORDER[11])
#             ObstacleMsg.CallMessage.Parameters.append(str(ObstacleDistance))
#             pub.send_multipart([AllTopic[1], '\0', ObstacleMsg.SerializeToString()])                       
        except Exception as err:
            print "LaserControl::SendDisCallBack except set controlcar.obstacle_flag error: ",err
            continue
        
#更新令牌位置
def SetTokenCallBack(recvmsg):
    Result = PackageMessage()
    ReturnMsg(Result, recvmsg.SessionId, recvmsg.From)    
    if (recvmsg.CallMessage.Parameters[0] != None) and (recvmsg.CallMessage.Parameters[0] != ""):
        global TOKEN
        TOKEN = "%s" %recvmsg.CallMessage.Parameters[0]
        #print"LaserControl::SetTokenCallBack TOKEN: ",TOKEN
        tokenpub.publish(TOKEN)
        Result.ResultMessage.ErrorCode = 0
    else:
        Result.ResultMessage.ErrorCode = 0xd003
    try:
        pub = ZmqPubConnect()
        pub.send_multipart([AllTopic[0], '\0', Result.SerializeToString()])
    except Exception as err:
        print "LaserControl::SetTokenCallBack except pub.send_multipart()"

#车体控制
def ControlFun(goal):
    #NowGoal = PackageMessage()
    global NowGoal
    NowGoal = copy.deepcopy(goal)
    
    if TOKEN == SENDER:
        try:
            CarControl_Thread = threading.Thread(target=CarControlBack ,args=(NowGoal,))
            CarControl_Thread.setDaemon(True)
            CarControl_Thread.start()
            print "LaserControl::ControlFun CarControl_Thread start"
        except Exception as err:    
            print "LaserControl::ControlFun CarControl_Thread error: ",err
            #return
    else:
        NowGoal.Clear()

def CarControlBack(ngoal):
    global controlcar
    goalmsg =Setgoalwork()
    cotrlgoal = Point()
    #turn_goal = Point()
    last_goal = Point()
    #global NowGoal
    #goal = NowGoal
    goal = ngoal
    #print"LaserControl::CarControlBack goal: ",goal
    #print"LaserControl::CarControlBack the lengh of Parameters: ",len(goal.CallMessage.Parameters)
    if ((len(goal.CallMessage.Parameters)) != 0) and (goal.CallMessage.Parameters[0] != ""):
        try:
            goalmsg.ParseFromString(goal.CallMessage.Parameters[0])
            #print "LaserControl::CarControlBack goalmsg: ",goalmsg
            msql = MySQLdb.connect(host="127.0.0.1", user="root", passwd="1", db="xrs2015", port=3306, charset="utf8")
            cur = msql.cursor()
            cur.execute("select lasermapX,lasermapY from rb_topologypoint where fname='%s';" %(goalmsg.fname))
            result = cur.fetchall() 
            cur.close()
            msql.close()
        except MySQLdb.Error,e:
            print "LaserControl::CarControlBack MySQL Error %d: %s" %(e.args[0], e.args[1])
            
        if len(result) != 0:
            last_goal.x = result[0][0]
            last_goal.y = result[0][1]
        else:
            last_goal.x = 0.0
            last_goal.y = 0.0
        cotrlgoal.x = goalmsg.x
        cotrlgoal.y = goalmsg.y
        #print "LaserControl::CarControlBack goalmsg.work: ",goalmsg.work
    else:
        print "LaserControl::CarControlBack goal.CallMessage.Parameters is None or NULL"
        return
    try:
        if goalmsg.work == AllWork[0] or goalmsg.work == AllWork[5]:
            #print "turn_angle-1"
            controlcar.turn_angle(last_goal,cotrlgoal, goalmsg.fname)
            time.sleep(1.5)
            #print "keepline-1"
            controlcar.keep_line(last_goal, cotrlgoal, goalmsg.work, goalmsg.stop, goalmsg.fnameend, goalmsg.fname)
        elif goalmsg.work == AllWork[1]:
            #print "d_keepline"
            controlcar.turn_angle_back(last_goal,cotrlgoal, goalmsg.fname)
            time.sleep(1.5)
            controlcar.d_keep_line(last_goal, cotrlgoal, goalmsg.work, goalmsg.stop, goalmsg.fnameend, goalmsg.fname)
        else:
            #print "turn_angle-2"
            controlcar.turn_angle(last_goal,cotrlgoal, goalmsg.fname)
            time.sleep(1.5)
            #print "keepline-2"
            controlcar.keep_line(last_goal,cotrlgoal, goalmsg.work, goalmsg.stop, goalmsg.fnameend, goalmsg.fname)
    except Exception as err:    
        print "LaserControl::CarControlBack excepttion error: ",err
    print "LaserControl::CarControlBack The Car ControlThread Done"
    #NowGoal.Clear()
#设置参数接口
def SetParamCallBack(recvmsg):
    return

#实时设置参数接口
def SetParamTimeCallBack(recvmsg):
    return

#手动前进
def ManualCallBack(recvmsg):
        global ctrltk
        global softkeypub
        if recvmsg.CallMessage.Function == Manualorder[0]:
            #ctrltk.forward()
            softkey = String()
            softkey.data = "w";
            softkeypub.publish(softkey)
        elif recvmsg.CallMessage.Function == Manualorder[1]:
            softkey = String()
            softkey.data = "x";
            softkeypub.publish(softkey)
            #ctrltk.speedslow_manual()
        elif recvmsg.CallMessage.Function == Manualorder[2]:
            softkey = String()
            softkey.data = "s";
            softkeypub.publish(softkey)
            #ctrltk.carstop_manual()
        elif recvmsg.CallMessage.Function == Manualorder[3]:
            softkey = String()
            softkey.data = "d";
            softkeypub.publish(softkey)
            #ctrltk.turnright()
        elif recvmsg.CallMessage.Function == Manualorder[4]:
            #ctrltk.turnleft()
            softkey = String()
            softkey.data = "a";
            softkeypub.publish(softkey)
        else:
            return
        
def ConfigCallBack(recvmsg):
    #print "LaserControl::ConfigCallBack recvmsg: ",recvmsg
    Configparam = CarConfig()
    Configparam.ParseFromString(recvmsg.CallMessage.Parameters[0])
    #print "LaserControl::ConfigCallBack Parameters: ",recvmsg.CallMessage.Parameters[0]
    print "LaserControl::ConfigCallBack Configparam:",Configparam
    
    ParamPubSetLaserThresholdSlow = rospy.Publisher("ParamPubSetLaserThresholdSlow", Float32, queue_size=2)
    ParamPubSetLaserThresholdSector = rospy.Publisher("ParamPubSetLaserThresholdSector", Int32, queue_size=2)
    
    if Configparam.HasField("MaxSpeed"):
        max_speed = float("%.2f"%Configparam.MaxSpeed)
        rospy.set_param('LaserControl/max_speed', max_speed)
        max_speed = rospy.get_param('LaserControl/max_speed', 0.8)
        print "LaserControl::ConfigCallBack max_speed:",max_speed,"Configparam.MaxSpeed",Configparam.MaxSpeed
        
    if Configparam.HasField("MinSpeed"):
        min_speed = float("%.2f"%Configparam.MinSpeed)
        rospy.set_param('LaserControl/min_speed', min_speed)
        min_speed = rospy.get_param('LaserControl/min_speed', 0.1)
        print "min_speed:",min_speed,"Configparam.min_speed",Configparam.MinSpeed
        
    if Configparam.HasField("AngleSpeed"):
        ang_speed = float("%.2f"%Configparam.AngleSpeed)
        rospy.set_param('LaserControl/ang_speed', ang_speed)
        ang_speed = rospy.get_param('LaserControl/ang_speed', 0.1)
        print "ang_speed:",ang_speed,"Configparam.ang_speed",Configparam.AngleSpeed
        
    if Configparam.HasField("DistanceSlow"):
        distance_slow = float("%.2f"%Configparam.DistanceSlow)
        rospy.set_param('LaserControl/distance_slow', distance_slow)
        distance_slow = rospy.get_param('LaserControl/distance_slow', 1.4)
        print "distance_slow:",distance_slow,"Configparam.ang_speed",Configparam.DistanceSlow
        
    if Configparam.HasField("CtrlEnableBobstacle"):
        global carObstacleEnable_B
        carObstacleEnable_B = bool(Configparam.CtrlEnableBobstacle)
        rospy.set_param('LaserControl/CtrlEnableBobstacle',Configparam.CtrlEnableBobstacle)
        #global ObstacleEnable_B
        carObstacleEnable_B = rospy.get_param('LaserControl/CtrlEnableBobstacle',True)
        print "ObstacleEnable_B",carObstacleEnable_B,"Configparam.CtrlEnableBobstacle",Configparam.CtrlEnableBobstacle
    
    if Configparam.HasField("SetLaserThreshold"):
        SetLaserThreshold = float("%.2f"%Configparam.SetLaserThreshold)
        rospy.set_param('LaserControl/SetLaserThreshold',SetLaserThreshold)
        global ObstacleStop
        ObstacleStop = rospy.get_param('LaserControl/SetLaserThreshold',0.5)
        print "SetLaserThreshold",ObstacleStop,"Configparam.SetLaserThreshold",Configparam.SetLaserThreshold
    
    if Configparam.HasField("SetLaserThresholdStop"):
        SetLaserThresholdStop = float("%.2f"%Configparam.SetLaserThresholdStop)
        rospy.set_param('LaserControl/SetLaserThresholdStop',SetLaserThresholdStop)
        global ObstacleSlow
        ObstacleSlow =  rospy.get_param('LaserControl/SetLaserThresholdStop',1.6)
        print "SetLaserThresholdStop",ObstacleSlow,"Configparam.SetLaserThresholdStop",Configparam.SetLaserThresholdStop
        ParamPubSetLaserThresholdSlow.publish(ObstacleSlow)
    
    if Configparam.HasField("SetLaserThresholdSector"):
        SetLaserThresholdSector = int(Configparam.SetLaserThresholdSector)
        rospy.set_param('LaserControl/SetLaserThresholdSector',SetLaserThresholdSector)
        ObstacleSector = rospy.get_param('LaserControl/SetLaserThresholdSector',19)
        print "SetLaserThresholdSector",ObstacleSector,"Configparam.SetLaserThresholdSector",Configparam.SetLaserThresholdSector
        ParamPubSetLaserThresholdSector.publish(ObstacleSector)

    Result = PackageMessage()
    ReturnMsg(Result,recvmsg.SessionId,recvmsg.From)
    Result.ResultMessage.ErrorCode = 0
    pub = ZmqPubConnect()
    time.sleep(0.1)
    pub.send_multipart([AllTopic[0],'\0',Result.SerializeToString()])
        
def GetConfigCallBack(recvmsg):
        Result = PackageMessage()
        LaserParam = CarConfig()
        LaserParam.MaxSpeed = rospy.get_param('LaserControl/max_speed', 0.8)
        LaserParam.MinSpeed = rospy.get_param('LaserControl/min_speed', 0.1)
        LaserParam.AngleSpeed = rospy.get_param('LaserControl/ang_speed', 0.1)
        LaserParam.DistanceSlow = rospy.get_param('LaserControl/distance_slow', 1.4)
        
        LaserParam.CtrlEnableBobstacle = rospy.get_param('LaserControl/CtrlEnableBobstacle',True)
        #global ObstacleEnable_B
        global carObstacleEnable_B
        LaserParam.CtrlEnableBobstacle = carObstacleEnable_B;
        LaserParam.SetLaserThreshold = rospy.get_param('LaserControl/SetLaserThreshold',0.5)
        LaserParam.SetLaserThresholdStop =  rospy.get_param('LaserControl/SetLaserThresholdStop',1.6)
        LaserParam.SetLaserThresholdSector = rospy.get_param('LaserControl/SetLaserThresholdSector',19)
        
        
        ReturnMsg(Result,recvmsg.SessionId,recvmsg.From)
        Result.ResultMessage.ErrorCode = 0
        Result.ResultMessage.ResultData = LaserParam.SerializeToString()
        pub = ZmqPubConnect()
        time.sleep(0.1)
        pub.send_multipart([AllTopic[0],'\0',Result.SerializeToString()])
#指令词典
ControlOrder = {ORDER[14]:InitialMapCallBack, 
                ORDER[3]:SetParamCallBack,ORDER[4]:SetParamTimeCallBack
                , ORDER[5]:SetTokenCallBack, ORDER[6]:ControlFun, Manualorder[0]:ManualCallBack, Manualorder[1]:ManualCallBack,
                Manualorder[2]:ManualCallBack, Manualorder[3]:ManualCallBack, Manualorder[4]:ManualCallBack, Configorder[0]:ConfigCallBack
                , Configorder[1]:GetConfigCallBack}

#监听障碍物距离
def SubObstacleDis(data):
    global ObstacleDistance
    ObstacleDistance = float(data.data)
    if ObstacleDistance <= 0.0:
        ObstacleDistance = 6.0  

#zmq发布者连接函数
def ZmqPubConnect():
    socket = CON.socket(zmq.PUB)
    socket.connect(SERVICEPUBIP)
    return socket

#zmq监听者连接函数
def ZmqSubConnect():
    socket = CON.socket(zmq.SUB)
    socket.connect(SERVICESUBIP)
    return socket
    
#结果返回赋值函数
def ReturnMsg(Result,sessionid,fromer):
    Result.Token = TOKEN
    Result.SessionId = sessionid
    Result.Time = int(time.time()*100000000)
    Result.From = SENDER
    Result.To = fromer

#发送赋值函数    
def SendMsg(Pubmsg,fromer,order):
    Pubmsg.Token = TOKEN
    Pubmsg.SessionId = '%s' %uuid.uuid1()
    Pubmsg.Time = int(time.time()*100000000)
    Pubmsg.From = SENDER
    Pubmsg.To = fromer
    Pubmsg.CallMessage.Function = order

def AmclState(data):
    global AmclStatus
    AmclStatus = data.data
    print"LaserControl::AmclState AmclStatus: ",AmclStatus 
          
class ini_Connector():
    def __init__(self,config_file_path):
        self.config_file_path = config_file_path
        #print config_file_path
        self.cf = ConfigParser.ConfigParser()
        self.cf.read(self.config_file_path)
        self.sec = self.cf.sections()
        #print 'section:',self.sec
        self.o = self.cf.options(self.sec[0])
        #print 'option:',self.o
        self.v = self.cf.items(self.sec[0])
        #print 'db:',self.v
        
        self.db_pose_x = self.cf.getfloat(self.sec[0],'pose_x')
        self.db_pose_y = self.cf.getfloat(self.sec[0],'pose_y')
        self.db_pose_z = self.cf.getfloat(self.sec[0],'pose_z')
        print "LaserControl::ini_Connector Db_Connector_init"
        #print self.db_pose_x,self.db_pose_y,self.db_pose_z
        
    def set_db(self,section,option,value):
        self.cf.set(section, option, value)
        print 
        self.cf.write(open(self.config_file_path, "w"))     
        
def main():
    rospy.init_node("LaserControl", anonymous=True)
    global AmclStatus
    rospy.Subscriber("AmclState", Bool, AmclState)
    global softkeypub
    softkeypub = rospy.Publisher("soft_key", String, queue_size=5)
    while AmclStatus == False:
        time.sleep(0.5)
    print"LaserControl::main start working...."   
     
    rospy.set_param('LaserControl/max_speed', 0.4)#0.5
    rospy.set_param('LaserControl/min_speed', 0.1)#0.05
    rospy.set_param('LaserControl/ang_speed', 0.1)
    rospy.set_param('LaserControl/distance_slow',1.4)#1.0
    
    rospy.set_param('LaserControl/d_max_speed', -0.3)
    rospy.set_param('LaserControl/d_max_speed', -0.1)
    rospy.set_param('LaserControl/d_ang_speed', -0.1)    # radians per second
    
    rospy.set_param('LaserControl/CtrlEnableBobstacle',False)
    rospy.set_param('LaserControl/SetLaserThreshold',0.5)
    rospy.set_param('LaserControl/SetLaserThresholdStop',1.6)
    rospy.set_param('LaserControl/SetLaserThresholdSector',19)
    
    global carObstacleEnable_B
    global ObstacleEnable_B
    global ObstacleStop
    global ObstacleSlow
    carObstacleEnable_B = rospy.get_param('LaserControl/CtrlEnableBobstacle',True)
    ObstacleStop = rospy.get_param('LaserControl/SetLaserThreshold',0.5)
    ObstacleSlow = rospy.get_param('LaserControl/SetLaserThresholdStop',1.6)
    
    global tf_relation   
    tf_relation = TfRelation()  #TF relation
    global ctrltk 
    ctrltk = TkCtrl()
    
    global controlcar 
    controlcar = ControlCar()
    ctrltk.CarStatu(controlcar)
    #print "LaserControl::main ctrltk and controlcar intial Done"
    ObThread = threading.Thread(target=SendDisCallBack, args=())
    ObThread.setDaemon(True)
    ObThread.start()
    
    #GoseThread = threading.Thread(target=tf_relation.PubPos, args=())
    #GoseThread.setDaemon(True)
    #GoseThread.start()   
    #print "LaserControl::main ObThread and GoseThread  Done"
    
    SynDataBase = PackageMessage()
    SynSock = ZmqPubConnect()
    SendMsg(SynDataBase, RECVER[1], ORDER[10])
    SynSock.send_multipart([AllTopic[0], '\0', SynDataBase.SerializeToString()])  
    RecvMsg = PackageMessage()
    sock = ZmqSubConnect()
    sock.setsockopt(zmq.SUBSCRIBE,"")
    #try:
        #Sqlcon = MySQLdb.connect(host = '127.0.0.1', user = 'root', passwd = '1', db = 'xrs2015', port = 3306, charset = 'utf8' )
        #cur = Sqlcon.cursor()
        #cur.execute("select posex,posey,posez from rb_point where id =1;" )
        #curpose = Vector3()
        #result = cur.fetchall()
        #cur.close()
        #Sqlcon.close()
        #if len(result) != 0:
            #if result[0][0] != None and result[0][1] != None and result[0][2] != None:
                #curpose.x = result[0][0]
                #curpose.y = result[0][1]
                #curpose.z = result[0][2]
                #print "curpose:",curpose.x,curpose.y,curpose.z
                #initrobotpose(curpose.x, curpose.y, curpose.z)
    #except MySQLdb.Error,e:
        #print "MySQL Error %d: %s" %(e.args[0], e.args[1])
        
    while not rospy.is_shutdown():
        #time.sleep(0.01)
        #print "LaserControl::main controlcar.obstacle_flag",controlcar.obstacle_flag
        try:
            topic,re,msg = sock.recv_multipart()  #flags=zmq.NOBLOCK
            if topic == AllTopic[0] or topic == AllTopic[4]:
                if (msg != None) and (msg != "") and (len(msg) != 0):
                    RecvMsg.ParseFromString(msg)
                    #print"LaserControl::main RecvMsg:",RecvMsg
                    if (RecvMsg.To == SENDER or RecvMsg.To == "*") and (RecvMsg.HasField("CallMessage") == True) and (RecvMsg.CallMessage.HasField("Function") == True) and (RecvMsg.CallMessage.Function != ORDER[12]):
                        try:
                            #print"LaserControl::main while RecvMsg:",RecvMsg
                            ControlOrder.get(RecvMsg.CallMessage.Function)(RecvMsg)
                            if RecvMsg.CallMessage.Function == "ConfigCar":
                                print "LaserControl::main The ConfigCar: ",RecvMsg
                        except Exception as err:    
                            print "LaserControl::main ControlOder.get() error is :",err
                            print "LaserControl::main the RecvMsg.CallMessage.Function :",RecvMsg.CallMessage.Function
                            continue
                    #RecvMsg.Clear()
                else:
                    print "LaserControl::main the receive msg is NULL or None"
                    continue
#             else:
#                 repub = ZmqPubConnect()
#                 repub.send_multipart([topic, '\0',msg ])
#                 print "lasercontrol repub",topic,msg
        except Exception as err:    
            print "LaserControl::main except error: ",err
            continue
    #ObThread.join()
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("LaserControl terminated.")
