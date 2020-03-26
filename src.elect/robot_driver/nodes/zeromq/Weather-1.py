#!/usr/bin/env python
# -*- coding: utf-8 -*-
import zmq
import time 
import serial
import re
import ConfigParser
import threading
from PackageMessage_pb2 import PackageMessage
from WeatherMessage_pb2 import WeatherMessage
from time import sleep
from _socket import timeout
from config import MESSAGEPUB,MESSAGESUB,TEMPADD,HUMIADD,SPEEDADD,RAINADD,TIMEADD,L,L_BUF,ORDER,DATA
from _ast import Pass
TEMPADD='B403000000019E6F'
HUMIADD='B40300010001CFAF'
SPEEDADD='B603000000019F8D'
RAINADD='B703000000019E5C'
TIMEADD='B403000000019E6F'
L=['B403000000019E6F','B40300010001CFAF','B603000000019F8D','B703000000019E5C','B403000000019E6F'] 
L_BUF=[0,0,0,0,0]
ORDER=["GetTemp","GetHumi","GetSpeed","GetRain","GetSnow"]
TIME=""
LAt=""
LON=""
NOSV=0
FS=0
GPSerreo=0
def WeatherMsg():
    while True:
        for n in range(5):
            buf=L[n].decode("hex")
            ser.write(buf)
            ss=ser.read(7)
            if ss!='':
                L_BUF[n]=Data(ss)
                ss=0
                sleep(0.5)
            else:
                L_BUF[n]=""
            print L_BUF
        sleep(5)
def crc16(x):
    b = 0xA001
    a = 0xFFFF
    for byte in x:
        a = a^byte
        for i in range(8):
            last = a%2
            a = a>>1
            if last ==1: a = a^b
    aa = '0'*(6-len(hex(a)))+hex(a)[2:]
    ll,hh = int(aa[:2],16),int(aa[2:],16)
    print hex(hh),hex(ll)
    return [hh,ll]
def Data(x):
    cc=(ord(x[3])*256+ord(x[4]))*0.1
    return round(cc,1)
def time(ff):
    hh=int(ff[:2])+8
    if hh>23:
        hh=hh-24
    tt=str(hh)+ff[2:]
    if len(tt)<9:
        tt='0'+tt
    return tt
    
def Gps():
    ser2=serial.Serial("/dev/ttySAC1",9600)
    if ser2 is None:
        print 'serial is error'
    else:
        while 1:
            buf=ser.readline()
            m=re.match(r'\SGNGGA\S',buf)
            if m:
                buf1=buf.split(",")
                TIME=time(buf1[1])  //time
                LAT=buf1[2]  //lat
                LON=buf[4]   //lon                                        
                NOSV=buf[7]   //nosv
                FS=buf[6]   //Fs
            else:
                GPSerreo=1
            
def SandP():
    MessageMsg=PackageMessage()
    PubMsg=PackageMessage()
    SS=WeatherMessage()
    context=zmq.Context()
    Sub=context.socket(zmq.SUB)
    Sub.connect(MESSAGESUB)
    Sub.setsockopt(zmq.SUBSCRIBE,"")
    while True:
        topic,recv,msg=Sub.recv_multipart()
        if topic == "WeatherRpc":
            if msg !=None:   
                MessageMsg.ParseFromString(msg)
                if MessageMsg.To == "Weather":                 
                    PubMsg.Time=int(time.time()*100000000)
                    PubMsg.SessionId=MessageMsg.SessionId
                    PubMsg.From ="Weather"
                    PubMsg.To =MessageMsg.From
                    AA=MessageMsg.CallMessage.Parameters
                    AA.split('') 
                    if MessageMsg.CallMessage.Function=="AutoSwitch":
                        if AA[0]=='1':                  
                            SS.GpsMessage.Time=TIME
                            SS.GpsMessage.Lat=LAT
                            SS.GpsMessage.Lon=LON
                            SS.GpsMessage.Nosv=NOSV
                            SS.GpsMessage.Fs=FS
                            SS.Mode.WorkMode=1
                            SS.Mode.Rate=AA[1]
                            SS.Mode.ErrorCode=0
                            SS.Mode=SS.Mode.SerializeToString()
                            SS.GpsMessage=SS.GpsMessage.SerializeToString()
                            SS.Temp=L_BUF[0]
                            SS.Humi=L_BUF[1]
                            SS.Speed=L_BUF[2]
                            SS.Rain=L_BUF[3]
                            PubMsg.ResultMessage.ResultData=SS.SerializeToString()
                            PubMsg.ResultMessage.ErrorCode=0 
                            Pub=context.socket(zmq.PUB)
                            Pub.connect(MESSAGEPUB)
                            Pub.send_multipart(["WeatherRpc", "\0", PubMsg.SerializeToString()])
                            cf.set("Mode","MODE",AA[0])
                            cf.set("Time","MIN",AA[1])
                            sleep(AA[1]*60)
                        else :
                            SS.Mode.WorkMode=0
                            SS.Mode.ErrorCode=0
                            SS.Mode=SS.Mode.SerializeToString()
                            PubMsg.ResultMessage.ResultData=SS.SerializeToString()
                            PubMsg.ResultMessage.ErrorCode=0 
                            Pub=context.socket(zmq.PUB)
                            Pub.connect(MESSAGEPUB)
                            Pub.send_multipart(["WeatherRpc", "\0", PubMsg.SerializeToString()])
                    elif MessageMsg.CallMessage.Function=="SetServerIp":
                        MESSAGEPUB=AA[0]
                        MESSAGESUB=AA[1]
                        cf.set("ServerIp","MESSAGEPUB",AA[0])
                        cf.set("ServerIp","MESSAGEsUB",AA[1])
                        PubMsg.ResultMessage.ErrorCode=0 
                        Pub=context.socket(zmq.PUB)
                        Pub.connect(MESSAGEPUB)
                        Pub.send_multipart(["WeatherRpc", "\0", PubMsg.SerializeToString()])
                    elif MessageMsg.CallMessage.Function=="AutoSend":
                        SS.GpsMessage.Time=TIME
                        SS.GpsMessage.Lat=LAT
                        SS.GpsMessage.Lon=LON
                        SS.GpsMessage.Nosv=NOSV
                        SS.GpsMessage.Fs=FS
                        SS.Mode.WorkMode=1
                        SS.Mode.Rate=AA[1]
                        SS.Mode.ErrorCode=0
                        SS.Mode=SS.Mode.SerializeToString()
                        SS.GpsMessage=SS.GpsMessage.SerializeToString()
                        SS.Temp=L_BUF[0]
                        SS.Humi=L_BUF[1]
                        SS.Speed=L_BUF[2]
                        SS.Rain=L_BUF[3]
                        PubMsg.CallMessage.Function="AutoSend"
                        PubMsg.CallMessage=SS.SerializeToString()
                        PubMsg.CallMessage.ErrorCode=0 
                        Pub=context.socket(zmq.PUB)
                        Pub.connect(MESSAGEPUB)
                        Pub.send_multipart(["WeatherRpc", "\0", PubMsg.SerializeToString()])
                    elif MessageMsg.CallMessage.Function=="GetAllData":
                        SS.GpsMessage.Time=TIME
                        SS.GpsMessage.Lat=LAT
                        SS.GpsMessage.Lon=LON
                        SS.GpsMessage.Nosv=NOSV
                        SS.GpsMessage.Fs=FS
                        SS.GpsMessage=SS.GpsMessage.SerializeToString()
                        SS.Temp=L_BUF[0]
                        SS.Humi=L_BUF[1]
                        SS.Speed=L_BUF[2]
                        SS.Rain=L_BUF[3]
                        PubMsg.ResultMessage.ResultData=SS.SerializeToString()
                        PubMsg.ResultMessage.ErrorCode=0 
                        Pub=context.socket(zmq.PUB)
                        Pub.connect(MESSAGEPUB)
                        Pub.send_multipart(["WeatherRpc", "\0", PubMsg.SerializeToString()])
                    elif MessageMsg.CallMessage.Function=="GetTemp":
                        if L_BUF[0]!='':
                            SS.Temp=L_BUF
                            PubMsg.ResultMessage.ResultMessage=SS.SerializeToString()
                            PubMsg.ResultMessage.ErrorCode=0
                        else:
                            PubMsg.ResultMessage.ErrorCode=1
                        Pub=context.socket(zmq.PUB)
                        Pub.connect(MESSAGEPUB)
                        Pub.send_multipart(["WeatherRpc", "\0", PubMsg.SerializeToString()])
                    elif MessageMsg.CallMessage.Function=="GetHumi":
                        if L_BUF[1]!='':
                            SS.Temp=L_BUF
                            PubMsg.ResultMessage.ResultMessage=SS.SerializeToString()
                            PubMsg.ResultMessage.ErrorCode=0
                        else:
                            PubMsg.ResultMessage.ErrorCode=1
                        Pub=context.socket(zmq.PUB)
                        Pub.connect(MESSAGEPUB)
                        Pub.send_multipart(["WeatherRpc", "\0", PubMsg.SerializeToString()])
                    elif MessageMsg.CallMessage.Function=="GetSpeed":
                        if L_BUF[2]!='':
                            SS.Temp=L_BUF
                            PubMsg.ResultMessage.ResultMessage=SS.SerializeToString()
                            PubMsg.ResultMessage.ErrorCode=0
                        else:
                            PubMsg.ResultMessage.ErrorCode=1
                        Pub=context.socket(zmq.PUB)
                        Pub.connect(MESSAGEPUB)
                        Pub.send_multipart(["WeatherRpc", "\0", PubMsg.SerializeToString()])
                    elif MessageMsg.CallMessage.Function=="GetRain":
                        if L_BUF[3]!='':
                            SS.Temp=L_BUF
                            PubMsg.ResultMessage.ResultMessage=SS.SerializeToString()
                            PubMsg.ResultMessage.ErrorCode=0
                        else:
                            PubMsg.ResultMessage.ErrorCode=1
                        Pub=context.socket(zmq.PUB)
                        Pub.connect(MESSAGEPUB)
                        Pub.send_multipart(["WeatherRpc", "\0", PubMsg.SerializeToString()])
                    elif MessageMsg.CallMessage.Function=="GetSnow":
                        if L_BUF[4]!='':
                            SS.Temp=L_BUF
                            PubMsg.ResultMessage.ResultMessage=SS.SerializeToString()
                            PubMsg.ResultMessage.ErrorCode=0
                        else:
                            PubMsg.ResultMessage.ErrorCode=1
                        Pub=context.socket(zmq.PUB)
                        Pub.connect(MESSAGEPUB)
                        Pub.send_multipart(["WeatherRpc", "\0", PubMsg.SerializeToString()])
                    elif MessageMsg.CallMessage.Function=="GetTime":
                        if TIME!='':
                            SS.Temp=TIME
                            PubMsg.ResultMessage.ResultMessage=SS.SerializeToString()
                            PubMsg.ResultMessage.ErrorCode=0
                        else:
                            PubMsg.ResultMessage.ErrorCode=1
                        Pub=context.socket(zmq.PUB)
                        Pub.connect(MESSAGEPUB)
                        Pub.send_multipart(["WeatherRpc", "\0", PubMsg.SerializeToString()])
                            
if __name__=="__main__":
    cf=ConfigParser.ConfigParser()
    cf.read("weather.conf")
    MESSAGEPUB=cf.get("ServerIp","MESSAGEPUB")
    MESSAGESUB=cf.get("ServerIp","MESSAGESUB")
    MIN=cf.get("Time","MIN")
    Mode=cf.get("Mode","mode")
    ser=serial.Serial('/dev/ttySAC2',9600,timeout=6)   
    t1=threading.Thread(target=WeatherMsg)
    t1.start()
    t2=threading.Thread(target=SandP)
    t2.start()
    t3=threading.Thread(target=Gps)
    t3.start()