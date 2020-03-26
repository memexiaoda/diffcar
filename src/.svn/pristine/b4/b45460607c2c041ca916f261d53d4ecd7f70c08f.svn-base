#!/usr/bin/env python
# -*- coding: utf-8 -*-
import zmq
import time 
import serial
import threading
from PackageMessage_pb2 import PackageMessage
from WeatherMessage_pb2 import WeatherMessage
from time import sleep
from _socket import timeout
from config import MESSAGEPUB,MESSAGESUB,TEMPADD,HUMIADD,SPEEDADD,RAINADD,TIMEADD,L,L_BUF,ORDER,DATA
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
                L_BUF[n]=0
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
                    PubMsg.ResultMessage.ErrorCode=0
                    if MessageMsg.CallMessage.Function!='GetallData':                    
                        for i in range(5):
                            if MessageMsg.CallMessage.Function==ORDER[i]:
                                PubMsg.ResultMessage.ResultData=str(L_BUF[i])
                    else:
                        SS.Time=int(time.time()*100000000)
                        SS.Temp=L_BUF[0]
                        SS.Humi=L_BUF[1]
                        SS.Speed=L_BUF[2]
                        SS.Rain=L_BUF[3]
                        PubMsg.ResultMessage.ResultData=SS.SerializeToString()
    
                Pub=context.socket(zmq.PUB)
                Pub.connect(MESSAGEPUB)
                Pub.send_multipart(["WeatherRpc", "\0", PubMsg.SerializeToString()])                                         
if __name__=="__main__":
    ser=serial.Serial('/dev/ttyUSB0',9600,timeout=6)   
    t1=threading.Thread(target=WeatherMsg)
    t1.start()
    t2=threading.Thread(target=SandP)
    t2.start()