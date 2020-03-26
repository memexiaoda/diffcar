#!/usr/bin/env python
# -*- coding:utf-8 -*-

import sys,os
import ConfigParser
#import RawConfigParser
#import SafeConfigParser
from configobj import ConfigObj

class Db_ConfigObj():
    def __init__(self,config_file_path):
        self.config_file_path = config_file_path
        print config_file_path
        self.cf  = ConfigObj(self.config_file_path,encoding='UTF8')
        try:
            self.sec = self.cf.keys()
            print 'section:',self.sec
            #self.db_pose_x = self.cf.keys()
            #self.db_pose_y = ConfigObj["initialpose"]["pose_x"]
        except:
            print "Db_ConfigObj cxcept"
class Db_SafeConfigParser():
    def __init__(self,config_file_path):
        self.config_file_path = config_file_path
        print config_file_path
        self.cf = ConfigParser.ConfigParser()
        self.cf.read(self.config_file_path)
        try:
            self.sec = self.cf.sections()
            print 'section:',self.sec
            self.o = self.cf.options(self.sec[0])
            print 'option:',self.o
            self.v = self.cf.items(self.sec[0])
            print 'db:',self.v
        
            self.db_pose_x = self.cf.getfloat(self.sec[0],'pose_x')
            self.db_pose_y = self.cf.getfloat(self.sec[0],'pose_y')
            self.db_pose_z = self.cf.getfloat(self.sec[0],'pose_z')
            print "Db_SafeConfigParser_init"
            print self.db_pose_x,self.db_pose_y,self.db_pose_z
        except:
            print "Db_RawConnector cxcept"
            
class Db_RawConnector():
    def __init__(self,config_file_path):
        self.config_file_path = config_file_path
        print config_file_path
        self.cf = ConfigParser.ConfigParser()
        self.cf.read(self.config_file_path)
        try:
            self.sec = self.cf.sections()
            print 'section:',self.sec
            self.o = self.cf.options(self.sec[0])
            print 'option:',self.o
            self.v = self.cf.items(self.sec[0])
            print 'db:',self.v
        
            self.db_pose_x = self.cf.getfloat(self.sec[0],'pose_x')
            self.db_pose_y = self.cf.getfloat(self.sec[0],'pose_y')
            self.db_pose_z = self.cf.getfloat(self.sec[0],'pose_z')
            print "Db_RawConnector_init"
            print self.db_pose_x,self.db_pose_y,self.db_pose_z
        except:
            print "Db_RawConnector cxcept"
                    
class Db_Connector():
    def __init__(self,config_file_path):
        self.config_file_path = config_file_path
        print config_file_path
        self.cf = ConfigParser.ConfigParser()
        self.cf.read(self.config_file_path)
        self.sec = self.cf.sections()
        print 'section:',self.sec
        self.o = self.cf.options(self.sec[0])
        print 'option:',self.o
        self.v = self.cf.items(self.sec[0])
        print 'db:',self.v
        
        self.db_pose_x = self.cf.getfloat(self.sec[0],'pose_x')
        self.db_pose_y = self.cf.getfloat(self.sec[0],'pose_y')
        self.db_pose_z = self.cf.getfloat(self.sec[0],'pose_z')
        print "Db_Connector_init"
        print self.db_pose_x,self.db_pose_y,self.db_pose_z
        
    def set_db(self,section,option,value):
        self.cf.set(section, option, value)
        self.cf.write(open(self.config_file_path, "w"))
