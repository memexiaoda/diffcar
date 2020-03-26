#!/usr/bin/env python
#-*- coding:utf-8 -*-

#import sys,os
#import ConfigParser
import rospy
from ConfigIni import Db_Connector, Db_RawConnector, Db_SafeConfigParser, Db_ConfigObj
#class Db_Connector():
    #def __init__(self,config_file_path):
        #self.config_file_path = config_file_path
        #print config_file_path
        #self.cf = ConfigParser.ConfigParser()
        #self.cf.read(config_file_path)
        
        #self.sec = self.cf.sections()
        #print 'section:',self.sec
        
        #self.opts = self.cf.options(self.sec[0]) 
        #print 'sec_a,options:',self.opts
        
        #self.kvs = self.cf.items(self.sec[0]) 
        #print 'sec_b:',self.kvs
        
        #read by type
        #str_val = self.cf.getint(self.sec[1], "b_key1") 
        #int_val = self.cf.get(self.sec[1], "b_key2")
        
        #print "value for sec_a's a_key1:" ,str_val
        #print "value for sec_a's a_key2:", int_val
        
        #self.cf.set("sec_b","b_key3","new-$r")
        #set a new value
        #self.cf.set("sec_b","b_newkey", "new-value") 
        #create a new section
        #self.cf.add_section('a_new_section') 
        #self.cf.set('a_new_section', 'new_key', 'new_value') 
        
        #write back to configure file
        #self.cf.write(open(config_file_path, "w")) 
        #wirte config
        #update vaule
        #self.cf.set("sec_b", option, value)
        
if __name__ == '__main__':
    #rospy.init_node('conftest', anonymous=False)
    try:
       
#         global fdb
#         fdb = Db_Connector("initpose.conf")
#         fdb.db_pose_x = fdb.cf.getfloat(fdb.sec[0],'pose_x')
#         fdb.db_pose_y = fdb.cf.getfloat(fdb.sec[0],'pose_y')
#         fdb.db_pose_z = fdb.cf.getfloat(fdb.sec[0],'pose_z')
#         print "set_initial",fdb.db_pose_x,fdb.db_pose_y,fdb.db_pose_z
        #initrobotpose(fdb.db_pose_x,fdb.db_pose_y,fdb.db_pose_z)
         global fdb
         fdb = Db_ConfigObj("initpose.conf")
    except:
        print "Connector except"
    #rospy.spin()
