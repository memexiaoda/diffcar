#!/usr/bin/env python 
# -*- coding: utf-8 -*-

from __future__ import division

import rospy
import serial  

import RPi.GPIO as GPIO
import binascii
import time
from std_msgs.msg import String,Float32


EN,R,G,B = 18,23,24,25

#GPIO.setmode(GPIO.BCM)
#ser = serial.Serial('/dev/ttyAMA0', 9600)

led_flag = 0

def initial():
    
    global led_flag
    
    GPIO.setmode(GPIO.BCM)

    GPIO.setup(EN, GPIO.OUT)
    GPIO.setup(R, GPIO.OUT)
    GPIO.setup(G, GPIO.OUT)
    GPIO.setup(B, GPIO.OUT)
    GPIO.output(R,True)
    time.sleep(0.25)
    GPIO.output(R,False)
    time.sleep(0.25)

    GPIO.output(G,True)
    time.sleep(0.25)
    GPIO.output(G,False)
    time.sleep(0.25)

    GPIO.output(B,True)
    time.sleep(0.25)
    GPIO.output(B,False)
    time.sleep(0.25)
     
    GPIO.output(EN, False)
    
    led_flag = 1

    print"serial_led::initial done"
#    command_str = "01100050000102010680a8"

#    GPIO.output(EN, True)
#    time.sleep(0.05)

#    ser.write(command_str)
#    print"send the initial command_str:",command_str    
#    time.sleep(0.01)
#    GPIO.output(EN, False)


def send_command(str):

    global ser
    
    if (str):
        ser.flushOutput()
        
        GPIO.output(EN, True)
        time.sleep(0.05)
        
        ser.write(str)

	    #print"serial_led::send_command send the command : ",binascii.b2a_hex(str)
        time.sleep(0.01)

	GPIO.output(EN, False)



def recv_data():
    
 #   ser.flushInput()
    global ser
    try:
        GPIO.output(EN, False)
        time.sleep(0.05)
        
        count = ser.inWaiting()  
        #print"serial_led::recv_data the count : ",count

        if count != 0:
            
            recv = ser.read(count)
            str_recv = binascii.b2a_hex(recv)
            
            #print"serial_led::recv_data receive the data: %s" % str_recv

            ser.flushInput()  

            return str_recv[6:10]

    except Exception as err:
            print"serial_led::recv_data recv_data() => read serial exception error:" ,err


def led():

    global led_flag
    
    #print "serial_led::led the led_flag:",led_flag

    if (led_flag == 1):
        GPIO.output(R,True)
        time.sleep(0.5)
        GPIO.output(R,False)
        time.sleep(0.5)
	
    elif(led_flag == 2):
        GPIO.output(G,True)
        time.sleep(0.5)
        GPIO.output(G,False)
        time.sleep(0.5)

    elif(led_flag == 3):
        GPIO.output(B,True)
        time.sleep(0.5)
        GPIO.output(B,False)
        time.sleep(0.5)

    else:
        GPIO.output(R,False)
        GPIO.output(G,False)
        GPIO.output(B,False)

def main(): 

    rospy.init_node("LedControl", anonymous=False)
    
    battery_Pub = rospy.Publisher("Battery_state", String, queue_size=5)

    battery_PubU = rospy.Publisher("Battery_stateU", Float32, queue_size=5)
    
    global led_flag
    
    global ser
    
    k = 0
    U_average = 0.0
    Value = 0.0
    
    initial()
        
    while not rospy.is_shutdown():
        ser = serial.Serial('/dev/ttyAMA0', 9600)
        if not ser.isOpen():
            print "serial_led::main Open the serial False"
            time.sleep(2)
            continue
        else:
            break        	

    ser.flushInput()
    
    while not rospy.is_shutdown():
        
        command_u = "\x01\x03\x00\x00\x00\x01\x84\x0a"
        
        try:
            send_command(command_u)
    
            data_u = recv_data()
    
            if (data_u):
                data_uu = int(data_u ,16) 
                value_u = (data_uu*3)/1000
    	        print"serial_led::main the data_uu is ",value_u
                
                battery_PubU.publish(value_u)
                
                k += 1
                Value += value_u
                
                if (k == 5):
                    U_average = Value/k
                    k = 0
                    Value = 0.0
                    
                    if (U_average < 12.3):
                        led_flag = 2
                        Send_charge = String()
                        Send_charge = "charging"
                        battery_Pub.publish(Send_charge)
                        print "serial_led::main(): the U_average is less 12.3 and need to charging"
    	    else:
                print "serial_led::main(): the receive data_u is null"
    #		    Send_charge = String()
    #        	Send_charge = "nocharging"
    #		    battery_Pub.publish(Send_charge)
    
    #	    time.sleep(0.5)
    
    
    #	    send_command(command_i)
    
    #	    data_i = recv_data()
    
    #	    if (data_i):
    #	        data_ii = int(data_i ,16)
    #	        value_i = data_ii/1000
    #	        print"the data_ii is ",data_ii , value_i	
    	   
    #	        if(value_i < 0.05):
    #		    led_flag = 2
    #		    Send_charge = String()
    #        	    Send_charge = "charging"
    #		    battery_Pub.publish(Send_charge)
    #	        else:
    #		    pass
    
    #	    time.sleep(0.5)
    
    #           send_command(command_p)
    #           data_p = recv_data()
    
    #           if (data_p):
    #               data_pp = int(data_p ,16)
    #               value_p = (data_pp*3)/100
    #               print"the data_pp is ",data_pp , value_p
    
    #               if(value_p < 100):
    #                   led_flag = 2
    #               else:
    #                   pass	
    	except Exception as err:
            
            print"serial_led::main() => read or send serial exception error:" ,err
        
        led()

        time.sleep(1.5) 
            
    #GPIO.cleanup()
    ser.close()

if __name__ == '__main__':  

    try:  

        main() 
 
    except Exception as err:

        print "serial_led exception is ",err 
	#GPIO.cleanup()
        if ser != None:  

            ser.close() 

