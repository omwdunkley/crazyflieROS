#!/usr/bin/env python
import roslib; 
roslib.load_manifest("crazyflieROS")
import rospy

from sensor_msgs.msg import Joy as JoyMSG
from crazyflieROS.msg import cmd as CFJoyMSG
from crazyflieROS.cfg import cmdJoyConfig as CFJoyCFG
from dynamic_reconfigure.server import Server as DynamicReconfigureServer
import tf
import tf.msg as TFMSG

from optparse import OptionParser
import time, sys
from math import pi as PI
from math import sqrt, sin, cos, degrees, radians, atan2, atan
import numpy as np


'''
    TODO:
        1) drop down menus to assign axis/buttons
        2) load/save default config
        3) non linear input
        4) implement rumblepack options
        5) battery warning on low battery
'''

def enum(*sequential, **named):
    enums = dict(zip(sequential, range(len(sequential))), **named)
    reverse = dict((value, key) for key, value in enums.iteritems())
    enums['to_str'] = reverse
    return type('Enum', (), enums)

Button = enum(L1=10,R1=11,Select=0,Start=3,L2=8,R2=9,Up=4,Right=5,Down=6,Left=7,Square=15,Cross=14,Triangle=12,Circle=13)
Axes = enum(SLL=0,SLU=1,SRL=2,SRU=3,Up=4,Right=5,Down=6,Left=7,L2=8,R2=9,L1=10,R1=11,Triangle=12,Circle=13,Cross=14,Square=15,AccL=16,AccF=17,AccU=18,GyroY=19)
#Button = enum(L1=10,R1=11,Select=0,Start=3,L2=8,R2=9,Up=4,Right=5,Down=6,Left=7,Square=15,Cross=14,Triangle=12,Circle=13)
#Axes = enum(SLL=0,SLU=1,SRL=2,SRU=3,Up=4,Right=5,Down=6,Left=7,L2=4+8,R2=4+9,L1=4+10,R1=4+11,Triangle=4+12,Circle=4+13,Cross=4+14,Square=4+15,AccL=16,AccF=17,AccU=18,GyroY=19)


MAX_THRUST = 60000.


# TODO:shouldnt min thrust be subtracted?
def thrustToPercentage( thrust):
    return ((float(thrust)/MAX_THRUST)*100.0)

def percentageToThrust( percentage):
    return int(MAX_THRUST*(percentage/100.0))

def deadband(value, threshold):
    if abs(value)<threshold:
        return 0.0
    elif value>0:
        return value-threshold
    else:
        return value+threshold
   
        

class JoyController:
    def __init__(self,options=None):     
        self.options = options
           
        self.joy_scale = [-1,1,-1,1,1] #RPYTY
        self.trim_roll = 0
        self.trim_pitch = 0
        self.max_angle = 30
        self.max_yawangle = 200
        
        
        self.max_thrust = 80.
        self.min_thrust = 25.
        self.max_thrust_raw = percentageToThrust(self.max_thrust)
        self.min_thrust_raw = percentageToThrust(self.min_thrust)       
        self.old_thurst_raw = 0
        
        self.slew_limit = 45
        self.slew_rate = 30
        self.slew_limit_raw = percentageToThrust(self.slew_limit)            
        self.slew_rate_raw = percentageToThrust(self.slew_rate)   
        
        self.dynserver = None
        self.prev_cmd = None
        self.curr_cmd = None
        self.yaw_joy = True
        
        self.sub_joy   = rospy.Subscriber("/joy", JoyMSG, self.new_joydata)
        self.sub_tf    = tf.TransformListener()         
        self.pub_tf    = tf.TransformBroadcaster()    
        self.pub_cfJoy = rospy.Publisher("/cfjoy", CFJoyMSG)
        
        # Dynserver               
        
        self.dynserver = DynamicReconfigureServer(CFJoyCFG, self.reconfigure)
        
        
    def released(self, id):
        return self.prev_cmd.buttons[id] and not self.curr_cmd.buttons[id]
    
    def pressed(self, id):
        return not self.prev_cmd.buttons[id] and self.curr_cmd.buttons[id]
    
    def held(self, id):
        return self.prev_cmd.buttons[id] and self.curr_cmd.buttons[id]    
        
    #Needed so the class can change the dynserver values with button presses    
    def set_dynserver(self, server):
        self.dynserver = server
        
        
    def thurstToRaw(self, joy_thrust):           
        
        # Deadman button or invalid thrust
        if not self.curr_cmd.buttons[Button.L1] or joy_thrust>1:
            return 0
        
        raw_thrust = 0
        if joy_thrust > 0.01:
            raw_thrust = self.min_thrust_raw + joy_thrust*(self.max_thrust_raw-self.min_thrust_raw)
              
        
        if self.slew_rate_raw>0 and self.slew_limit_raw > raw_thrust:
            if self.old_thurst_raw > self.slew_limit_raw:
                self.old_thurst_raw = self.slew_limit_raw
            if raw_thrust < (self.old_thurst_raw - (self.slew_rate_raw/100)):
                raw_thrust = self.old_thurst_raw - self.slew_rate_raw/100
            if joy_thrust < 0 or raw_thrust < self.min_thrust_raw:
                raw_thrust = 0
        self.old_thurst_raw = raw_thrust
        
        return raw_thrust        
    
    def new_joydata(self, joymsg):
        global Button
        global Axes
        #Should run at 100hz. Use launch files roslaunch crazyflie_ros joy.launch
        if self.prev_cmd == None:
            self.prev_cmd = joymsg
            print len(joymsg.axes)
            
            #USB mode
            if len(joymsg.axes) == 27:
                Button = enum(L1=10,R1=11,Select=0,Start=3,L2=8,R2=9,Up=4,Right=5,Down=6,Left=7,Square=15,Cross=14,Triangle=12,Circle=13)
                Axes = enum(SLL=0,SLU=1,SRL=2,SRU=3,Up=4,Right=5,Down=6,Left=7,L2=4+8,R2=4+9,L1=4+10,R1=4+11,Triangle=4+12,Circle=4+13,Cross=4+14,Square=4+15,AccL=16,AccF=17,AccU=18,GyroY=19)   
            return 
        
        self.curr_cmd = joymsg
        hover = False
        
        
#        print "AXES"
#        for i,x in enumerate(joymsg.axes):
#            print " ",i,":",x
#        print "BUTTONS
#        for i,x in enumerate(joymsg.buttons):
#            print " ",i,":",x
            
        x = 0
        y = 0
        z = 0
        r = 0
        r2 = 0
        # Get stick positions [-1 1]
        if self.curr_cmd.buttons[Button.L1]:         
            x = self.joy_scale[0] * self.curr_cmd.axes[Axes.SLL] # Roll
            y = self.joy_scale[1] * self.curr_cmd.axes[Axes.SLU] # Pitch
            r = self.joy_scale[2] * self.curr_cmd.axes[Axes.SRL] # Yaw
            z = self.joy_scale[3] * self.curr_cmd.axes[Axes.SRU] # Thrust            
            r2 = self.joy_scale[4] * (self.curr_cmd.axes[Axes.L2] - self.curr_cmd.axes[Axes.R2])
            hover = self.curr_cmd.axes[Axes.L1]<-0.75
        
        roll = x * self.max_angle
        pitch = y * self.max_angle        
        yaw = 0
        
        
        #TODO use dyn reconf to decide which to use
        if r2!=0:
            yaw = r2 * self.max_yawangle
        else:
            if self.yaw_joy:
                # Deadzone     
                if r < -0.2 or r > 0.2:
                    if r < 0:
                        yaw = (r + 0.2) * self.max_yawangle * 1.25
                    else:
                        yaw = (r - 0.2) * self.max_yawangle * 1.25
                
                
        if hover:
            thrust = int(round(deadband(z,0.2)*32767 + 32767)) #Convert to uint16
        else:    
            thrust = self.thurstToRaw(z)  
            
        trimmed_roll = roll + self.trim_roll
        trimmed_pitch = pitch + self.trim_pitch
        
            
        # Control trim manually        
        new_settings = {}       
        if self.curr_cmd.buttons[Button.Left]:
            new_settings["trim_roll"] = max(self.trim_roll + self.curr_cmd.axes[Axes.Left]/10.0, -10)                         
        if self.curr_cmd.buttons[Button.Right]:
            new_settings["trim_roll"] =  min(self.trim_roll - self.curr_cmd.axes[Axes.Right]/10.0, 10)  
        if self.curr_cmd.buttons[Button.Down]:
            new_settings["trim_pitch"] = max(self.trim_pitch + self.curr_cmd.axes[Axes.Down]/10.0, -10)                
        if self.curr_cmd.buttons[Button.Up]:
            new_settings["trim_pitch"] = min(self.trim_pitch - self.curr_cmd.axes[Axes.Up]/10.0, 10)
        
        # Set trim to current input
        if self.released(Button.R1):
            new_settings["trim_roll"] = min(10, max(trimmed_roll, -10))
            new_settings["trim_pitch"] = min(10, max(trimmed_pitch, -10))   
            rospy.loginfo("Trim updated Roll/Pitch: %r/%r", round(new_settings["trim_roll"],2), round(new_settings["trim_roll"],2))
        
        # Reset Trim
        if self.released(Button.Square):
            new_settings["trim_roll"] = 0
            new_settings["trim_pitch"] = 0       
            rospy.loginfo("Pitch reset to 0/0")        
            
        if new_settings != {} and self.dynserver!=None:
            self.dynserver.update_configuration(new_settings)            
       
        
        
        # TODO: new firmware doesnt need hover_change, it just uses thrust directly
        #(trimmed_roll,trimmed_pitch,yaw,thrust,hover,hover_set, z)
        msg = CFJoyMSG()
        msg.header.stamp = rospy.Time.now()
        msg.roll = trimmed_roll
        msg.pitch = trimmed_pitch
        msg.yaw = yaw
        msg.thrust = thrust
        msg.hover = hover
        #msg.calib = self.pressed(Button.Triangle)
        self.pub_cfJoy.publish(msg)       
        
        
        # Cache prev joy command
        self.prev_cmd = self.curr_cmd
        
    
    def reconfigure(self, config, level):
        self.trim_roll = config["trim_roll"]
        self.trim_pitch = config["trim_pitch"]
        self.max_angle = config["max_angle"]
        self.max_yawangle = config["max_yawangle"]
        self.max_thrust = config["max_thrust"]
        self.min_thrust = config["min_thrust"]
        self.slew_limit = config["slew_limit"]
        self.slew_rate = config["slew_rate"]  
        self.max_thrust_raw = percentageToThrust(self.max_thrust)
        self.min_thrust_raw = percentageToThrust(self.min_thrust)
        self.slew_limit_raw = percentageToThrust(self.slew_limit)
        self.slew_rate_raw = percentageToThrust(self.slew_rate) 
        self.yaw_joy =  config["yaw_joy"]            
        return config 

        

def run(args=None):    
    parser = OptionParser(description='Crazyflie ROS Driver')
#     parser.add_option('--uri', '-u',
#                         dest='uri',
#                         default='radio://0/4',
#                         help='URI to connect to')    
    (options, leftargs) = parser.parse_args(args=args)
        
    #Load some DB depending on options 
    rospy.init_node('CrazyFlieJoystickDriver')   
    joydriver = JoyController(options)

    #START NODE
    try:
        rospy.loginfo("Starting CrazyFlie joystick driver")
        rospy.spin()
    except KeyboardInterrupt:    
        rospy.loginfo('...exiting due to keyboard interrupt')


if __name__ == "__main__":   
    run(sys.argv)
