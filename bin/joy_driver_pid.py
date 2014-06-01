#!/usr/bin/env python
import roslib;
roslib.load_manifest("crazyflieROS")
import rospy

from sensor_msgs.msg import Joy as JoyMSG
from crazyflieROS.msg import cmd as CFJoyMSG
from crazyflieROS.msg import pid as PidMSG
from crazyflieROS.cfg import cmdJoyPIDConfig as CFJoyCFG
from dynamic_reconfigure.server import Server as DynamicReconfigureServer
from geometry_msgs.msg import PoseStamped as PoseMSG
import tf
import tf.msg as TFMSG
import os

from optparse import OptionParser
import time, sys
from math import pi as PI
from math import sqrt, sin, cos, degrees, radians, atan2, atan, copysign, asin, acos
import numpy as np
import time

# print 'Python Path: %s' % os.environ['PYTHONPATH'].split(os.pathsep)


def fATAN(dist, r):  return degrees(atan(dist))*sqrt(r)
def fASIN (dist, r): return degrees(asin(min(1.,max(-1, dist))))*r
def fLIN(dist, mag):   return dist*mag


tid = 0
a = 0.75
y = -90



# cam simple with rotation
trajectory = [(0.5,2.0,0.5,90), # take off
			  (0.5,2.0,1.5,90), # init
              (0.5,2.0,1.0,90), # level - hover
			  (1.5,2.0,1.0,90), # move x
  			  (0.5,2.0,1.0,90+45), #move back rotating 45
  			  (1.5,2.0,1.0,45), # move x rotating -90
  			  (0.5,2.0,1.0,90)] #move back rotaint 45


# updown, 1 meter
trajectory = [(0,0,0.5,0),
			  (0,0,1.5,0)  
              ]



# Left-right Trajectory. ALigned and 45degrees
trajectory = [(1.5,0,1,0),  #go left
              (-1.5,0,1,0), #go right
              (-1.5,0,1,45), #rotate
              (1.5,0,1,45), #go left
              (-1.5,0,1,45), #go right
		      (-1.5,0,1,0), #rotate back
              ]

# 3d circuit
U = 1.5
D = 0.5
L = -1.25
R = -L
F = 1.25
B = -F
rot = 0
M = 0
trajectory = [(M,F,D,rot), (L,F,D,rot), (L,B,D,rot), (R,B,D,rot), (R,F,D,rot), (R,F,U,rot), (L,F,U,rot), (L,B,U,rot), (M,B,U,rot), (M,M,U,rot), (M,M,1.0,rot),(M,M,0.5,rot),(M,M,0.2,rot)]




def enum(*sequential, **named):
    enums = dict(zip(sequential, range(len(sequential))), **named)
    reverse = dict((value, key) for key, value in enums.iteritems())
    enums['to_str'] = reverse
    return type('Enum', (), enums)

Button = enum(L1=10,R1=11,Select=0,Start=3,L2=8,R2=9,Up=4,Right=5,Down=6,Left=7,Square=15,Cross=14,Triangle=12,Circle=13)
#Axes = enum(SLL=0,SLU=1,SRL=2,SRU=3,Up=4,Right=5,Down=6,Left=7,L2=8,R2=9,L1=10,R1=11,Triangle=12,Circle=13,Cross=14,Square=15,AccL=16,AccF=17,AccU=18,GyroY=19)#default bluez + sixaxis
Axes = enum(SLL=0,SLU=1,SRL=2,SRU=3,Up=8,Right=9,Down=10,Left=11,L2=12,R2=13,L1=14,R1=15,Triangle=12,Circle=13,Cross=14,Square=15,AccL=4,AccF=5,AccU=6)#https://help.ubuntu.com/community/Sixaxis#Quick_Setup_Guide_for_12.10%3C/p%3E
Source = enum(Qualisys=0,Cam=1,Synthetic=2)
#Button = enum(L1=10,R1=11,Select=0,Start=3,L2=8,R2=9,Up=4,Right=5,Down=6,Left=7,Square=15,Cross=14,Triangle=12,Circle=13)
#Axes = enum(SLL=0,SLU=1,SRL=2,SRU=3,Up=4,Right=5,Down=6,Left=7,L2=4+8,R2=4+9,L1=4+10,R1=4+11,Triangle=4+12,Circle=4+13,Cross=4+14,Square=4+15,AccL=16,AccF=17,AccU=18,GyroY=19)



MAX_THRUST = 60000.
def percentageToThrust( percentage):
    return int(MAX_THRUST*(percentage/100.0))

def deadband(value, threshold):
    if abs(value)<threshold:
        return 0.0
    elif value>0:
        return value-threshold
    else:
        return value+threshold

class PID:
    """ Classic PID Controller """
    def __init__(self, P=1.0, I=0.0, D=0.0):
        self.P = P
        self.I = I
        self.D = D

        self.maxP = sys.float_info.max
        self.maxI = sys.float_info.max
        self.maxD = sys.float_info.max
        self.maxTotal = sys.float_info.max

        # Useful for I part
        self.error_sum = 0.0

        # Useful for D part
        self.last_time = rospy.Time.now()
        self.last_error = 0.0# sys.float_info.max
        self.last_output = 0.0
        return

    def get_command(self, error, rtime, d=None, i=None, f=lambda x: x):
        dt = (rtime-self.last_time).to_sec()


        if not dt>0:
            if dt==0:
                P = f(self.P * error)
                I = f(self.I * self.error_sum)
                self.last_time  = rtime
                self.last_error = error
                total = P+I
                self.last_output = copysign(min(abs(total), self.maxTotal), total)
                return self.last_output, P, I, 0
            rospy.logwarn("Negative time in PID controller: "+str(dt)+"s")
            return 0,0,0,0; #todo check -> probably reset stzff
        #TODO: add max limits



        P = f(self.P * error)



        if i is None:
            self.error_sum += (self.last_error+error)/2. * dt #trapazoidal integration
            I = f(self.I * self.error_sum)
        else:
            self.error_sum = i
            I = f(self.I * self.error_sum)

        if d is None:
            D = f(self.D * ((self.last_error-error) / dt))
            alpha = 0.5
            self.last_error = self.last_error*alpha + error*(1.-alpha)
        else:
            D = f(self.D * d)
            self.last_error = 0

        # Cache time for computing dt in next step
        self.last_time  = rtime

        # Compute command
        total = P+I-D
        self.last_output = copysign(min(abs(total), self.maxTotal), total)
        return self.last_output, P, I ,D

    def set_gains(self, P,I,D):
        self.P = P
        self.I = I
        self.D = D
        return

    def set_limits(self, maxTotal, maxP = float("inf"), maxI = float("inf"), maxD = float("inf")):
        self.maxP = maxP
        self.maxI = maxI
        self.maxD = maxD
        self.maxTotal = maxTotal

    def reset(self,time=None):
        # Useful for I part
        self.error_sum = 0.0

        # Useful for D part
        if time is None:
            self.last_time = rospy.Time.now()
        else:
            self.last_time = time
        self.last_error = 0.0#sys.float_info.max #why max??
        self.last_output = 0.0





def numpy2python(l):
    if isinstance(l,list):
        return [np.asscalar(e) if isinstance(e, np.generic) else e for e in l]
    if isinstance(l,tuple):
        return tuple([np.asscalar(e) if isinstance(e, np.generic) else e for e in l])
    if isinstance(l,dict):
        d = {}
        for k, v in l.iteritems():
            d[np.asscalar(k) if isinstance(k, np.generic) else k] = np.asscalar(v) if isinstance(v, np.generic) else v
        return d



class JoyController:
    def __init__(self,options=None):     
        self.options = options
        self.warntime = time.time()
        self.USB = False
           
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
        self.pub_cfJoy = rospy.Publisher("/cfjoy", CFJoyMSG, queue_size=50)
        self.pub_PID = rospy.Publisher("/pid", PidMSG,queue_size=50)

        # PIDs for R,P,Y,THROTTLE
        self.PID = (PID(), PID(), PID(), PID()) # XY, Throttle, Yaw
        self.PIDActive = (True, True, True, True) # XY, YAw, Throttle on/off
        self.PIDDelay = 0.0
        self.PIDSource = Source.Qualisys
        self.PIDControl = True
        self.goal = [0.0, 0.0, 0.0, 0.0] # X Y Z YAW
        self.prev_msg = None
        self.PIDSetCurrentAuto = True

        self.thrust = (40., 90., 66.) # Min, Max, Base thrust
        self.distFunc = fLIN
        self.wandControl = False

        # Dynserver
        self.dynserver = DynamicReconfigureServer(CFJoyCFG, self.reconfigure)

    def setXYRGoal(self, msg):
        new_settings = {}
        new_settings["x"] = msg.pose.position.x
        new_settings["y"] = msg.pose.position.y
        new_settings["rz"] = degrees(tf.transformations.euler_from_quaternion(
            [msg.pose.orientation.x,
             msg.pose.orientation.y,
             msg.pose.orientation.z,
             msg.pose.orientation.w])[2])
        os.system("beep -f 6000 -l 35&")
        print new_settings
        self.dynserver.update_configuration(new_settings)
        
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
        if joy_thrust > 0.016:
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
                rospy.logwarn("Experimental: Using USB mode. Axes/Buttons may be different")
                Button = enum(L1=10,R1=11,Select=0,Start=3,L2=8,R2=9,Up=4,Right=5,Down=6,Left=7,Square=15,Cross=14,Triangle=12,Circle=13)
                Axes = enum(SLL=0,SLU=1,SRL=2,SRU=3,Up=4+4,Right=5+4,Down=6+4,Left=7+4,L2=4+8,R2=4+9,L1=4+10,R1=4+11,Triangle=4+12,Circle=4+13,Cross=4+14,Square=4+15,AccL=16,AccF=17,AccU=18,GyroY=19)
                self.USB = True
            return


        # bp = [i for i in range(len(joymsg.buttons)) if joymsg.buttons[i]]
        # ap = [(i,round(joymsg.axes[i],2)) for i in range(len(joymsg.axes)) if abs(joymsg.axes[i])>0.51 ]
        # if len(bp)>0:
        #     print "Buttons:", bp
        # if len(ap)>0:
        #     print "Axes   :", ap

        if self.USB:
            # USB buttons go from 1 to -1, Bluetooth go from 0 to -1, so normalise
            joymsg.axes =np.array(joymsg.axes, dtype=np.float32)
            joymsg.axes[8:19] -=1
            joymsg.axes[8:19] /=2
            # Strange bug: left button as axis doesnt work. So use button instead
            joymsg.axes[11] = -joymsg.buttons[Button.Left]/2.

        self.curr_cmd = joymsg
        hover = False

        x = 0
        y = 0
        z = 0
        r = 0
        r2 = 0
        # Get stick positions [-1 1]
        if self.curr_cmd.buttons[Button.L1]:  # Deadman pressed (=allow flight)
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
        if joymsg.header.seq%10 == 0:

            if self.curr_cmd.buttons[Button.Left]:
                new_settings["trim_roll"] = max(self.trim_roll + self.curr_cmd.axes[Axes.Left], -20)
            if self.curr_cmd.buttons[Button.Right]:
                new_settings["trim_roll"] =  min(self.trim_roll - self.curr_cmd.axes[Axes.Right], 20)
            if self.curr_cmd.buttons[Button.Down]:
                new_settings["trim_pitch"] = max(self.trim_pitch + self.curr_cmd.axes[Axes.Down], -20)
            if self.curr_cmd.buttons[Button.Up]:
                new_settings["trim_pitch"] = min(self.trim_pitch - self.curr_cmd.axes[Axes.Up], 20)




        msg = CFJoyMSG()
        msg.header.stamp = rospy.Time.now()
        msg.roll = trimmed_roll
        msg.pitch = trimmed_pitch
        msg.yaw = yaw
        msg.thrust = thrust
        msg.hover = hover
        #msg.calib = self.pressed(Button.Triangle)



        # TODO: rotation is wrong! Cannot simply decompose x,y into r,p.
        # TODO: roll, pitch should be dependent from the distance and the angle.
        if self.PIDControl: # and hover:
            try:
                rtime = rospy.Time.now()

                # Update goal position
                if hover and joymsg.header.seq%10 == 0:
                    if self.PIDActive[0]:
                        if abs(x)>0.016:
                            new_settings["x"] = self.goal[0]-roll/70.
                        if abs(y)>0.016:
                            new_settings["y"] = self.goal[1]-pitch/70.

                    if self.PIDActive[1]:
                        if abs(z)>0.016:
                            new_settings["z"] = self.goal[2]+z/5

                    if self.PIDActive[2]:
                        if abs(yaw)>0:
                           new_settings["rz"] = self.goal[3]-yaw/self.max_yawangle*20

                    if  new_settings.has_key("x") or new_settings.has_key("y") or new_settings.has_key("z") or new_settings.has_key("rz"):
                        os.system("beep -f 6000 -l 15&")


                # Get error from current position to goal if possible
                [px,py,alt,rz] = self.getErrorToGoal()

                # Starting control mode
                if hover and not self.prev_msg.hover:
                    # Starting Goal should be from current position
                    if self.PIDSetCurrentAuto:
                        new_settings["SetCurrent"] = True

                    # Reset PID controllers
                    for p in range(4):
                        self.PID[p].reset(rtime)

                    # Set yaw difference as I part
                    #self.PID[3].error_sum = 0






                if hover:
                    msg.hover = False #TODO should be an option

                    pidmsg = PidMSG()
                    pidmsg.header.stamp = rtime


                    # Get PID control

                    #XY control depends on the asin of the distance
                    #cr, crp, cri, crd = self.PID[0].get_command(-py,rtime,f=lambda d: degrees(asin(d))/90.*self.max_angle)
                    #cp, cpp, cpi, cpd  = self.PID[1].get_command(px,rtime,f=lambda d: degrees(asin(d))/90.*self.max_angle)



                    #cr, crp, cri, crd = self.PID[0].get_command(-py,rtime)
                    #cp, cpp, cpi, cpd  = self.PID[1].get_command(px,rtime)


                    # Distance to angle (X direction = pitch, Y direction = roll)
                    cr, crp, cri, crd = self.PID[0].get_command(-py,rtime,f=self.distFunc )
                    cp, cpp, cpi, cpd  = self.PID[1].get_command(px,rtime,f=self.distFunc )

                    # THRUST
                    ct, ctp, cti, ctd  = self.PID[2].get_command(alt,rtime)
                    # YAW VEL
                    cy, cyp, cyi, cyd  = self.PID[3].get_command(-rz,rtime)#,d=rz,i=copysign(1,-rz)) #TODO: normalise rotation



                    # ROll/Pitch
                    if self.PIDActive[0]:
                        msg.roll = cr + self.trim_roll
                        msg.pitch = cp + self.trim_pitch

                        pidmsg.ypid = [cr, crp, cri, -crd, -py]
                        pidmsg.xpid = [cp, cpp, cpi, -cpd, px]



                    # THRUST
                    if self.PIDActive[1]:

                        # Add base thrust
                        ct += self.thrust[2]

                        # Add roll compensation
                        # TODO: done on the flie itself now @ 250hz


                        # Bound
                        ct = max(self.thrust[0],ct)
                        ct = min(self.thrust[1],ct)

                        # ct in thrust percent
                        msg.thrust = percentageToThrust(ct)
                        pidmsg.zpid= [ct, ctp, cti, ctd, alt]


                    # YAW
                    if self.PIDActive[2]:
                        msg.yaw = cy
                        pidmsg.rzpid= [cy, cyp, cyi, cyd, -rz]



                    self.pub_PID.publish(pidmsg)



                if self.pressed(Button.Cross):
                    new_settings["SetCurrent"] = True


                q = tf.transformations.quaternion_from_euler(radians(msg.roll-self.trim_roll), radians(msg.pitch - self.trim_pitch), radians(-msg.yaw))
                self.pub_tf.sendTransform((0,0,0), q,  rospy.Time.now(), "/cmd", "/cf_gt2d")





            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):

                if time.time() - self.warntime>2:
                    rospy.logwarn('Could not look up cf_gt2d -> goal')
                    self.warntime = time.time()


        global tid
        if self.pressed(Button.Circle):

            tid = (tid +1)%len(trajectory)
            new_settings["x"], new_settings["y"], new_settings["z"], new_settings["rz"] = trajectory[tid]

        # previous
        if self.pressed(Button.Triangle):
            tid = (tid -1)%len(trajectory)
            new_settings["x"], new_settings["y"], new_settings["z"], new_settings["rz"] = trajectory[tid]


        # Set trim to current input
        if self.released(Button.R1):
            new_settings["trim_roll"] = min(20, max(msg.roll, -20))
            new_settings["trim_pitch"] = min(20, max(msg.pitch, -20))
            rospy.loginfo("Trim updated Roll/Pitch: %r/%r", round(new_settings["trim_roll"],2), round(new_settings["trim_pitch"],2))

        # Reset Trim
        if self.released(Button.Square):
            new_settings["trim_roll"] = 0.0
            new_settings["trim_pitch"] = 0.0
            rospy.loginfo("Pitch reset to 0/0")


        if new_settings != {} and self.dynserver is not None:
            if self.USB:
                new_settings = numpy2python(new_settings)
            self.dynserver.update_configuration(new_settings)
        
        
        # Cache prev joy command

        self.pub_cfJoy.publish(msg)


        self.prev_cmd = self.curr_cmd
        self.prev_msg = msg
        self.prev_msg.hover = hover


    def lookupTransformInWorld(self, frame='/cf_gt', forceRealtime = False, ):
        now = rospy.Time(0)
        if self.PIDDelay > 0.00001 and not forceRealtime:
            now = rospy.Time.now()-rospy.Duration(self.PIDDelay)
        (trans,rot) = self.sub_tf.lookupTransform('/world', frame, now)
        return (trans,rot)


    def getErrorToGoal(self):
        # Look up flie position, convert to 2d, publish, return error

        if self.wandControl:
            #update goal using wand
            try:
                (t,r) = self.lookupTransformInWorld(frame='/wand', forceRealtime=True)
                e = tf.transformations.euler_from_quaternion(r)
                self.goal= [t[0], t[1], t[2], degrees(e[2]-e[0])]
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.loginfo('Could not find wand')


        # Publish GOAL from world frame
        q = tf.transformations.quaternion_from_euler(0, 0, radians(self.goal[3]))
        self.pub_tf.sendTransform((self.goal[0],self.goal[1],self.goal[2]), q, rospy.Time.now(), "/goal", "/world")


        # Look up 6D flie pose, publish 3d+yaw pose
        (trans,rot) = self.lookupTransformInWorld(frame='/cf_gt')
        euler = tf.transformations.euler_from_quaternion(rot)
        q = tf.transformations.quaternion_from_euler(0, 0, euler[2])
        self.pub_tf.sendTransform(trans, q, rospy.Time.now(), "/cf_gt2d", "/world")

        # Look up error goal->cf
        (trans,rot) = self.sub_tf.lookupTransform('/cf_gt2d', '/goal', rospy.Time(0))
        #self.pub_tf.sendTransform(trans, rot, rospy.Time.now(), "/goalCF", "/cf_gt2d")
        euler = tf.transformations.euler_from_quaternion(rot)
        return trans[0], trans[1], trans[2], degrees(euler[2])




    def setGoalFromCurrent(self,config={}):
        try:
            (trans,rot) = self.lookupTransformInWorld(frame='/cf_gt', forceRealtime=True)
            euler = tf.transformations.euler_from_quaternion(rot)
            config['x'],config['y'],config['z'],config['rz']= trans[0], trans[1], trans[2], degrees(euler[2])
            rospy.loginfo('Updated goal state: [%.2f %.2f %.2f %.1f]', trans[0], trans[1], trans[2], degrees(euler[2]))
            self.goal = [config['x'],config['y'],config['z'],config['rz']]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn('Could not look up transform world -> cf_gt')
        return config
    
    def reconfigure(self, config, level):
        # Manual control stuff
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


        #PID control stuff
        if config["PIDPreset"]>0:
            if config["PIDPreset"]==1:
                # Aggressive
                config["DistFunc"] = 1 # ATAN
                config["Pxy"] = 1.0
                config["Ixy"] = 0.0
                config["Dxy"] = 0.2
                config["Pyaw"] = 8
                config["Iyaw"] = 0.0
                config["Dyaw"] = 0.0
                config["Pz"] = 20
                config["Iz"] = 5
                config["Dz"] = 8
                config["RP_maxAngle"] = 35
                config["Y_maxVel"] = 250
                config["z_minThrust"] = 50
                config["z_maxThrust"] = 100
                config["z_baseThrust"] = 73
                config["Response"] = 0.3

            if config["PIDPreset"]==2:
                # Passive
                config["DistFunc"] = 2 # ASIN
                config["Pxy"] = 2.0
                config["Ixy"] = 0.0
                config["Dxy"] = 0.4
                config["Pyaw"] = 2.0
                config["Iyaw"] = 0.0
                config["Dyaw"] = 0.0
                config["Pz"] = 20
                config["Iz"] = 5
                config["Dz"] = 8
                config["RP_maxAngle"] = 7
                config["Y_maxVel"] = 80
                config["z_minThrust"] = 50
                config["z_maxThrust"] = 90
                config["z_baseThrust"] = 73
                config["Response"] = 0.175

            if config["PIDPreset"]==3:
                # Linear
                config["DistFunc"] = 0
                config["Pxy"] = 17
                config["Ixy"] = 0.0
                config["Dxy"] = 5
                config["Pyaw"] = 3.3
                config["Iyaw"] = 0.0
                config["Dyaw"] = 0.0
                config["Pz"] = 20
                config["Iz"] = 5
                config["Dz"] = 8
                config["RP_maxAngle"] = 25
                config["Y_maxVel"] = 300
                config["z_minThrust"] = 50
                config["z_maxThrust"] = 90
                config["z_baseThrust"] = 73
                config["Response"] = 1


            rospy.loginfo("Preset Loaded")
            config["PIDPreset"] = 0




        # Settings
        self.PIDDelay = config["Delay"]
        self.PIDSource = config["Source"]
        self.PIDControl = config["Control"]
        self.max_anglePID = config['RP_maxAngle']

        # Gains
        self.PID[0].set_gains(config["Pxy"], config["Ixy"], config["Dxy"] )
        self.PID[1].set_gains(config["Pxy"], config["Ixy"], config["Dxy"] )
        self.PID[2].set_gains(config["Pz"],  config["Iz"], config["Dz"] )
        self.PID[3].set_gains(config["Pyaw"], config["Iyaw"], config["Dyaw"] )

        # Goal
        self.wandControl = config['WandControl']
        if config['SetCurrent']:
            config['SetCurrent'] = False
            config = self.setGoalFromCurrent(config)

        if config['Set'] or config['LiveUpdate']:
            config['Set'] = False
            if not self.wandControl:
                self.goal = [config['x'],config['y'],config['z'],config['rz']]

        # Limits
        #self.limits = (config['RP_maxAngle'], config['Y_maxVel'], config['z_maxAcc'])
        self.PID[0].set_limits(config['RP_maxAngle'])
        self.PID[1].set_limits(config['RP_maxAngle'])
        self.PID[2].set_limits(100)
        self.PID[3].set_limits(config['Y_maxVel'])

        self.thrust = (config['z_minThrust'],config['z_maxThrust'], config['z_baseThrust'] )


        # On/OFF
        self.PIDActive = (config['xyControl'],config['thrustControl'],config['yawControl'])
        self.PIDSetCurrentAuto = config['SetCurrentAuto']

        self.distFunc = [lambda x: fLIN(x,config["Response"]),
                         lambda x: fATAN(x,config["Response"]),
                         lambda x: fASIN(x,config["Response"])][config['DistFunc']]



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
