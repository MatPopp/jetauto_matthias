#!/usr/bin/env python3
# encoding: utf-8
import rospy
from geometry_msgs.msg import Twist
from JoystickPanel import Joystick
import json

import panel as pn
import time

class DriveWidget:
    def __init__(self, lin_vel = 0.4, ang_vel=1, lin_dec = 0.8, ang_dec=2):
        self.lin_vel = lin_vel
        self.ang_vel = ang_vel
        self.lin_dec = lin_dec
        self.ang_dec = ang_dec

        self.last_t = time.time()
        self.last_change_t = time.time()

        try:
            self.mecanum_pub = rospy.Publisher('jetauto_controller/cmd_vel', Twist, queue_size=10)

        except rospy.ROSInterruptException:
            pass
        self.build_panel()

    def periodic_callback(self):
       # try:
            
          #  if True: #not self.lock_velocities.value:
          #      dt = time.time()-self.last_t
          #      if dt>0.05: 
          #          dt = 0.05 
                
              #  if abs(self.front_back.value) < 0.05:
              #      self.front_back.value = 0
              #  elif self.front_back.value > 0:
              #      self.front_back.value -= self.lin_dec*(dt)
              #  elif self.front_back.value < 0:
              #      self.front_back.value += self.lin_dec*(dt)

                
              #  if abs(self.left_right.value) < 0.05:
              #      self.left_right.value = 0
              #  elif self.left_right.value > 0:
              #      self.left_right.value -= self.lin_dec*(dt)
              #  elif self.left_right.value < 0:
              #      self.left_right.value += self.lin_dec*(dt)

                
             #   if abs(self.rotate.value) < 0.05:
             #       self.rotate.value = 0
             #   elif self.rotate.value > 0:
             #       self.rotate.value -= self.ang_dec*(dt)
             #   elif self.rotate.value < 0:
             #       self.rotate.value += self.ang_dec*(dt)
      
        #except Exception as e:
        #    print(e)
        try: 

            lin_dict = json.loads(self.linear_move_joystick.value)
            rot_dict = json.loads(self.rotate_joystick.value)

            twist = twist = Twist()
            twist.linear.x = 1.0*self.lin_vel*float(lin_dict["y"])/100
            twist.linear.y = 1.0*-self.lin_vel*float(lin_dict["x"])/100
            twist.linear.z = 0.0

            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = -self.ang_vel*float(rot_dict["x"])/100

            print("twist to be published: ", twist)
            self.mecanum_pub.publish(twist)
            
        except Exception as e:
            print(e)

        self.last_t = time.time()
        
        
    def build_panel(self):

        #self.front_back = pn.widgets.FloatSlider(name="front back",start = -self.lin_vel, end = self.lin_vel, step = 0.01, value = 0)
        #self.front_back.param.watch(self.front_back_callback,"value")
        
        #self.left_right = pn.widgets.FloatSlider(name="left rigth",start = -self.lin_vel, end = self.lin_vel, step = 0.01, value = 0)
        #self.left_right.param.watch(self.left_right_callback,"value")
        
        self.linear_move_joystick = Joystick()
        self.rotate_joystick = Joystick()
        #self.linear_move_joystick.param.watch(self.linear_move_callback,"value")

        #self.rotate = pn.widgets.FloatSlider(name="rotate", start = -self.ang_vel, end = self.ang_vel, step = 0.01, value = 0)
        #self.rotate.param.watch(self.rotate_callback,"value")

        self.lock_velocities = pn.widgets.Toggle(name="release brake")

        self.stick_row = pn.Row(self.rotate_joystick, pn.Spacer(sizing_mode="stretch_width"),self.linear_move_joystick)
        self.col = pn.Column(self.stick_row,self.lock_velocities)
        ### wait for initialization of servo values
        pn.state.add_periodic_callback(self.periodic_callback,50)

    def linear_move_callback(self,event):
        print(event.new)

    def front_back_callback(self,event):
        pass

    def left_right_callback(self,event):
        pass

    def rotate_callback(self,event):
        pass
        
    def panel(self):
        return(self.col)

        

if __name__=="__main__":
    
    
    rospy.init_node('drive_widget')
    rospy.loginfo('drive_widget node started')
    
    ### build panel app
    wid = DriveWidget().panel()

    def wrapper():
        return(pn.Column(wid))
    pn.serve(wrapper, port = 22221, allow_websocket_origin = ["*"])

    rospy.spin()
     
    