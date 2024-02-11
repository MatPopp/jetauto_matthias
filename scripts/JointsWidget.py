#!/usr/bin/env python3
# encoding: utf-8
import rospy
import math
import json
import threading
import time
from geometry_msgs.msg import Twist
from hiwonder_servo_msgs.msg import MultiRawIdPosDur, ServoStateList
from hiwonder_servo_controllers.bus_servo_control import set_servos
from kinematics.search_kinematics_solutions import SearchKinematicsSolutions
from JoystickPanel import Joystick

import panel as pn

class JointsWidget:
    def __init__(self):

        self.lin_vel = 0.3 ## m per second
        self.ang_vel = 30 ## degree per second

        self.last_t = time.time()
        self.last_IK = time.time()

        self.servo_id_list = range(1,6)
        self.initialized = False
        self.servo_goal_dict = {}
        self.servo_pos_dict = {}
        self.slider_dict = {}
        self.pos_indicator_dict = {}
        
        try:
            rospy.Subscriber("/servo_controllers/port_id_1/servo_states", ServoStateList,self.servo_msg_callback)
            self.joints_pub = rospy.Publisher('servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)

        except rospy.ROSInterruptException:
            pass

        self.build_panel()

    def servo_msg_callback(self,servo_msg):
        if not self.initialized:
            for i,servo_state in enumerate(servo_msg.servo_states):
                self.servo_goal_dict[servo_state.id] = servo_state.position
                self.servo_pos_dict[servo_state.id] = servo_state.position
            self.init_sliders()
            self.initialized = True
        else:
            for i,servo_state in enumerate(servo_msg.servo_states):
                self.servo_pos_dict[servo_state.id] = servo_state.position
                self.pos_indicator_dict[servo_state.id].value=str(servo_state.position)
        self.servo_states = servo_msg.servo_states
        
    def build_panel(self):

        self.linear_move_joystick=Joystick()
        self.rotate_joystick = Joystick()

        self.joystick_row = pn.Row(self.rotate_joystick,
                                   self.linear_move_joystick)

        self.y_slider = pn.widgets.FloatSlider(name="y in cm", start = 0, end=0.6,value = 0.2, step=0.001)
        self.y_slider.param.watch(self.IK_callback,"value")
        self.z_slider = pn.widgets.FloatSlider(name="z in cm", start = 0, end=0.6,value=0.4,  step=0.001)
        self.z_slider.param.watch(self.IK_callback,"value")
        self.phi_slider = pn.widgets.FloatSlider(name= "phi left / right",start = -90, end = 90, value=0, direction="rtl")
        self.phi_slider.param.watch(self.IK_callback,"value")
        self.alpha_slider = pn.widgets.FloatSlider(name = "alpha / down",start = -90, end = 90, value = 0,  step=0.1)
        self.alpha_slider.param.watch(self.IK_callback,"value")
        self.gripper_slider = pn.widgets.IntSlider(name = "grip", start = 0, end=999, value = 500)
        self.gripper_slider.param.watch(self.IK_callback,"value")
        
        self.angle_col = pn.Column(self.alpha_slider,self.phi_slider)
        self.lin_col = pn.Column(self.z_slider,self.y_slider)
        self.arm_row = pn.Row(self.angle_col,self.lin_col)
        self.IK_col = pn.Column(self.gripper_slider,self.arm_row)
        self.col = pn.Column(self.joystick_row,self.IK_col)
        ### wait for initialization of servo values

        #cb = pn.state.add_periodic_callback(self.periodic_callback,50)
        self.periodic_thread = threading.Thread(target=self.periodic_loop)
        self.periodic_thread.start()
        print("started thread")

    def panel(self):
        return(self.col)
    
    def periodic_loop(self):
        #print("periodic_loop started")
        while True: 
            self.periodic_callback()
            time.sleep(0.02)

    def periodic_callback(self):
        print("periodic_callback")
        try: 

            lin_dict = json.loads(self.linear_move_joystick.value)
            ang_dict = json.loads(self.rotate_joystick.value)
            
            dt = time.time()-self.last_t
            self.last_t = time.time()

            ## update linear values
            v_up = 1.0*self.lin_vel*float(lin_dict["y"])/100
            v_forward = 1.0*self.lin_vel*float(lin_dict["x"])/100
            
            self.y_slider.value += dt*(self.lin_vel*v_forward*math.cos(math.radians(self.alpha_slider.value)) - self.lin_vel*v_up*math.sin(math.radians(self.alpha_slider.value)))
            self.z_slider.value += dt*(+self.lin_vel*v_forward*math.sin(math.radians(self.alpha_slider.value)) + self.lin_vel*v_up*math.cos(math.radians(self.alpha_slider.value))) 
            
            ## update rotations

            omega_alpha = 1.0*self.ang_vel*float(ang_dict["y"])/100
            omega_phi = 1.0*-self.ang_vel*float(ang_dict["x"])/100

            self.alpha_slider.value += omega_alpha*dt
            self.phi_slider.value += omega_phi*dt

        except Exception as e:
            print(e)

    def IK_callback(self,event):
        
        print("IK_callback dt = ", self.last_IK-time.time())
        self.last_IK=time.time()
        solution_searcher = SearchKinematicsSolutions()
        res = solution_searcher.solveIK((0, self.y_slider.value, self.z_slider.value), math.radians(self.alpha_slider.value), math.radians(-90), math.radians(90))
        if res:
            joint_data = res[1]
            ## convert to int 
            for key,value in joint_data.items():
                #print(joint_data[key],"=>", max(0,int(joint_data[key])))
                joint_data[key] = max(0,int(joint_data[key]))

            phi_servo_val = int((self.phi_slider.value+90)/180*1000)
                
            set_servos(self.joints_pub, 0, ((1, self.gripper_slider.value),
                                                (2, joint_data['joint4']),
                                                (3, joint_data['joint3']),
                                                (4, joint_data['joint2']),
                                                (5, phi_servo_val)))
            #print("servos set")

    
    def init_sliders(self):
        for id, goal in self.servo_goal_dict.items():

            slider = pn.widgets.IntSlider(name="Servo "+str(id), start = 0, end = 999, step =1, value = goal)
            slider.param.watch(self.slider_callback,"value")
            self.slider_dict[id] = slider
            pos_indicator = pn.widgets.StaticText(name="curr pos "+ str(id), value = str(self.servo_pos_dict[id]))
            self.pos_indicator_dict[id] = pos_indicator
            self.col.append(pn.Row(slider,pos_indicator))
    
    def slider_callback(self,event):
        print(event)
        try:
            
            for id, slider in self.slider_dict.items():
                self.servo_goal_dict[id]=slider.value

            ## publish new servo goals
            set_servos(self.joints_pub, 0,
            ((1, self.servo_goal_dict[1]),
            (2, self.servo_goal_dict[2]),
            (3, self.servo_goal_dict[3]),
                (4, self.servo_goal_dict[4]),
                (5, self.servo_goal_dict[5])))
            print("servos set")
        except Exception as e:
            print(e)
     
    def panel(self):
        return(self.col)

        

if __name__=="__main__":
    
    
    rospy.init_node('servo_widget')
    rospy.loginfo('jetauto_widget node started')
    
    ### build panel app
    wid = JointsWidget().panel()
    def wrapper():
        return(pn.Column(wid))
    pn.serve(wrapper, port = 22221, allow_websocket_origin=["*"])

    rospy.spin()
     
    