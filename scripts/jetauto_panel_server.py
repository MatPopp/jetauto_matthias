#!/usr/bin/env python3
# encoding: utf-8
import rospy
from hiwonder_servo_msgs.msg import MultiRawIdPosDur
from hiwonder_servo_controllers.bus_servo_control import set_servos
from sensor_msgs.msg import Image
import cv2
import numpy as np
import io 
from PIL import Image as PIL_Image
import panel as pn

from JointsWidget import JointsWidget
from LidarWidget import LidarWidget
from DriveWidget import DriveWidget
from snippets.html_camera_stream import HtmlCameraPanel

import time

import sys, select, os
if os.name == 'nt':
  import msvcrt, time
else:
  import tty, termios

class RobotApp:
    def __init__(self):
        try:
            rospy.init_node('jetauto_panel_server')
            rospy.loginfo('jetauto_panel_server node started')
            ### build panel app
            self.build_panel()

            rospy.spin()
        except rospy.ROSInterruptException:
            pass
    
    def build_panel(self):
        def app_wrapper():
            self.stop_app_button = pn.widgets.Button(name="stop App")
            self.app_funcitons_row = pn.Row(self.stop_app_button)
            self.stop_app_button.on_click(self.stop_app_callback)

            #self.lidar_widget = LidarWidget().panel()
            #self.astra_image_panel = VideoWidget("/astra_cam/rgb/image_raw").panel()
            self.astra_image_panel = HtmlCameraPanel("192.168.178.36","8080","/astra_cam/rgb/image_raw").panel()
            self.drive_widget = DriveWidget().panel()
            
            self.drive_column = pn.Column(self.astra_image_panel,self.drive_widget)


            self.joints_widget = JointsWidget().panel()
            #self.usb_image_panel = VideoWidget("/usb_cam/image_raw").panel()
            self.usb_image_panel = HtmlCameraPanel("192.168.178.36","8080","/usb_cam/image_raw").panel()
            

            self.gripper_column = pn.Column(self.usb_image_panel,self.joints_widget)

            
            self.operator_row = pn.Row(self.gripper_column,
                                       #self.lidar_widget ,
                                       self.drive_column)
            
            
            self.col = pn.Column(self.app_funcitons_row,self.operator_row)
        
            return(self.col)
            
            ### actually start panel (wrapper to make periodic callback work)
        self.app = pn.serve(app_wrapper,threaded=True, port = 22222 ,allow_websocket_origin = ["*"])

    def stop_app_callback(self,event):
        print("stop app")
        self.app.stop()


    

class VideoWidget:
    def __init__(self,image_topic,streaming=True):
        self.image_topic = image_topic
        rospy.Subscriber(self.image_topic, Image, self.process_image)
        self.streaming=streaming
        self.build_panel()
        
    def process_image(self,msg):

        if self.streaming:
            try:
                # convert sensor_msgs/Image to OpenCV Image
                ros_image = msg
                rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data) # 原始 RGB 画面
                image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
                is_success, buffer = cv2.imencode(".jpg", image)
                self.usb_image_io = io.BytesIO(buffer)
                self.update_image()

            except Exception as err:
                print(err)

    
    def start_callback(self,event):
        self.streaming = True

    def stop_callback(self,event):
        self.streaming = False

    def build_panel(self):
        self.start_button = pn.widgets.Button(name="Start")
        self.start_button.on_click(self.start_callback)

        self.stop_button = pn.widgets.Button(name="Stop")
        self.stop_button.on_click(self.stop_callback)

        self.row = pn.Row(self.start_button,self.stop_button)
        self.image_pane = pn.pane.image.PNG(width=640, height = 480)
        self.col = pn.Column(self.row,self.image_pane)

    def update_image(self):
        
        try:
            self.image_pane.object = self.usb_image_io 

        except Exception as e:
            print(e)

    def panel(self):
        return self.col
    

if __name__ == '__main__':
    
    RobotApp()