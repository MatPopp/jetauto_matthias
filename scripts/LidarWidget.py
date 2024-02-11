#!/usr/bin/env python3
# encoding: utf-8
import rospy
from sensor_msgs.msg import LaserScan
import panel as pn
import numpy as np
from bokeh.plotting import figure
from bokeh.events import DoubleTap
from bokeh.models import ColumnDataSource
from functools import partial


class LidarWidget:
    def __init__(self,update_time_ms=200):

        self.update_time_ms = update_time_ms
        self.ranges = np.array([])
        self.angles = np.array([])
        self.source = ColumnDataSource(data=dict(x=[], y=[]))
        self.line_source = ColumnDataSource(data=dict(x=[], y=[]))
        self.add_source = ColumnDataSource(data=dict(x=[], y=[]))
        self.coord_list=[]
        self.build_panel()

        try:
            rospy.Subscriber("/scan", LaserScan ,self.scan_msg_callback)
           
        except rospy.ROSInterruptException:
            pass

    def scan_msg_callback(self,scan_msg):
        print("bin in callback")
        angle_min = scan_msg.angle_min
        angle_max= scan_msg.angle_max
        self.ranges = np.array(scan_msg.ranges)
        self.angles = np.linspace(angle_min,angle_max,len(self.ranges))
        


    def update_plot(self,source, line_source):
        print("bin in update_plot")
        x = -np.sin(self.angles)*self.ranges
        y = np.cos(self.angles)*self.ranges
        source.data = dict(x=x, y=y)

        x_origin = 0
        y_origin = 0

        x_line = np.array([np.ones(len(x))*x_origin,x]).flatten(order="F")
        y_line = np.array([np.ones(len(y))*y_origin,y]).flatten(order="F")

        line_source.data = dict(x=x_line, y=y_line)

        
    def build_panel(self):

        plot= figure()#tools=("tap",""))
        self.plot = pn.pane.Bokeh(plot, width=500, height = 480)
        plot.scatter('x', 'y', source=self.source)
        plot.line('x', 'y', source=self.line_source)
        self.col = pn.Column(self.plot)
        ## periodic callback for plot updating 
        cb = pn.state.add_periodic_callback(partial(self.update_plot, self.source, self.line_source), self.update_time_ms)
        #add a dot where the click happened
        
        #plot.circle(source=self.add_source,x='x',y='y') 
        #def callback(event):
        #    Coords=(event.x,event.y)
        #    self.coord_list.append(Coords) 
        #    self.add_source.data = dict(x=[i[0] for i in coordList], y=[i[1] for i in coordList])        
        #self.plot.on_event(DoubleTap, callback)


    def panel(self):
        return(self.col)

        

if __name__=="__main__":
    
    
    rospy.init_node('scan_widget')
    rospy.loginfo('scan_widget node started')
    
    

    def app_wrapper():
        ### build panel app
        wid = LidarWidget().panel()
        col = pn.Column(wid)
        return col
        
    pn.serve(app_wrapper,threaded = True, port = 22221)

    rospy.spin()
     
    