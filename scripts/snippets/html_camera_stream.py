#!/usr/bin/env python3
# encoding: utf-8

import panel as pn

class HtmlCameraPanel:
    def __init__(self,ip_address,port,topic):
        self.src = "http://"+ip_address+":"+port+"/stream?topic="+topic
        self.build_panel()

    def build_panel(self):
        self.html_panel = pn.pane.HTML(width=640, height=480)
        self.html_panel.object = '<img src="'+self.src+'" alt="video source not reachable: '+self.src+'">'
        self.col = pn.Column(self.html_panel)
    def panel(self):
        return(self.col)

if __name__=="__main__":

    ### problem: webpage never finishes loading
    ip_address = "192.168.178.36"
    port = "8080"

    astra_rgb = HtmlCameraPanel(ip_address,port,"/astra_cam/rgb/image_raw").panel()
    #astra_depth = HtmlCameraPanel(ip_address,port,"/astra_cam/depth/image").panel()
    
    astra_col = pn.Column(astra_rgb)

    usb_rgb = HtmlCameraPanel(ip_address,port,"/usb_cam/image_raw").panel()
    usb_col = pn.Column(usb_rgb)
    row = pn.Row(astra_col, usb_col)
    pn.serve(row,  port = 22222 ,allow_websocket_origin = ["*"])