"""----------------------------------------------------------------------------
@author:  Hao Yu, Edinburgh Centre for Robotics
Date of last update : 11/08/2023
Functions: building the GUI 
----------------------------------------------------------------------------"""
# import general libraries
import time
import matplotlib.pyplot as plt
import numpy as np
from scipy import signal

from std_msgs.msg import String, Float64, Int32

# tkinter librairies
import tkinter as tk

#import for ROS
import rospy
from dynamixel_msgs.msg import JointState as JointStateDynamixel

# Colors used
black_color = '#000000'
white_color = '#ffffff'
blue_color = "#1e8bc3"
grey_color = '#d9d9d9'

'''-------------------------------- Main page ------------------------------'''
class AppGUI:
# Contructor of the main page
    def __init__(self, master):    
        #Create the window
        self.master = master
        self.master.geometry ("900x400+40+10")
        self.master.title("Impedance data collector")
        self.master.configure(background=grey_color)

        # Create the Menu bar 
        menu = tk.Menu(self.master)
        self.master.config(menu=menu)
        
        # publish the assessment mode
        rospy.init_node('GUI', anonymous=True)
        self.pub_mode = rospy.Publisher('/Assessment_mode', String, queue_size=10)
        
        # get force sensor values
        rospy.Subscriber("/Sensor_value", String, self.callbackForce)
        self.forces = [0, 0, 0, 0]
        
        # get motor angle values
        rospy.Subscriber("/MCP_controller/state", JointStateDynamixel, self.callbackAngle(0))
        rospy.Subscriber("/PIP_controller/state", JointStateDynamixel, self.callbackAngle(1))
        rospy.Subscriber("/DIP_controller/state", JointStateDynamixel, self.callbackAngle(2))
        rospy.Subscriber("/IP_controller/state", JointStateDynamixel, self.callbackAngle(3))
        self.angles = [0, 0, 0, 0]
        
        # GUI Frame
        self.Ext =  tk.Button(self.master, text="extend", font='Helvetica 16 bold', command= lambda: self.setMode(0, 0))
        self.Ext.place(relx=0.2, rely=0.2, height=40, width=100)
        self.Ext.configure(background=blue_color)
        self.Ext.configure(foreground=white_color)  
        
        self.Fle =  tk.Button(self.master, text="flex", font='Helvetica 16 bold', command= lambda: self.setMode(0, 1))
        self.Fle.place(relx=0.2, rely=0.4, height=40, width=100)
        self.Fle.configure(background=blue_color)
        self.Fle.configure(foreground=white_color)  
        
        self.MoveLabel = tk.Label(self.master, text="input the moving distance",font='Helvetica 16 bold')
        self.MoveLabel.place(relx=0.4, rely=0.2, height=40, width=300)
        self.MoveLabel.configure(background=grey_color)
        self.MoveLabel.configure(foreground=black_color)
        
        self.MoveCommand = tk.Entry(self.master, width=35, font='Helvetica 16 bold',)
        self.MoveCommand.place(relx=0.8, rely=0.2, height=40, width=100)
        self.MoveCommand.configure(background=blue_color)
        self.MoveCommand.configure(foreground=white_color)
        
        self.VelLabel = tk.Label(self.master, text="input the velocity",font='Helvetica 16 bold')
        self.VelLabel.place(relx=0.4, rely=0.4, height=40, width=300)
        self.VelLabel.configure(background=grey_color)
        self.VelLabel.configure(foreground=black_color)
        
        self.VelCommand = tk.Entry(self.master, width=35, font='Helvetica 16 bold')
        self.VelCommand.place(relx=0.8, rely=0.4, height=40, width=100)
        self.VelCommand.configure(background=blue_color)
        self.VelCommand.configure(foreground=white_color)
        
        self.IPButton = tk.Button(self.master, text="IP", font='Helvetica 16 bold', command= lambda: self.setMode(1, 0))
        self.IPButton.place(relx=0.2, rely=0.6, height=60, width=100)
        self.IPButton.configure(background=blue_color)
        self.IPButton.configure(foreground=white_color)
        
        self.MCPButton = tk.Button(self.master, text="MCP", font='Helvetica 16 bold', command= lambda: self.setMode(1, 1))
        self.MCPButton.place(relx=0.4, rely=0.6, height=60, width=100)
        self.MCPButton.configure(background=blue_color)
        self.MCPButton.configure(foreground=white_color)

        self.PIPButton = tk.Button(self.master, text="PIP", font='Helvetica 16 bold', command= lambda: self.setMode(1, 2))
        self.PIPButton.place(relx=0.6, rely=0.6, height=60, width=100)
        self.PIPButton.configure(background=blue_color)
        self.PIPButton.configure(foreground=white_color)
        
        self.DIPButton = tk.Button(self.master, text="DIP", font='Helvetica 16 bold', command= lambda: self.setMode(1, 3))
        self.DIPButton.place(relx=0.8, rely=0.6, height=60, width=100)
        self.DIPButton.configure(background=blue_color)
        self.DIPButton.configure(foreground=white_color)

        self.ForceLabel0 = tk.Label(self.master, text="IP force [N]",font='Helvetica 16 bold' )
        self.ForceLabel0.place(relx=0.2, rely=0.8, height=60, width=150)
        self.ForceLabel0.configure(background=grey_color)
        self.ForceLabel0.configure(foreground=black_color)
        
        self.ForceLabel1 = tk.Label(self.master, text="MCP force [N]",font='Helvetica 16 bold' )
        self.ForceLabel1.place(relx=0.4, rely=0.8, height=60, width=150)
        self.ForceLabel1.configure(background=grey_color)
        self.ForceLabel1.configure(foreground=black_color)  
        
        self.ForceLabel2 = tk.Label(self.master, text="PIP force [N]",font='Helvetica 16 bold' )
        self.ForceLabel2.place(relx=0.6, rely=0.8, height=60, width=150)
        self.ForceLabel2.configure(background=grey_color)
        self.ForceLabel2.configure(foreground=black_color)
        
        self.ForceLabel3 = tk.Label(self.master, text="DIP force [N]",font='Helvetica 16 bold' )
        self.ForceLabel3.place(relx=0.8, rely=0.8, height=60, width=150)
        self.ForceLabel3.configure(background=grey_color)
        self.ForceLabel3.configure(foreground=black_color)

        self.Stop = tk.Button(self.master, text="Stop", font='Helvetica 16 bold', command= lambda: self.setMode(99))
        self.Stop.place(relx=0.9, rely=0.9, height=50, width=50)
        self.Stop.configure(background=grey_color)
        self.Stop.configure(foreground=black_color)        
        
        self.update_force()
        
    # send mode commands
    def setMode(self, modeID = 99, motorID = 0):
        if modeID == 0:
            message = str(modeID) + " " + str(motorID) # motorID = 0, extend; motorID = 1, flex
            self.pub_mode.publish(message)
        elif modeID == 1:
            distance = float(self.MoveCommand.get())
            max_vel = float(self.VelCommand.get())
            message = str(modeID) + " " + str(motorID) + " " + str(distance) + " " + str(max_vel)
            self.pub_mode.publish(message)
        else:
            self.pub_mode.publish(str(modeID))
        
    # This funtion is called everytime "/Sensor_value" receives a value
    def callbackForce(self, data):    
        force_values = data.data.split(",")
        if len(force_values) == 4:
            # Process the force values here
            self.forces[0] = float(force_values[0])/100
            self.forces[1] = float(force_values[1])/100
            self.forces[2] = float(force_values[2])/100
            self.forces[3] = float(force_values[3])/100            
        else:
            print("force data error")
    
    # Subscrib the current position of four motors 
    def callbackAngle(self, motor_ID):
        def callback(data):
            # Your callback logic here
            self.angles[motor_ID] = data.current_pos
        return callback

    def update_force(self):
        # Update the label with new force data
        self.ForceLabel0.configure(text="%s " % round(self.forces[0],3))
        self.ForceLabel1.configure(text="%s " % round(self.forces[1],3))
        self.ForceLabel2.configure(text="%s " % round(self.forces[2],3))
        self.ForceLabel3.configure(text="%s " % round(self.forces[3],3))
        
        # Schedule the next update, every 0.2 seconds
        self.master.after(200, self.update_force)

