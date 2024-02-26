#!/usr/bin/env python
# license removed for brevity

# import general libraries
from std_msgs.msg import String, Float64, Int32
import os

# import for ROS
import rospy
from dynamixel_msgs.msg import JointState as JointStateDynamixel

# import for saving data
import time
import matplotlib.pyplot as plt
import numpy as np
from scipy import signal

class MotorController():
    def __init__(self):
        self.count = 0
        self.current_directory = os.path.dirname(os.path.abspath(__file__))
        
        # joint name
        self.joints = ['IP', 'MCP', 'PIP', 'DIP']
        
        # publish velocity
        self.pubIP = rospy.Publisher('/IP_controller/command', Float64, queue_size=1)
        self.pubMCP = rospy.Publisher('/MCP_controller/command', Float64, queue_size=1)
        self.pubPIP = rospy.Publisher('/PIP_controller/command', Float64, queue_size=1)  
        self.pubDIP = rospy.Publisher('/DIP_controller/command', Float64, queue_size=1) 
        # add more motor here
        
        # get motor angle values
        rospy.Subscriber("/IP_controller/state", JointStateDynamixel, self.callbackAngle(0))
        rospy.Subscriber("/MCP_controller/state", JointStateDynamixel, self.callbackAngle(1))
        rospy.Subscriber("/PIP_controller/state", JointStateDynamixel, self.callbackAngle(2))
        rospy.Subscriber("/DIP_controller/state", JointStateDynamixel, self.callbackAngle(3))
        # add more motor here
        self.angles = [0, 0, 0, 0]
        
        # get force sensor values
        rospy.Subscriber("/Sensor_value", String, self.callbackForce)
        self.forces = [0, 0, 0, 0]
        
        # get the mode setting 
        rospy.init_node('Motor_controller', anonymous=True)
        rospy.Subscriber("/Assessment_mode", String, self.callbackMode)
        
        # PID controller
        self.rate = 50 # hz
        self.KP = 50
        self.KI = 0.1
        self.KD = 2

        self.e_i = 0
        self.e_old = 0
        self.dt = 1/self.rate

        # assessment procedure parameters
        self.mode = 99 # assign the mode
        self.inimode = 0 # motorID = 0, extend; motorID = 1, flex
        self.motor = 0 # assign the motor
        self.goal_position = 0 # set the goal_position, rad
        self.maxvel = 0 # set the max velocity, rad/s

        # data storage 
        self.angle_array = []
        self.force_array = []
        self.time_array = []
        self.starting_time = 0
    
    # Subscrib the current position of four motors 
    def callbackAngle(self, motor_ID):
        def callback(data):
            self.angles[motor_ID] = data.current_pos
        return callback
    
    # This funtion is called everytime "/Sensor_value" receives a value
    def callbackForce(self, data):    
        force_values = data.data.split(",")
        if len(force_values) == 4:
            # Process the force values here
            self.forces[0] = float(force_values[0])/100 # IP
            self.forces[1] = float(force_values[1])/100 # MCP
            self.forces[2] = float(force_values[2])/100 # PIP
            self.forces[3] = float(force_values[3])/100 # DIP          
        else:
            print("force data error")

    # This funtion is called everytime "/assessment_mode" receives a value        
    def callbackMode(self, data):
        params = data.data.split()
            
        # Extract individual parameters and convert them if necessary
        # Ensure that there are exactly 3 parameters in the message for safety
        if len(params) == 4:
            mode, motor, distance, maxvel = params  # Unpack the list into variables
            self.mode = int(mode)  # Convert mode to int
            self.motor = int(motor)  # Convert motor to int
            self.maxvel = float(maxvel)  # Convert maxvel to float
            rospy.loginfo(str(mode) + " " + str(distance) + " " + str(maxvel))
            
            current_position = self.angles[self.motor]
            self.goal_position = current_position + float(distance)
            self.starting_time = time.time()

            # count the number of assessment
            self.count += 1

        elif len(params) == 1:
            mode = params[0]
            self.mode = int(mode)
            rospy.loginfo(str(mode))
        elif len(params) == 2:
            mode, inimode = params
            self.mode = int(mode)
            self.inimode = int(inimode)
            rospy.loginfo(str(mode) + " " + str(inimode))
        else:
            self.mode = 99
        # TO DO: Add an else statement to handle the case where the message is not in the expected format
        # TO DO: different modes will send different parameters
    
    def setVel(self, ID, vel):
        # Check the ID of the motor
        if ID is 0:
            self.pubIP.publish(vel) 
        elif ID is 1:
            self.pubMCP.publish(vel)                      
        elif ID is 2:
            self.pubPIP.publish(vel)
        elif ID is 3:
            self.pubDIP.publish(vel)
        return False
    
    def setPos(self, ID, goal_position):
        current_position = self.angles[ID]
        e = goal_position - current_position
        maxvel = 6
        minvel = 2
        vel = self.KP*e + minvel
        if abs(vel) > maxvel:
            vel = maxvel
        self.setVel(ID, vel)
        
    def controlloop(self):
        rate = rospy.Rate(50)
        print("initialise assessment")
        while not rospy.is_shutdown():
            try:
                if self.mode == 0:
                    self.initialise()
                    print("0")
                elif self.mode == 1:
                    self.assess()
                    print("1")
                else:
                    self.stop()
            except:
                print("Be trying again")
        rate.sleep()
    
    # Stop the motors at the current position
    def stop(self):
        self.setVel(0, 0)
        self.setVel(1, 0)
        self.setVel(2, 0)
        self.setVel(3, 0)
        print ("Assessment is stopped")
    
    # Extend or flex the motors to the initial position
    def initialise(self):
        if self.inimode == 0:
            self.setPos(0, 0)
            self.setPos(1, 0)
            self.setPos(2, 0)
            self.setPos(3, 0)
        elif self.inimode == 1:
            self.setPos(0, 3)
            self.setPos(1, 3)
            self.setPos(2, 3)
            self.setPos(3, 3)
        else:
            print("Initialisation mode error")

    # Move the motor to the desired position
    def assess(self):
        ID = self.motor
        sampling_time = time.time() 
        time_tag = sampling_time - self.starting_time
        angle = round(self.angles[ID], 2)
        if  time_tag <= 0.5:
            self.setVel(ID, 0)
        elif time_tag <= 4.5 & time_tag > 0.5:      
            e = self.goal_position - angle
            e_d = (e - self.e_old)/self.dt
            self.e_i = self.e_i + e
            self.e_old = e
            vel = self.KP*e + self.KD*e_d + self.KI*self.e_i
            if abs(vel) > self.maxvel:
                vel = self.maxvel
            self.setVel(ID, vel)
        elif time_tag > 4.5:
            self.setVel(ID, 0)    
        else:
            print("Complete the assessment")
            # save data
            self.saveData(self.time_array, self.force_array, self.angle_array, self.joints(ID))
            # draw plot
            self.plotCurves(self.time_array, self.force_array, self.angle_array, self.joints(ID))
            # reset the mode
            self.mode = 99

        angle = round(self.angles[ID], 2)
        self.angle_array.append(angle)
        self.force_array.append(self.forces[ID])
        self.time_array.append(sampling_time-self.starting_time)

    # Save the data to a csv file
    def saveData(self, time_array, force_array, angle_array, joint_name):
        savepath = os.path.join(self.current_directory, 'data')
        filename = ''.join([joint_name,'_', str(self.count), '.csv'])
        savepath = os.path.join(savepath, filename)
        data = np.array([time_array,angle_array,force_array]).T
        var_name = 'time,angle,force'
        np.savetxt(savepath, data, delimiter=',',header =var_name)
        
    # Plot the force and angle curves
    def plotCurves(self, time_array, force_array, angle_array, joint_name):
        data_filtered = signal.savgol_filter(force_array, 11, 5)
        # draw plot
        fig, ax1 = plt.subplots()
        ax1.plot(time_array,data_filtered, color='green', label='force', alpha = 1)
        ax2 = ax1.twinx()
        ax2.plot(time_array,angle_array,color='red', label='angle', alpha = 1)
        # plt.legend()
        ax1.set_xlabel("Time (s)", fontsize=14) 
        # ax1.set_xlim(0, 1) 
        ax1.set_ylabel("Force (N) green", fontsize=14)
        # ax1.set_ylim(-5, 5)
        ax2.set_ylabel("Angle (rad) red", fontsize=14)
        # ax2.set_ylim(0, 1.57)
        savepath = os.path.join(self.current_directory, 'image')
        filename = ''.join([joint_name, '_', str(self.count), '.png'])
        savepath = os.path.join(savepath, filename)
        plt.savefig(savepath , dpi = 150)
        plt.show()

if __name__ == '__main__':  
    Motors = MotorController()
    try:
        Motors.controlloop()
    except rospy.ROSInterruptException:
        pass

