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
import pandas as pd
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
        self.loads = [0, 0, 0, 0]
        
        # get force sensor values
        rospy.Subscriber("/Sensor_value", String, self.callbackForce)
        self.forces = [0, 0, 0, 0]
        
        # get the mode setting 
        rospy.init_node('Motor_controller', anonymous=True)
        rospy.Subscriber("/Assessment_mode", String, self.callbackMode)
        
        # PID controller
        self.rate = 50 # hz
        self.KP = 50
        self.KD = 2
        self.KI = 0.2

        self.e_i = 0
        self.e_old = 0
        self.dt = 1/self.rate

        # assessment procedure parameters
        self.mode = 99 # assign the mode
        self.inimode = 0 # motorID = 0, extend; motorID = 1, flex
        self.motor = 0 # assign the motor
        self.goal_position = 0 # set the goal_position, rad
        self.maxvel = 0 # set the max velocity, rad/s

        # sensor data storage 
        self.angle_array = []
        self.force_array = []
        self.time_array = []
        self.load_array = []
        self.starting_time = 0

        # emg data storage
        self.emg_start_index = 0
        self.emg_end_index = 0
        self.file_path = '/mnt/hgfs/share/emg_data.csv'
    
    # Subscrib the current position of four motors 
    def callbackAngle(self, motor_ID):
        def callback(data):
            if motor_ID == 2:
                self.angles[motor_ID] = -data.current_pos
                self.loads[motor_ID] = -data.load
            else:
                self.angles[motor_ID] = data.current_pos
                self.loads[motor_ID] = data.load
            # print(str(motor_ID)+":"+str(self.angles[motor_ID]))
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
            rospy.loginfo("force data error")

    # This funtion is called everytime "/assessment_mode" receives a value        
    def callbackMode(self, data):
        params = data.data.split()
            
        # Extract individual parameters and convert them if necessary
        # Ensure that there are exactly 3 parameters in the message for safety
        if len(params) == 4:
            # count the number of assessment
            self.count += 1

            mode, motor, distance, maxvel = params  # Unpack the list into variables
            self.mode = int(mode)  # Convert mode to int
            self.motor = int(motor)  # Convert motor to int
            self.maxvel = float(maxvel)  # Convert maxvel to float

            df = pd.read_csv(self.file_path)
            self.emg_start_index = df.shape[0]
            
            current_position = self.angles[self.motor]
            self.goal_position = current_position + float(distance)
            self.starting_time = time.time()

            rospy.loginfo("exp" + str(self.count) + ": " + self.joints[self.motor] + " moves to " + str(self.goal_position) + "rad with max velocity " + str(self.maxvel) + "rad/s")
            self.time_array = [0]
            self.angle_array = [current_position]
            self.force_array = [self.forces[self.motor]]
            self.load_array = [self.loads[self.motor]]
            self.e_i = 0
            self.e_old = 0

        elif len(params) == 1:
            mode = params[0]
            self.mode = int(mode)
            rospy.loginfo("stop: " + str(mode))

        elif len(params) == 2:
            mode, inimode = params
            self.mode = int(mode)
            self.inimode = int(inimode)
            if self.inimode == 0:
                rospy.loginfo("extending")
            elif self.inimode == 1:
                rospy.loginfo("flexing")
            else:
                rospy.loginfo("error command for initialisation mode")
                self.mode = 99

        else:
            rospy.loginfo("error: " + str(data.data))
            self.mode = 99
        # TO DO: Add an else statement to handle the case where the message is not in the expected format
    
    def setVel(self, ID, vel):
        # Check the ID of the motor
        if ID is 0:
            self.pubIP.publish(vel) 
        elif ID is 1:
            self.pubMCP.publish(vel)                      
        elif ID is 2:
            self.pubPIP.publish(-vel)
        elif ID is 3:
            self.pubDIP.publish(vel)
        return False
    
    def setPos(self, ID, goal_position, complete = 0):
        current_position = self.angles[ID]
        e = goal_position - current_position
        maxvel = 2
        minvel = 0
        vel = self.KP*e + minvel
        if abs(vel) > maxvel:
            vel = maxvel*np.sign(vel)
        if abs(e) < 0.1:
            complete = 1
            vel = 0
        self.setVel(ID, vel)
        return complete
        
    def controlloop(self):
        rate = rospy.Rate(50)
        rospy.loginfo("initialised assessment")
        while not rospy.is_shutdown():
            try:
                if self.mode == 0:
                    self.initialise()
                elif self.mode == 1:
                    self.assess()
                else:
                    self.stop()
            except Exception as e:
                 rospy.logerr("Failed to set the mode: %s", str(e))  # Log the error message with the exception
            rate.sleep()
    
    # Stop the motors at the current position
    def stop(self):
        self.setVel(0, 0)
        self.setVel(1, 0)
        self.setVel(2, 0)
        self.setVel(3, 0)
    
    # Extend or flex the motors to the initial position
    def initialise(self):
        flag = [0,0,0,0]
        if self.inimode == 0:
            flag[0] = self.setPos(0, 0)
            # time.sleep(0.5)
            flag[1] = self.setPos(1, 0)
            # time.sleep(0.5)
            flag[2] = self.setPos(2, 0)
            # time.sleep(0.5)
            flag[3] = self.setPos(3, 0)
        elif self.inimode == 1:
            flag[0] = self.setPos(0, 1)
            # time.sleep(0.5)
            flag[1] = self.setPos(1, 1.8)
            # time.sleep(0.5)
            flag[2] = self.setPos(2, 1.5)
            # time.sleep(0.5)
            flag[3] = self.setPos(3, 1.0)
        if sum(flag) == 4:
            self.mode = 99
            rospy.loginfo("Initialisation complete")

    # Move the motor to the desired position
    def assess(self):
        ID = self.motor
        sampling_time = time.time() 
        time_tag = sampling_time - self.starting_time
        angle = round(self.angles[ID], 2)
        e = self.goal_position - angle
        if  time_tag <= 0.5:
            self.setVel(ID, 0)
            rospy.loginfo("Wait for 0.5s")
        elif time_tag <= 4.5 and time_tag > 0.5 and abs(e) > 0.05:
            e_d = (e - self.e_old)/self.dt
            self.e_i = self.e_i + e*self.dt
            self.e_old = e
            vel = self.KP*e + self.KD*e_d + self.KI*self.e_i
            if abs(vel) > self.maxvel:
                vel = self.maxvel*np.sign(vel)
            self.setVel(ID, vel)
            rospy.loginfo("Moving")
        elif time_tag <= 4.5 and time_tag > 0.5 and abs(e) <= 0.05:
            self.setVel(ID, 0)
            rospy.loginfo("arriving")
        elif time_tag > 4.5 and time_tag <= 5.0:
            self.setVel(ID, 0) 
            rospy.loginfo("Stop for 0.5s")   
        else:
            rospy.loginfo("Complete the assessment")
            # save data
            self.saveData(self.time_array, self.force_array, self.angle_array, self.load_array, self.joints[ID])
            # draw plot
            self.plotCurves(self.time_array, self.force_array, self.angle_array, self.load_array, self.joints[ID])
            # reset the mode
            self.mode = 99

        self.angle_array.append(angle)
        self.force_array.append(self.forces[ID])
        self.time_array.append(time_tag)
        self.load_array.append(self.loads[ID])

        df = pd.read_csv(self.file_path)
        self.emg_end_index = df.shape[0]

    # Save the data to a csv file
    def saveData(self, time_array, force_array, angle_array, load_array, joint_name):
        savepath = os.path.join(self.current_directory, 'data')
        filename = ''.join([joint_name,'_', str(self.count), '.csv'])
        savepath = os.path.join(savepath, filename)
        time_array = np.array(time_array)
        angle_array = np.array(angle_array)
        force_array = np.array(force_array)
        load_array = np.array(load_array)

        # data = np.array([time_array,angle_array,force_array,load_array]).T
        # var_name = 'time,angle,force,load'
        # np.savetxt(savepath, data, delimiter=',',header =var_name)
        combined_df = pd.DataFrame({
            'time': (time_array - time_array[0])*1000,
            'angle': angle_array,
            'force': force_array,
            'load': load_array
        })

        df = pd.read_csv(self.file_path)
        emg_time = df['Time'][self.emg_start_index:self.emg_end_index].values
        emg_time = emg_time - emg_time[0]
        emg_data1 = df['Sensor1'][self.emg_start_index:self.emg_end_index].values
        emg_data2 = df['Sensor2'][self.emg_start_index:self.emg_end_index].values

        emg_df = pd.DataFrame({
            'time': emg_time,
            'emg1': emg_data1,
            'emg2': emg_data2
        })

        # merge the two dataframes according to the emg time
        combined_df = pd.merge_asof(emg_df, combined_df, on='time', direction='nearest')
        # fill the missing values
        combined_df['angle'] = combined_df['angle'].interpolate(method='linear')
        combined_df['force'] = combined_df['force'].interpolate(method='linear')
        combined_df['load'] = combined_df['load'].interpolate(method='linear')

        # fill the remaining missing values
        combined_df['angle'].fillna(0, inplace=True)
        combined_df['force'].fillna(0, inplace=True)
        combined_df['load'].fillna(0, inplace=True)
        combined_df['emg1'].fillna(0, inplace=True)
        combined_df['emg2'].fillna(0, inplace=True)

        combined_df.to_csv(savepath, index=False)
        
    # Plot the force and angle curves
    def plotCurves(self, time_array, force_array, angle_array, load_array, joint_name):
        data_filtered = signal.savgol_filter(force_array, 11, 5)

        fig, (ax1, ax3) = plt.subplots(1, 2, figsize=(15, 10))
        # draw plot of angle and force sensor
        ax1.plot(time_array,data_filtered, color='green', label='force', alpha = 1)
        ax2 = ax1.twinx()
        ax2.plot(time_array,angle_array,color='red', label='angle', alpha = 1)
        ax1.set_xlabel("Time (s)", fontsize=14) 
        ax1.set_ylabel("Force (N) green", fontsize=14)
        ax2.set_ylabel("Angle (rad) red", fontsize=14)

        # draw the plot of angle and motor load
        ax3.plot(time_array,angle_array, color='red', label='angle', alpha = 1)
        ax4 = ax3.twinx()
        ax4.plot(time_array,load_array, color='blue', label='load', alpha = 1)
        ax3.set_xlabel("Time (s)", fontsize=14)
        ax3.set_ylabel("Angle (rad) red", fontsize=14)
        ax4.set_ylabel("Load (A) blue", fontsize=14)

        # save the plot
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

