#!/usr/bin/env python
# license removed for brevity

# import general libraries
from std_msgs.msg import String, Float64, Int32

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
        self.mode = 0
        
        # joint name
        self.Joints = ['MCP', 'PIP', 'DIP', 'IP']
        
        # publish velocity
        self.pubMCP = rospy.Publisher('/MCP_controller/command', Float64, queue_size=1)
        self.pubPIP = rospy.Publisher('/PIP_controller/command', Float64, queue_size=1)  
        self.pubDIP = rospy.Publisher('/DIP_controller/command', Float64, queue_size=1) 
        self.pubIP = rospy.Publisher('/IP_controller/command', Float64, queue_size=1)
        # add more motor here
        
        # get motor angle values
        rospy.Subscriber("/MCP_controller/state", JointStateDynamixel, self.callbackAngle(0))
        rospy.Subscriber("/PIP_controller/state", JointStateDynamixel, self.callbackAngle(1))
        rospy.Subscriber("/DIP_controller/state", JointStateDynamixel, self.callbackAngle(2))
        rospy.Subscriber("/IP_controller/state", JointStateDynamixel, self.callbackAngle(3))
        # add more motor here
        self.angles = [0, 0, 0, 0]
        
        # get force sensor values
        rospy.Subscriber("/Sensor_value", String, self.callbackForce)
        self.forces = [0, 0, 0, 0]
        
        # get the mode setting 
        rospy.init_node('Motor_controller', anonymous=True)
        rospy.Subscriber("/Assessment_mode", Int32, self.callbackMode)
        
        # PID controller
        self.rate = 50 # hz
        self.KP = 50
        self.KI = 0.1
        self.KD = 2
    
    # Subscrib the current position of four motors 
    def callbackAngle(self, motor_ID):
        def callback(data):
            # Your callback logic here
            self.angles[motor_ID] = data.current_pos
        return callback

    # This funtion is called everytime "/assessment_mode" receives a value        
    def callbackMode(self, data):
        self.mode = data.data
    
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
    
    def setVelocity(self, ID, vel):
        # Check the ID of the motor
        if ID is 0:
            self.pubMCP.publish(vel)
        elif ID is 1:
            self.pubPIP.publish(vel)                      
        elif ID is 2:
            self.pubDIP.publish(vel)
        elif ID is 3:
            self.pubIP.publish(vel) 
        return False
        
    def assessment(self):
        rate = rospy.Rate(50)
        print("initialise assessment")
        while not rospy.is_shutdown():
            try:
                if self.mode == 0:
                    self.setVelocity(3, 0)
                    #print("0")
                elif self.mode == 1:
                    self.setVelocity(3, 0)
                    #print("1")
                elif self.mode == 2:
                    self.setVelocity(3, 0)
                    #print("2")
                elif self.mode == 3:
                    self.setVelocity(3, 0)
                    #print("3")
                else:
                    print("error")
            except:
                print("Be trying again")
        rate.sleep()

if __name__ == '__main__':  
    Motors = MotorController()
    try:
        Motors.assessment()
    except rospy.ROSInterruptException:
        pass
    
    
'''    
    def extend(self):
        self.positionPID(0, 0)
        time.sleep(0.1)
        self.positionPID(1, 0)
        time.sleep(0.1)
        self.positionPID(2, 0)
        
    def flex(self):
        self.positionPID(0, 1.5)
        time.sleep(0.1)
        self.positionPID(1, 0)
        time.sleep(0.1)
        self.positionPID(2, 0)
                       
    def assess(self, ID):
        # stop according to time
        print('start')
        self.count += 1
            
        joint_name = self.Joints[ID]
        angle_array = []
        force_array = []
        time_array = []
        
        distance = float(self.MoveCommand.get())
        initial_position = self.Motors.Angles[ID]
        print('initial_position',initial_position)
        goal_position = initial_position + distance
        print('goal_position', goal_position)
        
        max_vel = float(self.VelCommand.get())
        if max_vel > 0:
            e = 0
            e_d = 0
            e_i = 0
            e_old = 0
            dt = 1/self.rate
            starting_time = time.time()
            sampling_time = starting_time
            while(sampling_time - starting_time <= 0.5):
                sampling_time = time.time() 
                angle = round(self.Motors.Angles[ID], 2)
                angle_array.append(angle)
                force_array.append(self.Motors.Forces[ID])
                time_array.append(sampling_time-starting_time)
                Last_time = time.time()-sampling_time
                if Last_time<dt: # control the sampling rate to 100Hz               
                    time.sleep(dt-Last_time)                
            
            while(sampling_time - starting_time <=6*abs(distance/max_vel)):
                sampling_time = time.time()             
                # record angle
                angle = round(self.Motors.Angles[ID], 2)
                angle_array.append(angle)
                # record force
                force_array.append(self.Motors.Forces[ID])
                time_array.append(sampling_time-starting_time)
                
                e = goal_position - angle
                e_d = (e - e_old)/dt
                e_i = e_i + e
                e_old = e
                vel = self.KP*e + self.KD*e_d + self.KI*e_i
                if abs(vel) > max_vel:
                    vel = max_vel*vel/abs(vel)
                self.Motors.move(ID, vel)
                Last_time = time.time()-sampling_time
                if Last_time<dt: # control the sampling rate to 100Hz               
                    time.sleep(dt-Last_time)
            self.Motors.move(ID, 0)   
            
            # save data
            self.saveData(time_array, force_array, angle_array, joint_name)
            # draw plot
            self.plotCurves(time_array, force_array, angle_array, joint_name)
            
    def assess2(self, ID):
        # stop according to position
        print('start')
        self.count += 1
            
        joint_name = self.Joints[ID]
        angle_array = []
        force_array = []
        time_array = []
        
        distance = float(self.MoveCommand.get())
        initial_position = self.Motors.Angles[ID]
        print('initial_position',initial_position)
        goal_position = initial_position + distance
        print('goal_position', goal_position)
        
        dt = 1/self.rate
        starting_time = time.time()
        sampling_time = starting_time
        while(sampling_time - starting_time <= 0.5):
            sampling_time = time.time() 
            angle = round(self.Motors.Angles[ID], 2)
            angle_array.append(angle)
            force_array.append(self.Motors.Forces[ID])
            time_array.append(sampling_time-starting_time)
            Last_time = time.time()-sampling_time
            if Last_time<dt: # control the sampling rate to 100Hz               
                time.sleep(dt-Last_time) 
        
        vel = float(self.VelCommand.get())
        self.Motors.move(ID, vel)
        if goal_position > initial_position:                          
            while(goal_position > angle):
                sampling_time = time.time()             
                # record angle
                angle = round(self.Motors.Angles[ID], 2)
                angle_array.append(angle)
                # record force
                force_array.append(self.Motors.Forces[ID])
                time_array.append(sampling_time-starting_time)
                Last_time = time.time()-sampling_time
                if Last_time<dt: # control the sampling rate to 100Hz               
                    time.sleep(dt-Last_time)
        else:
            while(goal_position < angle):
                sampling_time = time.time()             
                # record angle
                angle = round(self.Motors.Angles[ID], 2)
                angle_array.append(angle)
                # record force
                force_array.append(self.Motors.Forces[ID])
                time_array.append(sampling_time-starting_time)
                Last_time = time.time()-sampling_time
                if Last_time<dt: # control the sampling rate to 100Hz               
                    time.sleep(dt-Last_time)
        self.Motors.move(ID, 0)  
            
        # save data
        self.saveData(time_array, force_array, angle_array, joint_name)
        # draw plot
        self.plotCurves(time_array, force_array, angle_array, joint_name)

    def assess3(self, ID):
        # force sensor calibration 
        self.count += 1
            
        angle_array = []
        force_array = []
        time_array = []
        
        distance = float(self.MoveCommand.get())
        initial_position = self.Motors.Angles[ID]
        goal_position = initial_position + distance
        
        max_vel = float(self.VelCommand.get())
        if max_vel > 0:
            e = 0
            e_d = 0
            e_i = 0
            e_old = 0
            dt = 1/self.rate
            starting_time = time.time()
            sampling_time = starting_time                     
            while(sampling_time - starting_time <=3*abs(distance/max_vel)):
                sampling_time = time.time()             
                # record angle
                angle = round(self.Motors.Angles[ID], 2)                
                e = goal_position - angle
                e_d = (e - e_old)/dt
                e_i = e_i + e
                e_old = e
                vel = self.KP*e + self.KD*e_d + self.KI*e_i
                if abs(vel) > max_vel:
                    vel = max_vel*vel/abs(vel)
                self.Motors.move(ID, vel)
                Last_time = time.time()-sampling_time
                if Last_time<dt: # control the sampling rate to 100Hz               
                    time.sleep(dt-Last_time)
                    
            starting_time = time.time()      
            self.Motors.move(ID, 0)
            while(sampling_time - starting_time <= 1):
                sampling_time = time.time() 
                angle = round(self.Motors.Angles[ID], 2)
                angle_array.append(angle)
                force_array.append(self.Motors.Forces[ID])
                time_array.append(sampling_time-starting_time)
                Last_time = time.time()-sampling_time
                if Last_time<dt: # control the sampling rate to 100Hz               
                    time.sleep(dt-Last_time) 
                    
        average = sum(force_array) / len(force_array)
        print(average);
        
        
    def saveData(self, time_array, force_array, angle_array, joint_name):
        savepath = os.path.join(self.current_directory, 'data')
        filename = ''.join([joint_name,'_', str(self.count), '.csv'])
        savepath = os.path.join(savepath, filename)
        data = np.array([time_array,angle_array,force_array]).T
        var_name = 'time,angle,force'
        np.savetxt(savepath, data, delimiter=',',header =var_name)
        

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
    
    # Stop the motors at the current position
    def stop(self):
        self.move(0, 0)
        self.move(1, 0)
        self.move(2, 0)
        self.move(3, 0)
        print ("Assessment is stopped")
'''



