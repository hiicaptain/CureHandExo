#!/usr/bin/env python
# license removed for brevity
import serial
import rospy
from std_msgs.msg import String

# Try/catch to check if the connection is established 
try:
    PORT = '/dev/ttyACM0'
    SPEED = 115200
    connection = serial.Serial(PORT, SPEED, timeout=0.1)
except OSError as e:
    print("Error", "The connection with the Arduino is not establisehd, change the Serial Port and restart the GUI")

def talker():
    rospy.init_node('Sensor_value', anonymous=True)
    pub_value = rospy.Publisher('/Sensor_value', String, queue_size=10)
    rate = rospy.Rate(100)
    while not rospy.is_shutdown(): 
        while True:
            try:
                sensor_data = connection.readline()
                # print(sensor_data.decode('utf-8'))
                pub_value.publish(sensor_data.decode('utf-8'))
                break
            except:
                print("Be trying again")
    rate.sleep()

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
