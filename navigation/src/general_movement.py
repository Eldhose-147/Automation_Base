#! /bin/python3
import rospy
#import vector3 message type
from geometry_msgs.msg import Vector3
import numpy as np
from std_msgs.msg import Float64

RPM =0

def callback(rpm):
    RPM = rpm.data

def keyboard_node():
  
    #create publishers for each motor
    pub1=rospy.Publisher('keyboard_message1',Vector3,queue_size=10)
    pub2=rospy.Publisher('keyboard_message2',Vector3,queue_size=10)
    pub3=rospy.Publisher('keyboard_message3',Vector3,queue_size=10)
    pub4=rospy.Publisher('keyboard_message4',Vector3,queue_size=10)

    # pwm_publisher = rospy.Publisher('pwm_values',keyboard_pwm,queue_size=10)


    #initialize node
    rospy.init_node('keyboard_node',anonymous=True)



    #initialize speed variables
    mes1=Vector3()
    mes2=Vector3()
    mes3=Vector3()
    mes4=Vector3()


    #mobile base parameters
    l = 0.5
    w = 0.5
    r = 0.152/2

    #allocation matrix for the base (ref:Modern Robotic, mechanics ,planning and control pg.no. 519 http://hades.mech.northwestern.edu/images/7/7f/MR.pdf)
    allocation_matrix  = np.mat((1/r)*np.array([[-l-w ,1,-1],[l+w,1,1],[l+w,1,-1],[-l-w,1,1]]))



    while not rospy.is_shutdown():
        #get keyboard input from user yaw rate, velocity in x, y
        speed =input("Enter PWM value w,x,y: ")
        w=0
        x=0
        y=0
        if speed == 'w':
            w=0
            x=1
            y=0
        elif speed == 's':
            w=0
            x=-1
            y=0
        elif speed=='a':
            w=0
            x=0
            y=1
        elif speed=='d':
            w=0
            x=0
            y=-1
        elif speed=='wa':
            w=0
            x=1
            y=1
        elif speed=='wd':
            w=0
            x=1
            y=-1
        elif speed=='as':
            w=0
            x=-1
            y=1
        elif speed=='ds':
            w=0
            x=-1
            y=-1
        elif speed=='q' :
            w=1
            x=0
            y=0
        elif speed=='e':
            w=-1
            x=0
            y=0
        elif speed==' ':
            w=0
            x=0
            y=0
        

        # w,x,y = speed.split(" ")
        # print(RPM)
        
        twist = np.mat(np.array([float(w),float(x),float(y)])).T
        wheel_speeds = allocation_matrix*twist
        #convert to pwm values
        pwms = speed_to_pwm(wheel_speeds)
        
        mes1.x = -1*pwms[0] * 1.3
        mes2.x = -1*pwms[1]
        mes3.x = -1*pwms[2]
        mes4.x = -1*pwms[3]

        pub1.publish(mes1)
        pub2.publish(mes2)
        pub3.publish(mes3)
        pub4.publish(mes4)
        


def speed_to_pwm(speed):
    return speed*2.3


if __name__ == '__main__':
	try:
		keyboard_node()
	except rospy.ROSInterruptException:
		pass

				
		  

			
				
			
	


