#! /bin/python3
import rospy
from manipulator_pwm.msg import keyboard_pwm
#import vector3 message type
from geometry_msgs.msg import Vector3

def keyboard_node():
	pub1=rospy.Publisher('keyboard_message1',Vector3,queue_size=10)
	pub2=rospy.Publisher('keyboard_message2',Vector3,queue_size=10)
	pub3=rospy.Publisher('keyboard_message3',Vector3,queue_size=10)
	pub4=rospy.Publisher('keyboard_message4',Vector3,queue_size=10)


	rospy.init_node('keyboard_node',anonymous=True)

	mes1=Vector3()
	mes2=Vector3()
	mes3=Vector3()
	mes4=Vector3()

	y= int(input("Enter PWM value: "))
	if(y<0 or y>255):
		print("Youu IDIOT dont know about PWM range! Enter valid PWM value")	
	else:
		mes1.x=y
		mes2.x=y
		mes3.x=y
		mes4.x = y

	pwm = 30
	

	while not rospy.is_shutdown():
		x=input("give dir:")
		if x=='w':
			mes1.y = -150
			mes1.x = pwm

			mes2.y = -150
			mes2.x = pwm

			mes3.y = -150
			mes3.x = pwm

			mes4.y = -150
			mes4.x = pwm
		elif x=='s':
			mes1.y = 150
			mes1.x = pwm

			mes2.y = 150
			mes2.x = pwm

			mes3.y = 150
			mes3.x = pwm

			mes4.y = 150
			mes4.x = pwm
		elif x=='a':  #left
			mes1.y = 150
			mes1.x = pwm

			mes2.y = -150
			mes2.x = pwm

			mes3.y = 150
			mes3.x = pwm

			mes4.y = -150
			mes4.x = pwm

		elif x=='d':  #right
			mes1.y = -150
			mes1.x = pwm

			mes2.y = 150
			mes2.x = pwm

			mes3.y = -150
			mes3.x = pwm

			mes4.y = 150
			mes4.x = pwm

		elif x == "f":
			mes1.y = 0
			mes1.x = 0

			mes2.y = 150
			mes2.x = pwm

			mes3.y = 0
			mes3.x = 0

			mes4.y = 150
			mes4.x = pwm
		
		pub1.publish(mes1)
		pub2.publish(mes2)
		pub3.publish(mes3)
		pub4.publish(mes4)

if __name__ == '__main__':
	try:
		keyboard_node()
	except rospy.ROSInterruptException:
		pass

				
		  

			
				
			
	


