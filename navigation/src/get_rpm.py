
import rospy
from std_msgs.msg import Float64


RPM = 0


#callback function for rpm
def callback1(data):
    print("Wheel 1:",data.data)
def callback2(data):
    print("Wheel 2:",data.data)
def callback3(data):
    print("Wheel 3:",data.data)
def callback4(data):
    print("Wheel 4:",data.data)


def rpm_node():
    rospy.init_node('rpm_node',anonymous=True) #initialising the node
    rospy.Subscriber("rpm_value1",Float64,callback1) #subscribing to the topic
    rospy.Subscriber("rpm_value2",Float64,callback2) #subscribing to the topic
    rospy.Subscriber("rpm_value3",Float64,callback3) #subscribing to the topic
    rospy.Subscriber("rpm_value4",Float64,callback4) #subscribing to the topic

    # rpm_repub = rospy.Publisher("rpm_repub", Float64 , repub_callback)

    while not rospy.is_shutdown():
        pass


if __name__ == '__main__':
	try:
		rpm_node()
	except rospy.ROSInterruptException:
		pass