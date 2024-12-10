
import rospy
from std_msgs.msg import Float64
import message_filters


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


def filter_callback(speed1,speed2,speed3,speed4):
    #print the approximate time synchronized encoder data 
    print(speed1.data,speed2.data,speed3.data,speed4.data)
    



def rpm_node():
    rospy.init_node('rpm_node',anonymous=True) #initialising the node
    rospy.Subscriber("rpm_value1",Float64,callback1) #subscribing to the topic
    rospy.Subscriber("rpm_value2",Float64,callback2) #subscribing to the topic
    rospy.Subscriber("rpm_value3",Float64,callback3) #subscribing to the topic
    rospy.Subscriber("rpm_value4",Float64,callback4) #subscribing to the topic


    #create subscribers for passing to time synchronizer
    speed1_sub = message_filters.Subscriber("rpm_value1",Float64)
    speed2_sub = message_filters.Subscriber("rpm_value2",Float64)
    speed3_sub = message_filters.Subscriber("rpm_value3",Float64)
    speed4_sub = message_filters.Subscriber("rpm_value4",Float64)

    ts = message_filters.ApproximateTimeSynchronizer([speed1_sub,speed2_sub,speed3_sub,speed4_sub], 10, 0.1, allow_headerless=True)
    ts.registerCallback(filter_callback)

    while not rospy.is_shutdown():
        pass


if __name__ == '__main__':
	try:
		rpm_node()
	except rospy.ROSInterruptException:
		pass