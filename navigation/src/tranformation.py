#!/usr/bin/env python3
import rospy
import numpy as np
# from navigation.msg import encoders
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
import message_filters
from geometry_msgs.msg import Vector3



class mecanumwheelOdometry():
    """
        
        
        Attributes
        ----------
        initialPhi: float
            initial orientation relative to global fixed frame(scalar)
        initialX: float 
            initial  X relative to global fixed frame(scalar)
        initialY: float
            initial Y  relative to global fixed frame(scalar)
        
        l:float 
             x distance to the centre of the wheel from centre of body fixed frame(scalar)
        w:float
            y distance to the centre of the wheel from centre of body fixed frame(scalar)
        r: float
            radius of the wheel (scalr)
        time: float
            time interval in which the values are received from encoder
        
        Methods
        -------
        motion_model(action=None)
            Computes the Current Position of the bot from the action input
    """
    def __init__(self,initialPhi,initialX,initialY,l,w,r,time):
        # rospy.Subscriber("/base_encoders", encoders, self.base_encoders_callback)
        self.motors_pub = rospy.Publisher("/odom",Odometry, queue_size=1)
        


        #pose pulisher
        pose_pub = rospy.Publisher("/pose",Vector3, queue_size=10)

        #initialize variables
        self.prevPose = np.transpose(np.mat(np.array([[float(initialPhi),float(initialX),float(initialY)]])))
        self.l = float(l)
        self.w = float(w)
        self.r = float(r)
        self.time = float(time)
            
        #create subscribers for passing to time synchronizer
        speed1_sub = message_filters.Subscriber("rpm_value1",Float64)
        speed2_sub = message_filters.Subscriber("rpm_value2",Float64)
        speed3_sub = message_filters.Subscriber("rpm_value3",Float64)
        speed4_sub = message_filters.Subscriber("rpm_value4",Float64)
        #create subscriber for imu
        rospy.Subscriber("/imu/data",Imu,self.imu_callback)


        ts = message_filters.ApproximateTimeSynchronizer([speed1_sub,speed2_sub,speed3_sub,speed4_sub], 10,0.5, allow_headerless=True)
        ts.registerCallback(self.filter_callback)




        self.pose = np.mat(np.array([[0,0,0]])).T
        self.yaw = 0

        while rospy.is_shutdown() == False:
            #publish pose
            pose_msg = Vector3()
            pose_msg.z = self.pose[0]  #yaw
            pose_msg.x = self.pose[1]  #x
            pose_msg.y = self.pose[2]  #y

            pose_pub.publish(pose_msg)


    def imu_callback(self,data):    
        # yaw=(180/3.14)*data.angular_velocity.x
        # print(self.yaw)

        self.yaw = data.angular_velocity.x
        
              

    def filter_callback(self,speed1,speed2,speed3,speed4):


        self.pose = self.motion_model(np.mat(np.array([float(speed1.data),-1*float(speed2.data),float(speed3.data),-1*float(speed4.data)])).T)
        

    # Manipulating the encoder data
    def base_encoders_callback(self,msg):
        encoder1=msg.motor1
        encoder2=msg.motor2
        encoder3=msg.motor3
        encoder4=msg.motor4


        odom=Odometry()
        odom.header.stamp=rospy.Time.now()
        odom.header.frame_id = "base_link"

    def motion_model(self,action):
        

        """
        calculates the predicted position of the base from previous position and encoder rotations(action)
         ________                 X
        |1       |2              |
        |        |               | 
        |        |               |      
        |4_______|3       Y______|      
        "mecanum wheel 
        position number"
                                                    
      

        Parameters
        ----------
        action: numpy.mat
            action(in radian) = 4*1 matrix of wheel rotations,CCW rotation is positive
        
        Returns
        -------
            current position(phi,x,y)
                
        """
        matrix = np.mat(np.array(
            [[-1/(self.l+self.w), 1/(self.l+self.w), 1/(self.l+self.w),-1/(self.l+self.w)],
            [1, 1 , 1,1],
            [-1,1,-1,1]]))
        chasisPlanarTwist = (self.r/4) * matrix * np.mat(action) * self.time
        
        deltaQb1 = chasisPlanarTwist[0]
        deltaQb2 = chasisPlanarTwist[1]*np.sin(chasisPlanarTwist[0]) + chasisPlanarTwist[2]*(np.cos(chasisPlanarTwist[0])-1)/chasisPlanarTwist[0]
        deltaQb3 = chasisPlanarTwist[2]*np.sin(chasisPlanarTwist[0]) - chasisPlanarTwist[1]*(np.cos(chasisPlanarTwist[0])-1)/chasisPlanarTwist[0]
        if((float(chasisPlanarTwist[0])*180/np.pi) >50 or (float(chasisPlanarTwist[0])*180/np.pi)<-50):
            deltaQb  = np.transpose(np.mat(np.array([[deltaQb1,deltaQb2,deltaQb3]])))
        else:
            deltaQb = np.transpose(np.mat(np.array([[0,float(chasisPlanarTwist[1]),float(chasisPlanarTwist[2])]])))
        prev_phi = float(self.prevPose[0])
        transformation_matrix = np.mat(np.array([[1,0,0],[0,np.cos(prev_phi),-np.sin(prev_phi)],[0,np.sin(prev_phi),np.cos(prev_phi)]]))
        delQ = transformation_matrix * deltaQb
        self.prevPose = self.prevPose + delQ
        #update yaw from imu
        # print(self.yaw)
        self.prevPose[0] = self.yaw
        return self.prevPose
    

if __name__ == "__main__":
    try:
        rospy.init_node('base_odometry')
        mecanumwheelOdometry(0,0,0,0.5/2,0.64/2,0.152/2,1)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    



# s = mecanumwheelOdometry(0,0,0,0.5,0.5,0.152/2,1)
# print(s.motion_model(np.transpose(np.mat(np.array([2*np.pi,2*np.pi,2*np.pi,2*np.pi])))))
# print(s.motion_model(np.transpose(np.mat(np.array([2*np.pi,-2*np.pi,-2*np.pi,2*np.pi])))))
# print(s.motion_model(np.transpose(np.mat(np.array([2*np.pi,-2*np.pi,2*np.pi,-2*np.pi])))))
# action = np.transpose(np.mat(np.array([0,-2*np.pi,2*np.pi])))
# chasisPlanarTwist = self.r * np.mat(np.array([[-1/(3*self.d), -1/(3*self.d),-1/(3*self.d)],[2/3,-1/3,-1/3],[0,-1/(2*np.sin(np.pi/3)),1/(2*np.sin(np.pi/3))]])) * np.mat(action)
# deltaQb1 = chasisPlanarTwist[0]
# deltaQb2 = chasisPlanarTwist[1]*np.sin(chasisPlanarTwist[0]) + chasisPlanarTwist[2]*(np.cos(chasisPlanarTwist[0])-1)/chasisPlanarTwist[0]
# deltaQb3 = chasisPlanarTwist[2]*np.sin(chasisPlanarTwist[0]) - chasisPlanarTwist[1]*(np.cos(chasisPlanarTwist[0])-1)/chasisPlanarTwist[0]
# if((float(chasisPlanarTwist[0])*180/np.pi) >1 or (float(chasisPlanarTwist[0])*180/np.pi)<-1):
#     deltaQb  = np.transpose(np.mat(np.array([[deltaQb1,deltaQb2,deltaQb3]])))
# else:
#     deltaQb = np.transpose(np.mat(np.array([[0,float(chasisPlanarTwist[1]),float(chasisPlanarTwist[2])]])))

# prevPose = np.transpose(np.mat(np.array([0,0,0])))
# prev_phi = float(prevPose[0])
# transformation_matrix = np.mat(np.array([[1.0,0.0,0.0],[0.0,np.cos(prev_phi),-np.sin(prev_phi)],[0.0,np.sin(prev_phi),np.cos(prev_phi)]]))
# delQ = transformation_matrix * deltaQb
# print(prevPose + delQ)

        


