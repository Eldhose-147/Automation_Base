#! /usr/bin/env python3
import rospy
import numpy as np
import modern_robotics as mr
from geometry_msgs.msg import PoseStamped, Twist
from tf.transformations import euler_from_quaternion
import time
from geometry_msgs.msg import Vector3


class p2p():
    def __init__(self, trajectory, timegap, Xstart, Xend, samplingTime, T, kp=10, ki=0.0, l=0.5/2,w = 0.64/2, r=0.152/2):
        
        
        #trajectory parameters
        self.trajectory = trajectory
        self.timegap = timegap
        self.time = 0
        self.Rstart, self.pstart = mr.TransToRp(Xstart)
        self.Rend, self.pend = mr.TransToRp(Xend)
        self.samplingTime = samplingTime
        self.T = T

        #robot base parameters
        # self.l = l
        # self.r = r
        # self.w = w

        self.allocation_matrix  = np.mat((1/r)*np.array([[-l-w ,1,-1],[l+w,1,1],[l+w,1,-1],[-l-w,1,1]]))



        #control parameters PI gains
        self.Kp = kp * np.eye(4)
        self.Ki = ki * np.eye(4)

        #initialize integral terrm
        self.sum_Xe = 0

        self.matrixLog3 = mr.MatrixLog3(np.mat(self.Rstart).T * self.Rend)


        #initialize count to get reference trajectory
        self.count = 0
        
        #initialize pose
        self.phi = self.pstart[0]
        self.x = self.pstart[1]
        self.y = self.pstart[2]
        

        rospy.Subscriber("/pose", Vector3, self.poseUpdate)
        
        #create publishers for each motor
        self.pub1=rospy.Publisher('keyboard_message1',Vector3,queue_size=10)
        self.pub2=rospy.Publisher('keyboard_message2',Vector3,queue_size=10)
        self.pub3=rospy.Publisher('keyboard_message3',Vector3,queue_size=10)
        self.pub4=rospy.Publisher('keyboard_message4',Vector3,queue_size=10)


        #wait for 10 seconds
        time.sleep(3)


        #initialize speed variables for each motor
        self.mes1=Vector3()
        self.mes2=Vector3()
        self.mes3=Vector3()
        self.mes4=Vector3()

        while not rospy.is_shutdown():
            self.decoupledControl()
            rospy.Rate(1).sleep()

    def poseUpdate(self,msg):
        #update the current pose information from odometry
        self.x = msg.x
        self.y = msg.y
        self.phi = msg.z


    def decoupledControl(self):
        x = self.x  #x position
        y = self.y   #y position
        phi = self.phi  #orientation
        #get homogeneous transformation matrix
        pose = np.array([
            [np.cos(phi), -np.sin(phi), 0, x],
            [np.sin(phi), np.cos(phi), 0, y],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        


        #uncomment below for trajectory verification
        # pose  = self.trajectory[self.count]
        # x = pose[0][3]
        # y = pose[1][3]
        # phi = np.arccos(pose[0][0])


        Rt, pt = mr.TransToRp(pose)
        Rt = np.mat(Rt)

        matrix1 = np.eye(4)
        matrix1[0:3, 0:3] = np.transpose(Rt) * np.mat(np.eye(3))

        t = self.time
        sdot = 30 * (t**2 / self.T**3) - 60 * \
            (t**3/self.T ** 4) + 30 * (t**4 / self.T**5)

        var_exp3 = self.matrixLog3 * mr.QuinticTimeScaling(self.T, self.time)
        var_exp3 = np.reshape(np.array(var_exp3), (3, 3))
        if mr.NearZero(np.linalg.norm(var_exp3)):
            w = 0
        else:
            Omgtheta = mr.so3ToVec(var_exp3)
            Theta = mr.AxisAng3(Omgtheta)[1]
            Omgmat = var_exp3 / Theta
            Rdot = Omgmat * np.cos(Theta) * sdot + \
                np.sin(Theta) * np.dot(Omgmat, Omgmat) * sdot
            Rdot = self.Rstart @ Rdot
            wMatrix = Rdot @ self.trajectory[self.count][0:3, 0:3]
            w = mr.so3ToVec(wMatrix)[2]

        commanded_vel = np.array([
            w, sdot * (self.pend[0] - self.pstart[0]
                       ), sdot * (self.pend[1] - self.pstart[1]), 0

        ])
        commanded_vel = np.mat(np.reshape(commanded_vel, (-1, 1)))
        # print(commanded_vel)
        first_term = matrix1*commanded_vel

        we = float(np.array(mr.MatrixLog3(
            np.transpose(Rt) * np.mat(self.trajectory[self.count][0:3, 0:3])))[1][0])

        self.time += self.samplingTime

        if(self.time > self.T):
            ex = float(self.pend[0]) - x
            ey = float(self.pend[1]) - y
            ephi = np.arccos(float(self.Rend[0][0])) - phi
            print("x:", self.pend[0])
            Xe = np.mat(np.reshape(np.array([[ephi, ex, ey, 0]]), (4, -1)))

            self.sum_Xe += Xe
            temp = (self.Kp @ Xe + self.Ki @ self.sum_Xe)[0:3]
        else:
            desired_x = float(self.trajectory[self.count][0][3])
            desired_y = float(self.trajectory[self.count][1][3])
            self.count += 1

            ex = desired_x - x
            ey = desired_y - y

            Xe = np.mat(np.reshape(np.array([[we, ex, ey, 0]]), (4, -1)))
            self.sum_Xe += Xe
            temp = (first_term + self.Kp @ Xe + self.Ki @ self.sum_Xe)[0:3]

        # twist = Twist()
        # twist.angular.z = temp[0]
        # twist.linear.x = temp[1]
        # twist.linear.y = temp[2]
        temp = np.reshape(np.array(temp),(3,-1))

        wbz, vbx, vby = np.mat([
            [1, 0, 0],
            [0, np.cos(phi), np.sin(phi)],
            [0, -np.sin(phi), np.cos(phi)]
        ]) * np.mat(temp)

        wbz = float(wbz[0])
        vbx = float(vbx[0])
        vby = float(vby[0])

        body_twist = np.mat(np.array([wbz,vbx,vby])).T

        motor_speeds = self.allocation_matrix * np.mat(body_twist)
        #rospy.loginfo(motor_rpms)
        motor_pwms = self.speed_to_pwm(motor_speeds)
        motor_pwms = np.reshape(np.array(motor_pwms) , (4,1))
        a=np.array(motor_pwms)
        motor_pwms = np.clip(a,a_min=-60,a_max=60)
        print(motor_pwms)


        #publish motor commands
        self.mes1.x = motor_pwms[0] * 1.2
        self.mes2.x = motor_pwms[1]
        self.mes3.x = motor_pwms[2]
        self.mes4.x = motor_pwms[3]
        


        self.pub1.publish(self.mes1)
        self.pub2.publish(self.mes2)
        self.pub3.publish(self.mes3)
        self.pub4.publish(self.mes4)


        #print(twist)
        # self.twist_pub.publish(twist)
        # print(ex,ey)
        
        return temp


    def speed_to_pwm(self,speed):
        return speed*5






if __name__ == "__main__":
    try:
        rospy.init_node('point2point')

        theta_start = 0 * np.pi / 180  # initial orientation in radian
        theta_end = 0 * np.pi / 180  # final orientation in radian
        pos_initial = [0, 0]  # initial x,y in meters
        pos_final = [-2,0 ]  # final x,y in meters

        Xstart = np.array([[np.cos(theta_start), -np.sin(theta_start), 0, pos_initial[0]],
                          [np.sin(theta_start),   np.cos(theta_start), 0, pos_initial[1]],
                          [0,                     0,                   1,          0    ],
                          [0,                     0,                    0,          1]    ])
        Xend = np.array([[np.cos(theta_end),   -np.sin(theta_end),    0,   pos_final[0]],
                         [np.sin(theta_end),   np.cos(theta_end),     0,   pos_final[1]],
                         [0,                     0,           1,       0],
                         [0,                     0,           0,       1]])
        Tf = 20
        samplingTime = 1
        N = int(Tf/samplingTime)
        method = 5
        trajectory = mr.CartesianTrajectory(Xstart, Xend, Tf, N, method)
        timegap = Tf / (N - 1.0)
        controller = p2p(trajectory, timegap, Xstart, Xend, samplingTime , Tf)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass