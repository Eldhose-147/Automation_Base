from operator import imod


import numpy as np
import math
import matplotlib.pyplot as plt
import rospy
from geometry_msgs.msg import Pose


class UKF_pos():
    """
    This Class is used to implement the Unscented Kalman Filter (UKF) for the pos estimation. It fuses the //
    position predicted by the motion model with pwm ( converted to rpm/rad) and the pos measured by the encoder. //
    """

    def __init__(self, motion_model, l, w, r):
        """
        Initialize the UKF.
        """

        # subscribers
        self.sub_encoder = rospy.Subscriber(
            "/encoder_data", EncoderData, self.callback_encoder)
        self.sub_pwm = rospy.Subscriber(
            "/pwm_data", PwmData, self.callback_pwm)

        # publishers
        self.pub_pos = rospy.Publisher("/pos_data", Pose, queue_size=1)

        # initialize variables
        self.pose = np.array([0, 0, 0, 0, 0, 0])
        self.pwm = np.array([0, 0, 0, 0])
        self.encoder = np.array([0, 0, 0, 0])
        self.pwm_to_rad = np.array([0, 0, 0, 0])

        self.N = len(self.pose)
        self.k = 3 - self.N

        self.P = np.array([[0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [
                          0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0]])
        self.Q = np.array([[0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [
                          0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0]])
        self.R = np.array([[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [
                          0, 0, 0, 0]])

        # initialize variables for motion model and measurement model
        self.l = float(l)
        self.w = float(w)
        self.r = float(r)
        # self.time = float(time)

        while not rospy.is_shutdown():

            encoder = self.encoder

            ###prediction step###
            # calculate sigma points
            sigma_ponts = self.sigma_points(self.pose, self.P)
            # propogate sigma points through motion model
            motion_model_prediction = motion_model.compute_state(sigma_ponts)
            # calculate the mean and covariance of the motion model prediction
            mean_predicted, covariance_predicted, weights = self.calculate_mean_covariance(
                motion_model_prediction)

            ###correction step###
            y_sigma_points = self.measurement_model(motion_model_prediction)
            # calculate the mean and covariance of the measurement model prediction
            mean_measurement, covariance_measurement = self.calculate_mean_covariance_measurement(
                y_sigma_points)

            ###compute cross covariance and Kalman gain###
            K = self.kalman_gain(
                motion_model_prediction, mean_predicted, y_sigma_points, mean_measurement, covariance_measurement, weights)

            #conpute corrected mean and covariance
            self.pose = mean_predicted + np.mat(K) * np.mat(encoder - mean_measurement)
            self.P = covariance_predicted - np.mat(K) * np.mat(covariance_measurement) * np.mat(K).T

            # publish the pose
            estimate = Pose()
            estimate.Point.x = self.pose[1]  ##x coordinate
            estimate.Point.y = self.pose[2]  ##y coordinate
            estimate.Point.z = 0             ##z coordinate
            estimate.Orientation.x = self.pose[0]  ##yaw in radians

            self.pub_pos.publish(estimate)
            pass

    def callback_encoder(self, data):
        """
        Callback function for the encoder data.
        """
        self.encoder = np.array(
            [data.encoder1, data.encoder2, data.encoder3, data.encoder4])

    def callback_pwm(self, data):
        """
        Callback function for the pwm data.
        """
        self.pwm = np.array([data.pwm1, data.pwm2, data.pwm3, data.pwm4])
        self.pwm_to_rad = self.pwmToRPPM(self.pwm)

    def pwmToRPPM(self, pwm):
        """
        Converts the pwm to rpm and rad/s.
        """
        pwm_to_rad = np.array([0, 0, 0, 0])
        for i in range(4):
            pwm_to_rad[i] = pwm[i] * (2 * math.pi) / 60
        return pwm_to_rad

    def sigma_points(self, mean, P):
        """
        Calculates the sigma points.
        """
        # initialize 2n+1 sigma points
        sigma_points = np.zeros((2*len(mean)+1, len(mean)))
        # calculate the cholesky decompositin of P
        L = np.linalg.cholesky(P)
        # calculate the first sigma point
        sigma_points[0, :] = mean
        # calculate the rest of the sigma points
        for i in range(len(mean)):
            sigma_points[i+1, :] = mean + L[:, i]*math.sqrt(3)
            sigma_points[i+1+len(mean), :] = mean - L[:, i]*math.sqrt(3)

        return sigma_points

    def motion_model(self, sigma_points):
        """
        Propogates the sigma points through the motion model.
        """
        del_pos, velocity = self.compute_state(self.pwm_to_rad)
        motion_model_prediction = np.zeros(np.shape(sigma_points))
        # add first 3 elewments of del_pos to the first 3 elements of sigma_points
        for i in range(np.shape(sigma_points)[0]):
            motion_model_prediction[i, 0] = sigma_points[i, 0] + del_pos[0]
            motion_model_prediction[i, 1] = sigma_points[i, 1] + del_pos[1]
            motion_model_prediction[i, 2] = sigma_points[i, 2] + del_pos[2]
            motion_model_prediction[i, 3] = velocity[0]
            motion_model_prediction[i, 4] = velocity[1]
            motion_model_prediction[i, 5] = velocity[2]

        return motion_model_prediction

    def compute_state(self, action):
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
            [[-1/(self.l+self.w), 1/(self.l+self.w), 1/(self.l+self.w), -1/(self.l+self.w)],
             [1, 1, 1, 1],
             [-1, 1, -1, 1]]))
        chasisPlanarTwist = (self.r/4) * matrix * np.mat(action) * self.time

        deltaQb1 = chasisPlanarTwist[0]
        deltaQb2 = chasisPlanarTwist[1]*np.sin(chasisPlanarTwist[0]) + chasisPlanarTwist[2]*(
            np.cos(chasisPlanarTwist[0])-1)/chasisPlanarTwist[0]
        deltaQb3 = chasisPlanarTwist[2]*np.sin(chasisPlanarTwist[0]) - chasisPlanarTwist[1]*(
            np.cos(chasisPlanarTwist[0])-1)/chasisPlanarTwist[0]
        if((float(chasisPlanarTwist[0])*180/np.pi) > 1 or (float(chasisPlanarTwist[0])*180/np.pi) < -1):
            deltaQb = np.transpose(
                np.mat(np.array([[deltaQb1, deltaQb2, deltaQb3]])))
        else:
            deltaQb = np.transpose(np.mat(
                np.array([[0, float(chasisPlanarTwist[1]), float(chasisPlanarTwist[2])]])))
        prev_phi = float(self.pose[0])
        transformation_matrix = np.mat(np.array([[1, 0, 0], [0, np.cos(
            prev_phi), -np.sin(prev_phi)], [0, np.sin(prev_phi), np.cos(prev_phi)]]))
        delQ = transformation_matrix * deltaQb
        return delQ, chasisPlanarTwist

    def calculate_mean_covariance(self, motion_model_prediction):
        """
        Calculates the mean and covariance of the motion model prediction.
        """

        N = self.N
        k = self.k

        m = np.shape(motion_model_prediction)[0]
        mean_predicted = np.zeros((N, 1))
        covariance_predicted = np.zeros((N, N))

        # weight of each row
        weights = np.zeros([m, 1])
        weights[0] = k/(N+k)
        for i in range(1, m):
            weights[i] = 1/(2*(N+k))
        weights = np.transpose(np.mat(weights))

        # calculate the mean of the predicted state
        for i in range(m):
            mean_predicted = mean_predicted + \
                float(weights[i])*motion_model_prediction[i, :]

        # calculate the covariance of the predicted state
        for i in range(m):
            covariance_predicted = covariance_predicted + float(weights[i])*np.mat(
                (motion_model_prediction[i, :]-mean_predicted)) * np.mat((motion_model_prediction[i, :]-mean_predicted)).T

        # add guassian noise to covariance_preidcted
        covariance_predicted = covariance_predicted + self.Q
        return mean_predicted, covariance_predicted, weights

    def measurement_model(self, X):
        """
        Calculates the wheel velocities for given state.
        """
        # initialize predicted measurements m*4 matrix
        predicted_measurement = np.zeros(np.shape(X)[0], 4)
        # calculate the predicted measurements
        for i in range(np.shape(X)[0]):
            predicted_measurement[i, 0] = X[i, 3] * \
                (-self.l - self.w) + X[i, 4] - X[i, 5]
            predicted_measurement[i, 1] = X[i, 3] * \
                (self.l + self.w) + X[i, 4] + X[i, 5]
            predicted_measurement[i, 2] = X[i, 3] * \
                (self.l + self.w) + X[i, 4] - X[i, 5]
            predicted_measurement[i, 3] = X[i, 3] * \
                (-self.l - self.w) + X[i, 4] + X[i, 5]

        return predicted_measurement

    def calculate_mean_covariance_measurement(self, Y):
        """
        Calculates the mean and covariance of the motion measurement prediction.
        """

        N = self.N
        k = self.k

        m = np.shape(Y)[0]

        y_len = np.shape(Y)[1]
        mean_predicted = np.zeros((y_len, 1))
        covariance_predicted = np.zeros((y_len, y_len))

        # weight of each row
        weights = np.zeros([m, 1])
        weights[0] = k/(N+k)
        for i in range(1, m):
            weights[i] = 1/(2*(N+k))
        weights = np.transpose(np.mat(weights))

        # calculate the mean of the predicted state
        for i in range(m):
            mean_predicted = mean_predicted + \
                float(weights[i])*Y[i, :]

        # calculate the covariance of the predicted state
        for i in range(m):
            covariance_predicted = covariance_predicted + float(weights[i])*np.mat(
                (Y[i, :]-mean_predicted)) * np.mat((Y[i, :]-mean_predicted)).T

        # add guassian noise to covariance_preidcted
        covariance_predicted = covariance_predicted + self.R
        return mean_predicted, covariance_predicted

    def kalman_gain(self, X, X_mean, Y, Y_mean, Py,weights):
        """calculates crooss covariance between X and Y and Kalman Gain

        Args:
            X (_ndarray_): shape (13,6)
            X_mean (_ndarray_): shape (6,1)
            Y (_ndarray_): shape (13,4)
            Y_mean (_ndarray_): shape (4,1)
        """
        # calculate the cross covariance between X and Y
        cross_covariance = np.mat(np.zeros((6, 4)))
        for i in range(np.shape(X)[0]):
            cross_covariance = cross_covariance + \
                float(weights[i])*np.mat(X[i, :] - X_mean) * \
                np.mat(Y[i, :] - Y_mean).T
        # calculate the Kalman Gain
        Kalman_gain = cross_covariance * \
            np.linalg.inv(Py)  ##6*4 matrix

        return Kalman_gain


if __name__ == '__main__':
    rospy.init("Ukalman_filter")
    UKF = UKF_pos()
    rospy.spin()
