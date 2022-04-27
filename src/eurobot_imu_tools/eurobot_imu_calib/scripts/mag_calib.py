#!/usr/bin/env python3

import rospy
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
from sensor_msgs.msg import MagneticField
from geometry_msgs.msg import Twist
import numpy as np


class MagCalib:
    def __init__(self):
        self.mag_sub = rospy.Subscriber("mpu9250_mag_raw", MagneticField, self.MagCallback, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.service_start_calib = rospy.Service("~start_calib", Empty, self.start_calib_server)
        self.states = ["Idle", "Collect Data", "Apply Calib"]
        self.curr_state = "Idle"
        self.sample_amount = 3000
        self.sample_counter = 0
        self.mag_data = np.ndarray(shape=(), dtype=float)
        self.center = np.ndarray(shape=(), dtype=float)
        self.evecs = np.ndarray(shape=(), dtype=float)
        self.radii = np.ndarray(shape=(), dtype=float)
        self.v = np.ndarray(shape=(), dtype=float)

    def MagCallback(self, data: MagneticField):
        if self.curr_state == "Idle":
            return

        elif self.curr_state == "Collect Data":
            if self.sample_counter >= self.sample_amount:
                rospy.loginfo("Stop Rotating and Calculate params")
                cmd_vel = Twist()
                cmd_vel.linear.x = 0
                cmd_vel.linear.y = 0
                cmd_vel.angular.z = 0
                self.cmd_vel_pub.publish(cmd_vel)

                self.CalcCalibParams()

                self.curr_state = "Apply Calib"
                rospy.loginfo("Apply Calib")

            if self.sample_counter == 0:
                rospy.loginfo("Getting Mag Data")
                cmd_vel = Twist()
                cmd_vel.linear.x = 0
                cmd_vel.linear.y = 0
                cmd_vel.angular.z = 0.5
                self.cmd_vel_pub.publish(cmd_vel)

            self.sample_counter += 1
            self.mag_data = np.append(self.mag_data, (data.magnetic_field.x, data.magnetic_field.y, data.magnetic_field.z))

        elif self.curr_state == "Apply Calib":
            pass

    def start_calib_server(self, req: EmptyRequest):
        rospy.loginfo("now state is %s" % (self.curr_state))
        if self.curr_state == "Idle":
            self.curr_state = "Collect Data"
            rospy.loginfo("Start Collect Mag Data: %s" % (self.mag_sub.name))
        return EmptyResponse()

    def CalcCalibParams(self):
        self.ellipsoid_fit()

    def ellipsoid_fit(self):
        x = mag_data[:, 0]
        y = mag_data[:, 1]
        z = mag_data[:, 2]
        D = np.array([x * x + y * y - 2 * z * z, x * x + z * z - 2 * y * y, 2 * x * y, 2 * x * z, 2 * y * z, 2 * x, 2 * y, 2 * z, 1 - 0 * x])
        d2 = np.array(x * x + y * y + z * z).T  # rhs for LLSQ
        u = np.linalg.solve(D.dot(D.T), D.dot(d2))
        a = np.array([u[0] + 1 * u[1] - 1])
        b = np.array([u[0] - 2 * u[1] - 1])
        c = np.array([u[1] - 2 * u[0] - 1])
        v = np.concatenate([a, b, c, u[2:]], axis=0).flatten()
        A = np.array([[v[0], v[3], v[4], v[6]], [v[3], v[1], v[5], v[7]], [v[4], v[5], v[2], v[8]], [v[6], v[7], v[8], v[9]]])

        center = np.linalg.solve(-A[:3, :3], v[6:9])

        translation_matrix = np.eye(4)
        translation_matrix[3, :3] = center.T

        R = translation_matrix.dot(A).dot(translation_matrix.T)

        evals, evecs = np.linalg.eig(R[:3, :3] / -R[3, 3])
        evecs = evecs.T

        radii = np.sqrt(1.0 / np.abs(evals))
        radii *= np.sign(evals)
        
        self.center = center
        self.evecs = evecs
        self.radii = radii
        self.v = v


if __name__ == "__main__":
    rospy.init_node("mag_calib")
    mag_calib = MagCalib()
    while not rospy.is_shutdown():
        rospy.spin()
