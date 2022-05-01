#!/usr/bin/env python3

import rospy
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
from sensor_msgs.msg import MagneticField
from geometry_msgs.msg import Twist
import numpy as np


class MagCalib:
    def __init__(self):
        self.mag_sub = rospy.Subscriber("mpu9250_mag_raw", MagneticField, self.MagCallback, queue_size=4000)
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.mag_pub = rospy.Publisher("mpu9250_mag", MagneticField, queue_size=10)
        self.service_start_calib = rospy.Service("~start_calib", Empty, self.StartCalibServer)
        self.states = ["Idle", "Collect Data", "Apply Calib"]
        self.curr_state = "Idle"
        self.sample_amount = 3000
        self.sample_counter = 0
        self.mag_raw_data = []
        self.mag_calib_data = []
        self.output_mag = MagneticField()
        self.center = np.ndarray(shape=(), dtype=float)
        self.evecs = np.ndarray(shape=(), dtype=float)
        self.radii = np.ndarray(shape=(), dtype=float)
        self.v = np.ndarray(shape=(), dtype=float)
        self.TR = np.ndarray(shape=(), dtype=float)

    def MagCallback(self, msg: MagneticField):

        self.mag_raw_data = [msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z]

        if self.curr_state == "Idle":
            rospy.loginfo_throttle(5.0, "Waiting for service call")
            return

        elif self.curr_state == "Collect Data":
            if self.sample_counter == 0:
                rospy.loginfo("Getting Mag Data")

            if self.sample_counter >= self.sample_amount:
                rospy.loginfo("Stop Rotating and Calculate params")
                cmd_vel = Twist()
                cmd_vel.linear.x = 0
                cmd_vel.linear.y = 0
                cmd_vel.angular.z = 0
                self.cmd_vel_pub.publish(cmd_vel)
                try:
                    self.DoCalib()
                except np.linalg.LinAlgError as e:
                    rospy.logerr(str(e))
                    rospy.logwarn("Cannot Solve Mag Calibration Matrix")
                    while not rospy.is_shutdown():
                        pass

                self.curr_state = "Apply Calib"
                rospy.loginfo("Ready to Apply Calib")
                rospy.loginfo(self.center)
                rospy.loginfo(self.radii)
                rospy.loginfo(self.evecs)
                rospy.loginfo(self.TR)
                return

            curr_progress = int(float(self.sample_counter / self.sample_amount) * 100.0)
            rospy.loginfo_throttle(2.0, f"Collecting Data : {curr_progress:02d} %")
            cmd_vel = Twist()
            cmd_vel.linear.x = 0
            cmd_vel.linear.y = 0
            cmd_vel.angular.z = 0.5
            self.cmd_vel_pub.publish(cmd_vel)
            self.sample_counter += 1
            self.mag_calib_data.append(self.mag_raw_data)

        elif self.curr_state == "Apply Calib":
            mag = self.TR.dot(self.mag_raw_data - self.center.T).T
            self.output_mag.header = msg.header
            self.output_mag.magnetic_field.x = mag[0]
            self.output_mag.magnetic_field.y = mag[1]
            self.output_mag.magnetic_field.z = mag[2]
            self.output_mag.magnetic_field_covariance = msg.magnetic_field_covariance
            self.mag_pub.publish(self.output_mag)

    def StartCalibServer(self, req: EmptyRequest):
        rospy.loginfo("now state is %s" % (self.curr_state))
        self.curr_state = "Collect Data"
        rospy.loginfo("Start Collect Mag Data: %s" % (self.mag_sub.name))
        return EmptyResponse()

    def DoCalib(self):
        self.ellipsoid_fit()

    def ellipsoid_fit(self):
        self.mag_calib_data = np.array(self.mag_calib_data)
        x = self.mag_calib_data[:, 0]
        y = self.mag_calib_data[:, 1]
        z = self.mag_calib_data[:, 2]
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

        self.CalcTansformMatrix()

    def ellipse_fit(self):
        pass

    def CalcTansformMatrix(self):
        a, b, c = self.radii
        r = abs(a * b * c) ** (1.0 / 3.0)
        D = np.array([[r / a, 0.0, 0.0], [0.0, r / b, 0.0], [0.0, 0.0, r / c]])
        self.TR = self.evecs.dot(D).dot(self.evecs.T)


if __name__ == "__main__":
    rospy.init_node("mag_calib")
    mag_calib = MagCalib()
    while not rospy.is_shutdown():
        rospy.spin()
