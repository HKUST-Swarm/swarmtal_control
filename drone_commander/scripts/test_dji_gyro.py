#!/usr/bin/python
import rospy
from sensor_msgs.msg import Imu
import matplotlib.pyplot as plt
from pyquaternion import Quaternion
from tf.transformations import euler_from_quaternion, unit_vector


def update_quat_omg_dt(quat, omg, dt):
    ox, oy, oz = omg[0], omg[1], omg[2]
    qw, qx, qy, qz = quat[3], quat[0], quat[1], quat[2]
    dqw = -0.5*(ox*qx + oy*qy + oz*qz)
    dqx = 0.5*(ox*qw + oy*qz - oz*qy)
    dqy = 0.5*(oy*qw + oz*qx - ox*qz)
    dqz = 0.5*(oz*qw + ox*qy - oy*qx)


    q = [qx + dqx*dt, qy + dqy*dt, qz + dqz*dt, qw + dqw*dt]
    return unit_vector(q)

class GyroEasy:
    def __init__(self):
        self.roll = 0
        self.pitch = 0
        self.yaw = 0

        self.ts = rospy.get_time()
        self.roll_arr = []
        self.ts_arr = []

        self.last_ts = rospy.get_time()
        self.gyro_bias = [0, 0, 0]
        self.gyro_bias_sum = [0, 0, 0]
        self.gyro_bias_count = 0
        self.init_gyro_bias = True
        self.quat = [0, 0, 0, 1]
        self.init_time = 30

        rospy.loginfo("Start init Gyroscope")
        self.imu_sub = rospy.Subscriber("/dji_sdk_1/dji_sdk/imu", Imu, self.imu_callback, queue_size=1000)
        self.timer = rospy.Timer(rospy.Duration(1.0), self.timer_cb)
        

    def timer_cb(self, e):
        if len(self.ts_arr) == len(self.roll_arr) and len(self.ts_arr) > 0:
            plt.figure(0)
            plt.clf()
            plt.plot(self.ts_arr[-1000:-1], self.roll_arr[-1000:-1]   )
            plt.grid(which="both")
            plt.pause(0.1)
        print("RPY", self.roll*57.3, self.pitch*57.3, self.yaw*57.3)



    def imu_callback(self, _imu):
        omega = _imu.angular_velocity
        t = rospy.get_time()

        if t - self.ts < self.init_time:
            #10s to init
            self.gyro_bias_sum[0] = self.gyro_bias_sum[0] + omega.x
            self.gyro_bias_sum[1] = self.gyro_bias_sum[1] + omega.y
            self.gyro_bias_sum[2] = self.gyro_bias_sum[2] + omega.z
            self.gyro_bias_count += 1

            self.gyro_bias[0] = self.gyro_bias_sum[0] / self.gyro_bias_count
            self.gyro_bias[1] = self.gyro_bias_sum[1] / self.gyro_bias_count
            self.gyro_bias[2] = self.gyro_bias_sum[2] / self.gyro_bias_count
            return

        if self.init_gyro_bias:
            rospy.loginfo("finish init gyro bias {}".format(self.gyro_bias))
            self.init_gyro_bias = False

        omg = [omega.x - self.gyro_bias[0], omega.y - self.gyro_bias[1], omega.z - self.gyro_bias[2]]
        # self.roll = self.roll + (omega.x-self.gyro_bias[0]) * 0.0025


        self.quat = update_quat_omg_dt(self.quat, omg, 0.0025)
        self.roll, self.pitch, self.yaw = euler_from_quaternion(self.quat)
        """
        rospy.loginfo_throttle(1.0, "T {:3.4f} ROLL is {:7.6f}".format(
            t - self.ts,
            self.roll*57.3))
        """
        self.ts_arr.append(t - self.ts- self.init_time)
        self.roll_arr.append(self.roll*57.3)



if __name__ == "__main__":
    rospy.init_node("Gyro")
    rospy.loginfo("GRYO TEST")
    ge = GyroEasy()

    rospy.spin()