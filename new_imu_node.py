#!/usr/bin/env python3

import time
import smbus
import rospy
import numpy as np
from sensor_msgs.msg import Temperature, Imu
from tf.transformations import quaternion_from_euler, quaternion_multiply, euler_from_quaternion
from mpu_6050_driver.registers import PWR_MGMT_1, ACCEL_XOUT_H, ACCEL_YOUT_H, ACCEL_ZOUT_H, TEMP_H, GYRO_XOUT_H, GYRO_YOUT_H, GYRO_ZOUT_H

ADDR = None
bus = None
IMU_FRAME = None

def read_word(adr):
    high = bus.read_byte_data(ADDR, adr)
    low = bus.read_byte_data(ADDR, adr + 1)
    val = (high << 8) + low
    return val

def read_word_2c(adr):
    val = read_word(adr)
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val

def publish_temp(timer_event):
    temp_msg = Temperature()
    temp_msg.header.frame_id = IMU_FRAME
    temp_msg.temperature = read_word_2c(TEMP_H) / 340.0 + 36.53
    temp_msg.header.stamp = rospy.Time.now()
    temp_pub.publish(temp_msg)

# Gyroscope bias
gyro_bias = [0.0, 0.0, 0.0]

def calibrate_gyro():
    global gyro_bias
    gyro_x_total = 0.0
    gyro_y_total = 0.0
    gyro_z_total = 0.0
    num_samples = 1000
    for i in range(num_samples):
        gyro_x_total += read_word_2c(GYRO_XOUT_H) / 131.0
        gyro_y_total += read_word_2c(GYRO_YOUT_H) / 131.0
        gyro_z_total += read_word_2c(GYRO_ZOUT_H) / 131.0
        time.sleep(0.01)
    gyro_bias[0] = gyro_x_total / num_samples
    gyro_bias[1] = gyro_y_total / num_samples
    gyro_bias[2] = gyro_z_total / num_samples

prev_time = None
orientation_q = [1.0, 0.0, 0.0, 0.0]
alpha = 0.98  # Complementary filter constant

def integrate_gyroscope(gyro_x, gyro_y, gyro_z, dt):
    # Convert gyroscope degrees/sec to radians/sec
    gyro_x_rad = np.deg2rad(gyro_x)
    gyro_y_rad = np.deg2rad(gyro_y)
    gyro_z_rad = np.deg2rad(gyro_z)
    
    # Create the delta quaternion from the gyroscope readings
    delta_q = quaternion_from_euler(gyro_x_rad * dt, gyro_y_rad * dt, gyro_z_rad * dt)
    
    return delta_q

def publish_imu(timer_event):
    global prev_time, orientation_q

    imu_msg = Imu()
    imu_msg.header.frame_id = IMU_FRAME

    # Read the acceleration vals
    accel_x = read_word_2c(ACCEL_XOUT_H) / 16384.0
    accel_y = read_word_2c(ACCEL_YOUT_H) / 16384.0
    accel_z = read_word_2c(ACCEL_ZOUT_H) / 16384.0

    # Adjust accelerometer readings to match expected coordinate system
    accel_x = -accel_x
    accel_y = -accel_y  # y is reversed
    accel_z = accel_z  # z is reversed

    # Read the gyro vals and subtract the bias
    gyro_x = (read_word_2c(GYRO_XOUT_H) / 131.0) - gyro_bias[0]
    gyro_y = (read_word_2c(GYRO_YOUT_H) / 131.0) - gyro_bias[1]
    gyro_z = (read_word_2c(GYRO_ZOUT_H) / 131.0) - gyro_bias[2]

    # Adjust gyroscope readings to match expected coordinate system
    gyro_x = -gyro_x
    gyro_y = -gyro_y  # y is reversed
    gyro_z = gyro_z  # z is reversed

    current_time = rospy.Time.now()
    if prev_time is None:
        prev_time = current_time
        return

    dt = (current_time - prev_time).to_sec()
    prev_time = current_time

    # Integrate gyroscope data to get the delta quaternion
    delta_q = integrate_gyroscope(gyro_x, gyro_y, gyro_z, dt)
    
    # Update the current orientation quaternion using the complementary filter
    orientation_q = quaternion_multiply(orientation_q, delta_q)

    # Normalize the quaternion to avoid drift over time
    orientation_q = np.array(orientation_q)
    orientation_q /= np.linalg.norm(orientation_q)

    # Calculate the roll and pitch from accelerometer data
    accel_angle_pitch = np.arctan2(accel_y, np.sqrt(accel_x**2 + accel_z**2))
    accel_angle_roll = np.arctan2(-accel_x, np.sqrt(accel_y**2 + accel_z**2))

    # Convert the accelerometer angles to a quaternion
    accel_q = quaternion_from_euler(accel_angle_roll, accel_angle_pitch, 0.0)

    # Convert current orientation to roll, pitch, yaw
    current_rpy = euler_from_quaternion(orientation_q)

    # Use complementary filter to combine accelerometer and gyroscope data
    fused_roll = alpha * current_rpy[0] + (1.0 - alpha) * accel_angle_roll
    fused_pitch = alpha * current_rpy[1] + (1.0 - alpha) * accel_angle_pitch
    fused_yaw = current_rpy[2]  # Yaw is only updated by gyro

    # Convert fused RPY back to quaternion
    fused_q = quaternion_from_euler(fused_roll, fused_pitch, fused_yaw)
    
    # Load up the IMU message
    o = imu_msg.orientation
    o.x, o.y, o.z, o.w = fused_q

    imu_msg.linear_acceleration.x = accel_x
    imu_msg.linear_acceleration.y = accel_y
    imu_msg.linear_acceleration.z = accel_z

    imu_msg.angular_velocity.x = gyro_x
    imu_msg.angular_velocity.y = gyro_y
    imu_msg.angular_velocity.z = gyro_z

    imu_msg.header.stamp = current_time

    imu_pub.publish(imu_msg)

temp_pub = None
imu_pub = None

if __name__ == '__main__':
    rospy.init_node('imu_node')

    bus = smbus.SMBus(rospy.get_param('~bus', 1))
    ADDR = rospy.get_param('~device_address', 0x68)
    if type(ADDR) == str:
        ADDR = int(ADDR, 16)

    IMU_FRAME = rospy.get_param('~imu_frame', 'imu_link')

    # Retry initializing the bus
    for i in range(3):
        try:
            bus.write_byte_data(ADDR, PWR_MGMT_1, 0)
            break
        except OSError as e:
            if i == 2:
                rospy.logerr("Failed to initialize MPU-6050: {}".format(e))
                exit(1)
            time.sleep(1)

    # Calibrate gyroscope to find the bias
    calibrate_gyro()

    temp_pub = rospy.Publisher('temperature', Temperature, queue_size=10)
    imu_pub = rospy.Publisher('imu/data', Imu, queue_size=10)
    imu_timer = rospy.Timer(rospy.Duration(0.02), publish_imu)
    temp_timer = rospy.Timer(rospy.Duration(10), publish_temp)
    rospy.spin()

