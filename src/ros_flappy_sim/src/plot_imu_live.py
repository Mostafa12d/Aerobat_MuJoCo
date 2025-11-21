#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
import matplotlib.pyplot as plt
from collections import deque
import numpy as np

class IMUPlotter:
    def __init__(self, buffer_size=500):
        rospy.init_node('imu_plotter', anonymous=True)
        
        self.buffer_size = buffer_size
        
        # Data buffers for Core IMU
        self.core_time = deque(maxlen=buffer_size)
        self.core_gyro_x = deque(maxlen=buffer_size)
        self.core_gyro_y = deque(maxlen=buffer_size)
        self.core_gyro_z = deque(maxlen=buffer_size)
        self.core_accel_x = deque(maxlen=buffer_size)
        self.core_accel_y = deque(maxlen=buffer_size)
        self.core_accel_z = deque(maxlen=buffer_size)
        
        # Data buffers for Guard IMU
        self.guard_time = deque(maxlen=buffer_size)
        self.guard_gyro_x = deque(maxlen=buffer_size)
        self.guard_gyro_y = deque(maxlen=buffer_size)
        self.guard_gyro_z = deque(maxlen=buffer_size)
        self.guard_accel_x = deque(maxlen=buffer_size)
        self.guard_accel_y = deque(maxlen=buffer_size)
        self.guard_accel_z = deque(maxlen=buffer_size)
        
        self.start_time = None
        
        # Subscribe to both IMU topics
        rospy.Subscriber('/flappy/core/imu', Imu, self.core_callback)
        rospy.Subscriber('/flappy/guard/imu', Imu, self.guard_callback)
        
        # Setup plot
        plt.ion()
        self.fig, self.axes = plt.subplots(2, 2, figsize=(14, 8))
        self.fig.suptitle('Live IMU Data', fontsize=16)
        
        # Configure subplots
        self.axes[0, 0].set_title('Core Gyroscope (rad/s)')
        self.axes[0, 0].set_xlabel('Time (s)')
        self.axes[0, 0].set_ylabel('Angular Velocity')
        self.axes[0, 0].grid(True)
        
        self.axes[0, 1].set_title('Core Accelerometer (m/s²)')
        self.axes[0, 1].set_xlabel('Time (s)')
        self.axes[0, 1].set_ylabel('Acceleration')
        self.axes[0, 1].grid(True)
        
        self.axes[1, 0].set_title('Guard Gyroscope (rad/s)')
        self.axes[1, 0].set_xlabel('Time (s)')
        self.axes[1, 0].set_ylabel('Angular Velocity')
        self.axes[1, 0].grid(True)
        
        self.axes[1, 1].set_title('Guard Accelerometer (m/s²)')
        self.axes[1, 1].set_xlabel('Time (s)')
        self.axes[1, 1].set_ylabel('Acceleration')
        self.axes[1, 1].grid(True)
        
        plt.tight_layout()
        
    def core_callback(self, msg):
        if self.start_time is None:
            self.start_time = msg.header.stamp.to_sec()
        
        t = msg.header.stamp.to_sec() - self.start_time
        self.core_time.append(t)
        self.core_gyro_x.append(msg.angular_velocity.x)
        self.core_gyro_y.append(msg.angular_velocity.y)
        self.core_gyro_z.append(msg.angular_velocity.z)
        self.core_accel_x.append(msg.linear_acceleration.x)
        self.core_accel_y.append(msg.linear_acceleration.y)
        self.core_accel_z.append(msg.linear_acceleration.z)
        
    def guard_callback(self, msg):
        if self.start_time is None:
            self.start_time = msg.header.stamp.to_sec()
        
        t = msg.header.stamp.to_sec() - self.start_time
        self.guard_time.append(t)
        self.guard_gyro_x.append(msg.angular_velocity.x)
        self.guard_gyro_y.append(msg.angular_velocity.y)
        self.guard_gyro_z.append(msg.angular_velocity.z)
        self.guard_accel_x.append(msg.linear_acceleration.x)
        self.guard_accel_y.append(msg.linear_acceleration.y)
        self.guard_accel_z.append(msg.linear_acceleration.z)
        
    def update_plot(self):
        if len(self.core_time) == 0:
            return
        
        # Clear all axes
        for ax in self.axes.flat:
            ax.clear()
        
        # Core Gyroscope
        if len(self.core_time) > 0:
            self.axes[0, 0].plot(self.core_time, self.core_gyro_x, 'r-', label='X', linewidth=1)
            self.axes[0, 0].plot(self.core_time, self.core_gyro_y, 'g-', label='Y', linewidth=1)
            self.axes[0, 0].plot(self.core_time, self.core_gyro_z, 'b-', label='Z', linewidth=1)
            self.axes[0, 0].set_title('Core Gyroscope (rad/s)')
            self.axes[0, 0].set_xlabel('Time (s)')
            self.axes[0, 0].set_ylabel('Angular Velocity')
            self.axes[0, 0].legend(loc='upper right')
            self.axes[0, 0].grid(True, alpha=0.3)
            
            # Core Accelerometer
            self.axes[0, 1].plot(self.core_time, self.core_accel_x, 'r-', label='X', linewidth=1)
            self.axes[0, 1].plot(self.core_time, self.core_accel_y, 'g-', label='Y', linewidth=1)
            self.axes[0, 1].plot(self.core_time, self.core_accel_z, 'b-', label='Z', linewidth=1)
            self.axes[0, 1].set_title('Core Accelerometer (m/s²)')
            self.axes[0, 1].set_xlabel('Time (s)')
            self.axes[0, 1].set_ylabel('Acceleration')
            self.axes[0, 1].legend(loc='upper right')
            self.axes[0, 1].grid(True, alpha=0.3)
        
        # Guard Gyroscope
        if len(self.guard_time) > 0:
            self.axes[1, 0].plot(self.guard_time, self.guard_gyro_x, 'r-', label='X', linewidth=1)
            self.axes[1, 0].plot(self.guard_time, self.guard_gyro_y, 'g-', label='Y', linewidth=1)
            self.axes[1, 0].plot(self.guard_time, self.guard_gyro_z, 'b-', label='Z', linewidth=1)
            self.axes[1, 0].set_title('Guard Gyroscope (rad/s)')
            self.axes[1, 0].set_xlabel('Time (s)')
            self.axes[1, 0].set_ylabel('Angular Velocity')
            self.axes[1, 0].legend(loc='upper right')
            self.axes[1, 0].grid(True, alpha=0.3)
            
            # Guard Accelerometer
            self.axes[1, 1].plot(self.guard_time, self.guard_accel_x, 'r-', label='X', linewidth=1)
            self.axes[1, 1].plot(self.guard_time, self.guard_accel_y, 'g-', label='Y', linewidth=1)
            self.axes[1, 1].plot(self.guard_time, self.guard_accel_z, 'b-', label='Z', linewidth=1)
            self.axes[1, 1].set_title('Guard Accelerometer (m/s²)')
            self.axes[1, 1].set_xlabel('Time (s)')
            self.axes[1, 1].set_ylabel('Acceleration')
            self.axes[1, 1].legend(loc='upper right')
            self.axes[1, 1].grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.pause(0.01)
        
    def run(self):
        rate = rospy.Rate(10)  # 10 Hz update rate for plots
        print("IMU Plotter started. Waiting for data...")
        print("Press Ctrl+C to stop.")
        
        try:
            while not rospy.is_shutdown():
                self.update_plot()
                rate.sleep()
        except KeyboardInterrupt:
            print("\nShutting down IMU plotter...")
        
        plt.ioff()
        plt.show()

if __name__ == '__main__':
    try:
        plotter = IMUPlotter(buffer_size=500)  # 500 samples ≈ 2.5 seconds at 200Hz
        plotter.run()
    except rospy.ROSInterruptException:
        pass
