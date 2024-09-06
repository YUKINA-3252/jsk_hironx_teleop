#!/usr/bin/env python3

import numpy as np
import rospy
import serial
import serial.tools.list_ports
from jsk_hironx_teleop.msg import ContactSensorForceMoment

def publisher(force, moment):
    while not rospy.is_shutdown():
        msg = ContactSensorForceMoment()
        msg.force = force
        msg.moment = moment
        pub.publish(msg)
        rate.sleep()

class MultiEMAFilter:
    def __init__(self, alpha, num_elements):
        self.alpha = alpha
        self.ema_values = [None] * num_elements
    def update(self, new_values):
        smoothed_values = []
        for i, new_value in enumerate(new_values):
            if self.ema_values[i] is None:
                self.ema_values[i] = new_value
            else:
                self.ema_values[i] = self.alpha * new_value + (1 - self.alpha) * self.ema_values[i]
            smoothed_values.append(self.ema_values[i])
        return smoothed_values

Serial_Port = serial.Serial(port='/dev/ttyACM0', baudrate=230400, parity='N', timeout=1)

Serial_Port.write(b'020202\r\n')

pub = rospy.Publisher('contact_sensor_force_moment_topic', ContactSensorForceMoment, queue_size=10)
rospy.init_node('contact_sensor_force_moment_publisher', anonymous=True)
rate = rospy.Rate(92)

alpha = 0.3
num_elements = 6
ema_filter = MultiEMAFilter(alpha, num_elements)

try:
    while True:
        data = Serial_Port.readline().decode('utf-8').strip()

        if data:
            # convert to decimal
            sub_data = data[4:]
            decimal_data = [int(sub_data[i:i+4], 16) for i in range(0, len(sub_data), 4)]
            adjust_decimal_data = [(value - 2048) for value in decimal_data]
            adjust_decimal_data = [adjust_decimal_data[0] * 0.01,
                                   adjust_decimal_data[1] * 0.01,
                                   adjust_decimal_data[2] * 0.02,
                                   adjust_decimal_data[3] * 0.05,
                                   adjust_decimal_data[4] * 0.05,
                                   adjust_decimal_data[5] * 0.05
                                   ]
            smoothed_adjust_decimal_data = ema_filter.update(adjust_decimal_data)
            msg = ContactSensorForceMoment()
            msg.header.stamp = rospy.Time.now()
            msg.force = smoothed_adjust_decimal_data[:3]
            msg.moment = smoothed_adjust_decimal_data[3:]
            pub.publish(msg)
            rate.sleep()

            # print(adjust_decimal_data)

except KeyboardInterrupt:
    print("Program interrupted")

finally:
    Serial_Port.close()
