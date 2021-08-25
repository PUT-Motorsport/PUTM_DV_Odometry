#!/usr/bin/env python3

import can
import rospy

# Speed factor of the wheels
SPEED_FACTOR = 11/53


class CANReceiver:
    def __init__(self):
        # Frame ID for front wheels' rotation speed
        self.front_card_id = 0x0d
        # Same as before but for back wheels
        self.back_card_id = 0x60
        # Frame ID for accelometer
        self.acc_id = 0x5a
        # Mapping: [ front_left, front_right, back_right, back_left ]
        self.wheel_rotations = [ 0, 0, 0, 0 ]
        # CAN bus data
        self.interface = 'vcan0'
        self.bustype = 'socketcan'
        self.node_name = 'can2flexbe'
        self.bus = can.interface.Bus(self.interface, bustype=self.bustype)
        # Accelerometer data
        self.accX = 0
        self.accY = 0
        self.accZ = 0
        # Increasing sequence_id
        self.seq_id = 0

    
    # Convert array of 2 8-bit values into 16-bit value
    def to16bit(self, value):
        return ((value[0] & 0xFF) << 8) | (value[1] & 0xFF)
    

    def update_rotation(self, update_front, rot_left, rot_right):
        speed_left = self.to16bit(rot_left)
        speed_right = self.to16bit(rot_right)

        left_wheel = ((speed_left * SPEED_FACTOR * 1.63363) / 60) * 3.6
        right_wheel = ((speed_right * SPEED_FACTOR * 1.63363) / 60) * 3.6

        if update_front:
            self.wheel_rotations[0] = left_wheel
            self.wheel_rotations[1] = right_wheel
        
        else:
            self.wheel_rotations[2] = right_wheel
            self.wheel_rotations[3] = left_wheel
    

    def update_acceleration(self, acc):
        self.accX = self.to16bit(acc[0:2])
        self.accY = self.to16bit(acc[2:4])
        self.accZ = self.to16bit(acc[4:6])

    
    def translate(self, channelId, message):
        if channelId == self.front_card_id:
            self.update_rotation(True, message[1:3], message[3:5])
        elif channelId == self.back_card_id:
            self.update_rotation(True, message[0:2], message[2:4])
        elif channelId == self.acc_id:
            self.update_acceleration(message[0:6])


    def listen(self):
        message = self.bus.recv()
        self.translate(message.arbitration_id, message.data)
        self.sleep_node.sleep()


    def start(self):
        rospy.init_node(self.node_name, anonymous=True)
        self.sleep_node = rospy.Rate(10)
        try:
            while not rospy.is_shutdown():
                self.listen()
        except rospy.ROSInterruptException:
            pass
        except AttributeError:
            pass


if __name__ == '__main__':
    receiver = CANReceiver()
    print("Listener started")
    receiver.start()
