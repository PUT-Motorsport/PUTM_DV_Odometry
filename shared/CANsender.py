#!/usr/bin/env python3
import rospy
import time
import can


bustype = 'socketcan'
channel = 'vcan0'


class CANSender:
    def __init__(self):
        self.bustype = 'socketcan'
        self.channel = 'vcan0'
        self.bus = can.interface.Bus(channel=self.channel, bustype=self.bustype)
        self.nodeName = 'CANsender'
        self.sendDelay = 0.1


    def sendMessage(self, channelId, data):
        msg = can.Message(arbitration_id=channelId, extended_id=False, is_remote_frame=False, is_error_frame=False, data=data)
        self.bus.send(msg)
        time.sleep(self.sendDelay)


    def start(self):
        rospy.init_node(self.nodeName, anonymous=True)
        # Front wheel rotations / s
        self.sendMessage(0x0d, [0x0, 0x0, 0x64, 0x0, 0x64, 0x0, 0x0, 0x0])
        # Accelerometer
        self.sendMessage(0x5a, [0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0])
        # Back wheel rotations / s
        self.sendMessage(0x60, [0x0, 0x64, 0x0, 0x64, 0x0, 0x0, 0x0, 0x0])
        rospy.spin()


if __name__ == '__main__':
    sender = CANSender()
    sender.start()
