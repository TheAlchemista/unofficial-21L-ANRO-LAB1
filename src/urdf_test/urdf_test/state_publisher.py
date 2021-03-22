#! /usr/bin/env python
from math import sin, cos, pi
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped

# Project imports:
from urdf_test.Joint_move import Joint_move

class StatePublisher(Node):

  def __init__(self):
    rclpy.init()
    super().__init__('state_publisher')

    qos_profile = QoSProfile(depth=10)
    self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
    self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
    self.nodeName = self.get_name()
    self.get_logger().info("{0} started".format(self.nodeName))

    degree = pi / 180.0
    loop_rate = self.create_rate(30)

    joint_move = Joint_move()

    # robot state
    tilt = 0.
    tinc = degree
    swivel = 0.
    angle = 0.
    rotation = 0.

    # message declarations
    odom_trans = TransformStamped()
    odom_trans.header.frame_id = 'odom'
    odom_trans.child_frame_id = 'body'
    joint_state = JointState()

    try:
      while rclpy.ok():
        rclpy.spin_once(self)

        # update joint_state
        now = self.get_clock().now()
        joint_state.header.stamp = now.to_msg()
        joint_state.name = ['swivel', 'long', 'longer']
        joint_state.position = [swivel, joint_move.calc_height(), joint_move.calc_rotation()]

        # update transform
        # (moving in a circle with radius=2)
        odom_trans.header.stamp = now.to_msg()
        odom_trans.transform.translation.x = cos(angle)*0
        odom_trans.transform.translation.y = sin(angle)*0
        odom_trans.transform.translation.z = 0.0
        odom_trans.transform.rotation = \
            euler_to_quaternion(angle*0, 0, (angle + pi/2)*0) # roll,pitch,yaw

        # send the joint state and transform
        self.joint_pub.publish(joint_state)
        self.broadcaster.sendTransform(odom_trans)

        # Print new robot state
        #self.get_logger().info("Height: " + str(joint_move.height))

        
        # rotation speed actually
        rotation += 0.1
        # if height > 0.0 or height < 5.0:
        #     hinc *= -1
        # swivel += degree
        # angle += degree/4

        # This will adjust as needed per iteration
        loop_rate.sleep()

    except KeyboardInterrupt:
        pass

def get_height(height, hinc, go_up):
  if height < 0.4:
    go_up = True
  elif height > 2.0:
    go_up = False
  else:
    pass

  if go_up:
    height += hinc
  else:
    height -= hinc
  return height, go_up

def euler_to_quaternion(roll, pitch, yaw):
  qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
  qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
  qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
  qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
  return Quaternion(x=qx, y=qy, z=qz, w=qw)

def main():
  node = StatePublisher()

if __name__ == '__main__':
  main()