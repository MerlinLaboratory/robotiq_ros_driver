#!/usr/bin/env python3

import os
import rospy
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from std_msgs.msg import Float64, Float64MultiArray
from sensor_msgs.msg import JointState
from controller_manager_msgs.srv import ListControllers
from robotiq_msgs.msg import CModelCommand, CModelStatus


class ControllerEmulator(object):
  
  def __init__(self):
    
    # Setup publishers and subscribers
    # gripper_status_pub = rospy.Publisher('gripper/status', CModelStatus, queue_size=1)
    self.gripper_command_pub = rospy.Publisher('gripper_controller/command', Float64MultiArray, queue_size=1)
    
    rospy.Subscriber('joint_states', JointState, self.gripper_status_callback, queue_size=1)
    
    # Setup services that simulate services  
    open_gripper_service = rospy.Service('open_gripper', Trigger, self.open_gripper)
    close_gripper_service = rospy.Service('close_gripper', Trigger, self.close_gripper)
  
  def gripper_status_callback(self, msg):
    for joint_name, joint_pos in zip(msg.name, msg.position):
      if "hande_left_finger" in joint_name or "hande_right_finger" in joint_name:
        self.position = joint_pos
        break
  
  def open_gripper(self, req):
    msg = Float64MultiArray()
    msg.data.append(0.0)
    msg.data.append(0.0)
    
    self.gripper_command_pub.publish(msg)
    return TriggerResponse(True, "Gripper opened succesfully")
    
  
  def close_gripper(self, req):
    msg = Float64MultiArray()
    msg.data.append(0.5)
    msg.data.append(0.5)
    
    self.gripper_command_pub.publish(msg)
    return TriggerResponse(True, "Gripper closed succesfully")
    
  
if __name__ == "__main__":
  rospy.init_node("gripper_emulator")
  emulator = ControllerEmulator()
  rospy.spin()


