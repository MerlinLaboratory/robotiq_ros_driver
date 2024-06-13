#!/usr/bin/env python
import rospy
from robotiq_msgs.msg import CModelCommand, CModelStatus
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from sensor_msgs.msg import JointState
from time import sleep

class GripperController():
  
  def __init__(self):
    rospy.init_node('robotiq_controller')
    
    # Publishers/Subscribers
    self.pub_command = rospy.Publisher('command', CModelCommand, queue_size=3)
    self.joint_state_pub = rospy.Publisher('joint_states', JointState, queue_size=3)
    
    rospy.Subscriber('status', CModelStatus, self.gripper_status_cb)
    
    # TODO: obtain these from param file
    self.gripper_velocity = 255
    self.gripper_force = 150
    
    # TODO: get this from status value
    self.active = True
    
    # Instantiating services
    open_gripper_service = rospy.Service('open_gripper', Trigger, self.open_gripper)
    close_gripper_service = rospy.Service('close_gripper', Trigger, self.close_gripper)
    
  def open_gripper(self, req):
    self.check_is_active()
    command_open = self.gen_command_open()
    self.pub_command.publish(command_open)
    
    return TriggerResponse(True, "gripper open successfully")
  
  def close_gripper(self, req):
    self.check_is_active()
    command_close = self.gen_command_close()
    self.pub_command.publish(command_close)
    
    return TriggerResponse(True, "gripper closed successfully")
  
  def gripper_status_cb(self, msg):
    position = msg.gPO / 255.0 * 0.025 # meters
    
    joint_state_msg = JointState()
    joint_state_msg.header.frame_id = 'hand_e_link'
    joint_state_msg.header.stamp = rospy.Time.now()
    joint_state_msg.name = ['hande_left_finger_joint', 'hande_right_finger_joint']
    joint_state_msg.position = [position, position]
    joint_state_msg.velocity = [0.0, 0,0]
    joint_state_msg.effort   = [0.0, 0,0]
    
    self.joint_state_pub.publish(joint_state_msg)
    
  
  ##########################################################
  ##################### Utils functions ####################
  ##########################################################
  
  def check_is_active(self):
    if self.active == False:
      command_activate = self.gen_command_activate()
      self.pub_command(command_activate)
    
  def gen_command_activate(self):
    command = CModelCommand()
    command.rACT = 1
    command.rGTO = 1
    command.rSP  = self.gripper_velocity
    command.rFR  = self.gripper_force
    return command

  def gen_command_open(self):
    command = CModelCommand()
    command.rPR = 0
    return command

  def gen_command_close(self):
    command = CModelCommand()
    command.rPR = 255
    return command
  
  
if __name__ == '__main__':
  
  gripper_controller = GripperController()
  rospy.spin()
  