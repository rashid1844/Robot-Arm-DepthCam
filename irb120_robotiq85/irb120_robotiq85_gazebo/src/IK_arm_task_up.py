#! /usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg # includes Pose
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# Init stuff
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('moving_irb120_robot', anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
arm_group = moveit_commander.MoveGroupCommander("irb_120")
hand_group = moveit_commander.MoveGroupCommander("robotiq_85")

# Publish trajectory in RViz
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)

# Move the Robotiq gripper by master axis
def move_joint_hand(gripper_finger1_joint):
    joint_goal = hand_group.get_current_joint_values()
    joint_goal[2] = gripper_finger1_joint # Gripper master axis

    hand_group.go(joint_goal, wait=True)
    hand_group.stop() # To guarantee no residual movement


# Inverse Kinematics (IK): move TCP to given position and orientation
def move_pose_arm(x,y,z,roll,pitch,yaw):
    pose_goal = geometry_msgs.msg.Pose()
    quat = quaternion_from_euler(roll,pitch,yaw)
    pose_goal.orientation.x = quat[0]
    pose_goal.orientation.y = quat[1]
    pose_goal.orientation.z = quat[2]
    pose_goal.orientation.w = quat[3]
    pose_goal.position.x = x
    pose_goal.position.y = y
    pose_goal.position.z = z
    arm_group.set_pose_target(pose_goal)

    plan = arm_group.go(wait=True)

    arm_group.stop() # To guarantee no residual movement
    arm_group.clear_pose_targets()

def loop_function(box_vector):
    rospy.loginfo("Moving arm to HOME point")	
    move_pose_arm(0.4,0,0.6,0,0.8,0)
    rospy.loginfo("Opening gripper")	
    move_joint_hand(0)
    rospy.sleep(1)	
    rospy.loginfo("Moving arm to pick box")
    rospy.loginfo(box_vector)	
    #move_pose_arm(0.5,0,0.19,0,pi/2,0)  # remove
    move_pose_arm(box_vector[0],box_vector[1]*.666,box_vector[2]+.28,0,pi/2,0) # box top
    move_pose_arm(box_vector[0],box_vector[1]*.666,box_vector[2]+.20,0,pi/2,0)  # box
    rospy.sleep(1)
    rospy.loginfo("Closing gripper to 0.25")	
    move_joint_hand(0.25)
    rospy.sleep(1)
    rospy.loginfo("Moving arm to target location")
    move_pose_arm(box_vector[0],box_vector[1]*.666,box_vector[2]+.28,0,pi/2,0) # box top
    move_pose_arm(0,-0.5,box_vector[2]+.33,0,pi/2,0) # target top
    move_pose_arm(0,-0.5,box_vector[2]+.25,0,pi/2,0)  # target  
    rospy.sleep(1)
    rospy.loginfo("Opening gripper")	
    move_joint_hand(0)
    rospy.sleep(1)
    rospy.loginfo("Moving arm to Home point")	
    move_pose_arm(0,-0.5,box_vector[2]+.33,0,pi/2,0) # target top
    #rospy.sleep(1)
    move_pose_arm(0.4,0,0.6,0,0.8,0)
    #rospy.loginfo("Closing gripper to 0.6")	
    #move_joint_hand(0.6)
    #rospy.sleep(1)
    rospy.loginfo("All movements finished. Shutting down")	
    moveit_commander.roscpp_shutdown()
    rospy.signal_shutdown("Done.")




def callback(data):
    box_vector = [data.position.x, data.position.y, data.position.z]
    loop_function(box_vector)




if __name__ == '__main__':
    #loop_function(None)
    rospy.Subscriber('/block_pose', geometry_msgs.msg.Pose, callback)
    rospy.spin()  
	
    #rospy.loginfo("All movements finished. Shutting down")	
    #moveit_commander.roscpp_shutdown()
