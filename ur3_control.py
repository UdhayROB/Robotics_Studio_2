#! /usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from onrobot_rg_control.msg import OnRobotRGOutput
from msg_package.msg import action_msg

storage_dict = {"1":[False, 0.1, -0.25, 0.2],
                "2":[False, 0.0, -0.25, 0.2],
                "3":[False, -0.1, -0.25, 0.2]
                };

def genCommand(char, command):  
    """ Updates the command according to the input character.

        Args:
            char (str): set command service request message
            command (OnRobotRGOutput): command to be sent

        Returns:
            command: command message with parameters set
    """
    # command = OnRobotRGOutput()
    if gtype == 'rg2':
        max_force = 400
        max_width = 1100
    elif gtype == 'rg6':
        max_force = 1200
        max_width = 1600
    else:
        rospy.signal_shutdown(
            rospy.get_name() +
            ": Select the gripper type from rg2 or rg6.")

    if char == 'c':
        command.rGFR = 400
        command.rGWD = 500#450
        command.rCTR = 16
    elif char == 'o':
        command.rGFR = 400
        command.rGWD = 800 # max_width
        command.rCTR = 16
    elif char == 'i':
        command.rGFR += 25
        command.rGFR = min(max_force, command.rGFR)
        command.rCTR = 16
    elif char == 'd':
        command.rGFR -= 25
        command.rGFR = max(0, command.rGFR)
        command.rCTR = 16
    else:
        # If the command entered is a int, assign this value to rGWD
        try:
            command.rGFR = 400
            command.rGWD = min(max_width, int(char))
            command.rCTR = 16
        except ValueError:
            pass

    return command

def approach(pose_msg, gripper_cmd, command): #gripper_cmd is a string- 'c' or 'o'
    group.clear_pose_targets()
    pose_target = geometry_msgs.msg.Pose()
    pose_target = copy.deepcopy(pose_msg)
    # Maintain orientation
    pose_target.orientation.y = 1.0
    pose_target.orientation.w = 0
    # Account for gripper length
    pose_target.position.z += 0.21
    approach_pose = geometry_msgs.msg.Pose()
    approach_pose = copy.deepcopy(pose_msg)
    approach_pose.orientation.y = 1.0
    approach_pose.orientation.w = 0
    approach_pose.position.z += 0.27
    group.set_pose_target(approach_pose)
    success = group.go(wait=True)
    if(success):
        group.stop()
        group.clear_pose_targets()
        group.set_pose_target(pose_target)
        success = group.go(wait=True)
        if(success):
            print("Successful")
            cmd = genCommand(gripper_cmd, command)
            pub.publish(cmd)
            rospy.sleep(0.1)
        else:
            print("Failed to plan/ reach object")
    else:
        print("Failed to plan/ reach approach pose")
    group.stop()
    group.clear_pose_targets()
    return success

def retreat():
    group.clear_pose_targets()
    pose_target = group.get_current_pose().pose
    # Maintain orientation
    pose_target.orientation.y = 1.0
    pose_target.orientation.w = 0
    # Account for gripper length
    pose_target.position.z += 0.05
    group.set_pose_target(pose_target)
    success = group.go(wait=True)
    if(success):
        print("Successful")
    else:
        print("Failed to retreat")
    group.stop()
    group.clear_pose_targets()
    return success

def control_callback_full(data):
    #Keyword based on turtlebots perspective- pickup from storage, drop to store
    if(data.action == "pickup"):
        package_exists = False
        storage_position = 10 # To be used to change state of occupancy in particular storage location
        pickup_pose = geometry_msgs.msg.Pose()
        drop_pose = geometry_msgs.msg.Pose()
        for keys in storage_dict.keys():
            if(storage_dict[keys][0] == True):
                pickup_pose.position.x = storage_dict[keys][1]
                pickup_pose.position.y = storage_dict[keys][2]
                pickup_pose.position.z = storage_dict[keys][3]
                pickup_pose.orientation.y = 1.0
                pickup_pose.orientation.w = 0.0
                package_exists = True
                storage_position = copy.deepcopy(keys)
                break
        if(package_exists):
            drop_pose = copy.deepcopy(data.pose_msg)
            drop_pose.orientation.y = 1.0
            drop_pose.orientation.w = 0.0
            command = OnRobotRGOutput()
            if(approach(pickup_pose, 'c', command)):
                retreat()
                command = OnRobotRGOutput()
                if(approach(drop_pose, 'o', command)):
                    retreat()
                    storage_dict[storage_position][0] = False
                    print(storage_dict)
        else:
            print("No package exists for pickup")
    elif(data.action == "drop"):
        space_available = False
        storage_position = 10 # To be used to change state of occupancy in particular storage location
        pickup_pose = geometry_msgs.msg.Pose()
        drop_pose = geometry_msgs.msg.Pose()
        # storage_dict_rev = 
        for keys in storage_dict.keys():
            if(storage_dict[keys][0] == False):
                drop_pose.position.x = storage_dict[keys][1]
                drop_pose.position.y = storage_dict[keys][2]
                drop_pose.position.z = storage_dict[keys][3]
                drop_pose.orientation.y = 1.0
                drop_pose.orientation.w = 0.0
                space_available = True
                storage_position = copy.deepcopy(keys)
                break
        if(space_available):
            pickup_pose = copy.deepcopy(data.pose_msg)
            pickup_pose.orientation.y = 1.0
            pickup_pose.orientation.w = 0.0
            command = OnRobotRGOutput()
            if(approach(pickup_pose, 'c', command)):
                retreat()
                command = OnRobotRGOutput()
                if(approach(drop_pose, 'o', command)):
                    retreat()
                    storage_dict[storage_position][0] = True
                    print(storage_dict)
        else:
            print("No storage space available")

def setJointConstraints():
    jointConstraint0 = moveit_msgs.msg.JointConstraint()
    jointConstraint0.joint_name = "shoulder_pan_joint"
    jointConstraint0.position = -0.25
    jointConstraint0.tolerance_above = 2.25
    jointConstraint0.tolerance_below = 2.25
    jointConstraint0.weight = 1

    jointConstraint1 = moveit_msgs.msg.JointConstraint()
    jointConstraint1.joint_name = "elbow_joint"
    jointConstraint1.position = 1.67
    jointConstraint1.tolerance_above = 1.5
    jointConstraint1.tolerance_below = 1.5
    jointConstraint1.weight = 1

    jointConstraint2 = moveit_msgs.msg.JointConstraint()
    jointConstraint2.joint_name = "wrist_2_joint"
    jointConstraint2.position = -1.57
    jointConstraint2.tolerance_above = 0.3
    jointConstraint2.tolerance_below = 0.3
    jointConstraint2.weight = 1

    jointConstraint3 = moveit_msgs.msg.JointConstraint()
    jointConstraint3.joint_name = "wrist_3_joint"
    jointConstraint3.position = 0
    jointConstraint3.tolerance_above = 3.14
    jointConstraint3.tolerance_below = 3.14
    jointConstraint3.weight = 1

    jointConstraint4 = moveit_msgs.msg.JointConstraint()
    jointConstraint4.joint_name = "shoulder_lift_joint"
    jointConstraint4.position = 0.0
    jointConstraint4.tolerance_above = 0.7 #was 0.3
    jointConstraint4.tolerance_below = 2.3
    jointConstraint4.weight = 1

    urConstraints = moveit_msgs.msg.Constraints()
    urConstraints.joint_constraints.append(jointConstraint0)
    urConstraints.joint_constraints.append(jointConstraint1)
    urConstraints.joint_constraints.append(jointConstraint2)
    urConstraints.joint_constraints.append(jointConstraint3)
    urConstraints.joint_constraints.append(jointConstraint4)
    group.set_path_constraints(urConstraints)

if __name__ == '__main__':
    
    try:
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()    
        group = moveit_commander.MoveGroupCommander("ur_manipulator")
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
        
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
        setJointConstraints()
        gtype = rospy.get_param('/onrobot/gripper', 'rg2')

        pub = rospy.Publisher('OnRobotRGOutput', OnRobotRGOutput, queue_size=1)
        sub = rospy.Subscriber("/ur3pose", action_msg, control_callback_full)
        rospy.spin()
        
    except KeyboardInterrupt:
        print("Keyboard interrupt")
        moveit_commander.roscpp_shutdown()
