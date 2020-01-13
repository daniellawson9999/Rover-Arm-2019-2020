import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from moveit_msgs.msg import MoveGroupActionFeedback
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from sensor_msgs.msg import Joy
from moveit_commander.conversions import pose_to_list

class ArmController:
    def __init__(self):

        self.state = "IDLE"
        self.delay = 0
        self.data = {}
        self.increment = .1

        # init moveit and ros node
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('forward_kinematics_test', anonymous=True)

        # init robot model to get joint states / kinematic model
        robot = moveit_commander.RobotCommander()

        scene = moveit_commander.PlanningSceneInterface()

        # get the move group associated w/ the arm we are using
        group_name = "arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)
        # We can get the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        print "============ Planning frame: %s" % planning_frame

        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        print "============ End effector link: %s" % eef_link

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print "============ Available Planning Groups:", robot.get_group_names()

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print "============ Printing robot state"
        print robot.get_current_state()
        print ""
        ## END_SUB_TUTORIAL

        # Misc variables
        self.box_name = ''
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    def start(self):
        def callback_joy(data):
            self.data = data

        rospy.Subscriber("joy", Joy, callback_joy)

        def callback_feedback(data):
            self.state = data.feedback.state         
        
        rospy.Subscriber("move_group/feedback", MoveGroupActionFeedback, callback_feedback)
        
        while not rospy.core.is_shutdown():
            if self.state == "IDLE" and self.data:
                hip = self.data.axes[6]
                shoulder = self.data.axes[1]
                elbow = self.data.axes[4]
                wrist = self.data.axes[7]

                new_values =  self.move_group.get_current_joint_values()
                original_values = new_values[:]

                if hip > 0:
                    new_values[0] += self.increment
                elif hip < 0:
                    new_values[0] -= self.increment

                if shoulder > 0:
                    new_values[1] += self.increment
                elif shoulder < 0:
                    new_values[1] -= self.increment

                if elbow > 0:
                    new_values[2] += self.increment
                elif elbow < 0:
                    new_values[2] -= self.increment

                if wrist > 0:
                    new_values[3] += self.increment
                elif wrist < 0:
                    new_values[3] -= self.increment

                
                if original_values != new_values:
                    self.state = "RUNNING"
                    self.go_to_joint_state(*new_values)
            #rospy.rostime.wallsleep(0.5)




    # test function for arm controller
    def go_to_joint_state(self, hip = 0, shoulder = -.5 , elbow = .5 , wrist = 0):

        joint_goal = self.move_group.get_current_joint_values()
        '''
        rostopic pub /arm_controller/command trajectory_msgs/JointTrajectory '{joint_names: ["hip", "shoulder", "elbow", "wrist"], 
        points: [{positions: [0, -0.5, 0.5, 0], time_from_start: [.5, 0.0]}]}'
        '''
        # joints ["hip", "shoulder", "elbow", "wrist"]
        joint_goal[0] = hip
        joint_goal[1] = shoulder
        joint_goal[2] = elbow
        joint_goal[3] = wrist
    
        self.move_group.go(joint_goal, wait=False)

        # Calling ``stop()`` ensures that there is no residual movement
        #self.move_group.stop()



if __name__ == "__main__":
    arm_controller =  ArmController()
    arm_controller.start()
    #arm_controller.go_to_joint_state()
    