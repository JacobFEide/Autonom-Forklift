import rclpy
import math

from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory , JointTrajectoryPoint


class TrajectoryPublisher(Node):

    def __init__(self):
        super().__init__('topic_desired_trajectory_publisher_node')
        
        timer_period = 1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.trajectory_publisher = self.create_publisher(JointTrajectory,"/robot_manipulator_controller/joint_trajectory", 10)


    def timer_callback(self):

        # creating a point
        goal_positions_1 = [0.0,0.0,0.0,0.0]
        goal_positions_2 = [math.pi/2, math.pi/5.14, -(math.pi/1.89), -(math.pi/6)] # PLUKKE
        goal_positions_3 = [math.pi/2, math.pi / 4, -(math.pi / 1.89), -(math.pi / 4.5)] # DRA UT 
        goal_positions_4 = [0.0,-(6/20*math.pi),-(5/6*math.pi),(13/20*math.pi)] # HVIL PÅ BASE
        goal_positions_5 = [math.pi/2, math.pi/5.14, -(math.pi/1.89), -(math.pi/6)] # PLUKKE
        goal_positions_6 = [math.pi,-(math.pi/3),-(math.pi/1.5),math.pi/2] #  PLASSERE
        goal_positions_7 = [math.pi/2, math.pi/5.14, -(math.pi/1.89), -(math.pi/6)] # PLUKKE
        goal_positions_8 = [0.0,-(math.pi/3.25),-(math.pi/1.2),(math.pi/1.565)] # HVIL PÅ BASE
        
        point_msg_1 = JointTrajectoryPoint()
        point_msg_1.positions = goal_positions_1
        point_msg_1.time_from_start = Duration(sec=3)

        point_msg_2 = JointTrajectoryPoint()
        point_msg_2.positions = goal_positions_2
        point_msg_2.time_from_start = Duration(sec=8)

        point_msg_3 = JointTrajectoryPoint()
        point_msg_3.positions = goal_positions_3
        point_msg_3.time_from_start = Duration(sec=13)

        point_msg_4 = JointTrajectoryPoint()
        point_msg_4.positions = goal_positions_4
        point_msg_4.time_from_start = Duration(sec=18)

        point_msg_5 = JointTrajectoryPoint()
        point_msg_5.positions = goal_positions_5
        point_msg_5.time_from_start = Duration(sec=23)

        point_msg_6 = JointTrajectoryPoint()
        point_msg_6.positions = goal_positions_6
        point_msg_6.time_from_start = Duration(sec=28)

        point_msg_7 = JointTrajectoryPoint()
        point_msg_7.positions = goal_positions_7
        point_msg_7.time_from_start = Duration(sec=32)

        point_msg_8 = JointTrajectoryPoint()
        point_msg_8.positions = goal_positions_8
        point_msg_8.time_from_start = Duration(sec=37)


        # adding newly created point into trajectory message
        joints = ['arm_base_help_joint', 'arm_link_1_joint', 'arm_link_2_joint','arm_link_3_joint']

        my_trajectory_msg = JointTrajectory()
        my_trajectory_msg.joint_names = joints
        my_trajectory_msg.points.append(point_msg_1)
        my_trajectory_msg.points.append(point_msg_2)
        my_trajectory_msg.points.append(point_msg_3)
        my_trajectory_msg.points.append(point_msg_4)
        my_trajectory_msg.points.append(point_msg_5)
        my_trajectory_msg.points.append(point_msg_6)
        my_trajectory_msg.points.append(point_msg_7)
        my_trajectory_msg.points.append(point_msg_8)
        
        self.trajectory_publisher.publish(my_trajectory_msg)


def main(args=None):

    rclpy.init(args=args)
    joint_trajectory_object = TrajectoryPublisher()

    rclpy.spin(joint_trajectory_object)
    
    joint_trajectory_object.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
