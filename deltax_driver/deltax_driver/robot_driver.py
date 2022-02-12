import threading
import rclpy
import time
from math import sin, cos, pi

# import pyximport
# pyximport.install(setup_args={"script_args" : ["--verbose"]})

from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped
from geometry_msgs.msg import Quaternion

from deltax_driver.deltax_kinematic import Kinematic
from deltax_driver.deltax_robot_interface import DeltaXRobotInterface

class DeltaXRobotStatesPublisher(object):
    def __init__(self, robot_interface, node):
        self.qos_profile = QoSProfile(depth=10)
        self.robot_interface = robot_interface
        self.__node = node
        
        self.joint_pub = self.__node.create_publisher(JointState, 'joint_states', QoSProfile(depth=10))
        self.broadcaster = TransformBroadcaster(self.__node, qos=QoSProfile(depth=10))

        self.odom_trans = TransformStamped()
        self.odom_trans.header.frame_id = 'odom'
        self.odom_trans.child_frame_id = 'base_link'
        self.joint_state = JointState()

        self.deltaxs_kinematic = Kinematic()
        self.deltaxs_kinematic.set_robot_parameter(rd_rf = 291.77, rd_re = 736.00, rd_e = 120.00, rd_f = 511.15, rd_of = 179.00)
        self.__thread = None
        self.__keep_running = False

        self.__loop_rate = self.__node.create_rate(100)

    def start(self):
        self.__keep_running = True
        self.__thread = threading.Thread(name="DeltaXRobotJointStatePublisher",
                                         target=self.__run)
        self.__thread.daemon = True
        self.__thread.start()

    def __run(self):
        while self.__keep_running:   
            #rclpy.spin_once(self.__node)        
            [x, y, z] = self.robot_interface.get_position()
            self.deltaxs_kinematic.inverse(x, y, z)
            theta1, theta2, theta3 = self.deltaxs_kinematic.get_theta()
            [ball_top1, ball_top2, ball_top3, re12, re34, re56, re_ball, ball_moving] = self.deltaxs_kinematic.get_component_state()
            
            now = self.__node.get_clock().now()
            self.joint_state.header.stamp = now.to_msg()
            
            self.joint_state.name = ['theta1', 'theta2', 'theta3',
                                    'ball_top1', 'ball_top2', 'ball_top3',
                                    're1', 're2', 're3',
                                    're4', 're5', 're6',
                                    're_ball', 'ball_moving']
            self.joint_state.position = [theta1, theta2, theta3,
                                        ball_top1, ball_top2, ball_top3,
                                        re12, re12, re34,
                                        re34, re56, re56,
                                        re_ball, ball_moving]

            self.odom_trans.header.stamp = now.to_msg()
            self.odom_trans.transform.translation.x = 0.
            self.odom_trans.transform.translation.y = 0.
            self.odom_trans.transform.translation.z = 1.
            self.odom_trans.transform.rotation = \
                euler_to_quaternion(0., 0., 0.) # roll,pitch,yaw

            self.joint_pub.publish(self.joint_state)
            self.broadcaster.sendTransform(self.odom_trans)
            self.__loop_rate.sleep()

def euler_to_quaternion(roll, pitch, yaw):
    qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
    qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
    qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
    qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)


def main():
    #deltax_driver_node = Deltax_Driver_Node()
    #rclpy.spin(deltax_driver_node)
    rclpy.init()
    node = rclpy.create_node('deltax_node')
     
    deltax = DeltaXRobotInterface(port = "COM1")
    if deltax.connect():
        print ("connected")
        deltax.send_gcode_to_robot('M100 A1 B10')
        deltax.send_gcode_to_robot('Position')
        deltax.send_gcode_to_robot('G0 Z-780')

    
    deltax_states_publisher = DeltaXRobotStatesPublisher(deltax, node)
    deltax_states_publisher.start()
    #rclpy.spin(node)

    gcocde = "G0 X100"
    rever = True

    while rclpy.ok():
        rclpy.spin_once(node)
        
        if deltax.is_moving() == False:
            continue

        if rever == True:
            rever = False
            gcocde = "G2 F2000 A30000 I100 J0"
        else:
            rever = True
            gcocde = "G2 I100 J0"

        deltax.send_gcode_to_robot(gcocde)


if __name__ == '__main__':
    main()
    

