#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import sys
import threading
import time
import mujoco
import mujoco.viewer
from nav_msgs.msg import Odometry
from mujoco_msgs.msg import Control

class MuJoCoSimulationNode(Node):
    def __init__(self, param):
        super().__init__('mujoco_simulation_node')

        # Load the MuJoCo model and data
        self.model = mujoco.MjModel.from_xml_path(param)
        # Create the infomation for the simulation
        self.data = mujoco.MjData(self.model)

        # Name body drone
        self.name_drone = "drone"
        self.name_payload = "payload_ball"
        
        # Set initial states 
        init_pose = np.array([1.0, 1.0, 3.0, 0.93, 0.0, 0.0, -0.34])
        init_pose_payload = np.array([1.0, 1.0, 1.0, 1, 0.0, 0.0, 0])
        self.set_states(init_pose)
        self.set_payload(init_pose_payload)

        # get Parameters of the system
        mass_drone_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, self.name_drone)
        mass_payload_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, self.name_payload)

        self.g = -self.model.opt.gravity[2]
        self.mass_drone = self.model.body_mass[mass_drone_id]
        self.mass_payload = self.model.body_mass[mass_payload_id]
        

        # Hover Control Actions
        #self.f = self.g * self.mass_drone + self.g*self.mass_payload
        self.f = self.g * self.mass_drone 
        #self.f = 0.0
        self.mx = 0.0
        self.my = 0.0
        self.mz = 0.0


        # Defintion of the sample time
        self.ts = self.model.opt.timestep
        self.t_final = 200
        self.t = np.arange(0, self.t_final + self.ts, self.ts, dtype =np.double)

        # Create states Publishe
         # Publisher properties
        self.publisher_ = self.create_publisher(Odometry, "odom", 10)
        self.odometry_msg = Odometry()
        self.odometry_hz = 150
        self.timer_odometry = self.create_timer(1.0 / self.odometry_hz, self.callback_odometry)

        self.publisherpayload_ = self.create_publisher(Odometry, "load", 10)
        self.odometry_msg_payload = Odometry()
        self.odometry_hz_load = 150
        self.timer_odometry_load = self.create_timer(1.0 / self.odometry_hz_load, self.callback_odometry_load)

        # Subscriber control action
        self.subscriber_ = self.create_subscription(Control, "cmd", self.callback_control_value, 10)

        # Create a thread to run the simulation and viewer
        self.simulation_thread = threading.Thread(target=self.run_simulation)
        # Start thread for the simulation
        self.simulation_thread.start()

        # Create a timer to periodically check the simulation status
        self.timer = self.create_timer(0.05, self.check_simulation_status)

    def run_simulation(self):
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            # Close the viewer automatically after 30 wall-seconds.
            for k in range(0, self.t.shape[0]):
                if not viewer.is_running():
                    print("Viewer has stopped running")
                    break
                tic = time.time()
                # Apply veloicites inertial frame
                #self.data.qvel[0] = -0.1
                #self.data.qvel[1] = 0.1
                # Control Section update section
                # Set initial control action
                self.data.ctrl[0] = self.f
                self.data.ctrl[1] = self.mx
                self.data.ctrl[2] = self.my
                self.data.ctrl[3] = self.mz


                # mj_step can be replaced with code that also evaluates
                mujoco.mj_step(self.model, self.data)

                # Example modification of a viewer option: toggle contact points every two seconds.
                with viewer.lock():
                    viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(self.data.time % 2)
                # Pick up changes to the physics state, apply perturbations, update options from GUI.
                viewer.sync()

                # Section to guarantee same sample times
                while (time.time() - tic <= self.model.opt.timestep):
                    None
                toc = time.time() - tic
                #print(toc)

    def check_simulation_status(self):
        # Check if the simulation thread is still alive
        if not self.simulation_thread.is_alive():
            self.get_logger().info('Simulation has ended, shutting down node.')
            rclpy.shutdown()
        return None

    def callback_odometry(self):
        # Acces states of the drone
        joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, self.name_drone)
        joint_qposadr = self.model.jnt_qposadr[joint_id]
        x = self.data.qpos[joint_qposadr:joint_qposadr+7]

        # Get information from the sensors
        p_noise = self.data.sensor("drone_position").data.copy()
        q_noise = self.data.sensor("drone_quat").data.copy()
        p_dot_noise = self.data.sensor("drone_linear_velocity").data.copy()
        w_noise = self.data.sensor("drone_angular_velocity").data.copy()

        # setting the values to the message
        self.odometry_msg.header.frame_id = "map"
        self.odometry_msg.header.stamp = self.get_clock().now().to_msg()

        self.odometry_msg.pose.pose.position.x = p_noise[0]
        self.odometry_msg.pose.pose.position.y = p_noise[1]
        self.odometry_msg.pose.pose.position.z = p_noise[2]

        self.odometry_msg.pose.pose.orientation.x = q_noise[1]
        self.odometry_msg.pose.pose.orientation.y = q_noise[2]
        self.odometry_msg.pose.pose.orientation.z = q_noise[3]
        self.odometry_msg.pose.pose.orientation.w = q_noise[0]

        self.odometry_msg.twist.twist.linear.x = p_dot_noise[0]
        self.odometry_msg.twist.twist.linear.y = p_dot_noise[1]
        self.odometry_msg.twist.twist.linear.z = p_dot_noise[2]

        self.odometry_msg.twist.twist.angular.x = w_noise[0]
        self.odometry_msg.twist.twist.angular.y = w_noise[1]
        self.odometry_msg.twist.twist.angular.z = w_noise[2]

        # Send Message
        self.publisher_.publish(self.odometry_msg)
        return None 
        
    def callback_odometry_load(self):
        # Acces states of the drone
        joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, self.name_payload)
        joint_qposadr = self.model.jnt_qposadr[joint_id]
        x = self.data.qpos[joint_qposadr:joint_qposadr+7]


        # setting the values to the message
        self.odometry_msg_payload.header.frame_id = "map"
        self.odometry_msg_payload.header.stamp = self.get_clock().now().to_msg()

        self.odometry_msg_payload.pose.pose.position.x = x[0]
        self.odometry_msg_payload.pose.pose.position.y = x[1]
        self.odometry_msg_payload.pose.pose.position.z = x[2]

        self.odometry_msg_payload.pose.pose.orientation.x = x[4]
        self.odometry_msg_payload.pose.pose.orientation.y = x[5]
        self.odometry_msg_payload.pose.pose.orientation.z = x[6]
        self.odometry_msg_payload.pose.pose.orientation.w = x[3]

        # Send Message
        self.publisherpayload_.publish(self.odometry_msg_payload)
        return None 
        
    def set_states(self, init_position):
        joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, self.name_drone)
        joint_qposadr = self.model.jnt_qposadr[joint_id]
        self.data.qpos[joint_qposadr:joint_qposadr+7] = init_position

        # Set initial control action
        self.data.ctrl[0] = 0.0
        self.data.ctrl[1] = 0.0
        self.data.ctrl[2] = 0.0
        self.data.ctrl[3] = 0.0
        return None
        
    def set_payload(self, init_position):
        joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, self.name_payload)
        joint_qposadr = self.model.jnt_qposadr[joint_id]
        self.data.qpos[joint_qposadr:joint_qposadr+7] = init_position
        return None

    def callback_control_value(self, msg):
        fz = msg.thrust
        mx = msg.torque_x
        my = msg.torque_y
        mz = msg.torque_z
        self.f = fz
        self.mx = mx
        self.my = my
        self.mz = mz
        return None

def main(args=None):
    rclpy.init(args=args)

    # We check for at least one argument in this node
    if len(sys.argv) < 2:
        print("Usage: ros2 run my_ros2_package my_ros2_node.py <param1>")
        return
    param1 = sys.argv[1]
    node = MuJoCoSimulationNode(param1)
    try:
        rclpy.spin(node)  # Will run until manually interrupted
    except KeyboardInterrupt:
        node.get_logger().info('Simulation stopped manually.')
        rclpy.shutdown()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return None

if __name__ == '__main__':
    main()