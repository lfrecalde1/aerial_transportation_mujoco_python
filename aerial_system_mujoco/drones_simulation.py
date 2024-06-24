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

class MuJoCoSimulationNode(Node):
    def __init__(self, param):
        super().__init__('mujoco_simulation_node')

        # Load the MuJoCo model and data
        self.model = mujoco.MjModel.from_xml_path(param)
        # Create the infomation for the simulation
        self.data = mujoco.MjData(self.model)

        # Name body drone
        self.name = "root_drone"
        
        # Set initial states 
        init_position = np.array([1.0, 1.0, 2.0, 1.0, 0.0, 0.0, 0.0])
        self.set_states(init_position)


        # Defintion of the sample time
        self.ts = self.model.opt.timestep
        self.t_final = 60
        self.t = np.arange(0, self.t_final + self.ts, self.ts, dtype =np.double)

        # Create states Publishe
         # Publisher properties
        self.publisher_ = self.create_publisher(Odometry, "odom", 10)
        self.odometry_msg = Odometry()
        self.odometry_hz = 150
        self.timer_odometry = self.create_timer(1.0 / self.odometry_hz, self.callback_odometry)

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
                
                # Control Section update section

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

    def set_body_position_by_name(self, body_name, new_position):
        body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        if body_id == -1:
            raise ValueError(f"Body with name {body_name} not found")
        self.data.qpos[self.model.jnt_qposadr[self.model.body_jntadr[body_id]]] = new_position
        return None

    def callback_odometry(self):
        # Acces states of the drone
        joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, self.name)
        joint_qposadr = self.model.jnt_qposadr[joint_id]
        x = self.data.qpos[joint_qposadr:joint_qposadr+7]
        p_noise = self.data.sensor("drone_position").data.copy()
        q_noise = self.data.sensor("drone_quat").data.copy()
        print(x.shape)
        print(x[0], p_noise[0])
        print(x[1], p_noise[1])
        print(x[2], p_noise[2])
        print(x[3], q_noise[0])
        print(x[4], q_noise[1])
        print(x[5], q_noise[2])
        print(x[6], q_noise[3])

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

        # Send Message
        self.publisher_.publish(self.odometry_msg)
        return None 
        
        
    def set_states(self, init_position):
        joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, self.name)
        joint_qposadr = self.model.jnt_qposadr[joint_id]
        self.data.qpos[joint_qposadr:joint_qposadr+7] = init_position

        # Set initial control action
        self.data.ctrl[0] = 0.0
        self.data.ctrl[1] = 0.0
        self.data.ctrl[2] = 0.0
        self.data.ctrl[3] = 0.0
        return None

def main(args=None):
    rclpy.init(args=args)
    # We check for at least one argument in this node
    if len(sys.argv) < 2:
        print("Usage: ros2 run my_ros2_package my_ros2_node.py <param1>")
        return

    param1 = sys.argv[1]
    node = MuJoCoSimulationNode(param1)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except(KeyboardInterrupt):
        print("Error system")
        pass
    else:
        print("Complete Execution")
        pass
    