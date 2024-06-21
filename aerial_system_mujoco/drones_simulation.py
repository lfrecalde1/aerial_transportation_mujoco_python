import rclpy
from rclpy.node import Node
import sys
import threading
import time
import mujoco
import mujoco.viewer

class MuJoCoSimulationNode(Node):
    def __init__(self, param):
        super().__init__('mujoco_simulation_node')

        # Load the MuJoCo model and data
        self.model = mujoco.MjModel.from_xml_path(param)
        self.data = mujoco.MjData(self.model)
        self.ts = self.model.opt.timestep
        self.t_final = 60

        # Create a thread to run the simulation and viewer
        self.simulation_thread = threading.Thread(target=self.run_simulation)
        self.simulation_thread.start()

        # Create a timer to periodically check the simulation status
        self.timer = self.create_timer(0.05, self.check_simulation_status)

    def run_simulation(self):
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            # Close the viewer automatically after 30 wall-seconds.
            start = time.time()
            while viewer.is_running() and time.time() - start < self.t_final:
                tic = time.time()

                # mj_step can be replaced with code that also evaluates
                # a policy and applies a control signal before stepping the physics.
                mujoco.mj_step(self.model, self.data)

                # Example modification of a viewer option: toggle contact points every two seconds.
                with viewer.lock():
                    viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(self.data.time % 2)

                # Pick up changes to the physics state, apply perturbations, update options from GUI.
                viewer.sync()

                 # Section to guarantee same sample times
                while (time.time() - tic <= self.model.opt.timestep):
                    None

    def check_simulation_status(self):
        # Check if the simulation thread is still alive
        if not self.simulation_thread.is_alive():
            self.get_logger().info('Simulation has ended, shutting down node.')
            rclpy.shutdown()

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
    main()