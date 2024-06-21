import sys
import rclpy
import mujoco
from mujoco import MjViewer
def main(args=None):
    rclpy.init(args=args)

    # We check for at least one argument in this node
    if len(sys.argv) < 2:
        print("Usage: ros2 run my_ros2_package my_ros2_node.py <param1>")
        return

    param1 = sys.argv[1]
    model = mujoco.MjModel.from_xml_path(param1)

    # Print names of the geometrical objects in the xml file
    geometric_names = [] 
    body_names = [] 
    body_id = []
    for i in range(model.ngeom):
        geometric_names.append(model.geom(i).name)

    for i in range(model.nbody):
        body_names.append(model.body(i).name)
    
    for i in range(model.nbody):
        body_id.append(model.body(i).id)
        print(model.body(i).name)
        print(model.body_inertia[i])
        print(model.body_mass[i])


    ## 
    
    return None

if __name__ == '__main__':
    try: 
        main()
    except(KeyboardInterrupt):
        print("Error system")
        pass
    else:
        print("Complete Execution")
        pass
    