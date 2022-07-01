from simple_launch import SimpleLauncher, GazeboBridge
import sys

def generate_launch_description():
    
    sl = SimpleLauncher(use_sim_time = True)
    
    sl.declare_arg('nx', default_value = 1, description = 'how many turbines in X-axis')
    sl.declare_arg('ny', default_value = 1, description = 'how many turbines in Y-axis')    
    
    sl.declare_arg('x', default_value = 0., description = 'X-position of first turbine')
    sl.declare_arg('y', default_value = 0., description = 'Y-position of first turbine')
    sl.declare_arg('yaw', default_value = .7, description = 'Yaw of turbines')
    sl.declare_arg('scale', default_value = 200., description = 'Distances of turbines')
    sl.declare_arg('velocity', default_value = -2., description = 'Velocity of turbines')
    
    ns = 'farm'
                
    with sl.group(ns=ns):
        
        description = sl.robot_description('floatgen', 'farm.xacro', 
                                           xacro_args=sl.arg_map(('x', 'y', 'yaw', 'nx', 'ny', 'scale', 'velocity')))
        
        # run RSP on URDF
        sl.node('robot_state_publisher', 'robot_state_publisher', parameters={'robot_description': description})    
        
        # spawn in gazebo
        sl.spawn_gz_model(ns, only_new = False)
        
        # joint_state bridge
        gz_js_topic = sl.name_join(GazeboBridge.model_prefix(ns),'/joint_state')
        js_bridge = GazeboBridge(gz_js_topic, 'joint_states', 'sensor_msgs/JointState', GazeboBridge.gz2ros)        
        sl.create_gz_bridge(js_bridge, 'turbine_bridge')
    
    return sl.launch_description()
