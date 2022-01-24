from simple_launch import SimpleLauncher, IgnitionBridge
import sys

def generate_launch_description():
    
    sl = SimpleLauncher()
    
    sl.declare_arg('nx', default_value = 1, description = 'how many turbines in X-axis')
    sl.declare_arg('ny', default_value = 1, description = 'how many turbines in Y-axis')    
    
    sl.declare_arg('x', default_value = 0., description = 'X-position of first turbine')
    sl.declare_arg('y', default_value = 0., description = 'Y-position of first turbine')
    sl.declare_arg('yaw', default_value = .7, description = 'Yaw of turbines')
    sl.declare_arg('scale', default_value = 200., description = 'Distances of turbines')
    sl.declare_arg('velocity', default_value = -15., description = 'Velocity of turbines')
    
    ns = 'farm'
                
    with sl.group(ns=ns):
        
        description = sl.robot_description('floatgen', 'farm.xacro', 
                                           xacro_args=sl.arg_map(('x', 'y', 'yaw', 'nx', 'ny', 'scale', 'velocity')))
        
        # run RSP on URDF
        sl.node('robot_state_publisher', 'robot_state_publisher', parameters={'robot_description': description})    
        
        # spawn in gazebo
        sl.spawn_ign_model(ns)
        
        #
        ign_js_topic = sl.name_join(IgnitionBridge.model_prefix(ns),'/joint_state')
        js_bridge = IgnitionBridge(ign_js_topic, 'joint_states', 'sensor_msgs/JointState', IgnitionBridge.ign2ros)        
        sl.create_ign_bridge(js_bridge, 'turbine_bridge')
        #description_topic = sl.py_eval("'/' + '", ns, "' + '/robot_description'")
        #spawn_args = ['-topic','robot_description']
        #spawn_args += ['-gazebo_namespace','/gazebo','-entity', ns,'-robot_namespace', ns]
        #sl.node('gazebo_ros', 'spawn_entity.py', parameters={'use_sim_time': True}, arguments=spawn_args)
    
    return sl.launch_description()
