from simple_launch import SimpleLauncher


def generate_launch_description():
    
    sl = SimpleLauncher(use_sim_time = True)
    
    seabed = sl.declare_arg('seabed', default_value='seabed')
    sl.declare_arg('depth', -20)
                
    with sl.group(ns=seabed):
        
        description = sl.robot_description('floatgen', 'seabed.xacro', xacro_args=sl.arg_map('seabed','depth'))
        
        # run RSP on URDF
        sl.node('robot_state_publisher', 'robot_state_publisher', parameters={'robot_description': description})
    
    return sl.launch_description()
