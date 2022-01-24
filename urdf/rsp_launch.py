from simple_launch import SimpleLauncher

def generate_launch_description():
    
    sl = SimpleLauncher()
    
    sl.declare_arg('nx', default_value = 1, description = 'how many turbines in X-axis')
    sl.declare_arg('ny', default_value = 1, description = 'how many turbines in Y-axis')    
    
    with sl.group(ns = 'farm'):
        sl.robot_state_publisher('floatgen', 'farm.xacro', 
                                xacro_args={'floating': 'true','height': '1.','base_color': '".8 .8 0"',
                                            'nx': sl.arg('nx'), 'ny': sl.arg('ny'), 'yaw': '.5'})
        sl.joint_state_publisher('True')
    
    sl.node('rviz2','rviz2',arguments=['-d',sl.find('floatgen', 'farm.rviz')])

    return sl.launch_description()
