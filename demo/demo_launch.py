from simple_launch import SimpleLauncher, GazeboBridge
import sys

def generate_launch_description():
    
    sl = SimpleLauncher(use_sim_time = False)
    
    sl.declare_arg('nx', default_value = 1, description = 'how many turbines in X-axis')
    sl.declare_arg('ny', default_value = 1, description = 'how many turbines in Y-axis')
    
    sl.declare_arg('x', default_value = 0., description = 'X-position of first turbine')
    sl.declare_arg('y', default_value = 0., description = 'Y-position of first turbine')
    sl.declare_arg('yaw', default_value = .0, description = 'Yaw of turbines')
    sl.declare_arg('scale', default_value = 200., description = 'Distances of turbines')
    sl.declare_arg('velocity', default_value = -2., description = 'Velocity of turbines')
    
    ns = 'farm'
                
    with sl.group(ns='farm'):
        
        description = sl.robot_description('floatgen', 'farm.xacro',
                                           xacro_args=sl.arg_map(('x', 'y', 'yaw', 'nx', 'ny', 'scale', 'velocity')))
        
        # run RSP on URDF
        sl.node('robot_state_publisher', 'robot_state_publisher', parameters={'robot_description': description})
        
    with sl.group(ns='bluerov2'):
        sl.robot_state_publisher('bluerov2_description', 'bluerov2.xacro')
        sl.joint_state_publisher(True)

    sl.include('coral', 'track_launch.py',
               launch_arguments = {'link': 'bluerov2/base_link',
                                   'yaw': 3.14,
                                   'x': 3.})

    sl.node('coral', 'spawn')

    return sl.launch_description()
