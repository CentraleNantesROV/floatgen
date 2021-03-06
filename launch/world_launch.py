from simple_launch import SimpleLauncher
import sys

def generate_launch_description():
    
    sl = SimpleLauncher()
    
    sl.declare_arg('gui', default_value=True)
    
    with sl.group(if_arg='gui'):
        sl.include('ros_ign_gazebo','ign_gazebo.launch.py',launch_arguments={'ign_args': sl.name_join('-r ', sl.find('floatgen', 'floatgen_world.sdf'))})
        
    with sl.group(unless_arg='gui'):
        sl.include('ros_ign_gazebo','ign_gazebo.launch.py',launch_arguments={'ign_args': sl.name_join('-s -r ', sl.find('floatgen', 'floatgen_world.sdf'))})
        
    sl.create_gz_clock_bridge()
        
    return sl.launch_description()
