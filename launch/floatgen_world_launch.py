from simple_launch import SimpleLauncher, IgnitionBridge
import sys

def generate_launch_description():
    
    sl = SimpleLauncher()
    
    sl.declare_arg('gui', default_value=True)    
    
    sl.include('ros_ign_gazebo','ign_gazebo.launch.py',launch_arguments={'ign_args': sl.name_join('-r ', sl.find('floatgen', 'floatgen_world.sdf'))})
    sl.create_ign_bridge(IgnitionBridge.clock(), 'ign_clock')    
    
    return sl.launch_description()
