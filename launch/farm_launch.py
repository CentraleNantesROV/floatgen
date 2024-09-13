from simple_launch import SimpleLauncher, GazeboBridge


sl = SimpleLauncher(use_sim_time = True)

sl.declare_arg('nx', default_value = 1, description = 'how many turbines in X-axis')
sl.declare_arg('ny', default_value = 1, description = 'how many turbines in Y-axis')

sl.declare_arg('x', default_value = 0., description = 'X-position of first turbine')
sl.declare_arg('y', default_value = 0., description = 'Y-position of first turbine')
sl.declare_arg('yaw', default_value = 0., description = 'Yaw of turbines')
sl.declare_arg('scale', default_value = 200., description = 'Distances of turbines')
sl.declare_arg('velocity', default_value = -2., description = 'Velocity of turbines')

sl.declare_arg('gz_gui', True)

sl.declare_arg('gz', True)
sl.declare_arg('spawn', True)


def launch_setup():

    if sl.arg('gz'):
        gz_args = '-r'
        if not sl.arg('gz_gui'):
            gz_args += ' -s'
        sl.gz_launch(sl.find('floatgen', 'floatgen_world.sdf'), gz_args)

    if sl.arg('spawn'):
        ns = 'farm'
        with sl.group(ns=ns):
            # run RSP with given parameters
            sl.robot_state_publisher('floatgen', 'farm.xacro',
                                            xacro_args=sl.arg_map('x', 'y', 'yaw', 'nx', 'ny', 'scale', 'velocity'))

            # spawn in Gazebo
            sl.spawn_gz_model(ns)

            # joint_state bridge
            gz_js_topic = GazeboBridge.model_prefix(ns)+'/joint_state'
            js_bridge = GazeboBridge(gz_js_topic, 'joint_states', 'sensor_msgs/JointState', GazeboBridge.gz2ros)
            sl.create_gz_bridge([GazeboBridge.clock(), js_bridge], 'turbine_bridge')

    return sl.launch_description()


generate_launch_description = sl.launch_description(launch_setup)
