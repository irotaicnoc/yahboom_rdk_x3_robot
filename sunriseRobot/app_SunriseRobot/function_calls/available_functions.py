function_list = [
    {
        'name': 'set_beep',
        'containing_object': 'robot_body',
    },
    {
        'name': 'set_car_motion',
        'containing_object': 'robot_body',
    },
    {
        'name': 'toggle_rigid',
        'containing_object': 'arm',
    },
    {
        'name': 'set_desired_angles',
        'containing_object': 'arm',
    },
    {
        'name': 'next_light_effect',
        'containing_object': 'light',
    },
    {
        'name': 'stop',
        'containing_object': 'light',
    },
]

object_list = [
    {
        'name': 'robot_head',
        'excluded_functions': ['add_mode_callback', 'add_sub_mode_callback', 'all_sub_modes'],
    },
]
