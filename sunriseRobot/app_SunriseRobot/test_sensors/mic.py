def change_range(val, original_min_val, original_max_val, new_min_val, new_max_val):
    assert original_min_val < original_max_val, \
        f'Invalid input: original_min_val {original_min_val} must be smaller than original_max_val {original_max_val}'
    assert new_min_val < new_max_val, \
        f'Invalid input: new_min_val {new_min_val} must be smaller than new_max_val {new_max_val}'
    assert original_min_val <= val, \
        f'Invalid input: value {val} must be bigger than original_min_val {original_min_val}'
    assert val <= original_max_val, \
        f'Invalid input: value {val} must be smaller than original_max_val {original_max_val}'
    return (val - original_min_val) * (new_max_val - new_min_val) / (original_max_val - original_min_val) + new_min_val


def sound_angle_to_robot_speed(sound_angle: float,
                               turn_only_angle: float,
                               forward_speed_range: list,
                               angular_speed_range: list,
                               ) -> (float, float):
    # sound_angle: [-turn_only_angle, turn_only_angle]
    # speed_x: [0, 0.6] forward-stationary (no backward movement)
    # speed_z: [-3, 3] left-right
    speed_x = change_range(
        val=-abs(sound_angle),
        original_min_val=-turn_only_angle,
        original_max_val=0,
        new_min_val=forward_speed_range[0],
        new_max_val=forward_speed_range[1],
    )
    speed_x = abs(speed_x)

    speed_z = change_range(
        val=abs(sound_angle),
        original_min_val=0,
        original_max_val=turn_only_angle,
        new_min_val=angular_speed_range[0],
        new_max_val=angular_speed_range[1],
    )
    if sound_angle > 0:
        speed_z *= -1
    return speed_x, speed_z


def microphone_angle_to_robot_angle(direction_of_arrival: float, microphone_robot_angle: float) -> float:
    assert 0 <= direction_of_arrival < 360, f'Invalid DOA angle: {direction_of_arrival}°'
    assert 0 <= microphone_robot_angle < 360, f'Invalid microphone forward angle: {microphone_robot_angle}°'

    # The DOA angle is the angle of the sound source relative to the microphone array.
    # The microphone array is mounted on the robot with a rotation of microphone_robot_angle°.

    converted_doa = direction_of_arrival + microphone_robot_angle
    while converted_doa >= 180:
        converted_doa -= 360
    assert -180 <= converted_doa < 180, f'Error in DOA conversion: {converted_doa}°'

    return converted_doa


if __name__ == '__main__':
    forward_speed_range = [0, 0.6]
    angular_speed_range = [0, 3.0]
    turn_only_angle = 50.0
    speed_coefficient = 0.3
    microphone_robot_angle = 64.0

    # target_angle_microphone = 301
    # print(f'target_angle_microphone: {target_angle_microphone}°')
    #
    # target_angle_robot = microphone_angle_to_robot_angle(
    #     direction_of_arrival=target_angle_microphone,
    #     microphone_robot_angle=microphone_robot_angle,
    # )

    target_angle_robot = -40
    print(f'target_angle_robot: {target_angle_robot}°')

    if abs(target_angle_robot) > turn_only_angle:
        print('Turn only')
        speed_x = 0
        speed_z = angular_speed_range[1] * speed_coefficient

    else:
        print('Turn and Advance')
        speed_x, speed_z = sound_angle_to_robot_speed(
            sound_angle=target_angle_robot,
            turn_only_angle=turn_only_angle,
            forward_speed_range=forward_speed_range,
            angular_speed_range=angular_speed_range,
        )
        speed_x *= speed_coefficient
        speed_z *= speed_coefficient

    print(f'\tspeed_x: {speed_x}')
    print(f'\tspeed_z: {speed_z}')
