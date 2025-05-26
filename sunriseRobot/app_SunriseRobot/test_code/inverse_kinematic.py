import time
import numpy as np
import matplotlib.pyplot as plt
from ikpy.chain import Chain
# import ikpy.urdf.utils as urdf_utils
from ikpy.inverse_kinematics import inverse_kinematic_optimization


def ikpy_to_degree_conversion(angles: np.array) -> np.array:
    converted_angles = np.rad2deg(angles) + 90
    return converted_angles


def degree_to_ikpy_conversion(angles: np.array) -> np.array:
    converted_angles = np.deg2rad(angles - 90)
    return converted_angles


def main():
    my_chain = Chain.from_urdf_file(
        urdf_file='urdf/arm.urdf',
        base_elements=['base_link'],
        name='arm',
        active_links_mask=[False, True, True, True, True, False],
    )
    # my_chain.to_json_file(force=True)
    # x y z coordinates of the end effector
    # target_position = np.array([0.1, 0.03, 0.1])
    # start_time = time.time()
    # joint_pos = my_chain.inverse_kinematics(target_position=target_position)
    # print(f'chain IK time: {round(time.time() - start_time, 3)} seconds')
    # print(f'joint_pos: {ikpy_to_degree_conversion(joint_pos)}')
    # for target_position [0.1, 0.15, 0.1]
    joint_initial_pos = np.array([0, 90, 45, 35, 35, 90])
    print(f'joint_initial_pos: {joint_initial_pos}')
    joint_initial_pos = degree_to_ikpy_conversion(joint_initial_pos)
    print(f'joint_initial_pos: {joint_initial_pos}')
    transformation_matrix = my_chain.forward_kinematics(joints=joint_initial_pos)
    gripper_position = transformation_matrix[:3, 3]
    print(f'gripper position :\n{gripper_position}')
    target_frame = np.zeros(shape=(3, 3))
    target_frame[:3, -1] = gripper_position
    start_time = time.time()
    joint_pos_2 = inverse_kinematic_optimization(
        chain=my_chain,
        target_frame=target_frame,
        starting_nodes_angles=joint_initial_pos,
    )
    print(f'standalone IK time: {round(time.time() - start_time, 3)} seconds')
    print(f'joint_pos: {ikpy_to_degree_conversion(joint_pos_2)}')
    # pos 0: ???
    # pos 1: servo 1 (rotate base)
    # pos 2: servo 2 (tilt whole arm)
    # transformation_matrix = my_chain.forward_kinematics(joints=joint_pos)
    # gripper_position = transformation_matrix[:3, 3]
    # print(f'gripper position :\n{gripper_position}')
    # transformation_matrix_2 = my_chain.forward_kinematics(joints=joint_pos_2)
    # print(f'gripper position_2 :\n{transformation_matrix_2[:3, 3]}')

    # plot the arm in 3D in two subplots
    fig = plt.figure(figsize=(12, 6))
    ax1 = fig.add_subplot(121, projection='3d')
    ax2 = fig.add_subplot(122, projection='3d')

    my_chain.plot(joint_initial_pos, ax1)
    ax1.set_title('my_chain')
    my_chain.plot(joint_pos_2, ax2)
    ax2.set_title('my_chain_2')

    plt.show()

    # Get the URDF tree
    # dot, urdf_tree = urdf_utils.get_urdf_tree(
    #     urdf_path='arm.urdf',
    #     out_image_path='C:/Users/Marco/GIT/yahboom_rdk_x3_robot/sunriseRobot/app_SunriseRobot/img',
    #     root_element='base_link',
    #     legend=True
    # )


if __name__ == '__main__':
    main()
