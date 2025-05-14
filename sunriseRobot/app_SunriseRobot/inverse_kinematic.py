import numpy as np
import matplotlib.pyplot
from ikpy.chain import Chain
import ikpy.urdf.utils as urdf_utils


my_chain = Chain.from_urdf_file(urdf_file='urdf/arm.urdf', base_elements=['base_link'], name='arm')
my_chain.to_json_file(force=True)
# x y z coordinates of the end effector
target_position = np.array([0.1, 0.1, 0.1])
joint_pos = my_chain.inverse_kinematics(target_position=target_position)
print(f'joint_pos: {joint_pos}')
transformation_matrix = my_chain.forward_kinematics(joints=joint_pos)
print(f'transformation_matrix:\n{transformation_matrix}')
# plot the arm in 3D
ax = matplotlib.pyplot.figure().add_subplot(111, projection='3d')
my_chain.plot(joint_pos, ax)
matplotlib.pyplot.show()

# Get the URDF tree
# dot, urdf_tree = urdf_utils.get_urdf_tree(
#     urdf_path='arm.urdf',
#     out_image_path='C:/Users/Marco/GIT/yahboom_rdk_x3_robot/sunriseRobot/app_SunriseRobot/img',
#     root_element='base_link',
#     legend=True,
# )
