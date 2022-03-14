import open3d as o3d
import numpy as np
from scipy.spatial.transform import Rotation as scipyR

# Read ply files
source = o3d.read_point_cloud("/home/nuc/code/open3d_icp_test/03-103-3f_source.pcd")
target = o3d.read_point_cloud("/home/nuc/code/open3d_icp_test/03-103-3f_partial.pcd")

# Show in different colors
source.paint_uniform_color([1, 0.706, 0])    # yellow
target.paint_uniform_color([0, 0.651, 0.929])# blue

# outlier removal
processed_source, outlier_index = o3d.geometry.radius_outlier_removal(source,
                                              nb_points=16,
                                              radius=0.5)

processed_target, outlier_index = o3d.geometry.radius_outlier_removal(target,
                                              nb_points=16,
                                              radius=0.5)
threshold = 1.0  # for moving

rotationMat = scipyR.from_euler('z', 0.0, degrees=True).as_dcm()
translationVec = np.array([0,0,0])

trans_init = np.eye([4,4])


trans_init[:3, :3] = rotationMat
trans_init[3, :3]= translationVec

# icp
# reg_p2p = o3d.registration.registration_icp(
#         processed_source, processed_target, threshold, trans_init,
#         o3d.registration.TransformationEstimationPointToPoint())

reg_p2p = o3d.registration.registration_icp(
        processed_source, processed_target, threshold, trans_init,
        o3d.registration.TransformationEstimationPointToPlane())

#将我们的矩阵依照输出的变换矩阵进行变换
print(reg_p2p)
processed_source.transform(reg_p2p.transformation)

#创建一个 o3d.visualizer class
vis = o3d.visualization.Visualizer()
vis.create_window()

#将两个点云放入visualizer
vis.add_geometry(processed_source)
vis.add_geometry(processed_target)

#让visualizer渲染点云
vis.update_geometry()
vis.poll_events()
vis.update_renderer()

vis.run()