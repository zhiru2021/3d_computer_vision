import os
import numpy as np
import cv2

import open3d as o3d

if __name__ == '__main__':

    print()
    print('main ...')

    ratio = 4032/640.

    intrinsic = np.array([
                    [2748.01/ratio, 0, 2016.61/ratio],
                    [0, 2748.01/ratio, 1513.30/ratio],
                    [0, 0, 1]
                ])
    extrinsic = np.eye(4)

    print()
    print('intrinsic:')
    print(intrinsic)

    print()
    print('extrinsic:')
    print(extrinsic) 

    # READ o3d RGBD
    color_file = './data/o3d_format/image00000.png'
    color = o3d.io.read_image(color_file)
    depth_file = './data/o3d_format/depth00000.png'
    depth = o3d.io.read_image(depth_file)

    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
                    color,
                    depth,
                    depth_trunc=3.0,
                    convert_rgb_to_intensity=True
                 )

    print()

    o3d_intrinsic = o3d.open3d.camera.PinholeCameraIntrinsic()
    o3d_intrinsic.set_intrinsics(640, 480, fx=intrinsic[0,0], fy=intrinsic[1,1], cx=intrinsic[0,2], cy=intrinsic[1,2])
    print('o3d_intrinsic')
    print('intrinsic matrix')
    print(o3d_intrinsic.intrinsic_matrix)

    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
            rgbd_image,
            o3d_intrinsic
          )
    # OUTPUT o3d colored point cloud
    o3d.io.write_point_cloud("reconstruction.ply", pcd, False, True)
    