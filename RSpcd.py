import pyrealsense2 as rs
import numpy as np
import cv2
import open3d as o3d


align = rs.align(rs.stream.color)

config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 10)
config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 10)
pipeline = rs.pipeline()
profile = pipeline.start(config)

# get camera intrinsics
intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
print(intr.width, intr.height, intr.fx, intr.fy, intr.ppx, intr.ppy)
pinhole_camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(intr.width, intr.height, intr.fx, intr.fy, intr.ppx, intr.ppy)

while True:
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)

    color_frame = aligned_frames.get_color_frame()
    color_image = np.asanyarray(color_frame.get_data())

    cv2.namedWindow('color image', cv2.WINDOW_AUTOSIZE)
    cv2.imshow('color image', cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR))

    if cv2.waitKey(1) != -1:
        print('finish')
        break

depth_frame = aligned_frames.get_depth_frame()
depth = o3d.geometry.Image(np.asanyarray(depth_frame.get_data()))
color = o3d.geometry.Image(color_image)

rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(color, depth);
pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, pinhole_camera_intrinsic)                                                   
pcd.transform([[1,0,0,0],[0,-1,0,0],[0,0,-1,0],[0,0,0,1]])

pipeline.stop()
o3d.io.write_point_cloud('./rs_color.pcd', pcd)
o3d.geometry.draw_geometries([pcd])