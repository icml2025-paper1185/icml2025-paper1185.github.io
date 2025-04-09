import open3d as o3d
import numpy as np
import cv2
import os


point_cloud = o3d.io.read_point_cloud("output_video6.ply")  

num_points = len(point_cloud.points)  
gray_color = np.array([0.5, 0.5, 0.5])  
point_cloud.colors = o3d.utility.Vector3dVector(np.tile(gray_color, (num_points, 1)))  # 设置所有点为灰色

# if hasattr(point_cloud, 'colors'):
#     point_cloud.colors = None

vis = o3d.visualization.Visualizer()
vis.create_window(visible=False,width=800, height=800)
vis.add_geometry(point_cloud)


angle_step = -10*np.pi / 180 
rotation_axis = np.array([0, 1, 0])  
frames = 72 


output_dir = "frames"
os.makedirs(output_dir, exist_ok=True)


for i in range(frames):
    
    R = o3d.geometry.get_rotation_matrix_from_axis_angle(rotation_axis * angle_step)
    point_cloud.rotate(R, center=(0, 0, 0))  

    vis.update_geometry(point_cloud)
    vis.poll_events()
    vis.update_renderer()

    
    image_path = os.path.join(output_dir, f"frame_{i:03d}.png")
    vis.capture_screen_image(image_path)


vis.destroy_window()

# 合成视频
video_path = "output_video.mp4"
frame_rate = 30  # 设置帧率
frame_size = None


images = [os.path.join(output_dir, f) for f in sorted(os.listdir(output_dir)) if f.endswith(".png")]


if images:
    first_frame = cv2.imread(images[0])
    frame_size = (first_frame.shape[1], first_frame.shape[0])


fourcc = cv2.VideoWriter_fourcc(*"MJPG")  
video_writer = cv2.VideoWriter(video_path, fourcc, frame_rate, frame_size)


for image_path in images:
    frame = cv2.imread(image_path)
    video_writer.write(frame)


video_writer.release()

for image_path in images:
    os.remove(image_path)
os.rmdir(output_dir)

print(f"Video saved to {video_path}")
