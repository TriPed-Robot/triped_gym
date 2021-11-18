import pybullet as p
import numpy as np


def render_image(base_pos, cam_dist, cam_yaw, cam_pitch, render_width, render_height):
    view_matrix = p.computeViewMatrixFromYawPitchRoll(
        cameraTargetPosition=base_pos,
        distance=cam_dist,
        yaw=cam_yaw,
        pitch=cam_pitch,
        roll=0,
        upAxisIndex=2)

    proj_matrix = p.computeProjectionMatrixFOV(
        fov=60, aspect=float(render_width)/render_height,
        nearVal=0.1, farVal=100.0)

    (_, _, px, _, _) = p.getCameraImage(
        width=render_width, height=render_height, viewMatrix=view_matrix,
        projectionMatrix=proj_matrix,
        renderer=p.ER_TINY_RENDERER)

    rgb_array = np.array(px)
    rgb_array = rgb_array[:, :, :3]

    return rgb_array
