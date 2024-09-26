import os.path
import cv2
import numpy as np
import open3d as o3d
import argparse
from utils.visualization import display_single_points, display_two_points, display_three_points, display_points_path
from utils.funcs import ransac_iterations, construct_plane_from_normal_vector, construct_from_plane, remove_points, \
    choose_points, inverse_transform, apply_transform


class PCDProcessor:
    def __init__(self, root, pose_file="optimized_poses.txt", global_pcd="cloudGlobal.pcd", points_folder="Scans",
                 pcd_saving_name="map_removed_ground_top.pcd", png_saving_name="map_removed_ground_top.png",
                 display_steps=False, display_remove=False,
                 floor_height=0.4, height_threshold=4.0, frame_separate=10, radius=30, resolution=0.1,
                 neighbor_threshold_init=[5, 1], neighbor_threshold=[10, 0.5], distance_threshold=0.4, ransac_num=3):
        self.display_steps = display_steps
        self.display_remove = display_remove
        self.resolution = resolution
        self.frame_separate = frame_separate
        self.neighbor_threshold_init = neighbor_threshold_init
        self.neighbor_threshold = neighbor_threshold
        self.distance_threshold = distance_threshold
        self.ransac_num = ransac_num
        self.floor_height = floor_height
        self.height_threshold = height_threshold
        self.radius = radius
        self.pcd_saving_dir = os.path.join(root, pcd_saving_name)
        self.png_saving_dir = os.path.join(root, png_saving_name)

        self.root = root
        self.pcd_files_root = os.path.join(root, points_folder)
        self.pcd_files = os.listdir(self.pcd_files_root)
        self.pcd_files.sort()
        self.poses = self.read_poses_txt(os.path.join(root, pose_file))
        assert len(self.poses) == len(self.pcd_files), "the pcd files have the different number with recorded poses"
        self.global_pcd = o3d.io.read_point_cloud(os.path.join(root, global_pcd))
        gpts = self.global_pcd.select_by_index(range(int(len(self.global_pcd.points) /2)))

        if self.display_steps:
            display_single_points(np.asarray(gpts.points))

    @staticmethod
    def read_poses_txt(file):
        poses = []
        with open(file) as f:
            lines = f.readlines()
            for line in lines:
                values = line.split(" ")
                pose = np.array(values).reshape((3, 4))
                poses.append(pose)
        return np.array(poses)

    def process_global(self):
        global_pts = np.asarray(self.global_pcd.points, dtype=np.float32)
        total_plane_mask = None
        for idx in range(0, self.poses.shape[0], self.frame_separate):
            pose = np.asarray(self.poses[idx], dtype=float)
            local_points = o3d.io.read_point_cloud(os.path.join(self.pcd_files_root, self.pcd_files[idx]))
            floor_mask = self.remove_ground(all_points=global_pts, pose=pose, local_points=local_points)

            if total_plane_mask is None:
                total_plane_mask = floor_mask
            else:
                total_plane_mask *= floor_mask

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(global_pts[total_plane_mask])
        cl, ind = pcd.remove_radius_outlier(nb_points=self.neighbor_threshold[0], radius=self.neighbor_threshold[1])
        pcd = pcd.select_by_index(ind)
        if self.display_steps:
            res = np.asarray(pcd.points, dtype=np.float32)
            display_single_points(res)

        o3d.io.write_point_cloud(self.pcd_saving_dir, pcd)
        cost_map = self.pts_to_map(np.asarray(pcd.points, dtype=np.float32))
        cv2.imwrite(self.png_saving_dir, cost_map * 255)

    def pts_to_map(self, pts):
        indices, return_counts = np.unique(np.floor(pts / self.resolution).astype(int)[:, :2], axis=0,
                                           return_counts=True)
        indices = indices[return_counts > 0]
        min_value = np.min(indices, axis=0)
        cost_map = np.zeros(np.max(indices - min_value, axis=0) + 1)
        for (x, y) in indices - min_value:
            cost_map[x, y] = 1
        return cost_map

    def _get_lane_mask(self, points, plane_model):
        ptsx = points[:, 0] * plane_model[0]
        ptsy = points[:, 1] * plane_model[1]
        ptsz = points[:, 2] * plane_model[2]
        distance = ptsz + ptsy + ptsx + plane_model[3]
        plane_mask = abs(distance) > self.floor_height
        return plane_mask

    def remove_ground(self, all_points, pose, local_points):
        cl, ind = local_points.remove_radius_outlier(nb_points=self.neighbor_threshold_init[0],
                                                     radius=self.neighbor_threshold_init[1])
        pcd = local_points.select_by_index(ind)
        # if self.display_remove:
        #     display_two_points(ref=local_points.points, src=pcd.points, src_point_size=4)

        plane_model, inliers = pcd.segment_plane(distance_threshold=self.distance_threshold,
                                                 ransac_n=self.ransac_num, num_iterations=1000)

        points = inverse_transform(all_points, transformation=pose)
        plane_mask = self._get_lane_mask(points=points, plane_model=plane_model)

        plane_mask &= points[:, 2] < self.height_threshold

        distances = np.linalg.norm(points[:, :2], axis=1)
        plane_mask += (distances > self.radius)

        if self.display_remove:
            plane_points = construct_from_plane(plane_model, dis_range=5)
            display_three_points(pts1=all_points[plane_mask], pts2=all_points[~plane_mask], pts3=plane_points,
                                 point_size1=2, point_size2=2, point_size3=3,
                                 color1=[1, 0, 0, 1], color2=[0, 1, 0, 1], color3=[0, 0, 1, 1])
        return plane_mask


def get_args():
    parser = argparse.ArgumentParser(description='process rosbags')
    parser.add_argument('--display_steps', action='store_true', default=False,
                        help="if to display the points before and after removing ground points")
    parser.add_argument('--display_remove', action='store_true', default=False,
                        help="if to display the each step of removing points.")
    parser.add_argument('--frame_separate', type=int, default=20,
                        help="the separation of frames used to remove the ground points")
    parser.add_argument('--folder', type=str, help="the folder of the lio-sam results",
                        default="results/iribe2")
    return parser.parse_args()


if __name__ == "__main__":
    args = get_args()
    PCD_processor = PCDProcessor(root=args.folder, frame_separate=args.frame_separate,
                                 display_steps=args.display_steps, display_remove=args.display_remove)
    PCD_processor.process_global()
