import os
import copy
import pickle
import time
import numpy as np
import open3d as o3d
import teaserpp_python
import cv2
import argparse
from utils.visualization import display_two_points, display_single_points

NOISE_BOUND = 0.05
TEASER_MAX_NUM = 12000


def register_points(src, dst):
    src = np.transpose(src)
    dst = np.transpose(dst)

    solver_params = teaserpp_python.RobustRegistrationSolver.Params()
    solver_params.cbar2 = 1
    solver_params.noise_bound = NOISE_BOUND
    solver_params.estimate_scaling = False
    solver_params.rotation_estimation_algorithm = teaserpp_python.RobustRegistrationSolver.ROTATION_ESTIMATION_ALGORITHM.GNC_TLS
    solver_params.rotation_gnc_factor = 1.4
    solver_params.rotation_max_iterations = 1000
    solver_params.rotation_cost_threshold = 1e-12

    solver = teaserpp_python.RobustRegistrationSolver(solver_params)
    start = time.time()
    solver.solve(src, dst)
    end = time.time()

    solution = solver.getSolution()

    # print("Estimated rotation: ")
    # print(solution.rotation)
    #
    # print("Estimated translation: ")
    # print(solution.translation)

    R = np.matrix(solution.rotation)
    T = np.matrix(solution.translation)
    I = np.matrix([0, 0, 0, 1])
    Trans = np.concatenate([R, T.transpose()], axis=1)
    Trans = np.concatenate([Trans, I])

    print("transformation matrix: ")
    print(Trans.transpose().transpose())
    print("Number of correspondences: ", src.shape[1])
    print("Time taken (s): ", end - start)
    return Trans


def select_points(pts, threshold, choose_type=0):
    if pts.shape[0] > TEASER_MAX_NUM:
        if choose_type == 0:
            indices = np.where((pts[:, 0] > threshold[0]) * (pts[:, 0] < threshold[1]) * \
                               (pts[:, 1] > threshold[2]) * (pts[:, 1] < threshold[3]))
            return pts[indices]
        elif choose_type == 1:
            selected_indices = np.random.choice(pts.shape[0], TEASER_MAX_NUM)
            return pts[selected_indices]
        elif choose_type == 2:
            return pts[:TEASER_MAX_NUM]
    else:
        return pts


def process_files(src_folder, dst_folder, root, src_threshold, dst_threshold, resolution=0.1, display=False, choose_type=0):
    output_folder = os.path.join(root, "merged")
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    src_file = os.path.join(root, src_folder, 'map_removed_ground_top.pcd')
    dst_file = os.path.join(root, dst_folder, 'map_removed_ground_top.pcd')

    src_cloud = o3d.io.read_point_cloud(src_file)
    dst_cloud = o3d.io.read_point_cloud(dst_file)

    src = np.asarray(src_cloud.points)
    dst = np.asarray(dst_cloud.points)

    src_selected = select_points(pts=src, threshold=src_threshold, choose_type=choose_type)
    assert src_selected.shape[0] <= TEASER_MAX_NUM, \
        "teaser can process at most 12000 points, the size of src is {}".format(src.shape[0])
    if display:
        print("displaying src points")
        display_single_points(src)
        print("displaying selected src points")
        display_single_points(src_selected)

    dst_selected = select_points(pts=dst, threshold=dst_threshold, choose_type=choose_type)
    assert dst_selected.shape[0] <= TEASER_MAX_NUM, \
        "teaser can process at most 12000 points, the size of dst is {}".format(dst.shape[0])
    if display:
        print("displaying dst points")
        display_single_points(dst)
        print("displaying selected dst points")
        display_single_points(dst_selected)

    trans = register_points(src=src_selected, dst=dst_selected)
    src_cloud.transform(trans)
    # if display:
    #     o3d.visualization.draw_geometries([src_cloud])
    pcd = src_cloud + dst_cloud
    if display:
        o3d.visualization.draw_geometries([pcd])
    # o3d.io.write_point_cloud(os.path.join(root, 'merged/{}_{}_merged.pcd'.format(src_folder, dst_folder)), pcd)

    res = np.asarray(pcd.points, dtype=np.float32)
    indices, return_counts = np.unique(np.floor(res / resolution).astype(np.int)[:, :2], axis=0, return_counts=True)
    min_value = np.min(indices, axis=0)
    cost_map = np.zeros(np.max(indices - min_value, axis=0) + 1)
    for (x, y) in indices - min_value:
        cost_map[x, y] = 1
    cv2.imwrite(os.path.join(root, 'merged/{}_{}_merged.png'.format(src_folder, dst_folder)), cost_map * 255)

    with open(os.path.join(output_folder, "{}_{}_merged.pkl".format(src_folder, dst_folder)), 'wb') as handle:
        pickle.dump({
            "min_xy": min_value,
            "transformation": trans,
            "resolution": resolution,
            "points": np.asarray(pcd.points)
        }, handle, protocol=pickle.HIGHEST_PROTOCOL)


def get_args():
    parser = argparse.ArgumentParser(description='process rosbags')
    parser.add_argument('--display', action='store_true', default=False, help="if to display the points for debugging")
    parser.add_argument('--choose_type', type=int, default=2,
                        help="methods to choose points: 0: use threshold; 1: downsample; 2: first 12000;")
    parser.add_argument('--src_folder', type=str, help="the folder name of the source points", default="avw1")
    parser.add_argument('--dst_folder', type=str, help="the folder name of the destination points", default="iribe2")
    parser.add_argument('--root', type=str, help="the folder of the all lio-sam results", default="results")
    parser.add_argument('--src_threshold', nargs='+', type=float, default=(0, 0),
                        help="threshold the src points: xmin, xmax, ymin, ymax")
    parser.add_argument('--dst_threshold', nargs='+', type=float, default=(0, 0),
                        help="threshold the src points: xmin, xmax, ymin, ymax")
    return parser.parse_args()


if __name__ == "__main__":
    args = get_args()
    process_files(src_folder=args.src_folder, dst_folder=args.dst_folder, root=args.root, resolution=0.1,
                  src_threshold=tuple(args.src_threshold), dst_threshold=tuple(args.dst_threshold),
                  display=args.display, choose_type=args.choose_type)
