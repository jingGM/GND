from tqdm import tqdm
import numpy as np
import open3d as o3d


def _get_a_plane(points):
    indices = np.random.choice(range(len(points)), size=3)
    pts = points[indices]
    v1 = pts[2] - pts[0]
    v2 = pts[1] - pts[0]
    a, b, c = cp = np.cross(v1, v2)
    if np.dot(cp, np.array([0, 0, 1])) > 0:
        a, b, c = cp = np.cross(v2, v1)
    d = -np.dot(cp, pts[2])
    return np.array([a, b, c, d]), np.array([a, b, c]) / np.linalg.norm([a, b, c])


def calculate_disance(pt, plane):
    projection = abs(np.dot(pt, plane[:3]) + plane[-1])
    length = np.linalg.norm(plane[:3])
    return projection / length


def _calculate_all_distance(points, plane):
    all_distance = 0
    item = 0
    for pt in points:
        projection = abs(np.dot(pt, plane[:3]) + plane[-1])
        length = np.linalg.norm(plane[:3])
        # with warnings.catch_warnings():
        #     warnings.filterwarnings('error')
        #     try:
        #         dis = projection / length
        #     except Warning as e:
        #         print("test")
        #         continue
        all_distance += projection / length
        item += 1
    return all_distance / item


def ransac_iterations(points, iteration_num):
    best_plane = None
    best_norm_vec = None
    distance = np.infty
    for i in tqdm(range(iteration_num)):
        plane, norm_vec = _get_a_plane(points)
        # self.construct_from_plane(plane=plane, display=True)
        dis = _calculate_all_distance(points=points, plane=plane)
        if dis < distance:
            best_plane = np.array(plane)
            best_norm_vec = norm_vec
    return best_plane, best_norm_vec


def construct_plane_from_normal_vector(normal):
    point = np.array([0, 0, 0])
    normalized_normal = normal / np.linalg.norm(normal)
    d = -np.dot(normalized_normal, point)
    plane = np.append(normalized_normal, d)
    return plane


def reconstruct_plane(plane, norm_vec, dis):
    new_norm = norm_vec * dis
    plane[3] += np.dot(new_norm, plane[:3])
    return plane


def remove_points(pts, plane, top_plane=None, top_dis=None, radius=None):
    assert len(pts.shape) == 2 and pts.shape[1] == 3, "the points shape is not correct"
    ptsx = pts[:, 0] * plane[0]
    ptsy = pts[:, 1] * plane[1]
    ptsz = pts[:, 2] * plane[2]
    distance = ptsz + ptsy + ptsx + plane[3]
    mask = distance < 0

    if top_plane is not None:
        ptsx = pts[:, 0] * top_plane[0]
        ptsy = pts[:, 1] * top_plane[1]
        ptsz = pts[:, 2] * top_plane[2]
        distance = ptsz + ptsy + ptsx + top_plane[3]
        mask &= distance > 0
    if top_dis is not None:
        mask &= pts[:, 2] < top_dis

    if radius is not None:
        distances = np.linalg.norm(pts[:, :2], axis=1)
        mask += (distances > radius)
    return mask


def construct_from_plane(plane, points_num=1000, dis_range=2):
    pts = []
    if type(dis_range) == int:
        xmin = 0
        xmax = dis_range
        ymin = -dis_range
        ymax = dis_range
    else:
        xmin = dis_range[0][0]
        xmax = dis_range[0][1]
        ymin = dis_range[1][0]
        ymax = dis_range[1][1]
    for i in range(points_num):
        x = xmin + (xmax - xmin) * np.random.uniform(0, 1)
        y = ymin + (ymax - ymin) * np.random.uniform(0, 1)
        z = -(plane[0] * x + plane[1] * y + plane[3]) / plane[2]
        pts.append([x, y, z])
    return np.asarray(pts)


def choose_points(points, pts_range=((0, 3), (-1, 1)), downsample_pth=None):
    mask = (points[:, 0] < pts_range[0][1]) * (points[:, 0] > pts_range[0][0]) * \
           (points[:, 1] < pts_range[1][1]) * (points[:, 1] > pts_range[1][0])
    masked_pts = points[mask]

    if downsample_pth is not None:
        vx_size = 0.1
        value = masked_pts.shape[0]
        while value > downsample_pth:
            process_points = o3d.geometry.PointCloud()
            process_points.points = o3d.utility.Vector3dVector(masked_pts)
            n_pts = np.asarray(process_points.voxel_down_sample(voxel_size=vx_size).points)
            value = n_pts.shape[0]
            if value > downsample_pth:
                vx_size += 0.1
            else:
                masked_pts = n_pts
    return masked_pts






# def inverse_transform(pts, transformation):
#     assert len(pts.shape) == 2 and pts.shape[1] == 3, "The points shape is not correct"
#     pts = pts - transformation[:, 3].reshape(1, 3)
#     pts = np.matmul(pts, transformation[:3, :3])
#     return pts



def inverse_transform(pts, transformation):
    assert len(pts.shape) == 2 and pts.shape[1] == 3, "The points shape is not correct"
    
    # Transpose
    rotation_inv = transformation[:3, :3].T

    # Apply inverse rotation
    pts_rotated = np.dot(pts, rotation_inv)

    # Apply inverse translation
    translation_inv = -np.dot(transformation[:3, 3], rotation_inv)
    pts_transformed = pts_rotated + translation_inv

    return pts_transformed






def apply_transform(pts, transformation):
    assert len(pts.shape) == 2 and pts.shape[1] == 3, "The points shape is not correct"
    pts = np.matmul(pts, np.transpose(transformation[:3, :3]))
    pts = pts + transformation[:, 3].reshape(1, 3)
    return pts