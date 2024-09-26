import copy

import open3d
import open3d as o3d
# import torch
import numpy as np
from multiprocessing import Process


def display_single_points(pts, color=None, point_size=2, frame_size=2, frame_coordinate=None):
    app = o3d.visualization.gui.Application.instance
    app.initialize()
    window = app.create_window("Open3d", 1024, 768)
    widget3d = o3d.visualization.gui.SceneWidget()
    widget3d.scene = o3d.visualization.rendering.Open3DScene(window.renderer)
    widget3d.scene.set_background([0, 0, 0, 1.0])
    window.add_child(widget3d)
    if color is None:
        color = [1, 0, 0, 1]
    add_cloud(widget3d.scene, "ref", pts, color, point_size)

    if frame_coordinate is None:
        pose = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    elif frame_coordinate.shape[0] == 3 and len(frame_coordinate.shape) == 1:
        pose = np.array([[1, 0, 0, frame_coordinate[0]], [0, 1, 0, frame_coordinate[1]],
                         [0, 0, 1, frame_coordinate[2]], [0, 0, 0, 1]])
    else:
        pose = frame_coordinate
    add_frame(scene=widget3d.scene, name="ref_frame", frame_size=frame_size, frame=pose)

    widget3d.setup_camera(60, widget3d.scene.bounding_box, [0, 0, 0])
    app.run()


def display_two_points(ref, src, esrc=None, point_size=2, draw_lines=False, src_point_size=2, src_frame=None, frame_size=2,):
    app = o3d.visualization.gui.Application.instance
    app.initialize()
    window = app.create_window("Open3d", 1024, 768)
    widget3d = o3d.visualization.gui.SceneWidget()
    widget3d.scene = o3d.visualization.rendering.Open3DScene(window.renderer)
    widget3d.scene.set_background([0, 0, 0, 1.0])
    window.add_child(widget3d)

    add_cloud(widget3d.scene, "ref", ref, [1, 0, 0, 1], point_size)
    add_cloud(widget3d.scene, "src", src, [0, 1, 0, 1], src_point_size)
    if esrc is not None:
        add_cloud(widget3d.scene, "esrc", esrc, [0, 0, 1, 1], src_point_size)

    add_frame(scene=widget3d.scene, name="ref_frame", frame_size=frame_size,
              frame=np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]))
    if src_frame is not None:
        add_frame(scene=widget3d.scene, name="src_frame", frame_size=frame_size, frame=src_frame)

    if draw_lines:
        assert ref.shape[0] == src.shape[0], Exception("the two poins have different sizes")
        add_edges(widget3d.scene, "nd_edges", ref, src, [0, 1, 1, 1], 2)

    widget3d.setup_camera(60, widget3d.scene.bounding_box, [0, 0, 0])
    app.run()

def display_three_points(pts1, pts2, pts3,
                         point_size1=2, point_size2=2,  point_size3=2,
                         color1=[1, 0, 0, 1], color2=[0, 1, 0, 1], color3=[0, 0, 1, 1],  frame_size=2):
    app = o3d.visualization.gui.Application.instance
    app.initialize()
    window = app.create_window("Open3d", 1024, 768)
    widget3d = o3d.visualization.gui.SceneWidget()
    widget3d.scene = o3d.visualization.rendering.Open3DScene(window.renderer)
    widget3d.scene.set_background([0, 0, 0, 1.0])
    window.add_child(widget3d)

    add_cloud(widget3d.scene, "pts1", pts1, color1, point_size1)
    add_cloud(widget3d.scene, "pts2", pts2, color2, point_size2)
    add_cloud(widget3d.scene, "pts3", pts3, color3, point_size3)

    add_frame(scene=widget3d.scene, name="ref_frame", frame_size=frame_size,
              frame=np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]))

    widget3d.setup_camera(60, widget3d.scene.bounding_box, [0, 0, 0])
    app.run()

def add_frame(scene, name, frame, frame_size):
    # if isinstance(frame, torch.Tensor):
    #     try:
    #         frame = frame.cpu().numpy()
    #     except:
    #         frame = frame.detach().cpu().numpy()
    mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(size=frame_size, origin=np.array([0.0, 0.0, 0.0]))
    mesh = copy.deepcopy(mesh)
    mesh.rotate(frame[:3, :3], center=(0, 0, 0))
    mesh.translate((frame[0, 3], frame[1, 3], frame[2, 3]))

    mtl = o3d.visualization.rendering.MaterialRecord()
    mtl.base_color = [1.0, 1.0, 1.0, 1.0]
    mtl.shader = "defaultUnlit"
    scene.add_geometry(name, mesh, mtl)


def add_cloud(scene, name, pts, color, size):
    # if isinstance(pts, torch.Tensor):
    #     try:
    #         pts = pts.cpu().numpy()
    #     except:
    #         pts = pts.detach().cpu().numpy()
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(pts)

    material = o3d.visualization.rendering.MaterialRecord()
    material.shader = "defaultUnlit"
    material.base_color = color
    material.point_size = size
    scene.add_geometry(name, cloud, material)


def add_edges(scene, name, pt0, pt1, color, size):
    # if isinstance(pt0, torch.Tensor):
    #     pt0 = pt0.cpu().numpy()
    # if isinstance(pt1, torch.Tensor):
    #     pt1 = pt1.cpu().numpy()
    assert pt0.shape[0] == pt1.shape[0], Exception("the two poins have different sizes")
    lns = np.array([[i, i + len(pt0)] for i in range(len(pt0))])
    # if isinstance(lns, torch.Tensor):
    #     lns = lns.cpu().numpy()
    assert lns.shape[1] == 2, Exception("the lines shape is not correct")
    lines = open3d.geometry.LineSet()
    lines.points = open3d.utility.Vector3dVector(np.concatenate((pt0, pt1), axis=0)[:, :3])
    lines.lines = open3d.utility.Vector2iVector(lns)
    # ln_c = np.zeros((lns.shape[0], 3))
    # ln_c[:, 1] += 1
    # lines.colors = open3d.utility.Vector3dVector(ln_c)

    material = o3d.visualization.rendering.MaterialRecord()
    material.shader = "defaultUnlit"
    material.base_color = color
    material.point_size = size
    scene.add_geometry(name, lines, material)


def display_points_path(pts, path, pts1=None, point_size=2, path_width=2, pts_color=None, path_color=None,
                        frame_size=2, frame_coordinate=None):
    app = o3d.visualization.gui.Application.instance
    app.initialize()
    window = app.create_window("Open3d", 1024, 768)
    widget3d = o3d.visualization.gui.SceneWidget()
    widget3d.scene = o3d.visualization.rendering.Open3DScene(window.renderer)
    widget3d.scene.set_background([0, 0, 0, 1.0])
    window.add_child(widget3d)
    if pts_color is None:
        color = [1, 0, 0, 1]
    add_cloud(widget3d.scene, "ref", pts, color, point_size)

    if pts1 is not None:
        color1 = [1, 1, 0, 1]
        add_cloud(widget3d.scene, "src", pts1, color1, point_size)

    if frame_coordinate is None:
        pose = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    elif frame_coordinate.shape[0] == 3 and len(frame_coordinate.shape) == 1:
        pose = np.array([[1, 0, 0, frame_coordinate[0]], [0, 1, 0, frame_coordinate[1]],
                         [0, 0, 1, frame_coordinate[2]], [0, 0, 0, 1]])
    else:
        pose = frame_coordinate
    add_frame(scene=widget3d.scene, name="ref_frame", frame_size=frame_size, frame=pose)

    if path_color is None:
        path_color = [0, 1, 1, 1]
    if type(path) == list:
        for p in path:
            add_edges(widget3d.scene, "nd_edges", p[1:, :], p[:-1, :], path_color, path_width)
    else:
        add_edges(widget3d.scene, "nd_edges", path[1:, :], path[:-1, :], path_color, path_width)

    widget3d.setup_camera(60, widget3d.scene.bounding_box, [0, 0, 0])
    app.run()

def display_pts_nds_knn(ref, src, rk=None, sk=None, rsc=None, ssc=None, rse=None, sse=None, rsg=None, ssg=None):
    app = o3d.visualization.gui.Application.instance
    app.initialize()
    window = app.create_window("Open3d", 1024, 768)
    widget3d = o3d.visualization.gui.SceneWidget()
    widget3d.scene = o3d.visualization.rendering.Open3DScene(window.renderer)
    widget3d.scene.set_background([0, 0, 0, 1.0])
    window.add_child(widget3d)

    pt_color = 0.6
    add_cloud(widget3d.scene, "ref_pts", ref, [pt_color, 0, 0, 1], 2)
    add_cloud(widget3d.scene, "src_pts", src, [0, 0, pt_color, 1], 2)

    nd_color = 1
    if rk is not None:
        add_cloud(widget3d.scene, "ref_kns", rk, [nd_color, 0, 0, 1], 5)
    if sk is not None:
        add_cloud(widget3d.scene, "src_kns", sk, [0, 0, nd_color, 1], 5)

    if rsc is not None and ssc is not None:
        add_edges(widget3d.scene, "common_edges", rsc, ssc, [1, 0, 0, 1], 2)
    if rse is not None and sse is not None:
        add_edges(widget3d.scene, "est_edges", rse, sse, [0, 0.5, 0, 1], 2)
    if rsg is not None and ssg is not None:
        add_edges(widget3d.scene, "gt_edges", rsg, ssg, [0.5, 0.5, 0, 1], 2)

    widget3d.setup_camera(60, widget3d.scene.bounding_box, [0, 0, 0])
    app.run()


def display_coarse_result(rng, sng, rkg, skg, rk=None, sk=None, rn=None, sn=None):
    app = o3d.visualization.gui.Application.instance
    app.initialize()
    window = app.create_window("Open3d", 1024, 768)
    widget3d = o3d.visualization.gui.SceneWidget()
    widget3d.scene = o3d.visualization.rendering.Open3DScene(window.renderer)
    widget3d.scene.set_background([0, 0, 0, 1.0])
    window.add_child(widget3d)

    add_cloud(widget3d.scene, "rkg", rkg, [0.6, 0, 0, 1], 2)
    add_cloud(widget3d.scene, "skg", skg, [0, 0, 0.6, 1], 2)
    add_cloud(widget3d.scene, "rng", rng, [0, 1, 1, 1], 5)
    add_cloud(widget3d.scene, "sng", sng, [0, 1, 0, 1], 5)
    add_edges(widget3d.scene, "gt_nd_edges", rng, sng, [0.6, 0, 0.6, 1], 2)

    if rn is not None:
        add_cloud(widget3d.scene, "rk", rk, [0, 0.6, 0, 1], 2)
        add_cloud(widget3d.scene, "sk", sk, [0, 0.6, 0.6, 1], 2)
        add_edges(widget3d.scene, "nd_edges", rn, sn, [0.6, 0.6, 0, 1], 2)
    #

    widget3d.setup_camera(60, widget3d.scene.bounding_box, [0, 0, 0])
    app.run()


def display_points_lines(pts, point_colors, ref, src, point_size=2, draw_origin=True):
    # if isinstance(pts, torch.Tensor):
    #     pts = pts.cpu().numpy()
    # if isinstance(point_colors, torch.Tensor):
    #     point_colors = point_colors.cpu().numpy()
    # if isinstance(ref, torch.Tensor):
    #     ref = ref.cpu().numpy()
    # if isinstance(src, torch.Tensor):
    #     src = src.cpu().numpy()
    ref_points = ref[:, :3]
    src_points = src[:, :3]
    all_points = np.concatenate((ref_points, src_points), axis=0)
    pts = pts[:, :3]

    # point_colors = np.zeros_like(all_points)
    # point_colors[:, 0] += 1

    vis = open3d.visualization.Visualizer()
    vis.create_window()
    vis.get_render_option().background_color = np.zeros(3)
    vis.get_render_option().point_size = point_size
    if draw_origin:
        axis_pcd = open3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0, origin=[0, 0, 0])
        vis.add_geometry(axis_pcd)

    pt_geo = open3d.geometry.PointCloud()
    pt_geo.points = open3d.utility.Vector3dVector(pts[:, :3])
    if point_colors is None:
        pt_geo.colors = open3d.utility.Vector3dVector(np.ones((pts.shape[0], 3)))
    elif len(point_colors.shape) == 1:
        point_colors /= np.amax(point_colors)
        point_colors_all = np.stack((point_colors, 1 - point_colors, 1 - point_colors), axis=1)
        pt_geo.colors = open3d.utility.Vector3dVector(point_colors_all)
    else:
        pt_geo.colors = open3d.utility.Vector3dVector(point_colors)
    vis.add_geometry(pt_geo)

    assert ref.shape[0] == src.shape[0], Exception("the two poins have different sizes")
    lns = np.array([[i, i + len(ref)] for i in range(len(ref))])
    # if isinstance(lns, torch.Tensor):
    #     lns = lns.cpu().numpy()
    assert lns.shape[1] == 2, Exception("the lines shape is not correct")
    ln_c = np.zeros((lns.shape[0], 3))
    ln_c[:, 1] += 1
    lines = open3d.geometry.LineSet()
    lines.points = open3d.utility.Vector3dVector(all_points[:, :3])
    lines.lines = open3d.utility.Vector2iVector(lns)
    lines.colors = open3d.utility.Vector3dVector(ln_c)
    vis.add_geometry(lines)

    vis.run()
    vis.destroy_window()


def display_points(pts, point_colors=None, point_size=2, draw_origin=True):
    # if isinstance(pts, torch.Tensor):
    #     pts = pts.cpu().numpy()
    # if point_colors is not None:
        # if isinstance(point_colors, torch.Tensor):
        #     point_colors = point_colors.cpu().numpy()
    all_points = pts[:, :3]

    vis = open3d.visualization.Visualizer()
    vis.create_window()
    vis.get_render_option().background_color = np.zeros(3)
    vis.get_render_option().point_size = point_size
    if draw_origin:
        axis_pcd = open3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0, origin=[0, 0, 0])
        vis.add_geometry(axis_pcd)

    pts = open3d.geometry.PointCloud()
    pts.points = open3d.utility.Vector3dVector(all_points[:, :3])
    if point_colors is None:
        pts.colors = open3d.utility.Vector3dVector(np.ones((all_points.shape[0], 3)))
    elif len(point_colors.shape) == 1:
        point_colors /= np.amax(point_colors)
        point_colors_all = np.stack((point_colors, 1 - point_colors, 1 - point_colors), axis=1)
        pts.colors = open3d.utility.Vector3dVector(point_colors_all)
    else:
        pts.colors = open3d.utility.Vector3dVector(point_colors)
    vis.add_geometry(pts)

    vis.run()
    vis.destroy_window()


class Visualization:
    def __init__(self, draw_origin: bool = True, points_size=2):
        # self.vis = open3d.visualization.Visualizer()
        self.vis = open3d.visualization.VisualizerWithKeyCallback()
        self.vis.create_window()
        self.vis.get_render_option().background_color = np.zeros(3)
        self.vis.get_render_option().point_size = points_size
        # self.vis.register_key_callback(key, your_update_function)
        if draw_origin:
            axis_pcd = open3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0, origin=[0, 0, 0])
            self.vis.add_geometry(axis_pcd)

        self.geometry = None
        self.coarse_points = None
        self.coarse_cooresponds = None

        self.vis.run()
        # self.thread = Process(target=self.vis.run)
        # self.thread.start()

    def destroy_window(self):
        # self.thread.join()
        self.vis.destroy_window()

    def _render(self):
        self.vis.poll_events()
        self.vis.update_renderer()

    def _update_geo(self, points, point_colors):
        tag = False
        if self.geometry is None:
            self.geometry = open3d.geometry.PointCloud()
            tag = True
        self.geometry.points = open3d.utility.Vector3dVector(points[:, :3])
        if point_colors is None:
            self.geometry.colors = open3d.utility.Vector3dVector(np.ones((points.shape[0], 3)))
        elif len(point_colors.shape) == 1:
            point_colors /= np.amax(point_colors)
            point_colors_all = np.stack((point_colors, 1 - point_colors, 1 - point_colors), axis=1)
            self.geometry.colors = open3d.utility.Vector3dVector(point_colors_all)
        else:
            self.geometry.colors = open3d.utility.Vector3dVector(point_colors)
        if tag:
            self.vis.add_geometry(self.geometry)

    def _update_coarse(self, points, coors, colors=None, lnc=None):
        assert coors.shape[1] == 2, Exception("the lines shape is not correct")
        # if isinstance(coors, torch.Tensor):
        #     coors = coors.cpu().numpy()

        tag = False
        if self.coarse_cooresponds is None:
            self.coarse_cooresponds = open3d.geometry.LineSet()
            tag = True
        self.coarse_cooresponds.points = open3d.utility.Vector3dVector(points[:, :3])
        self.coarse_cooresponds.lines = open3d.utility.Vector2iVector(coors)
        if lnc is None:
            lnc = np.zeros((coors.shape[0], 3))
            lnc[:, 1] += 1
        self.coarse_cooresponds.colors = open3d.utility.Vector3dVector(lnc)
        if tag:
            self.vis.add_geometry(self.coarse_cooresponds)
        else:
            self.vis.update_geometry(self.coarse_cooresponds)

        tag = False
        if self.coarse_points is None:
            self.coarse_points = open3d.geometry.PointCloud()
            tag = True
        self.coarse_points.points = open3d.utility.Vector3dVector(points[:, :3])
        if colors is None:
            self.coarse_points.colors = open3d.utility.Vector3dVector(np.ones((points.shape[0], 3)))
        else:
            self.coarse_points.colors = open3d.utility.Vector3dVector(colors)
        if tag:
            self.vis.add_geometry(self.coarse_points)
        else:
            self.vis.update_geometry(self.coarse_points)
        self.vis.update_geometry(self.coarse_points)

    def update_points(self, points, point_colors=None, point_size=None):
        # if isinstance(points, torch.Tensor):
        #     points = points.cpu().numpy()

        if point_size is not None:
            self.vis.get_render_option().point_size = point_size
        self._update_geo(points=points, point_colors=point_colors)
        self.vis.update_geometry(self.geometry)
        self._render()

    def draw_points(self, points, point_colors=None, point_size=2):
        # if isinstance(points, torch.Tensor):
        #     points = points.cpu().numpy()
        # if isinstance(point_colors, torch.Tensor):
        #     point_colors = point_colors.cpu().numpy()

        self.vis.get_render_option().point_size = point_size

        self._update_geo(points, point_colors)

        self.vis.add_geometry(self.geometry)
        self._render()

    def draw_two_points(self, ref, src, point_size=5, colors=None, lnc=None):
        # if isinstance(ref, torch.Tensor):
        #     ref = ref.cpu().numpy()
        # if isinstance(src, torch.Tensor):
        #     src = src.cpu().numpy()
        all_points = np.concatenate((ref, src), axis=0)

        self.vis.get_render_option().point_size = point_size

        if colors is None:
            src_color = np.zeros_like(src)
            src_color[:, 0] += 1
            ref_color = np.zeros_like(ref)
            ref_color[:, [0, 1]] += 1
            all_color = np.concatenate((ref_color, src_color), axis=0)
        else:
            all_color = colors

        assert ref.shape[0] == src.shape[0], Exception("the two poins have different sizes")
        lines = np.array([[i, i + len(ref)] for i in range(len(ref))])

        self._update_coarse(points=all_points, coors=lines, colors=all_color, lnc=lnc)
        self._render()

    def visualize_GD(self, gaussians):
        pass


if __name__ == "__main__":
    p1 = np.array([[1, 5, 2], [2, 3, 4], [1, 3, 4]])
    p2 = np.array([[2, 5, 1], [3, 3, 1], [2, 2, 2]])
    ref_points = p1[:, :3]
    src_points = p2[:, :3]
    # display_two_points(ref_points, src_points)
    display_single_points(ref_points)
    # all_points = np.concatenate((src_points, ref_points), axis=0)
    #
    # vis = Visualization()
    # vis.draw_two_points(p1, p2, draw_line=True, point_size=5)
    # vis.draw_points(all_points, point_size=5)

    print("test")
