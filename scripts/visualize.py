import open3d as o3d
import numpy as np

def plot_polytope_from_file(filename):
    vertices = np.genfromtxt(filename, delimiter=',')
    if vertices.ndim != 2 or vertices.shape[1] != 3:
        print("Error: file must contain 3 columns (x, y, z).")
        return

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(vertices)
    hull_mesh, _ = pcd.compute_convex_hull()
    hull_mesh.compute_vertex_normals()

    # Faint translucent faces
    face_mat = o3d.visualization.rendering.MaterialRecord()
    face_mat.shader = "defaultLitTransparency"
    face_mat.base_color = [1.0, 0.0, 0.0, 0.7]   # RGBA

    # Black edges
    wire = o3d.geometry.LineSet.create_from_triangle_mesh(hull_mesh)
    wire.paint_uniform_color([0, 0, 0])
    wire_mat = o3d.visualization.rendering.MaterialRecord()
    wire_mat.shader = "unlitLine"
    wire_mat.line_width = 2.0

    # Vertices as spheres so they're clearly visible at any zoom
    spheres = o3d.geometry.TriangleMesh()
    for v in vertices:
        s = o3d.geometry.TriangleMesh.create_sphere(radius=0.08, resolution=10)
        s.translate(v)
        spheres += s
    spheres.compute_vertex_normals()
    spheres.paint_uniform_color([0.0, 0.3, 0.9])
    vert_mat = o3d.visualization.rendering.MaterialRecord()
    vert_mat.shader = "defaultLit"

    o3d.visualization.draw(
        [
            {"name": "faces",    "geometry": hull_mesh, "material": face_mat},
            {"name": "edges",    "geometry": wire,      "material": wire_mat},
            {"name": "vertices", "geometry": spheres,   "material": vert_mat},
        ],
        title="Polytope Visualization",
        width=1280, height=720,
        show_skybox=False,
    )

if __name__ == "__main__":
    plot_polytope_from_file('./scripts/vertices.txt')