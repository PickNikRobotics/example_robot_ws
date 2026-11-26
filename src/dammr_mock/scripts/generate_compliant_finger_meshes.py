#!/usr/bin/env python3
"""Generate the compliant fingertip meshes used by the DAMMR gripper.

Writes ASCII STL files to ../meshes. Units are meters, in the finger link frame:
the contact face is the plane y = 0, the fin grows toward +y (away from the
opposing finger) and toward +z (distal), and it is centered on x = 0.

Usage: python3 generate_compliant_finger_meshes.py
"""

import math
import pathlib

FIN_DEPTH = 0.025  # Extent along x.
FIN_LENGTH = 0.10  # Extent along z, from the finger base to the tip.
FIN_BASE_WIDTH = 0.03  # Extent along y at the finger base.
FIN_TIP_WIDTH = 0.004  # Extent along y at the tip.
BEAM_THICKNESS = 0.0025
RIB_THICKNESS = 0.002
RIB_COUNT = 4
RIB_DROP = 0.006  # Ribs slope down toward the back beam, so the finger wraps around a pressed object.


def back_beam_y(z: float) -> float:
    """Return the y of the outer edge of the back beam at height z."""
    return FIN_BASE_WIDTH + (FIN_TIP_WIDTH - FIN_BASE_WIDTH) * z / FIN_LENGTH


def prism(quad_yz):
    """Extrude a convex (y, z) quadrilateral along x into 12 outward-facing triangles."""
    half = FIN_DEPTH / 2.0
    front = [(-half, y, z) for y, z in quad_yz]
    back = [(half, y, z) for y, z in quad_yz]
    faces = [front[::-1], back]
    for i in range(4):
        j = (i + 1) % 4
        faces.append([front[i], front[j], back[j], back[i]])
    centroid = [sum(p[k] for p in front + back) / 8.0 for k in range(3)]
    triangles = []
    for face in faces:
        for tri in ((face[0], face[1], face[2]), (face[0], face[2], face[3])):
            normal = _normal(*tri)
            to_face = [tri[0][k] - centroid[k] for k in range(3)]
            if sum(normal[k] * to_face[k] for k in range(3)) < 0.0:
                tri = (tri[0], tri[2], tri[1])
            triangles.append(tri)
    return triangles


def _normal(a, b, c):
    u = [b[k] - a[k] for k in range(3)]
    v = [c[k] - a[k] for k in range(3)]
    n = [
        u[1] * v[2] - u[2] * v[1],
        u[2] * v[0] - u[0] * v[2],
        u[0] * v[1] - u[1] * v[0],
    ]
    length = math.sqrt(sum(x * x for x in n)) or 1.0
    return [x / length for x in n]


def write_stl(path: pathlib.Path, name: str, triangles) -> None:
    lines = [f"solid {name}"]
    for tri in triangles:
        n = _normal(*tri)
        lines.append(f"  facet normal {n[0]:.6f} {n[1]:.6f} {n[2]:.6f}")
        lines.append("    outer loop")
        for p in tri:
            lines.append(f"      vertex {p[0]:.6f} {p[1]:.6f} {p[2]:.6f}")
        lines.append("    endloop")
        lines.append("  endfacet")
    lines.append(f"endsolid {name}")
    path.write_text("\n".join(lines) + "\n")


def visual_triangles():
    """Front beam, back beam, base plate and sloped ribs of the finger."""
    tris = []
    # Front (contact) beam along the contact face.
    tris += prism(
        [
            (0.0, 0.0),
            (BEAM_THICKNESS, 0.0),
            (BEAM_THICKNESS, FIN_LENGTH),
            (0.0, FIN_LENGTH),
        ]
    )
    # Back beam, tapering from the base width to the tip.
    tris += prism(
        [
            (FIN_BASE_WIDTH - BEAM_THICKNESS, 0.0),
            (FIN_BASE_WIDTH, 0.0),
            (FIN_TIP_WIDTH, FIN_LENGTH),
            (FIN_TIP_WIDTH - BEAM_THICKNESS, FIN_LENGTH),
        ]
    )
    # Base plate joining both beams.
    tris += prism(
        [(0.0, 0.0), (FIN_BASE_WIDTH, 0.0), (FIN_BASE_WIDTH, 0.004), (0.0, 0.004)]
    )
    # Ribs between the beams.
    for i in range(1, RIB_COUNT + 1):
        z_front = FIN_LENGTH * i / (RIB_COUNT + 1)
        z_back = z_front - RIB_DROP
        y_back = back_beam_y(z_back) - BEAM_THICKNESS / 2.0
        tris += prism(
            [
                (BEAM_THICKNESS / 2.0, z_front),
                (y_back, z_back),
                (y_back, z_back + RIB_THICKNESS),
                (BEAM_THICKNESS / 2.0, z_front + RIB_THICKNESS),
            ]
        )
    return tris


def collision_triangles():
    """Convex wedge enclosing the visual fin."""
    return prism(
        [
            (0.0, 0.0),
            (FIN_BASE_WIDTH, 0.0),
            (FIN_TIP_WIDTH, FIN_LENGTH),
            (0.0, FIN_LENGTH),
        ]
    )


def main() -> None:
    mesh_dir = pathlib.Path(__file__).resolve().parent.parent / "meshes"
    mesh_dir.mkdir(exist_ok=True)
    write_stl(
        mesh_dir / "compliant_finger_visual.stl",
        "compliant_finger_visual",
        visual_triangles(),
    )
    write_stl(
        mesh_dir / "compliant_finger_collision.stl",
        "compliant_finger_collision",
        collision_triangles(),
    )


if __name__ == "__main__":
    main()
