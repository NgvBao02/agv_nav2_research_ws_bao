#!/usr/bin/env python3
"""Render accurate 3D explanatory figures from the Gazebo STL/SDF geometry.

These figures are explanatory illustrations. They deliberately do not replace
RViz2/Gazebo screenshots used as experimental evidence.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import PolyCollection
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / "src" / "vacuum_robot_gazebo" / "models" / "vacuum_robot"
OUT = ROOT / "docs" / "pstmo_bao_cao_toan_dien_assets" / "visuals_3d"

NAVY = "#102A43"
BLUE = "#1D6FB8"
TEAL = "#0F766E"
GREEN = "#169C52"
ORANGE = "#E8871E"
PURPLE = "#7257B5"
INK = "#17212B"
MUTED = "#5B6876"
LIGHT = "#F4F7FA"
SILVER = "#AEB8C2"
WHEEL = "#20262C"
BRASS = "#B88734"


@dataclass(frozen=True)
class MeshPart:
    name: str
    triangles: np.ndarray
    color: str
    alpha: float = 1.0


def load_stl(path: Path) -> np.ndarray:
    """Read a binary or ASCII STL and return (n, 3, 3) vertices."""
    data = path.read_bytes()
    if len(data) >= 84:
        count = struct.unpack_from("<I", data, 80)[0]
        if 84 + count * 50 == len(data):
            dtype = np.dtype(
                [
                    ("normal", "<f4", (3,)),
                    ("vertices", "<f4", (3, 3)),
                    ("attribute", "<u2"),
                ]
            )
            return np.frombuffer(data, dtype=dtype, count=count, offset=84)["vertices"].astype(float)

    vertices: list[list[float]] = []
    for raw in data.decode("utf-8", errors="ignore").splitlines():
        line = raw.strip().split()
        if len(line) == 4 and line[0].lower() == "vertex":
            vertices.append([float(line[1]), float(line[2]), float(line[3])])
    if not vertices or len(vertices) % 3:
        raise ValueError(f"Không đọc được STL: {path}")
    return np.asarray(vertices, dtype=float).reshape(-1, 3, 3)


def transform(triangles: np.ndarray, translation: tuple[float, float, float], scale: float = 0.001) -> np.ndarray:
    return triangles * scale + np.asarray(translation, dtype=float)


def mesh_parts(*, exploded: bool = False) -> list[MeshPart]:
    # SDF: base_link is at z=0.0425 m. Visual translations below are relative
    # to their links. Wheel link translations are included explicitly.
    base = transform(load_stl(MODEL / "meshes" / "base_link.stl"), (-0.035315, 0.200943, -0.169198))
    left = transform(load_stl(MODEL / "meshes" / "left_wheel_link_1.stl"), (-0.0353034, 0.200943, -0.1666883))
    right = transform(load_stl(MODEL / "meshes" / "right_wheel_link_1.stl"), (-0.0353129, 0.200943, -0.1666955))
    if exploded:
        base = base + np.array([0.0, 0.0, 0.105])
        left = left + np.array([0.0, 0.105, 0.0])
        right = right + np.array([0.0, -0.105, 0.0])
    return [
        MeshPart("Thân CAD", base, SILVER),
        MeshPart("Bánh trái", left, WHEEL),
        MeshPart("Bánh phải", right, WHEEL),
    ]


def add_mesh(ax, part: MeshPart, *, edge: str = "#4D5966", linewidth: float = 0.05) -> None:
    poly = Poly3DCollection(
        part.triangles,
        facecolors=part.color,
        edgecolors=edge,
        linewidth=linewidth,
        alpha=part.alpha,
        shade=True,
    )
    ax.add_collection3d(poly)


def box_faces(center, size):
    cx, cy, cz = center
    sx, sy, sz = size
    x0, x1 = cx - sx / 2, cx + sx / 2
    y0, y1 = cy - sy / 2, cy + sy / 2
    z0, z1 = cz - sz / 2, cz + sz / 2
    p = np.array(
        [
            [x0, y0, z0], [x1, y0, z0], [x1, y1, z0], [x0, y1, z0],
            [x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1],
        ]
    )
    return [p[[0, 1, 2, 3]], p[[4, 5, 6, 7]], p[[0, 1, 5, 4]], p[[1, 2, 6, 5]], p[[2, 3, 7, 6]], p[[3, 0, 4, 7]]]


def add_box(ax, center, size, color, *, alpha=1.0, edge="#4D5966", linewidth=0.25):
    ax.add_collection3d(
        Poly3DCollection(box_faces(center, size), facecolors=color, edgecolors=edge, linewidth=linewidth, alpha=alpha, shade=True)
    )


def add_cylinder_y(ax, center, radius, length, color, *, alpha=1.0):
    cx, cy, cz = center
    theta = np.linspace(0, 2 * np.pi, 40)
    yy = np.linspace(cy - length / 2, cy + length / 2, 2)
    tt, yy = np.meshgrid(theta, yy)
    xx = cx + radius * np.cos(tt)
    zz = cz + radius * np.sin(tt)
    ax.plot_surface(xx, yy, zz, color=color, linewidth=0, antialiased=True, alpha=alpha, shade=True)


def add_sphere(ax, center, radius, color, *, alpha=1.0):
    u = np.linspace(0, 2 * np.pi, 30)
    v = np.linspace(0, np.pi, 18)
    x = center[0] + radius * np.outer(np.cos(u), np.sin(v))
    y = center[1] + radius * np.outer(np.sin(u), np.sin(v))
    z = center[2] + radius * np.outer(np.ones_like(u), np.cos(v))
    ax.plot_surface(x, y, z, color=color, linewidth=0, alpha=alpha, shade=True)


def add_robot(ax, *, exploded=False, offset=(0.0, 0.0, 0.0), alpha=1.0, with_components=True):
    shift = np.asarray(offset)
    for part in mesh_parts(exploded=exploded):
        add_mesh(ax, MeshPart(part.name, part.triangles + shift, part.color, alpha))
    if not with_components:
        return
    extra = 0.105 if exploded else 0.0
    motor_y = 0.190 if exploded else 0.0934
    add_cylinder_y(ax, shift + np.array([0.0, motor_y, 0.0425]), 0.0125, 0.068, BRASS, alpha=alpha)
    add_cylinder_y(ax, shift + np.array([0.0, -motor_y, 0.0425]), 0.0125, 0.068, BRASS, alpha=alpha)
    # BNO055 board in the lower tray and the actual LiDAR sample origin.
    imu_z = -0.005 if exploded else 0.0297
    add_box(ax, shift + np.array([0.0, 0.0, imu_z]), (0.025, 0.020, 0.005), GREEN, alpha=alpha)
    lidar_z = 0.15142 + extra
    add_sphere(ax, shift + np.array([0.0, 0.0, lidar_z]), 0.008, BLUE, alpha=alpha)


def style_axes(ax, limits, *, elev=25, azim=-55, grid=False):
    (xmin, xmax), (ymin, ymax), (zmin, zmax) = limits
    ax.set_xlim(xmin, xmax)
    ax.set_ylim(ymin, ymax)
    ax.set_zlim(zmin, zmax)
    ax.set_box_aspect((xmax - xmin, ymax - ymin, zmax - zmin))
    ax.view_init(elev=elev, azim=azim)
    ax.set_proj_type("persp", focal_length=0.9)
    ax.set_axis_off()
    if grid:
        ax.set_axis_on()


def save(fig, name):
    OUT.mkdir(parents=True, exist_ok=True)
    path = OUT / name
    fig.savefig(path, dpi=220, bbox_inches="tight", facecolor=fig.get_facecolor())
    plt.close(fig)
    return path


def render_robot_isometric():
    fig = plt.figure(figsize=(12, 7.6), facecolor="white")
    ax = fig.add_axes([0.01, 0.02, 0.70, 0.92], projection="3d")
    add_robot(ax)
    # Ground shadow/footprint matches the configured conservative envelope.
    add_box(ax, (0.0, 0.0, 0.001), (0.44, 0.34, 0.002), BLUE, alpha=0.08, edge=BLUE, linewidth=0.7)
    # A less oblique azimuth keeps the near wheel at the chassis side instead
    # of projecting it visually into the front-centre opening.
    style_axes(ax, ((-0.27, 0.27), (-0.25, 0.25), (-0.02, 0.27)), elev=28, azim=-30)
    fig.text(0.055, 0.925, "MÔ HÌNH 3D THEO STL/SDF", fontsize=21, weight="bold", color=NAVY)
    fig.text(0.055, 0.885, "Hình học đúng với mô hình vacuum_robot trong Gazebo", fontsize=11.5, color=MUTED)
    fig.text(0.735, 0.80, "Cấu phần được đối chiếu", fontsize=15, weight="bold", color=NAVY)
    entries = [
        (SILVER, "Thân CAD · 4,6 kg"),
        (WHEEL, "Hai bánh chủ động · y=±0,1274 m"),
        (BRASS, "Hai động cơ GA25"),
        (GREEN, "IMU BNO055"),
        (BLUE, "Gốc đo LiDAR · z=0,10892 m"),
    ]
    for idx, (color, label) in enumerate(entries):
        y = 0.72 - idx * 0.09
        fig.text(0.74, y, "●", fontsize=17, color=color, va="center")
        fig.text(0.77, y, label, fontsize=11.5, color=INK, va="center")
    fig.text(0.735, 0.205, "Hình bao đánh giá", fontsize=13.2, weight="bold", color=TEAL)
    fig.text(0.735, 0.155, "0,44 × 0,34 m", fontsize=18, weight="bold", color=TEAL)
    fig.text(0.735, 0.105, "Vệt bánh vật lý: 0,2548 m", fontsize=11.2, color=MUTED)
    return save(fig, "robot_isometric_from_stl_sdf.png")


def render_robot_clean():
    """Text-free model render for slide layouts with native callouts."""
    fig = plt.figure(figsize=(9.6, 6.4), facecolor="white")
    ax = fig.add_axes([0.01, 0.01, 0.98, 0.98], projection="3d")
    add_robot(ax)
    add_box(ax, (0.0, 0.0, 0.001), (0.44, 0.34, 0.002), BLUE, alpha=0.08, edge=BLUE, linewidth=0.7)
    style_axes(ax, ((-0.27, 0.27), (-0.25, 0.25), (-0.02, 0.27)), elev=28, azim=-30)
    return save(fig, "robot_isometric_clean.png")


def render_robot_exploded():
    fig = plt.figure(figsize=(12, 7.6), facecolor="white")
    ax = fig.add_axes([0.01, 0.02, 0.72, 0.92], projection="3d")
    add_robot(ax, exploded=True)
    # Assembly guide lines.
    for y0 in (-0.19, 0.19):
        ax.plot([0, 0], [y0, np.sign(y0) * 0.1274], [0.0425, 0.0425], color=ORANGE, linewidth=1.8, linestyle="--")
    ax.plot([0, 0], [0, 0], [0.155, 0.0425], color=BLUE, linewidth=1.8, linestyle="--")
    # Both separated wheels are visible from this view.
    style_axes(ax, ((-0.30, 0.30), (-0.36, 0.36), (-0.04, 0.34)), elev=28, azim=-15)
    fig.text(0.055, 0.925, "CẤU TẠO ROBOT VI SAI", fontsize=21, weight="bold", color=NAVY)
    fig.text(0.055, 0.885, "Hình tách lớp để trình bày; kích thước và chi tiết lấy từ SDF/STL", fontsize=11.5, color=MUTED)
    labels = [
        ("1", "Thân CAD", "Bao kết cấu chính; khối lượng 4,6 kg", SILVER),
        ("2", "Cụm bánh chủ động", "Hai bánh cao su; bán kính 0,0425 m", WHEEL),
        ("3", "Cụm truyền động", "Hai động cơ GA25 bố trí đối xứng", BRASS),
        ("4", "Cảm biến", "LiDAR 2D và IMU BNO055", BLUE),
    ]
    for idx, (n, title, detail, color) in enumerate(labels):
        y = 0.76 - idx * 0.16
        fig.text(0.755, y, n, fontsize=13, weight="bold", color="white", ha="center", va="center", bbox=dict(boxstyle="circle,pad=0.35", fc=color, ec="none"))
        fig.text(0.79, y + 0.015, title, fontsize=12.5, weight="bold", color=NAVY, va="center")
        fig.text(0.79, y - 0.028, detail, fontsize=10.2, color=MUTED, va="center")
    fig.text(0.755, 0.10, "Lưu ý: vị trí tách rời chỉ phục vụ minh họa lắp ghép.", fontsize=9.8, color=MUTED, style="italic")
    return save(fig, "robot_exploded_from_stl_sdf.png")


def render_footprint():
    fig = plt.figure(figsize=(12, 7.6), facecolor="white")
    ax = fig.add_axes([0.00, 0.03, 0.76, 0.91], projection="3d")
    add_robot(ax, alpha=0.97)
    # Full 0.44 x 0.34 m body footprint, not just the two wheel contact points.
    add_box(ax, (0.0, 0.0, 0.004), (0.44, 0.34, 0.008), TEAL, alpha=0.20, edge=TEAL, linewidth=1.0)
    ax.plot([-0.22, 0.22], [-0.19, -0.19], [0.005, 0.005], color=BLUE, linewidth=2)
    ax.plot([-0.22, -0.22], [-0.19, -0.17], [0.005, 0.005], color=BLUE, linewidth=2)
    ax.plot([0.22, 0.22], [-0.19, -0.17], [0.005, 0.005], color=BLUE, linewidth=2)
    ax.text(0.0, -0.205, 0.005, "0,44 m", color=BLUE, fontsize=10, ha="center")
    ax.plot([0.26, 0.26], [-0.17, 0.17], [0.005, 0.005], color=PURPLE, linewidth=2)
    ax.text(0.275, 0.0, 0.005, "0,34 m", color=PURPLE, fontsize=10, ha="center")
    style_axes(ax, ((-0.31, 0.31), (-0.27, 0.27), (-0.02, 0.25)), elev=31, azim=-30)
    fig.text(0.055, 0.925, "HÌNH BAO CHIẾM CHỖ CỦA ROBOT", fontsize=21, weight="bold", color=NAVY)
    fig.text(0.055, 0.885, "Footprint dùng để kiểm tra an toàn trên bản đồ chi phí", fontsize=11.5, color=MUTED)
    fig.text(0.77, 0.76, "Không kiểm tra riêng tâm robot", fontsize=13.5, weight="bold", color=TEAL)
    fig.text(0.77, 0.68, "Mỗi tư thế phải biến đổi toàn bộ đa giác\nhình bao từ hệ base_link sang hệ map.", fontsize=11.2, color=INK, linespacing=1.45)
    fig.text(0.77, 0.51, "F_map(x,y,ψ)", fontsize=16, weight="bold", color=PURPLE)
    fig.text(0.77, 0.45, "= [x,y]ᵀ + R(ψ)F_body", fontsize=13.2, color=PURPLE)
    fig.text(0.77, 0.27, "Kích thước thử nghiệm", fontsize=12.2, weight="bold", color=NAVY)
    fig.text(0.77, 0.21, "0,44 × 0,34 m", fontsize=18, weight="bold", color=NAVY)
    fig.text(0.77, 0.13, "Phần màu xanh là diện tích chiếm chỗ\nđược quét dọc chuyển động.", fontsize=10.3, color=MUTED)
    return save(fig, "robot_footprint_3d.png")


def bezier_quintic(points, samples=120):
    t = np.linspace(0.0, 1.0, samples)
    coeff = np.array([1, 5, 10, 10, 5, 1], dtype=float)
    curve = np.zeros((samples, 2))
    for i in range(6):
        b = coeff[i] * (1 - t) ** (5 - i) * t**i
        curve += b[:, None] * points[i]
    return curve


def render_swept_footprint():
    fig = plt.figure(figsize=(12, 7.6), facecolor="white")
    ax = fig.add_axes([0.00, 0.02, 0.77, 0.92], projection="3d")
    # A representative G2 corner transition; dimensions are explanatory.
    p = np.array([[-1.10, -0.78], [-0.82, -0.78], [-0.54, -0.78], [0.78, 0.54], [0.78, 0.82], [0.78, 1.10]])
    curve = bezier_quintic(p)
    ax.plot(curve[:, 0], curve[:, 1], np.full(len(curve), 0.025), color=TEAL, linewidth=4)
    # Extruded corridor obstacles leave a safe channel around the transition.
    add_box(ax, (-1.12, 0.35, 0.22), (0.48, 1.25, 0.44), NAVY, alpha=0.90)
    add_box(ax, (0.34, -1.12, 0.22), (1.25, 0.48, 0.44), NAVY, alpha=0.90)
    sample_ids = np.linspace(2, len(curve) - 3, 7).astype(int)
    for idx in sample_ids:
        pt = curve[idx]
        tangent = curve[min(idx + 1, len(curve) - 1)] - curve[max(idx - 1, 0)]
        yaw = np.arctan2(tangent[1], tangent[0])
        corners = np.array([[-0.22, -0.17], [0.22, -0.17], [0.22, 0.17], [-0.22, 0.17]])
        rot = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
        xy = corners @ rot.T + pt
        verts = np.column_stack([xy, np.full(4, 0.028)])
        ax.add_collection3d(Poly3DCollection([verts], facecolors=GREEN, edgecolors=GREEN, linewidth=1.2, alpha=0.20))
    style_axes(ax, ((-1.55, 1.35), (-1.55, 1.35), (0.0, 0.80)), elev=42, azim=-58)
    fig.text(0.055, 0.925, "VÙNG QUÉT HÌNH BAO DỌC ĐOẠN CHUYỂN TIẾP", fontsize=19.5, weight="bold", color=NAVY)
    fig.text(0.055, 0.885, "Kiểm tra liên tục dọc đường cong, không chỉ tại các điểm đầu–cuối", fontsize=11.5, color=MUTED)
    fig.text(0.78, 0.76, "Quy trình kiểm tra", fontsize=14, weight="bold", color=NAVY)
    steps = [
        ("1", "Nội suy vị trí và góc hướng"),
        ("2", "Biến đổi đa giác hình bao"),
        ("3", "Quét toàn bộ diện tích chiếm chỗ"),
        ("4", "Loại phương án nếu có giao cắt"),
    ]
    for i, (n, label) in enumerate(steps):
        y = 0.68 - i * 0.12
        fig.text(0.79, y, n, fontsize=10.5, weight="bold", color="white", ha="center", va="center", bbox=dict(boxstyle="circle,pad=0.3", fc=[BLUE, TEAL, PURPLE, ORANGE][i], ec="none"))
        fig.text(0.825, y, label, fontsize=10.8, color=INK, va="center")
    fig.text(0.78, 0.14, "Màu xanh lá: hợp các hình bao\nMàu xanh đậm: vật cản", fontsize=10.5, color=MUTED, linespacing=1.45)
    return save(fig, "swept_footprint_3d.png")


def render_wheel_layout():
    """Show both drive wheels through a transparent chassis and verify b."""
    fig = plt.figure(figsize=(12, 7.6), facecolor="white")
    ax = fig.add_axes([0.00, 0.02, 0.76, 0.92], projection="3d")
    parts = mesh_parts()
    add_mesh(ax, MeshPart(parts[0].name, parts[0].triangles, SILVER, 0.18), edge="#83909C", linewidth=0.04)
    add_mesh(ax, parts[1], edge="#0E1216", linewidth=0.08)
    add_mesh(ax, parts[2], edge="#0E1216", linewidth=0.08)
    add_cylinder_y(ax, (0.0, 0.0934, 0.0425), 0.0125, 0.068, BRASS, alpha=0.75)
    add_cylinder_y(ax, (0.0, -0.0934, 0.0425), 0.0125, 0.068, BRASS, alpha=0.75)
    # Physical rolling-tread centres from the wheel-link origins in SDF.
    y_left, y_right = 0.1274, -0.1274
    ax.plot([0, 0], [y_right, y_left], [0.0425, 0.0425], color=ORANGE, linewidth=3)
    ax.scatter([0, 0], [y_left, y_right], [0.0425, 0.0425], color=[BLUE, PURPLE], s=45, depthshade=False)
    style_axes(ax, ((-0.29, 0.29), (-0.27, 0.27), (-0.02, 0.24)), elev=48, azim=-25)
    fig.text(0.055, 0.925, "BỐ TRÍ HAI BÁNH CHỦ ĐỘNG", fontsize=21, weight="bold", color=NAVY)
    fig.text(0.055, 0.885, "Tâm vệt lăn lấy từ link bánh trong SDF", fontsize=11.5, color=MUTED)
    fig.text(0.77, 0.75, "Bánh trái", fontsize=13.5, weight="bold", color=BLUE)
    fig.text(0.77, 0.70, "y = +0,1274 m", fontsize=12, color=INK)
    fig.text(0.77, 0.59, "Bánh phải", fontsize=13.5, weight="bold", color=PURPLE)
    fig.text(0.77, 0.54, "y = −0,1274 m", fontsize=12, color=INK)
    fig.text(0.77, 0.39, "Vệt bánh vật lý", fontsize=12.5, weight="bold", color=NAVY)
    fig.text(0.77, 0.31, "b = 0,2548 m", fontsize=20, weight="bold", color=ORANGE)
    fig.text(0.77, 0.20, "Bán kính bánh: 0,0425 m\nBề rộng vệt lăn va chạm: 0,0300 m", fontsize=10.8, color=MUTED, linespacing=1.45)
    fig.text(0.77, 0.10, "Thân được làm trong để nhìn rõ cả hai bánh;\nđây không phải thay đổi hình học robot.", fontsize=9.8, color=MUTED, style="italic")
    return save(fig, "wheel_layout_3d.png")


def render_wheel_layout_clean():
    """Text-free wheel-layout render for native slide labels."""
    fig = plt.figure(figsize=(9.6, 6.4), facecolor="white")
    ax = fig.add_axes([0.01, 0.01, 0.98, 0.98], projection="3d")
    parts = mesh_parts()
    add_mesh(ax, MeshPart(parts[0].name, parts[0].triangles, SILVER, 0.18), edge="#83909C", linewidth=0.04)
    add_mesh(ax, parts[1], edge="#0E1216", linewidth=0.08)
    add_mesh(ax, parts[2], edge="#0E1216", linewidth=0.08)
    add_cylinder_y(ax, (0.0, 0.0934, 0.0425), 0.0125, 0.068, BRASS, alpha=0.75)
    add_cylinder_y(ax, (0.0, -0.0934, 0.0425), 0.0125, 0.068, BRASS, alpha=0.75)
    ax.plot([0, 0], [-0.1274, 0.1274], [0.0425, 0.0425], color=ORANGE, linewidth=3)
    ax.scatter([0, 0], [0.1274, -0.1274], [0.0425, 0.0425], color=[BLUE, PURPLE], s=45, depthshade=False)
    style_axes(ax, ((-0.29, 0.29), (-0.27, 0.27), (-0.02, 0.24)), elev=48, azim=-25)
    return save(fig, "wheel_layout_clean.png")


def render_warehouse_hero():
    """Deterministic Gazebo-like hero using the exact STL/SDF robot."""
    fig = plt.figure(figsize=(16, 9), facecolor="#F6F8FA")
    ax = fig.add_axes([0.0, 0.0, 1.0, 1.0], projection="3d")
    # Gazebo-like ground grid. Lines avoid the depth-sorting artefact produced
    # by one very large polygon in Matplotlib's 3D renderer.
    for x in np.linspace(-0.58, 0.62, 13):
        ax.plot([x, x], [-0.52, 0.52], [-0.015, -0.015], color="#D6DEE7", linewidth=0.7)
    for y in np.linspace(-0.52, 0.52, 11):
        ax.plot([-0.58, 0.62], [y, y], [-0.015, -0.015], color="#D6DEE7", linewidth=0.7)
    add_box(ax, (-0.02, 0.39, 0.13), (0.28, 0.22, 0.26), "#5F7F9C", alpha=1.0)
    add_box(ax, (0.40, 0.38, 0.12), (0.30, 0.22, 0.24), "#6B879F", alpha=1.0)
    add_box(ax, (-0.43, 0.34, 0.11), (0.21, 0.21, 0.22), "#D88A43", alpha=1.0)
    add_robot(ax, offset=(0.07, -0.22, 0.0))
    style_axes(ax, ((-0.60, 0.64), (-0.54, 0.54), (-0.03, 0.56)), elev=24, azim=-30)
    return save(fig, "robot_gazebo_warehouse_hero.png")


def render_swept_footprint_clean():
    """Text-free swept-footprint scene for a slide-native explanation."""
    fig = plt.figure(figsize=(9.6, 6.4), facecolor="white")
    ax = fig.add_axes([0.01, 0.01, 0.98, 0.98], projection="3d")
    p = np.array([[-1.10, -0.78], [-0.82, -0.78], [-0.54, -0.78], [0.78, 0.54], [0.78, 0.82], [0.78, 1.10]])
    curve = bezier_quintic(p)
    ax.plot(curve[:, 0], curve[:, 1], np.full(len(curve), 0.025), color=TEAL, linewidth=4)
    add_box(ax, (-1.12, 0.35, 0.22), (0.48, 1.25, 0.44), NAVY, alpha=0.90)
    add_box(ax, (0.34, -1.12, 0.22), (1.25, 0.48, 0.44), NAVY, alpha=0.90)
    for idx in np.linspace(2, len(curve) - 3, 7).astype(int):
        pt = curve[idx]
        tangent = curve[min(idx + 1, len(curve) - 1)] - curve[max(idx - 1, 0)]
        yaw = np.arctan2(tangent[1], tangent[0])
        corners = np.array([[-0.22, -0.17], [0.22, -0.17], [0.22, 0.17], [-0.22, 0.17]])
        rot = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
        xy = corners @ rot.T + pt
        verts = np.column_stack([xy, np.full(4, 0.028)])
        ax.add_collection3d(Poly3DCollection([verts], facecolors=GREEN, edgecolors=GREEN, linewidth=1.2, alpha=0.20))
    style_axes(ax, ((-1.55, 1.35), (-1.55, 1.35), (0.0, 0.80)), elev=42, azim=-58)
    return save(fig, "swept_footprint_clean.png")


def main():
    outputs = [
        render_robot_isometric(),
        render_robot_clean(),
        render_robot_exploded(),
        render_footprint(),
        render_wheel_layout(),
        render_wheel_layout_clean(),
        render_swept_footprint(),
        render_swept_footprint_clean(),
        render_warehouse_hero(),
    ]
    # Guard against the exact failure that prompted this revision. The wheel
    # meshes must remain on opposite sides of the robot and mirror each other.
    parts = mesh_parts()
    left_bounds = parts[1].triangles.reshape(-1, 3)[:, 1]
    right_bounds = parts[2].triangles.reshape(-1, 3)[:, 1]
    assert left_bounds.min() > 0 and right_bounds.max() < 0
    assert np.isclose(left_bounds.min(), -right_bounds.max(), atol=1e-6)
    assert np.isclose(left_bounds.max(), -right_bounds.min(), atol=1e-6)
    for path in outputs:
        print(path)


if __name__ == "__main__":
    main()
