"""
PyVista 3D robot-arm simulator
==============================

index.html은 건드리지 않고, 발표용으로 볼 수 있는 3D 기구학 시뮬레이터를
Python/PyVista로 구현한 버전입니다.

실행:
    pip install -r requirements.txt
    python simul.py

계산:
    - J1: 베이스 yaw
    - J2/J3: 4절 링크 폐루프를 풀어서 팔 위치 계산
    - J4/Tilt: 휴대폰 방향/기울기
    - 토크: 기구 자세 기반 정적 중력 토크 + 움직임 속도 기반 보정
      표시 단위는 kg.cm이며, 실제 센서값이 아닌 모델 기반 추정치입니다.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Dict, Iterable, List, Tuple

import numpy as np
import pyvista as pv


DEG = math.pi / 180.0
NM_TO_KG_CM = 10.19716213
TORQUE_GRAPH_MAX = 15.0


@dataclass
class Pose:
    j1: float = 0.0
    j2: float = 25.0
    j3: float = 260.0
    j4: float = 0.0
    tilt: float = 0.0


class RobotArm3DSim:
    def __init__(self) -> None:
        self.pose = Pose()
        self.face = {"x": -1.45, "z": 0.52, "h": 0.28, "yaw": 60.0, "roll": 0.0}
        self.show_face = True
        self.auto_track = False
        self.history: Dict[str, List[float]] = {k: [] for k in ("j1", "j2", "j3", "j4", "tilt")}
        self.prev_pose: Pose | None = None
        self.dynamic_actors: List[object] = []
        self.text_actors: List[object] = []

        # Dimensions are meters and follow the web simulator proportions.
        self.bb_y = 0.04
        self.bh = 0.36
        self.s_y = self.bb_y + self.bh - 0.04
        self.s_z = 0.10
        self.r_y = self.bb_y + self.bh - 0.04
        self.r_z = -0.10
        self.l_arm = 0.45
        self.l_j3_crank = 0.20
        self.l_j3_rod = 0.42
        self.l_fa = 0.50

        self.colors = {
            "acrylic": "#9aa6b2",
            "servo": "#2080c4",
            "pivot": "#4b5563",
            "phone": "#111827",
            "screen": "#ffffff",
            "skin": "#f0c7a8",
            "hair": "#2b1f1a",
            "j1": "#0284c7",
            "j2": "#16a34a",
            "j3": "#ea580c",
            "j4": "#7c3aed",
            "tilt": "#dc2626",
        }

        self.plotter = pv.Plotter(window_size=(1280, 860))
        self.plotter.set_background("#f6f8fb")
        self.plotter.add_axes(line_width=2)
        self._build_static_scene()
        self._build_widgets()
        self.update_scene()

    # ------------------------------------------------------------------
    # Kinematics
    # ------------------------------------------------------------------
    def mechanism_pose(self, j2: float, j3: float) -> Dict[str, float]:
        t2 = j2 * DEG
        t3 = j3 * DEG
        ey = self.s_y + self.l_arm * math.cos(t2)
        ez = self.s_z + self.l_arm * math.sin(t2)
        py = self.r_y + self.l_j3_crank * math.cos(t3)
        pz = self.r_z + self.l_j3_crank * math.sin(t3)

        coupler_y = -0.38
        coupler_radius = abs(coupler_y)
        dy = py - ey
        dz = pz - ez
        dist = max(1e-4, math.hypot(dy, dz))
        a = (coupler_radius**2 - self.l_j3_rod**2 + dist**2) / (2 * dist)
        h = math.sqrt(max(0.0, coupler_radius**2 - a**2))
        uy = dy / dist
        uz = dz / dist
        by = ey + a * uy
        bz = ez + a * uz
        c1y = by - h * uz
        c1z = bz + h * uy
        c2y = by + h * uz
        c2z = bz - h * uy
        cy, cz = (c1y, c1z) if c1z < c2z else (c2y, c2z)
        fa_rot = math.atan2((cz - ez) / coupler_y, (cy - ey) / coupler_y)
        forearm_angle = math.pi / 2 - fa_rot

        return {
            "t2": t2,
            "t3": t3,
            "ey": ey,
            "ez": ez,
            "py": py,
            "pz": pz,
            "cy": cy,
            "cz": cz,
            "fa_rot": fa_rot,
            "forearm_angle": forearm_angle,
        }

    def local_to_world(self, radial: float, height: float, j1: float | None = None) -> np.ndarray:
        theta = (self.pose.j1 if j1 is None else j1) * DEG
        return np.array([radial * math.cos(theta), radial * math.sin(theta), height], dtype=float)

    def world_points(self) -> Dict[str, np.ndarray]:
        m = self.mechanism_pose(self.pose.j2, self.pose.j3)
        shoulder = self.local_to_world(self.s_y, self.s_z)
        elbow = self.local_to_world(m["ey"], m["ez"])
        crank = self.local_to_world(m["py"], m["pz"])
        coupler = self.local_to_world(m["cy"], m["cz"])
        wrist_r = m["ey"] + self.l_fa * math.sin(m["forearm_angle"])
        wrist_h = m["ez"] + self.l_fa * math.cos(m["forearm_angle"])
        wrist = self.local_to_world(wrist_r, wrist_h)
        return {
            "base": np.array([0.0, 0.0, 0.0]),
            "turntable": self.local_to_world(0.0, self.s_z),
            "shoulder": shoulder,
            "elbow": elbow,
            "crank": crank,
            "coupler": coupler,
            "wrist": wrist,
            "m": m,
        }

    def potential_energy(self, j2: float, j3: float) -> float:
        m = self.mechanism_pose(j2, j3)
        gravity = 9.80665
        mass = {
            "upper": 0.13,
            "rod": 0.07,
            "fore": 0.18,
            "phone": 0.32,
        }
        upper_z = self.s_z + (self.l_arm * 0.48) * math.sin(m["t2"])
        rod_z = (m["pz"] + m["cz"]) / 2
        fore_z = m["ez"] + (self.l_fa * 0.48) * math.cos(m["forearm_angle"])
        phone_z = m["ez"] + (self.l_fa + 0.12) * math.cos(m["forearm_angle"])
        return gravity * (
            mass["upper"] * upper_z
            + mass["rod"] * rod_z
            + mass["fore"] * fore_z
            + mass["phone"] * phone_z
        )

    def estimate_torques(self) -> Dict[str, float]:
        eps = 0.25
        d_rad = (eps * 2) * DEG
        j2_static = abs((self.potential_energy(self.pose.j2 + eps, self.pose.j3) -
                         self.potential_energy(self.pose.j2 - eps, self.pose.j3)) / d_rad)
        j3_static = abs((self.potential_energy(self.pose.j2, self.pose.j3 + eps) -
                         self.potential_energy(self.pose.j2, self.pose.j3 - eps)) / d_rad)

        prev = self.prev_pose or self.pose

        def speed_deg(key: str) -> float:
            now = getattr(self.pose, key)
            old = getattr(prev, key)
            if key == "j1":
                diff = ((now - old + 540) % 360) - 180
            else:
                diff = now - old
            return abs(diff) * DEG / 0.033

        torques_nm = {
            "j1": 0.025 + speed_deg("j1") * 0.018,
            "j2": j2_static + speed_deg("j2") * 0.025,
            "j3": j3_static + speed_deg("j3") * 0.022,
            "j4": abs(math.sin(self.pose.j4 * DEG)) * 0.05 + speed_deg("j4") * 0.016 + 0.018,
            "tilt": abs(math.sin(self.pose.tilt * DEG)) * 0.13 + speed_deg("tilt") * 0.018 + 0.024,
        }
        self.prev_pose = Pose(**self.pose.__dict__)
        return {k: v * NM_TO_KG_CM for k, v in torques_nm.items()}

    # ------------------------------------------------------------------
    # Mesh helpers
    # ------------------------------------------------------------------
    def _track(self, actor: object) -> object:
        self.dynamic_actors.append(actor)
        return actor

    def add_cylinder_between(self, p0: Iterable[float], p1: Iterable[float], radius: float,
                             color: str, name: str | None = None) -> None:
        a = np.asarray(p0, dtype=float)
        b = np.asarray(p1, dtype=float)
        direction = b - a
        length = float(np.linalg.norm(direction))
        if length < 1e-6:
            return
        mesh = pv.Cylinder(center=(a + b) / 2, direction=direction, radius=radius,
                           height=length, resolution=24)
        self._track(self.plotter.add_mesh(mesh, color=color, smooth_shading=True, name=name))

    def add_sphere(self, point: Iterable[float], radius: float, color: str) -> None:
        mesh = pv.Sphere(radius=radius, center=point, theta_resolution=32, phi_resolution=16)
        self._track(self.plotter.add_mesh(mesh, color=color, smooth_shading=True))

    def add_box(self, center: Iterable[float], lengths: Tuple[float, float, float], color: str,
                axes: Tuple[np.ndarray, np.ndarray, np.ndarray] | None = None,
                opacity: float = 1.0) -> None:
        cube = pv.Cube(center=(0, 0, 0), x_length=1, y_length=1, z_length=1)
        if axes is None:
            axes = (np.array([1.0, 0.0, 0.0]), np.array([0.0, 1.0, 0.0]), np.array([0.0, 0.0, 1.0]))
        mat = np.eye(4)
        for i, axis in enumerate(axes):
            axis = np.asarray(axis, dtype=float)
            axis = axis / max(1e-8, np.linalg.norm(axis))
            mat[:3, i] = axis * lengths[i]
        mat[:3, 3] = np.asarray(center, dtype=float)
        cube.transform(mat, inplace=True)
        self._track(self.plotter.add_mesh(cube, color=color, opacity=opacity))

    # ------------------------------------------------------------------
    # Scene
    # ------------------------------------------------------------------
    def _build_static_scene(self) -> None:
        floor = pv.Plane(center=(0, 0, -0.02), direction=(0, 0, 1), i_size=5.0, j_size=5.0)
        self.plotter.add_mesh(floor, color="#eef2f7")
        for x in np.linspace(-2.5, 2.5, 11):
            self.plotter.add_mesh(pv.Line((x, -2.5, -0.018), (x, 2.5, -0.018)), color="#d6dce5", line_width=1)
            self.plotter.add_mesh(pv.Line((-2.5, x, -0.018), (2.5, x, -0.018)), color="#d6dce5", line_width=1)

        self.plotter.add_text("4-DOF Face Tracking Rig", position="upper_left",
                              font_size=13, color="#26313f")
        self.plotter.add_text("Model-based torque estimate, kg.cm", position="lower_left",
                              font_size=9, color="#667486")

        self.plotter.camera_position = [(2.0, -2.8, 1.45), (0.15, 0.0, 0.45), (0, 0, 1)]

    def _build_widgets(self) -> None:
        slider_specs = [
            ("j1", "J1 base", 0, 360, 0.0),
            ("j2", "J2 lift", -30, 80, 25.0),
            ("j3", "J3 crank", 220, 350, 260.0),
            ("j4", "J4 phone yaw", -90, 90, 0.0),
            ("tilt", "Phone tilt", -80, 80, 0.0),
        ]
        for i, (key, title, lo, hi, val) in enumerate(slider_specs):
            y = 0.92 - i * 0.065
            self.plotter.add_slider_widget(
                callback=lambda value, k=key: self._set_pose(k, value),
                rng=(lo, hi),
                value=val,
                title=title,
                pointa=(0.68, y),
                pointb=(0.96, y),
                style="modern",
            )

    def _set_pose(self, key: str, value: float) -> None:
        setattr(self.pose, key, float(value))
        self.update_scene()

    def remove_dynamic(self) -> None:
        for actor in self.dynamic_actors + self.text_actors:
            try:
                self.plotter.remove_actor(actor)
            except Exception:
                pass
        self.dynamic_actors.clear()
        self.text_actors.clear()

    def update_scene(self) -> None:
        self.remove_dynamic()
        points = self.world_points()
        torques = self.estimate_torques()
        for key, value in torques.items():
            self.history[key].append(value)
            self.history[key] = self.history[key][-90:]

        self._draw_robot(points)
        if self.show_face:
            self._draw_person()
        self._draw_phone(points)
        self._draw_torque_panel(torques)
        self._draw_status_text(points, torques)
        self.plotter.render()

    def _draw_robot(self, p: Dict[str, np.ndarray]) -> None:
        c = self.colors
        base_center = np.array([0.0, 0.0, 0.02])
        self.add_box(base_center, (1.0, 0.75, 0.04), c["acrylic"], opacity=0.85)
        self.add_cylinder_between((0, 0, 0.02), (0, 0, self.s_z), 0.08, c["servo"])
        self.add_cylinder_between((-0.26, 0, self.s_z), (0.26, 0, self.s_z), 0.018, c["pivot"])

        shoulder = p["shoulder"]
        elbow = p["elbow"]
        crank = p["crank"]
        coupler = p["coupler"]
        wrist = p["wrist"]
        theta = self.pose.j1 * DEG
        side = np.array([-math.sin(theta), math.cos(theta), 0.0])
        offset = side * 0.045

        self.add_cylinder_between(shoulder + offset, elbow + offset, 0.018, c["acrylic"])
        self.add_cylinder_between(shoulder - offset, elbow - offset, 0.018, c["acrylic"])
        self.add_cylinder_between(elbow + offset, wrist + offset, 0.016, c["acrylic"])
        self.add_cylinder_between(elbow - offset, wrist - offset, 0.016, c["acrylic"])
        self.add_cylinder_between(crank, coupler, 0.012, c["acrylic"])
        self.add_cylinder_between(coupler, wrist, 0.018, c["acrylic"])

        for point in (shoulder, elbow, crank, coupler, wrist):
            self.add_sphere(point, 0.028, c["pivot"])

        self.add_box(shoulder + side * 0.18, (0.12, 0.10, 0.10), c["servo"])
        self.add_box(crank - side * 0.18, (0.12, 0.10, 0.10), c["servo"])
        self.add_box(wrist, (0.12, 0.10, 0.09), c["servo"])

    def _phone_axes(self, wrist: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        theta = (self.pose.j1 + self.pose.j4) * DEG
        normal = np.array([math.cos(theta), math.sin(theta), 0.0])
        right = np.array([-math.sin(theta), math.cos(theta), 0.0])
        up = np.array([0.0, 0.0, 1.0])
        tilt = self.pose.tilt * DEG
        tilted_normal = normal * math.cos(tilt) + up * math.sin(tilt)
        tilted_up = -normal * math.sin(tilt) + up * math.cos(tilt)
        center = wrist + tilted_up * 0.02 + tilted_normal * 0.085
        return center, right, tilted_normal, tilted_up

    def _draw_phone(self, p: Dict[str, np.ndarray]) -> None:
        center, right, normal, up = self._phone_axes(p["wrist"])
        self.add_box(center, (0.22, 0.018, 0.34), self.colors["phone"], axes=(right, normal, up))
        screen_center = center + normal * 0.011
        self.add_box(screen_center, (0.19, 0.004, 0.29), self.colors["screen"], axes=(right, normal, up))

        # White screen plus dark blocks that read as a presentation title from camera distance.
        # Korean text rendering in VTK depends on local fonts, so this is intentionally geometric.
        self.add_box(screen_center + up * 0.045, (0.115, 0.006, 0.032), "#111827", axes=(right, normal, up))
        self.add_box(screen_center - up * 0.030, (0.080, 0.006, 0.035), "#111827", axes=(right, normal, up))
        self.add_box(screen_center - up * 0.080, (0.080, 0.006, 0.035), "#111827", axes=(right, normal, up))
        self.text_actors.append(
            self.plotter.add_point_labels(
                [screen_center + up * 0.17],
                ["종설\n4조"],
                text_color="#111827",
                font_size=18,
                point_size=0,
                shape_opacity=0.0,
                always_visible=True,
            )
        )

    def _draw_person(self) -> None:
        yaw = self.face["yaw"] * DEG
        pos = np.array([self.face["x"], self.face["z"], self.face["h"]])
        self.add_sphere(pos + np.array([0, 0, 0.18]), 0.13, self.colors["skin"])
        self.add_sphere(pos + np.array([0, 0, 0.28]), 0.12, self.colors["hair"])
        self.add_cylinder_between(pos + np.array([0, 0, 0.02]), pos + np.array([0, 0, 0.12]), 0.04, self.colors["skin"])
        shoulder_axis = np.array([math.cos(yaw), math.sin(yaw), 0.0])
        self.add_cylinder_between(pos - shoulder_axis * 0.18, pos + shoulder_axis * 0.18, 0.035, self.colors["acrylic"])
        normal = np.array([math.sin(yaw), -math.cos(yaw), 0.0])
        face_center = pos + np.array([0, 0, 0.18]) + normal * 0.12
        self.add_box(face_center, (0.28, 0.004, 0.24), "#22c55e",
                     axes=(shoulder_axis, normal, np.array([0, 0, 1])), opacity=0.18)

    def _draw_torque_panel(self, torques: Dict[str, float]) -> None:
        origin = np.array([1.15, -0.95, 0.05])
        bar_w = 0.07
        gap = 0.06
        for i, spec in enumerate(("j1", "j2", "j3", "j4", "tilt")):
            value = torques[spec]
            height = 0.42 * min(value / TORQUE_GRAPH_MAX, 1.0)
            x = origin[0] + i * (bar_w + gap)
            color = self.colors[spec]
            self.add_box((x, origin[1], origin[2] + 0.21), (bar_w, 0.035, 0.42), "#eef2f7", opacity=0.75)
            self.add_box((x, origin[1] - 0.002, origin[2] + height / 2), (bar_w, 0.045, max(height, 0.01)), color)
        label = "Estimated torque (kg.cm)\n" + "  ".join(f"{k.upper()} {v:4.1f}" for k, v in torques.items())
        self.text_actors.append(self.plotter.add_text(label, position=(18, 80), font_size=9, color="#26313f"))

    def _draw_status_text(self, points: Dict[str, np.ndarray], torques: Dict[str, float]) -> None:
        wrist = points["wrist"]
        status = (
            f"J1 {self.pose.j1:5.1f}  J2 {self.pose.j2:5.1f}  "
            f"J3 {self.pose.j3:5.1f}  J4 {self.pose.j4:5.1f}  Tilt {self.pose.tilt:5.1f}\n"
            f"Wrist xyz = ({wrist[0]:.2f}, {wrist[1]:.2f}, {wrist[2]:.2f}) m\n"
            f"Max torque = {max(torques.values()):.1f} kg.cm"
        )
        self.text_actors.append(self.plotter.add_text(status, position=(18, 720), font_size=9, color="#26313f"))

    def show(self) -> None:
        self.plotter.show(title="4-DOF PyVista Robot Arm Simulator")


if __name__ == "__main__":
    RobotArm3DSim().show()
