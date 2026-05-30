"""
4-DOF 아크릴 로봇암 운동학 시뮬레이터
============================================
GrabCAD "Acrylic Robot - 4 DOF" 구조를 운동학적으로 재현한 시뮬레이터.
부품 30여 개 중 실제로 움직임을 만드는 조인트 4개(+그리퍼)만 모델링.

조인트 구성:
    J1 - 베이스 회전 (수직축, 평면 밖 회전 -> 턴테이블로 표시)
    J2 - 어깨
    J3 - 팔꿈치
    J4 - 손목
    + 그리퍼 개폐

3가지 모드:
    [FK]   슬라이더로 관절 직접 제어
    [IK]   캔버스 클릭 -> 그 지점으로 손끝 이동 (역기구학)
    [TRAJ] 집기->옮기기->놓기 궤적 자동 재생

실행:
    pip install matplotlib numpy
    python robot_arm_sim.py

링크 길이(LINKS)는 비례만 맞춘 근사치입니다. 실측값으로 바꾸면
실제 팔과 동일한 작업영역이 됩니다.
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button, RadioButtons

# ----------------------------------------------------------------------
# 로봇 파라미터 (단위: mm) — 실측값으로 교체하세요
# ----------------------------------------------------------------------
LINKS = {
    "base":  70.0,   # 베이스 기둥 높이 (어깨 피벗까지)
    "upper": 150.0,  # 어퍼암 (어깨 -> 팔꿈치)
    "fore":  160.0,  # 포어암 (팔꿈치 -> 손목)
    "wrist": 55.0,   # 손목 -> 손끝
}

# 관절 가동 범위 (degree)
LIMITS = {
    "j1": (-90, 90),
    "j2": (-10, 120),
    "j3": (-130, 20),
    "j4": (-90, 90),
}


# ----------------------------------------------------------------------
# 정기구학 (Forward Kinematics)
# 수직 평면상의 어깨/팔꿈치/손목 위치를 각도로부터 계산
# ----------------------------------------------------------------------
def forward_kinematics(j2, j3, j4):
    """관절 각도(deg) -> 각 조인트의 평면 좌표 [shoulder, elbow, wrist, tip]."""
    a2 = np.radians(j2)
    a3 = a2 + np.radians(j3)   # 팔꿈치 각도는 어깨에 누적
    a4 = a3 + np.radians(j4)   # 손목도 누적

    shoulder = np.array([0.0, LINKS["base"]])
    elbow = shoulder + LINKS["upper"] * np.array([np.cos(a2), np.sin(a2)])
    wrist = elbow + LINKS["fore"] * np.array([np.cos(a3), np.sin(a3)])
    tip = wrist + LINKS["wrist"] * np.array([np.cos(a4), np.sin(a4)])
    return shoulder, elbow, wrist, tip


# ----------------------------------------------------------------------
# 역기구학 (Inverse Kinematics)
# 목표점(tx, ty)에 손끝을 보내기 위한 어깨/팔꿈치 각도를 코사인법칙으로 역산
# ----------------------------------------------------------------------
def inverse_kinematics(tx, ty):
    """목표 평면좌표 -> (j2, j3, j4, 도달가능여부)."""
    sy = LINKS["base"]              # 어깨 높이
    dx, dy = tx, ty - sy
    r = np.hypot(dx, dy)

    l1 = LINKS["upper"]
    l2 = LINKS["fore"] + LINKS["wrist"] * 0.5   # 포어암+손목 절반을 유효길이로

    # 코사인법칙으로 팔꿈치 끼인각
    cos_e = (r**2 - l1**2 - l2**2) / (2 * l1 * l2)
    reachable = -1.0 <= cos_e <= 1.0 and r <= (l1 + l2 - 1)
    cos_e = np.clip(cos_e, -1.0, 1.0)
    e = np.arccos(cos_e)

    base_ang = np.arctan2(dy, dx)
    adj = np.arctan2(l2 * np.sin(e), l1 + l2 * np.cos(e))

    j2 = np.degrees(base_ang + adj)   # 어깨
    j3 = np.degrees(-e)               # 팔꿈치 (음수 = 아래로 굽힘)
    j4 = 10.0                         # 손목은 적당히 고정
    return j2, j3, j4, reachable


# ----------------------------------------------------------------------
# 궤적: 집기 -> 옮기기 -> 놓기 (j1,j2,j3,j4,grip)
# ----------------------------------------------------------------------
WAYPOINTS = [
    dict(j1=0,   j2=60, j3=-70, j4=10, grip=80, label="Home pose"),
    dict(j1=-40, j2=30, j3=-40, j4=20, grip=80, label="Approach above object"),
    dict(j1=-40, j2=18, j3=-25, j4=15, grip=80, label="Descend"),
    dict(j1=-40, j2=18, j3=-25, j4=15, grip=15, label="Grip (close)"),
    dict(j1=-40, j2=45, j3=-55, j4=15, grip=15, label="Lift"),
    dict(j1=45,  j2=45, j3=-55, j4=15, grip=15, label="Rotate to other side"),
    dict(j1=45,  j2=20, j3=-30, j4=15, grip=15, label="Descend"),
    dict(j1=45,  j2=20, j3=-30, j4=15, grip=85, label="Release (open)"),
    dict(j1=0,   j2=60, j3=-70, j4=10, grip=80, label="Return home"),
]


# ----------------------------------------------------------------------
# 시뮬레이터 UI
# ----------------------------------------------------------------------
class RobotArmSim:
    def __init__(self):
        self.q = dict(j1=0.0, j2=60.0, j3=-70.0, j4=10.0)
        self.grip = 60.0
        self.mode = "FK"
        self.target = [180.0, 120.0]
        self._anim_running = False

        self.fig = plt.figure(figsize=(11, 7))
        self.fig.canvas.manager.set_window_title("4-DOF Robot Arm Simulator")

        # 메인 작업영역
        self.ax = self.fig.add_axes([0.05, 0.30, 0.62, 0.65])
        self.ax.set_aspect("equal")
        self.ax.set_xlim(-260, 320)
        self.ax.set_ylim(-60, 380)
        self.ax.set_xlabel("X (mm)")
        self.ax.set_ylabel("Y (mm)")
        self.ax.grid(True, alpha=0.3)
        self.ax.set_title("Joint control (FK)")

        # 그림 요소
        (self.base_line,)  = self.ax.plot([], [], lw=10, color="#888780",
                                           solid_capstyle="round")
        (self.upper_line,) = self.ax.plot([], [], lw=8,  color="#378ADD",
                                           solid_capstyle="round")
        (self.fore_line,)  = self.ax.plot([], [], lw=8,  color="#1D9E75",
                                           solid_capstyle="round")
        (self.wrist_line,) = self.ax.plot([], [], lw=6,  color="#D85A30",
                                           solid_capstyle="round")
        (self.joints,)     = self.ax.plot([], [], "o", ms=9, color="#26215C",
                                           mec="white", mew=1.5, zorder=5)
        (self.grip_lines,) = self.ax.plot([], [], lw=3, color="black")
        (self.target_mark,)= self.ax.plot([], [], "+", ms=18, mew=2,
                                           color="#E24B4A")
        self.ground = self.ax.axhline(0, color="#B4B2A9", lw=1)
        self.info_text = self.ax.text(0.02, 0.97, "", transform=self.ax.transAxes,
                                      va="top", fontsize=10, family="monospace")

        self._build_controls()
        self.update()
        self.fig.canvas.mpl_connect("button_press_event", self.on_click)

    # -- 컨트롤 위젯 --------------------------------------------------
    def _build_controls(self):
        ax_radio = self.fig.add_axes([0.72, 0.72, 0.24, 0.22])
        ax_radio.set_title("Mode", fontsize=10)
        self.radio = RadioButtons(ax_radio, ("FK joints", "IK target", "TRAJ play"))
        self.radio.on_clicked(self._on_mode)

        # FK 슬라이더
        self.sliders = {}
        names = [("j1", "J1 base"), ("j2", "J2 shoulder"),
                 ("j3", "J3 elbow"), ("j4", "J4 wrist")]
        for i, (key, label) in enumerate(names):
            ax_s = self.fig.add_axes([0.10, 0.22 - i * 0.05, 0.55, 0.03])
            lo, hi = LIMITS[key]
            s = Slider(ax_s, label, lo, hi, valinit=self.q[key], valstep=1)
            s.on_changed(self._on_slider)
            self.sliders[key] = s

        ax_grip = self.fig.add_axes([0.10, 0.02, 0.55, 0.03])
        self.grip_slider = Slider(ax_grip, "Gripper %", 0, 100, valinit=self.grip, valstep=1)
        self.grip_slider.on_changed(self._on_slider)

        # IK 목표 슬라이더 (FK와 같은 자리, 모드에 따라 숨김 토글은 생략—항상 보이게 둠)
        ax_tx = self.fig.add_axes([0.72, 0.55, 0.24, 0.03])
        self.tx_slider = Slider(ax_tx, "Target X", -200, 260, valinit=self.target[0], valstep=1)
        ax_ty = self.fig.add_axes([0.72, 0.49, 0.24, 0.03])
        self.ty_slider = Slider(ax_ty, "Target Y", -40, 320, valinit=self.target[1], valstep=1)
        self.tx_slider.on_changed(self._on_target)
        self.ty_slider.on_changed(self._on_target)

        ax_play = self.fig.add_axes([0.72, 0.40, 0.24, 0.05])
        self.play_btn = Button(ax_play, "Play trajectory")
        self.play_btn.on_clicked(self._on_play)

    # -- 콜백 ---------------------------------------------------------
    def _on_mode(self, label):
        self.mode = label.split()[0]
        titles = {"FK": "Joint control (FK)",
                  "IK": "Reach target (IK) - click canvas",
                  "TRAJ": "Trajectory playback"}
        self.ax.set_title(titles[self.mode])
        if self.mode == "IK":
            self._solve_ik()
        self.update()

    def _on_slider(self, _):
        if self.mode != "FK":
            return
        for k, s in self.sliders.items():
            self.q[k] = s.val
        self.grip = self.grip_slider.val
        self.update()

    def _on_target(self, _):
        self.target = [self.tx_slider.val, self.ty_slider.val]
        if self.mode == "IK":
            self._solve_ik()
            self.update()

    def _solve_ik(self):
        j2, j3, j4, ok = inverse_kinematics(*self.target)
        self.q.update(j2=j2, j3=j3, j4=j4)
        self._ik_ok = ok

    def on_click(self, event):
        if self.mode != "IK" or event.inaxes != self.ax:
            return
        self.target = [event.xdata, event.ydata]
        self.tx_slider.set_val(np.clip(event.xdata, -200, 260))
        self.ty_slider.set_val(np.clip(event.ydata, -40, 320))

    def _on_play(self, _):
        if self._anim_running:
            return
        self.mode = "TRAJ"
        self.ax.set_title("Trajectory playback")
        self._anim_running = True
        self._run_trajectory()
        self._anim_running = False

    def _run_trajectory(self):
        for i in range(len(WAYPOINTS) - 1):
            A, B = WAYPOINTS[i], WAYPOINTS[i + 1]
            for s in np.linspace(0, 1, 25):
                for k in ("j1", "j2", "j3", "j4"):
                    self.q[k] = A[k] + (B[k] - A[k]) * s
                self.grip = A["grip"] + (B["grip"] - A["grip"]) * s
                self.update(status=f"{i+1}/{len(WAYPOINTS)-1} · {B['label']}")
                plt.pause(0.012)

    # -- 렌더 ---------------------------------------------------------
    def update(self, status=None):
        sh, el, wr, tip = forward_kinematics(self.q["j2"], self.q["j3"], self.q["j4"])
        base0 = np.array([0.0, 0.0])

        self.base_line.set_data([base0[0], sh[0]], [base0[1], sh[1]])
        self.upper_line.set_data([sh[0], el[0]], [sh[1], el[1]])
        self.fore_line.set_data([el[0], wr[0]], [el[1], wr[1]])
        self.wrist_line.set_data([wr[0], tip[0]], [wr[1], tip[1]])
        self.joints.set_data([sh[0], el[0], wr[0]], [sh[1], el[1], wr[1]])

        # 그리퍼 턱
        a4 = np.radians(self.q["j2"] + self.q["j3"] + self.q["j4"])
        opening = 6 + self.grip / 100 * 14
        nx, ny = np.cos(a4 + np.pi / 2), np.sin(a4 + np.pi / 2)
        gx, gy = [], []
        for sgn in (1, -1):
            jb = tip + np.array([nx, ny]) * opening * sgn
            jt = jb + np.array([np.cos(a4), np.sin(a4)]) * 16
            gx += [tip[0], jb[0], jt[0], np.nan]
            gy += [tip[1], jb[1], jt[1], np.nan]
        self.grip_lines.set_data(gx, gy)

        if self.mode == "IK":
            self.target_mark.set_data([self.target[0]], [self.target[1]])
        else:
            self.target_mark.set_data([], [])

        reach = np.hypot(tip[0], tip[1])
        lines = [f"Tip X   = {tip[0]:7.1f} mm",
                 f"Tip Y   = {tip[1]:7.1f} mm",
                 f"Reach   = {reach:6.1f} mm",
                 f"J1={self.q['j1']:.0f}  J2={self.q['j2']:.0f}  "
                 f"J3={self.q['j3']:.0f}  J4={self.q['j4']:.0f}"]
        if self.mode == "IK":
            ok = getattr(self, "_ik_ok", True)
            lines.append("Reachable" if ok else "Out of workspace")
        if status:
            lines.append(status)
        self.info_text.set_text("\n".join(lines))

        self.fig.canvas.draw_idle()


if __name__ == "__main__":
    sim = RobotArmSim()
    plt.show()