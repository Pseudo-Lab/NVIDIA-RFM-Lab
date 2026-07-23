# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# TeleopStatusUI — scene-direct teleop UI (2026-07-24 분리형 대시보드).
#   "OMX Joints"    : 좌측 도킹 — 타이틀·연결/FPS·관절 표
#   "OMX Recording" : 뷰포트 하단 도킹 — 녹화 상태·operator·키 안내·Replay 컨트롤
#   뷰포트 우측 하단 HUD: REC 상태만(캡처 PNG 에는 안 찍힘)
#   저장 다이얼로그: S 종료 시 이름/메모 입력 → commit
#   배경 이미지: 각 패널은 ZStack 구조라 ui.Image 를 뒤에 깔면 됨(로고 파일 지정 시).
#   text field 입력 중에는 editing=True 로 S/C/R/Z 키보드 컨트롤을 차단한다.
import math

import omni.ui as ui

_HUD_BG = 0xAA111111
_TITLE = 0xFFFFB050

import os as _os
_FONT_DIR = _os.path.abspath(_os.path.join(_os.path.dirname(__file__), "..", "assets", "fonts"))
_FONT = _os.path.join(_FONT_DIR, "Pretendard-Regular.otf")
_FONT_BOLD = _os.path.join(_FONT_DIR, "Pretendard-Bold.otf")


def _st(color=None, size=None, bold=False):
    """공통 스타일: Pretendard(한글 포함) 폰트 + 색/크기. 폰트 없으면 기본 폰트."""
    st = {}
    f = _FONT_BOLD if bold else _FONT
    if _os.path.exists(f):
        st["font"] = f
    if color is not None:
        st["color"] = color
    if size is not None:
        st["font_size"] = size
    return st


class TeleopStatusUI:

    ROW_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "gripper_1"]

    def __init__(self, recorder, replay, update_every: int = 5, on_screenshot=None):
        self.recorder = recorder
        self.replay = replay
        self.on_screenshot = on_screenshot
        self.update_every = max(1, update_every)
        self._tick = 0
        self._rows = []
        self._episodes = []
        self._editing_fields = set()
        self._font_size = 17  # 패널 폭 적응형 폰트 버킷

        self._build_joints_panel()
        self._build_rec_panel()
        self._build_save_dialog()
        self.recorder.on_pending = self._open_save_dialog
        self.refresh_episodes()
        self._dock_panels()

    # --- "OMX Joints" (좌측) ---
    def _build_joints_panel(self):
        import os
        self.joints_window = ui.Window("OMX Joints", width=420, height=520)
        bg = __import__("os").path.expanduser("~/Desktop/dashboard_bg.png")
        with self.joints_window.frame:
            with ui.ZStack():
                if __import__("os").path.exists(bg):
                    ui.Image(bg, fill_policy=ui.FillPolicy.PRESERVE_ASPECT_CROP)
                    ui.Rectangle(style={"background_color": 0xAA000000})  # 가독성 딤 레이어
                with ui.VStack(spacing=6, style={"margin_width": 4, "margin_height": 8}):
                    # 로고: ~/Desktop/pseudolab_logo.png 가 있으면 자동 표시 (파일만 놓으면 됨)
                    logo = os.path.expanduser("~/Desktop/pseudolab_logo.png")
                    if os.path.exists(logo):
                        ui.Image(logo, height=64, fill_policy=ui.FillPolicy.PRESERVE_ASPECT_FIT)
                    self.title_label = ui.Label("PseudoLab · OMX Sim2Real", height=30,
                                                 style={"color": _TITLE, "font_size": 22})
                    self.conn_label = ui.Label("-", height=22,
                                               style=_st(0xFFDDDDDD, 17))
                    ui.Spacer(height=4)
                    self._headers = []
                    with ui.HStack(height=24):
                        for i, text in enumerate(("joint", "n", "target", "actual")):
                            h = ui.Label(text, width=ui.Fraction(1),
                                         alignment=(ui.Alignment.LEFT_CENTER if i == 0
                                                    else ui.Alignment.RIGHT_CENTER),
                                         style=_st(0xFF33CCFF, 20, bold=True))
                            self._headers.append(h)
                    self._name_labels = []
                    for name in self.ROW_NAMES:
                        with ui.HStack(height=28):
                            nm = ui.Label(name, width=ui.Fraction(1),
                                          alignment=ui.Alignment.LEFT_CENTER,
                                          style=_st(0xFFCCCCCC, 17))
                            n = ui.Label("-", width=ui.Fraction(1),
                                         alignment=ui.Alignment.RIGHT_CENTER,
                                         style=_st(size=17))
                            t = ui.Label("-", width=ui.Fraction(1),
                                         alignment=ui.Alignment.RIGHT_CENTER,
                                         style=_st(size=17))
                            a = ui.Label("-", width=ui.Fraction(1),
                                         alignment=ui.Alignment.RIGHT_CENTER,
                                         style=_st(size=17))
                            self._name_labels.append(nm)
                            self._rows.append((n, t, a))
                    ui.Spacer()

    # --- "OMX Recording" (뷰포트 하단) ---
    def _build_rec_panel(self):
        self.rec_window = ui.Window("OMX Recording", width=700, height=200)
        bg = __import__("os").path.expanduser("~/Desktop/dashboard_bg.png")
        with self.rec_window.frame:
            with ui.ZStack():
                if __import__("os").path.exists(bg):
                    ui.Image(bg, fill_policy=ui.FillPolicy.PRESERVE_ASPECT_CROP)
                    ui.Rectangle(style={"background_color": 0xAA000000})
                with ui.HStack(spacing=20, style={"margin_width": 10, "margin_height": 2}):
                    with ui.VStack(spacing=2, width=340):
                        self.rec_label = ui.Label("idle", height=24,
                                                  style=_st(0xFF55CC55, 22, bold=True))
                        with ui.HStack(height=24):
                            ui.Label("operator", width=90, style=_st(size=17))
                            self.operator_field = ui.StringField(height=24)
                            self.operator_field.model.set_value("pnltoen")
                        ui.Label("S rec/save  C cancel  R reset  Z zero  P shot  L layout", height=18,
                                 style=_st(0xFF3399FF, 16))
                    with ui.VStack(spacing=2):
                        ui.Label("Replay", height=18, style=_st(0xFF888888, 16))
                        self._replay_box = ui.VStack(height=26)
                        with ui.HStack(height=26, spacing=6):
                            ui.Button("Refresh", clicked_fn=self.refresh_episodes)
                            ui.Button("Play", clicked_fn=self._on_play)
                            ui.Button("Stop", clicked_fn=self.replay.stop)
                            if self.on_screenshot is not None:
                                ui.Button("Screenshot", clicked_fn=self.on_screenshot)
        self._guard_field(self.operator_field, "operator")

        try:
            import omni.kit.ui
            menu = omni.kit.ui.get_editor_menu()
            self._menus = [
                menu.add_item("PseudoLab/OMX Joints",
                              lambda m, v: setattr(self.joints_window, "visible", bool(v)),
                              toggle=True, value=True),
                menu.add_item("PseudoLab/OMX Recording",
                              lambda m, v: setattr(self.rec_window, "visible", bool(v)),
                              toggle=True, value=True),
            ]
        except Exception as e:
            print(f"[UI] PseudoLab 메뉴 추가 실패: {e}", flush=True)

    def _dock_panels(self):
        # Joints = 뷰포트 왼쪽 분할, Recording = 뷰포트 아래 분할 (Unity 에디터식 배치)
        try:
            vp = ui.Workspace.get_window("Viewport")
            if vp is not None:
                self.joints_window.dock_in(vp, ui.DockPosition.LEFT, 0.20)
                self.rec_window.dock_in(vp, ui.DockPosition.BOTTOM, 0.16)
                print("[UI] Joints 좌측 · Recording 하단 도킹 완료", flush=True)
        except Exception as e:
            print(f"[UI] 도킹 실패(플로팅 유지): {e}", flush=True)

    # --- 키보드 가드 ---
    @property
    def editing(self) -> bool:
        return bool(self._editing_fields) or self.save_dialog.visible

    def _guard_field(self, field, key):
        try:
            field.model.add_begin_edit_fn(lambda m, k=key: self._editing_fields.add(k))
            field.model.add_end_edit_fn(lambda m, k=key: self._editing_fields.discard(k))
        except Exception:
            pass

    # --- 저장 다이얼로그 ---
    def _build_save_dialog(self):
        self.save_dialog = ui.Window("Save Episode", width=340, height=170, visible=False)
        with self.save_dialog.frame:
            with ui.VStack(spacing=6, style={"margin": 8}):
                self.dialog_info = ui.Label("-", height=18)
                with ui.HStack(height=22):
                    ui.Label("name", width=50)
                    self.dialog_name = ui.StringField(height=22)
                with ui.HStack(height=22):
                    ui.Label("note", width=50)
                    self.dialog_note = ui.StringField(height=22)
                with ui.HStack(height=26, spacing=8):
                    ui.Button("Save", clicked_fn=self._on_dialog_save)
                    ui.Button("Discard", clicked_fn=self._on_dialog_discard)

    def _open_save_dialog(self):
        self.dialog_info.text = (f"episode_{self.recorder.episode_idx:04d} — "
                                 f"{self.recorder.frame_idx} frames")
        self.dialog_name.model.set_value(self.operator_field.model.get_value_as_string())
        self.dialog_note.model.set_value("")
        self.save_dialog.visible = True

    def _on_dialog_save(self):
        self.recorder.commit(save=True,
                             operator=self.dialog_name.model.get_value_as_string().strip(),
                             note=self.dialog_note.model.get_value_as_string().strip())
        self.save_dialog.visible = False
        self.refresh_episodes()

    def _on_dialog_discard(self):
        self.recorder.commit(save=False)
        self.save_dialog.visible = False

    # --- 리플레이 ---
    def refresh_episodes(self):
        self._episodes = self.recorder.list_saved_episodes()
        self._replay_box.clear()
        with self._replay_box:
            if self._episodes:
                self.episode_combo = ui.ComboBox(
                    max(0, len(self._episodes) - 1),
                    *[label for _, label in self._episodes], height=24,
                )
            else:
                self.episode_combo = None
                ui.Label("no saved episodes", height=24, style={"color": 0xFF888888})

    def _on_play(self):
        if not self._episodes or self.episode_combo is None:
            return
        i = self.episode_combo.model.get_item_value_model().as_int
        i = max(0, min(i, len(self._episodes) - 1))
        self.replay.request(self._episodes[i][0])

    def _apply_font_scale(self):
        """Joints 패널 실측 폭에 맞춰 폰트 크기 재계산 — 폭을 줄이면 글자도 줄어드는 적응형."""
        try:
            w = self.joints_window.frame.computed_width or 420
        except Exception:
            return
        size = max(15, min(22, int(w / 22)))
        if size == self._font_size:
            return
        self._font_size = size
        self.title_label.style = _st(_TITLE, size + 7, bold=True)
        self.conn_label.style = _st(0xFFDDDDDD, size)
        for h in self._headers:
            h.style = _st(0xFF33CCFF, size + 3, bold=True)
        for nm in self._name_labels:
            nm.style = _st(0xFFCCCCCC, size)
        for n_lbl, t_lbl, a_lbl in self._rows:
            for lbl in (n_lbl, t_lbl, a_lbl):
                lbl.style = _st(size=size)

    # --- 루프 갱신 ---
    def update(self, leader_norm, target_rad, actual_rad, fps: float, leader_connected: bool):
        self._tick += 1
        if self._tick % self.update_every != 0:
            return
        self.recorder.operator = self.operator_field.model.get_value_as_string().strip()
        self._apply_font_scale()
        self.conn_label.text = (f"leader {'connected' if leader_connected else 'DISCONNECTED'}"
                                f"   loop {fps:5.1f} fps")
        for i, (n_lbl, t_lbl, a_lbl) in enumerate(self._rows):
            n_lbl.text = f"{float(leader_norm[i]):7.2f}"
            t_lbl.text = f"{math.degrees(float(target_rad[i])):7.1f}"
            a_lbl.text = f"{math.degrees(float(actual_rad[i])):7.1f}"
        text = self.replay.status_text or self.recorder.status_text
        color = (0xFF55AAFF if text.startswith("REPLAY")
                 else 0xFF5555EE if text.startswith("REC")
                 else 0xFFD29922 if "PENDING" in text
                 else 0xFF55CC55)
        self.rec_label.text = text
        self.rec_label.style = {"color": color, "font_size": 20}
