#!/usr/bin/env python3
"""f1tenth_gym 정적 장애물 생성기 GUI.

맵(yaml)을 골라 캔버스에 점을 찍어 정적 장애물을 배치한 뒤 [적용 & 실행]을
누르면, 장애물이 구워진(baked) 맵 이미지/야믈(`<이름>_obs.*`)을 저장하고
f1tenth_gym_ros의 obstacle_sim_launch.py 로 시뮬레이터를 자동 실행한다.

ROS 를 import 하지 않는 독립 도구다. 실행:
    python3 tools/obstacle_map_maker.py
"""

import math
import os
import signal
import subprocess
import sys
import time

import yaml

try:
    import tkinter as tk
    from tkinter import filedialog, messagebox, ttk
except ImportError:  # headless import (테스트용) 허용
    tk = None

from PIL import Image, ImageDraw

REPO_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_MAP_DIRS = [os.path.join(REPO_DIR, 'f1tenth_gym_ros', 'maps')]
ROS_SETUP = '/opt/ros/humble/setup.bash'
WS_SETUP = os.path.join(REPO_DIR, 'install', 'setup.bash')
LAUNCH_PKG = 'f1tenth_gym_ros'
LAUNCH_FILE = 'obstacle_sim_launch.py'
SIM_LOG = os.path.join('/tmp', 'obstacle_sim_launch.log')

OBS_SUFFIX = '_obs'
SIDECAR_SUFFIX = '.obstacles.yaml'


# ---------------------------------------------------------------------------
# 순수 로직 (GUI 비의존) — 헤드리스 테스트 가능
# ---------------------------------------------------------------------------

class Obstacle:
    """월드 좌표(m) 기준 정적 장애물. type: 'circle' | 'rect'(축 정렬)."""

    def __init__(self, type_, x, y, radius=0.25, width=0.5, height=0.5):
        self.type = type_
        self.x = float(x)
        self.y = float(y)
        self.radius = float(radius)
        self.width = float(width)
        self.height = float(height)

    def to_dict(self):
        if self.type == 'circle':
            return {'type': 'circle', 'x': self.x, 'y': self.y,
                    'radius': self.radius}
        return {'type': 'rect', 'x': self.x, 'y': self.y,
                'width': self.width, 'height': self.height}

    @staticmethod
    def from_dict(d):
        return Obstacle(d['type'], d['x'], d['y'],
                        radius=d.get('radius', 0.25),
                        width=d.get('width', 0.5),
                        height=d.get('height', 0.5))


class MapModel:
    """맵 yaml + 이미지 로드, 월드<->픽셀 변환, 장애물 베이크/저장."""

    def __init__(self, yaml_path):
        self.yaml_path = os.path.abspath(yaml_path)
        with open(self.yaml_path, 'r') as f:
            self.meta = yaml.safe_load(f)
        if not isinstance(self.meta, dict) or 'image' not in self.meta \
                or 'resolution' not in self.meta or 'origin' not in self.meta:
            raise ValueError(f'맵 yaml 형식이 아닙니다: {yaml_path}')

        img_rel = self.meta['image']
        img_path = img_rel if os.path.isabs(img_rel) else \
            os.path.join(os.path.dirname(self.yaml_path), img_rel)
        if not os.path.exists(img_path):
            raise FileNotFoundError(f'맵 이미지가 없습니다: {img_path}')
        self.img_path = img_path
        self.img_ext = os.path.splitext(img_path)[1]
        self.image = Image.open(img_path)
        self.image.load()
        self.gray = self.image.convert('L')
        self.width, self.height = self.image.size
        self.resolution = float(self.meta['resolution'])
        self.origin = [float(v) for v in self.meta['origin'][:2]]
        self.negate = int(self.meta.get('negate', 0))

    # 월드(m) -> 이미지 픽셀(float, 좌상단 원점)
    def world_to_px(self, x, y):
        u = (x - self.origin[0]) / self.resolution
        v = self.height - (y - self.origin[1]) / self.resolution
        return u, v

    # 이미지 픽셀 -> 월드(m)
    def px_to_world(self, u, v):
        x = self.origin[0] + u * self.resolution
        y = self.origin[1] + (self.height - v) * self.resolution
        return x, y

    def value_at(self, x, y):
        """해당 월드 좌표 픽셀의 그레이스케일 값(0~255). 범위 밖이면 None."""
        u, v = self.world_to_px(x, y)
        ui, vi = int(math.floor(u)), int(math.floor(v))
        if 0 <= ui < self.width and 0 <= vi < self.height:
            return self.gray.getpixel((ui, vi))
        return None

    def bake(self, obstacles):
        """장애물을 검정(=점유)으로 구운 그레이스케일 이미지를 반환."""
        img = self.gray.copy()
        draw = ImageDraw.Draw(img)
        fill = 255 if self.negate else 0
        for ob in obstacles:
            u, v = self.world_to_px(ob.x, ob.y)
            if ob.type == 'circle':
                r = ob.radius / self.resolution
                draw.ellipse((u - r, v - r, u + r, v + r), fill=fill)
            else:
                hw = 0.5 * ob.width / self.resolution
                hh = 0.5 * ob.height / self.resolution
                draw.rectangle((u - hw, v - hh, u + hw, v + hh), fill=fill)
        return img

    def save_baked(self, out_stem, obstacles, start_pose=None):
        """<out_stem><ext>, <out_stem>.yaml, <out_stem>.obstacles.yaml 저장.

        반환: (image_path, yaml_path, sidecar_path)
        """
        out_img = out_stem + self.img_ext
        out_yaml = out_stem + '.yaml'
        sidecar = out_stem + SIDECAR_SUFFIX

        self.bake(obstacles).save(out_img)

        meta = dict(self.meta)
        meta['image'] = os.path.basename(out_img)
        meta['negate'] = self.negate
        with open(out_yaml, 'w') as f:
            yaml.safe_dump(meta, f, default_flow_style=None, sort_keys=False)

        side = {
            'source_map': self.yaml_path,
            'obstacles': [ob.to_dict() for ob in obstacles],
        }
        if start_pose is not None:
            side['start_pose'] = {'x': start_pose[0], 'y': start_pose[1],
                                  'theta': start_pose[2]}
        with open(sidecar, 'w') as f:
            yaml.safe_dump(side, f, default_flow_style=None, sort_keys=False)
        return out_img, out_yaml, sidecar


def default_out_stem(yaml_path):
    """foo.yaml -> <dir>/foo_obs (이미 _obs 로 끝나면 중복 접미사 방지)."""
    stem = os.path.splitext(os.path.abspath(yaml_path))[0]
    if stem.endswith(OBS_SUFFIX):
        stem = stem[:-len(OBS_SUFFIX)]
    return stem + OBS_SUFFIX


def load_sidecar(path):
    """사이드카 yaml -> (obstacles, start_pose|None, source_map|None)."""
    with open(path, 'r') as f:
        side = yaml.safe_load(f) or {}
    obstacles = [Obstacle.from_dict(d) for d in side.get('obstacles', [])]
    sp = side.get('start_pose')
    start_pose = (sp['x'], sp['y'], sp['theta']) if sp else None
    return obstacles, start_pose, side.get('source_map')


def build_launch_command(map_stem, map_ext, start_pose=None, num_agent=None,
                         rviz=True):
    """f1sim(obstacle_sim_launch.py) 실행용 bash 명령 문자열 생성."""
    args = [f'"map_path:={map_stem}"', f'map_img_ext:={map_ext}']
    if start_pose is not None:
        args += [f'sx:={start_pose[0]:.6f}', f'sy:={start_pose[1]:.6f}',
                 f'stheta:={start_pose[2]:.6f}']
    if num_agent is not None:
        args.append(f'num_agent:={int(num_agent)}')
    if not rviz:
        args.append('rviz:=false')
    launch = f'ros2 launch {LAUNCH_PKG} {LAUNCH_FILE} ' + ' '.join(args)
    return (f'source {ROS_SETUP} && source "{WS_SETUP}" && exec {launch}')


# ---------------------------------------------------------------------------
# GUI
# ---------------------------------------------------------------------------

class ObstacleMakerApp(tk.Tk if tk else object):
    ZOOM_MIN, ZOOM_MAX = 0.05, 40.0

    def __init__(self):
        super().__init__()
        self.title('f1tenth 정적 장애물 생성기')
        self.geometry('1280x860')

        self.model = None
        self.obstacles = []
        self.undo_stack = []            # ('add', ob) | ('del', ob, index)
        self.start_pose = None          # (x, y, theta) | None
        self._drag_start = None         # 시작 포즈 드래그 앵커 (월드)
        self._pan_anchor = None
        self.scale = 1.0                # 화면px / 이미지px
        self.off = [0.0, 0.0]           # 캔버스 (0,0) 의 이미지 좌표
        self._photo = None
        self.sim_proc = None
        self._sim_log_f = None

        self._build_widgets()
        self._bind_canvas()
        self._refresh_map_list()
        self.protocol('WM_DELETE_WINDOW', self._on_close)
        self._poll_sim()

    # ------------------------------------------------------------- widgets
    def _build_widgets(self):
        top = ttk.Frame(self, padding=4)
        top.pack(side=tk.TOP, fill=tk.X)

        ttk.Label(top, text='맵:').pack(side=tk.LEFT)
        self.map_var = tk.StringVar()
        self.map_combo = ttk.Combobox(top, textvariable=self.map_var,
                                      width=48, state='readonly')
        self.map_combo.pack(side=tk.LEFT, padx=4)
        self.map_combo.bind('<<ComboboxSelected>>', lambda e: self._load_selected_map())
        ttk.Button(top, text='찾아보기…', command=self._browse_map).pack(side=tk.LEFT)
        ttk.Button(top, text='장애물 불러오기…', command=self._load_obstacles_file).pack(side=tk.LEFT, padx=4)

        bar = ttk.Frame(self, padding=4)
        bar.pack(side=tk.TOP, fill=tk.X)

        self.mode_var = tk.StringVar(value='obstacle')
        ttk.Radiobutton(bar, text='장애물 배치', variable=self.mode_var,
                        value='obstacle').pack(side=tk.LEFT)
        ttk.Radiobutton(bar, text='시작 포즈(드래그=방향)', variable=self.mode_var,
                        value='start').pack(side=tk.LEFT, padx=(4, 12))

        self.shape_var = tk.StringVar(value='circle')
        ttk.Radiobutton(bar, text='원', variable=self.shape_var,
                        value='circle').pack(side=tk.LEFT)
        ttk.Radiobutton(bar, text='사각형', variable=self.shape_var,
                        value='rect').pack(side=tk.LEFT, padx=(2, 8))

        ttk.Label(bar, text='반지름[m]').pack(side=tk.LEFT)
        self.radius_var = tk.DoubleVar(value=0.25)
        ttk.Spinbox(bar, textvariable=self.radius_var, from_=0.05, to=5.0,
                    increment=0.05, width=6).pack(side=tk.LEFT, padx=(2, 8))
        ttk.Label(bar, text='가로x세로[m]').pack(side=tk.LEFT)
        self.rw_var = tk.DoubleVar(value=0.5)
        self.rh_var = tk.DoubleVar(value=0.5)
        ttk.Spinbox(bar, textvariable=self.rw_var, from_=0.05, to=10.0,
                    increment=0.05, width=6).pack(side=tk.LEFT, padx=2)
        ttk.Spinbox(bar, textvariable=self.rh_var, from_=0.05, to=10.0,
                    increment=0.05, width=6).pack(side=tk.LEFT, padx=(2, 8))

        ttk.Button(bar, text='되돌리기(Ctrl+Z)', command=self._undo).pack(side=tk.LEFT, padx=2)
        ttk.Button(bar, text='전체 삭제', command=self._clear_obstacles).pack(side=tk.LEFT, padx=2)
        ttk.Button(bar, text='화면 맞춤', command=self._fit_view).pack(side=tk.LEFT, padx=2)

        self.canvas = tk.Canvas(self, background='#202020',
                                highlightthickness=0, cursor='crosshair')
        self.canvas.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        bottom = ttk.Frame(self, padding=4)
        bottom.pack(side=tk.BOTTOM, fill=tk.X)

        ttk.Label(bottom, text='출력 이름:').pack(side=tk.LEFT)
        self.out_var = tk.StringVar()
        ttk.Entry(bottom, textvariable=self.out_var, width=28).pack(side=tk.LEFT, padx=4)

        ttk.Label(bottom, text='에이전트:').pack(side=tk.LEFT, padx=(8, 0))
        self.agent_var = tk.StringVar(value='1')
        ttk.Combobox(bottom, textvariable=self.agent_var, width=8,
                     state='readonly', values=('기본', '1', '2')).pack(side=tk.LEFT, padx=2)

        self.rviz_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(bottom, text='RViz', variable=self.rviz_var).pack(side=tk.LEFT, padx=6)

        ttk.Button(bottom, text='맵만 저장', command=self._save_only).pack(side=tk.LEFT, padx=4)
        self.apply_btn = ttk.Button(bottom, text='적용 & 실행',
                                    command=self._apply_and_run)
        self.apply_btn.pack(side=tk.LEFT, padx=4)
        ttk.Button(bottom, text='시뮬 종료', command=self._stop_sim).pack(side=tk.LEFT, padx=4)

        self.sim_status = ttk.Label(bottom, text='시뮬: 꺼짐')
        self.sim_status.pack(side=tk.RIGHT, padx=6)

        self.status = ttk.Label(self, anchor=tk.W, padding=(6, 2))
        self.status.pack(side=tk.BOTTOM, fill=tk.X)
        self._set_status('맵을 선택하세요. 좌클릭=배치 / 우클릭=삭제 / 휠=줌 / '
                         '휠클릭·Ctrl+드래그=이동')

    def _bind_canvas(self):
        c = self.canvas
        c.bind('<ButtonPress-1>', self._on_left_down)
        c.bind('<B1-Motion>', self._on_left_drag)
        c.bind('<ButtonRelease-1>', self._on_left_up)
        c.bind('<ButtonPress-3>', self._on_right_click)
        c.bind('<ButtonPress-2>', self._on_pan_start)
        c.bind('<B2-Motion>', self._on_pan_move)
        c.bind('<Control-ButtonPress-1>', self._on_pan_start)
        c.bind('<Control-B1-Motion>', self._on_pan_move)
        c.bind('<Button-4>', lambda e: self._on_zoom(e, 1.15))
        c.bind('<Button-5>', lambda e: self._on_zoom(e, 1 / 1.15))
        c.bind('<MouseWheel>', lambda e: self._on_zoom(e, 1.15 if e.delta > 0 else 1 / 1.15))
        c.bind('<Motion>', self._on_motion)
        c.bind('<Configure>', lambda e: self._redraw())
        self.bind('<Control-z>', lambda e: self._undo())

    def _set_status(self, text):
        self.status.configure(text=text)

    # --------------------------------------------------------- map loading
    def _refresh_map_list(self):
        entries = {}
        for d in DEFAULT_MAP_DIRS:
            if not os.path.isdir(d):
                continue
            for name in sorted(os.listdir(d)):
                if not name.endswith('.yaml') or name.endswith(SIDECAR_SUFFIX):
                    continue
                path = os.path.join(d, name)
                try:
                    with open(path, 'r') as f:
                        meta = yaml.safe_load(f)
                    if isinstance(meta, dict) and 'image' in meta:
                        entries[name] = path
                except Exception:
                    continue
        self._map_paths = entries
        self.map_combo['values'] = list(entries.keys())

    def _browse_map(self):
        path = filedialog.askopenfilename(
            title='맵 yaml 선택',
            filetypes=[('map yaml', '*.yaml *.yml'), ('모든 파일', '*')])
        if path:
            self._load_map(path)

    def _load_selected_map(self):
        name = self.map_var.get()
        if name in self._map_paths:
            self._load_map(self._map_paths[name])

    def _load_map(self, yaml_path):
        try:
            model = MapModel(yaml_path)
        except Exception as e:
            messagebox.showerror('맵 로드 실패', str(e))
            return
        self.model = model
        self.obstacles = []
        self.undo_stack = []
        self.start_pose = None
        name = os.path.basename(yaml_path)
        if name in getattr(self, '_map_paths', {}):
            self.map_var.set(name)
        else:
            self.map_var.set(yaml_path)
        self.out_var.set(os.path.basename(default_out_stem(yaml_path)))
        self._fit_view()
        self._set_status(
            f'맵 로드: {name}  ({model.width}x{model.height}px, '
            f'{model.resolution:.4f} m/px)')
        # 이전 세션 사이드카가 있으면 복원 제안
        sidecar = default_out_stem(yaml_path) + SIDECAR_SUFFIX
        if os.path.exists(sidecar):
            if messagebox.askyesno('장애물 복원',
                                   '이전에 저장한 장애물 배치가 있습니다.\n불러올까요?'):
                self._apply_sidecar(sidecar)

    def _load_obstacles_file(self):
        path = filedialog.askopenfilename(
            title='장애물 yaml 선택',
            filetypes=[('obstacles yaml', f'*{SIDECAR_SUFFIX}'), ('모든 파일', '*')])
        if path:
            self._apply_sidecar(path)

    def _apply_sidecar(self, path):
        try:
            obstacles, start_pose, source_map = load_sidecar(path)
        except Exception as e:
            messagebox.showerror('불러오기 실패', str(e))
            return
        if self.model is None:
            if source_map and os.path.exists(source_map):
                self._load_map(source_map)
            if self.model is None:
                messagebox.showwarning('불러오기', '먼저 맵을 선택하세요.')
                return
        self.obstacles = obstacles
        if start_pose is not None:
            self.start_pose = start_pose
        self.undo_stack = []
        self._redraw()
        self._set_status(f'장애물 {len(obstacles)}개 불러옴: {os.path.basename(path)}')

    # ------------------------------------------------------------ view/draw
    def _fit_view(self):
        if self.model is None:
            return
        cw = max(self.canvas.winfo_width(), 50)
        ch = max(self.canvas.winfo_height(), 50)
        if cw < 60 or ch < 60:  # 초기 레이아웃 전이면 잠시 후 재시도
            self.after(80, self._fit_view)
            return
        self.scale = min(cw / self.model.width, ch / self.model.height) * 0.97
        self.scale = max(self.ZOOM_MIN, min(self.ZOOM_MAX, self.scale))
        self.off = [self.model.width / 2 - cw / (2 * self.scale),
                    self.model.height / 2 - ch / (2 * self.scale)]
        self._redraw()

    def _img_to_canvas(self, u, v):
        return (u - self.off[0]) * self.scale, (v - self.off[1]) * self.scale

    def _canvas_to_img(self, cx, cy):
        return self.off[0] + cx / self.scale, self.off[1] + cy / self.scale

    def _world_to_canvas(self, x, y):
        u, v = self.model.world_to_px(x, y)
        return self._img_to_canvas(u, v)

    def _canvas_to_world(self, cx, cy):
        u, v = self._canvas_to_img(cx, cy)
        return self.model.px_to_world(u, v)

    def _redraw(self):
        c = self.canvas
        c.delete('all')
        if self.model is None:
            return
        cw, ch = c.winfo_width(), c.winfo_height()
        if cw < 2 or ch < 2:
            return
        s = self.scale
        x0 = max(0.0, self.off[0]); y0 = max(0.0, self.off[1])
        x1 = min(float(self.model.width), self.off[0] + cw / s)
        y1 = min(float(self.model.height), self.off[1] + ch / s)
        if x1 > x0 and y1 > y0:
            ix0, iy0 = int(math.floor(x0)), int(math.floor(y0))
            ix1, iy1 = int(math.ceil(x1)), int(math.ceil(y1))
            crop = self.model.gray.crop((ix0, iy0, ix1, iy1))
            dw = max(1, int(round((ix1 - ix0) * s)))
            dh = max(1, int(round((iy1 - iy0) * s)))
            resample = Image.NEAREST if s >= 1.0 else Image.BILINEAR
            disp = crop.resize((dw, dh), resample).convert('RGB')
            from PIL import ImageTk
            self._photo = ImageTk.PhotoImage(disp)
            px, py = self._img_to_canvas(ix0, iy0)
            c.create_image(px, py, image=self._photo, anchor=tk.NW)

        # 장애물
        for ob in self.obstacles:
            cx, cy = self._world_to_canvas(ob.x, ob.y)
            if ob.type == 'circle':
                r = ob.radius / self.model.resolution * s
                c.create_oval(cx - r, cy - r, cx + r, cy + r,
                              outline='#ff4040', width=2,
                              fill='#ff4040', stipple='gray25')
            else:
                hw = 0.5 * ob.width / self.model.resolution * s
                hh = 0.5 * ob.height / self.model.resolution * s
                c.create_rectangle(cx - hw, cy - hh, cx + hw, cy + hh,
                                   outline='#ff4040', width=2,
                                   fill='#ff4040', stipple='gray25')
        # 시작 포즈
        if self.start_pose is not None:
            x, y, th = self.start_pose
            cx, cy = self._world_to_canvas(x, y)
            L = max(18.0, 0.5 / self.model.resolution * s)
            ex = cx + L * math.cos(th)
            ey = cy - L * math.sin(th)
            c.create_line(cx, cy, ex, ey, fill='#30d060', width=3,
                          arrow=tk.LAST, arrowshape=(10, 13, 5))
            c.create_oval(cx - 5, cy - 5, cx + 5, cy + 5,
                          outline='#30d060', width=2)

    # ------------------------------------------------------------- events
    def _on_zoom(self, event, factor):
        if self.model is None:
            return
        new_scale = max(self.ZOOM_MIN, min(self.ZOOM_MAX, self.scale * factor))
        if new_scale == self.scale:
            return
        u, v = self._canvas_to_img(event.x, event.y)
        self.scale = new_scale
        self.off = [u - event.x / new_scale, v - event.y / new_scale]
        self._redraw()

    def _on_pan_start(self, event):
        self._pan_anchor = (event.x, event.y, self.off[0], self.off[1])
        return 'break'

    def _on_pan_move(self, event):
        if self._pan_anchor is None:
            return 'break'
        ax, ay, ox, oy = self._pan_anchor
        self.off = [ox - (event.x - ax) / self.scale,
                    oy - (event.y - ay) / self.scale]
        self._redraw()
        return 'break'

    def _on_left_down(self, event):
        if self.model is None or event.state & 0x0004:  # Ctrl -> 팬
            return
        x, y = self._canvas_to_world(event.x, event.y)
        if self.mode_var.get() == 'start':
            th = self.start_pose[2] if self.start_pose else 0.0
            self.start_pose = (x, y, th)
            self._drag_start = (x, y)
            self._redraw()
        else:
            self._add_obstacle(x, y)

    def _on_left_drag(self, event):
        if self.model is None or self._drag_start is None:
            return
        if self.mode_var.get() != 'start':
            return
        x0, y0 = self._drag_start
        x, y = self._canvas_to_world(event.x, event.y)
        if math.hypot(x - x0, y - y0) > 0.05:
            th = math.atan2(y - y0, x - x0)
            self.start_pose = (x0, y0, th)
            self._redraw()

    def _on_left_up(self, event):
        if self._drag_start is not None and self.start_pose is not None:
            x, y, th = self.start_pose
            self._set_status(f'시작 포즈: x={x:.3f}  y={y:.3f}  θ={th:.3f} rad')
        self._drag_start = None

    def _num(self, var, fallback):
        """스핀박스가 비었거나 숫자가 아니면 fallback 사용."""
        try:
            return float(var.get())
        except tk.TclError:
            var.set(fallback)
            return fallback

    def _add_obstacle(self, x, y):
        if self.shape_var.get() == 'circle':
            ob = Obstacle('circle', x, y,
                          radius=max(0.01, self._num(self.radius_var, 0.25)))
        else:
            ob = Obstacle('rect', x, y,
                          width=max(0.01, self._num(self.rw_var, 0.5)),
                          height=max(0.01, self._num(self.rh_var, 0.5)))
        val = self.model.value_at(x, y)
        self.obstacles.append(ob)
        self.undo_stack.append(('add', ob))
        self._redraw()
        note = ''
        if val is not None and val < 100:
            note = '  (주의: 이미 점유된 영역입니다)'
        self._set_status(f'장애물 추가 ({len(self.obstacles)}개): '
                         f'x={x:.3f}, y={y:.3f}{note}')

    def _on_right_click(self, event):
        if self.model is None or not self.obstacles:
            return
        cx, cy = event.x, event.y
        best, best_d = None, None
        for i, ob in enumerate(self.obstacles):
            ox, oy = self._world_to_canvas(ob.x, ob.y)
            d = math.hypot(ox - cx, oy - cy)
            if best_d is None or d < best_d:
                best, best_d = i, d
        ob = self.obstacles[best]
        # 도형 내부이거나 15px 이내일 때만 삭제
        if ob.type == 'circle':
            hit_r = ob.radius / self.model.resolution * self.scale
        else:
            hit_r = 0.5 * max(ob.width, ob.height) / self.model.resolution * self.scale
        if best_d <= max(15.0, hit_r):
            del self.obstacles[best]
            self.undo_stack.append(('del', ob, best))
            self._redraw()
            self._set_status(f'장애물 삭제 (남은 {len(self.obstacles)}개)')

    def _undo(self):
        if not self.undo_stack:
            return
        action = self.undo_stack.pop()
        if action[0] == 'add':
            try:
                self.obstacles.remove(action[1])
            except ValueError:
                pass
        else:
            _, ob, idx = action
            self.obstacles.insert(min(idx, len(self.obstacles)), ob)
        self._redraw()

    def _clear_obstacles(self):
        if self.obstacles and messagebox.askyesno('전체 삭제', '장애물을 모두 지울까요?'):
            self.obstacles = []
            self.undo_stack = []
            self._redraw()

    def _on_motion(self, event):
        if self.model is None:
            return
        x, y = self._canvas_to_world(event.x, event.y)
        val = self.model.value_at(x, y)
        occ = '' if val is None else f'  px={val}'
        self._set_status(f'x={x:.3f}  y={y:.3f}{occ}   '
                         f'장애물 {len(self.obstacles)}개   줌 {self.scale:.2f}x')

    # -------------------------------------------------------------- output
    def _resolve_out_stem(self):
        if self.model is None:
            messagebox.showwarning('저장', '먼저 맵을 선택하세요.')
            return None
        name = self.out_var.get().strip()
        if not name:
            name = os.path.basename(default_out_stem(self.model.yaml_path))
            self.out_var.set(name)
        if os.sep in name:
            return os.path.abspath(name)
        return os.path.join(os.path.dirname(self.model.yaml_path), name)

    def _save(self):
        out_stem = self._resolve_out_stem()
        if out_stem is None:
            return None
        out_abs = os.path.abspath(out_stem)
        if out_abs == os.path.splitext(self.model.yaml_path)[0] \
                or out_abs + self.model.img_ext == os.path.abspath(self.model.img_path):
            messagebox.showerror('저장', '원본 맵을 덮어쓸 수 없습니다. 출력 이름을 바꿔주세요.')
            return None
        try:
            img_p, yaml_p, _ = self.model.save_baked(
                out_stem, self.obstacles, self.start_pose)
        except Exception as e:
            messagebox.showerror('저장 실패', str(e))
            return None
        self._set_status(f'저장됨: {img_p}')
        return out_stem

    def _save_only(self):
        out_stem = self._save()
        if out_stem:
            messagebox.showinfo(
                '저장 완료',
                f'{out_stem}{self.model.img_ext}\n{out_stem}.yaml\n'
                f'{out_stem}{SIDECAR_SUFFIX}')

    # ----------------------------------------------------------- simulator
    def _apply_and_run(self):
        out_stem = self._save()
        if out_stem is None:
            return
        if not os.path.exists(WS_SETUP):
            messagebox.showerror(
                '워크스페이스 없음',
                f'{WS_SETUP} 가 없습니다.\n'
                f'{REPO_DIR} 에서 ./install.sh (또는 colcon build) 를 먼저 실행하세요.')
            return
        self._stop_sim(quiet=True)
        num_agent = None
        if self.agent_var.get() in ('1', '2'):
            num_agent = int(self.agent_var.get())
        cmd = build_launch_command(out_stem, self.model.img_ext,
                                   start_pose=self.start_pose,
                                   num_agent=num_agent,
                                   rviz=self.rviz_var.get())
        try:
            self._sim_log_f = open(SIM_LOG, 'w')
            self.sim_proc = subprocess.Popen(
                ['bash', '-c', cmd],
                stdout=self._sim_log_f, stderr=subprocess.STDOUT,
                start_new_session=True)
        except Exception as e:
            messagebox.showerror('실행 실패', str(e))
            return
        self._set_status(f'시뮬레이터 실행 중… (로그: {SIM_LOG})')

    def _stop_sim(self, quiet=False):
        proc = self.sim_proc
        self.sim_proc = None
        if proc is None or proc.poll() is not None:
            if not quiet:
                self._set_status('실행 중인 시뮬레이터가 없습니다.')
            return
        try:
            pgid = os.getpgid(proc.pid)
            os.killpg(pgid, signal.SIGINT)
            for _ in range(40):                 # 최대 8초 대기
                if proc.poll() is not None:
                    break
                time.sleep(0.2)
            if proc.poll() is None:
                os.killpg(pgid, signal.SIGTERM)
                time.sleep(1.0)
            if proc.poll() is None:
                os.killpg(pgid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        if self._sim_log_f:
            self._sim_log_f.close()
            self._sim_log_f = None
        if not quiet:
            self._set_status('시뮬레이터 종료됨')

    def _poll_sim(self):
        if self.sim_proc is not None:
            code = self.sim_proc.poll()
            if code is None:
                self.sim_status.configure(text='시뮬: 실행 중', foreground='#208030')
            else:
                self.sim_status.configure(
                    text=f'시뮬: 종료(코드 {code})', foreground='#b03030')
                self.sim_proc = None
                if self._sim_log_f:
                    self._sim_log_f.close()
                    self._sim_log_f = None
        else:
            self.sim_status.configure(text='시뮬: 꺼짐', foreground='')
        self.after(700, self._poll_sim)

    def _on_close(self):
        if self.sim_proc is not None and self.sim_proc.poll() is None:
            if messagebox.askyesno('종료', '시뮬레이터도 함께 종료할까요?\n'
                                   '(아니오 = 시뮬레이터는 계속 실행)'):
                self._stop_sim(quiet=True)
        self.destroy()


def main():
    if tk is None:
        print('tkinter 를 사용할 수 없습니다 (python3-tk 설치 필요).',
              file=sys.stderr)
        return 1
    app = ObstacleMakerApp()
    app.mainloop()
    return 0


if __name__ == '__main__':
    sys.exit(main())
