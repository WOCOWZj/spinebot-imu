#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
rmu_vis.py -- 实时可视化 rmu 姿态：3D 姿态指示 + roll/pitch/yaw 曲线

依赖:
    pip install pyserial matplotlib -i https://mirrors.ustc.edu.cn/pypi/web/simple
    sudo apt install -y python3-tk          # 若报 No module named '_tkinter'

用法:
    python3 tools/rmu_vis.py --port /dev/rmu              # 正常跑，日志写 rmu_vis.log
    python3 tools/rmu_vis.py --model plane                # 用飞机模型替代立方体
    python3 tools/rmu_vis.py --model both                 # 立方体 + 飞机叠加
    python3 tools/rmu_vis.py --diag --snapshot diag.png   # 不接硬件，自检 GUI 并出图
    python3 tools/rmu_vis.py --headless                   # 无 GUI，只写日志/快照
    python3 tools/rmu_vis.py -v                           # 控制台也打详细日志

排障思路（白屏/刷屏时）:
    1) 先跑 --diag：如果能出图 -> GUI 环境没问题，问题在数据/串口
    2) 如果 --diag 也白屏 -> 把 rmu_vis.log 前 40 行给我，里面记录了
       backend / matplotlib / numpy / tkinter / DISPLAY / WAYLAND_DISPLAY
    3) 后台运行:  nohup python3 tools/rmu_vis.py >/dev/null 2>&1 &   然后看 log

设计说明:
  * 3D 姿态由四元数直接构造旋转矩阵，与固件 toRotationMatrix() 同式
  * 所有异常写入日志文件；Tkinter 回调异常被接管，不再刷屏 stderr
  * 同一异常只记录一次完整 traceback，之后只累加计数
"""

import argparse
import logging
import math
import os
import sys
import time
import traceback
from collections import deque

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

try:
    from rmu_reader import FrameParser
except ImportError:
    print("找不到 rmu_reader.py，请把它和本脚本放在同一目录（tools/）",
          file=sys.stderr)
    sys.exit(2)

LOG = logging.getLogger("rmu_vis")
DIAG_BURST = 50          # diag 模式每步灌入的帧数，让曲线填满窗口


# ---------------------------------------------------------------- 日志
def setup_logging(path, verbose):
    LOG.setLevel(logging.DEBUG)
    LOG.handlers.clear()
    fh = logging.FileHandler(path, mode="w", encoding="utf-8")
    fh.setFormatter(logging.Formatter("%(asctime)s %(levelname)-5s %(message)s"))
    LOG.addHandler(fh)
    ch = logging.StreamHandler(sys.stdout)
    ch.setLevel(logging.DEBUG if verbose else logging.WARNING)
    ch.setFormatter(logging.Formatter("%(levelname)-5s %(message)s"))
    LOG.addHandler(ch)
    logging.captureWarnings(True)          # matplotlib 的 warning 也进日志
    return LOG


def install_tk_exception_hook():
    """接管 Tkinter 回调里的异常：同类只记一次完整栈，之后只计数，避免刷屏。"""
    try:
        import tkinter
        seen = {}

        def _report(self, exc, val, tb):
            key = f"{exc.__name__}: {val}"
            n = seen.get(key, 0) + 1
            seen[key] = n
            if n == 1:
                LOG.error("Tkinter 回调异常: %s\n%s", key,
                          "".join(traceback.format_exception(exc, val, tb)))
                print(f"[warn] Tkinter 回调异常: {key}\n"
                      f"       完整栈已写入日志（同类异常后续只计数）", file=sys.stderr)
            elif n in (10, 100, 1000, 10000):
                LOG.error("同类 Tkinter 回调异常已重复 %d 次: %s", n, key)
            else:
                LOG.debug("Tkinter 回调异常(重复 #%d): %s", n, key)

        tkinter.Tk.report_callback_exception = _report
    except Exception:
        LOG.debug("无法安装 Tk 异常钩子:\n%s", traceback.format_exc())


def log_environment():
    LOG.info("cwd = %s", os.getcwd())
    for k in ("DISPLAY", "WAYLAND_DISPLAY", "XDG_SESSION_TYPE", "MPLBACKEND"):
        LOG.info("env %s = %s", k, os.environ.get(k))
    try:
        import matplotlib
        LOG.info("matplotlib %s  backend=%s", matplotlib.__version__,
                 matplotlib.get_backend())
        import numpy
        LOG.info("numpy %s", numpy.__version__)
    except Exception:
        LOG.info("matplotlib/numpy 版本读取失败:\n%s", traceback.format_exc())
    try:
        import tkinter
        LOG.info("tkinter TkVersion=%s TclVersion=%s",
                 tkinter.TkVersion, tkinter.TclVersion)
    except Exception as e:
        LOG.warning("tkinter 不可用: %s", e)
    try:
        import serial
        LOG.info("pyserial %s", serial.__version__)
    except Exception:
        LOG.warning("pyserial 不可用")


# ---------------------------------------------------------------- 几何
def quat_to_R(q):
    """四元数 (w,x,y,z) -> 旋转矩阵，与固件 toRotationMatrix() 同式。"""
    import numpy as np
    w, x, y, z = q
    n = math.sqrt(w * w + x * x + y * y + z * z)
    if n < 1e-9:
        return np.eye(3)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


CUBE_V = None
CUBE_E = [(0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6),
          (6, 7), (7, 4), (0, 4), (1, 5), (2, 6), (3, 7)]

# 简单飞机模型，机体坐标系: +X 机头, +Y 右翼, +Z 向上
# 配色刻意避开机体轴的 红(X)/绿(Y)/蓝(Z)，否则会看不出来
PLANE_PARTS = [
    ("fuselage", [(0.95, 0.0, 0.0), (-0.78, 0.0, 0.0)], "#111111", 2.8),
    ("wing", [(-0.15, -0.85, 0.0), (0.15, -0.10, 0.0),
              (0.15, 0.10, 0.0), (-0.15, 0.85, 0.0)], "#ff7f0e", 2.2),
    ("tailplane", [(-0.80, -0.30, 0.0), (-0.62, -0.05, 0.0),
                   (-0.62, 0.05, 0.0), (-0.80, 0.30, 0.0)], "#9467bd", 1.8),
    ("fin", [(-0.78, 0.0, 0.0), (-0.62, 0.0, 0.0),
             (-0.78, 0.0, 0.40), (-0.80, 0.0, 0.0)], "#9467bd", 1.8),
]


def cube_vertices():
    import numpy as np
    global CUBE_V
    if CUBE_V is None:
        CUBE_V = np.array([[-1, -1, -1], [+1, -1, -1], [+1, +1, -1], [-1, +1, -1],
                           [-1, -1, +1], [+1, -1, +1], [+1, +1, +1], [-1, +1, +1]],
                          float) * 0.5
    return CUBE_V


def edges_xyz(V):
    """12 条棱拼成一条 NaN 分隔的折线，供单个 Line3D 一次性更新。

    注意: 必须返回 numpy 数组。matplotlib >= 3.11 在 Line3D.draw() 里会访问
    verts[0].shape，传 Python list 会抛 AttributeError: 'list' object has no
    attribute 'shape'（表现为窗口白屏 + Tkinter 回调刷屏）。
    """
    import numpy as np
    xs, ys, zs = [], [], []
    for a, b in CUBE_E:
        for i in (a, b):
            xs.append(V[i, 0]); ys.append(V[i, 1]); zs.append(V[i, 2])
        xs.append(float("nan")); ys.append(float("nan")); zs.append(float("nan"))
    return (np.asarray(xs, float), np.asarray(ys, float), np.asarray(zs, float))


def set3d(line, x, y, z):
    """统一入口：把任意序列转成 numpy 数组再交给 set_data_3d。"""
    import numpy as np
    line.set_data_3d(np.asarray(x, float), np.asarray(y, float),
                     np.asarray(z, float))


# ---------------------------------------------------------------- 图形
def build_figure(args, plt):
    import numpy as np
    fig = plt.figure(figsize=(11, 5.2))
    ax3 = fig.add_subplot(1, 2, 1, projection="3d")
    ax2 = fig.add_subplot(1, 2, 2)

    ax3.set_xlim(-1, 1); ax3.set_ylim(-1, 1); ax3.set_zlim(-1, 1)
    try:
        ax3.set_box_aspect((1, 1, 1))
    except Exception:
        LOG.debug("set_box_aspect 不支持(旧版 matplotlib):\n%s", traceback.format_exc())
    ax3.set_xlabel("X"); ax3.set_ylabel("Y"); ax3.set_zlabel("Z")
    ax3.set_title(f"attitude ({args.model})")

    empty = np.zeros(0)          # 必须用数组, 见 edges_xyz 的注释

    cube = None
    if args.model in ("cube", "both"):
        cube, = ax3.plot(empty, empty, empty, "-", lw=1.2, color="#3b6ea5")

    plane = []
    if args.model in ("plane", "both"):
        for _name, pts, color, lw in PLANE_PARTS:
            ln, = ax3.plot(empty, empty, empty, "-", lw=lw, color=color,
                           solid_capstyle="round")
            plane.append((ln, np.asarray(pts, float)))

    # 有模型时把机体轴画细、半透明，避免盖住机身
    axis_lw = 2.8 if args.model == "cube" else 1.2
    axis_alpha = 1.0 if args.model == "cube" else 0.55
    axes_lines = [ax3.plot(empty, empty, empty, "-", lw=axis_lw, alpha=axis_alpha,
                           color=c)[0]
                  for c in ("#d62728", "#2ca02c", "#1f77b4")]
    hor = np.array([[-0.9, -0.9, 0], [0.9, -0.9, 0], [0.9, 0.9, 0],
                    [-0.9, 0.9, 0]], float)
    ax3.plot(hor[:, 0], hor[:, 1], hor[:, 2], "--", lw=0.8, color="0.6")
    # 视角：飞机需要机翼展开的斜俯视，立方体用默认斜视即可
    elev, azim = (25.0, -60.0) if args.model == "cube" else (30.0, -120.0)
    if args.view:
        try:
            elev, azim = (float(x) for x in args.view.split(","))
        except Exception:
            LOG.warning("--view 格式应为 'elev,azim'，已忽略: %r", args.view)
    try:
        ax3.view_init(elev=elev, azim=azim)
    except Exception:
        LOG.debug("view_init 失败:\n%s", traceback.format_exc())

    ax2.set_title("euler angle (deg)")
    ax2.set_xlabel("t (s)")
    ax2.set_ylabel("deg")
    ax2.grid(alpha=0.3)
    curves = [ax2.plot([], [], color=c, lw=1.0, label=l)[0]
              for c, l in zip(("#d62728", "#2ca02c", "#1f77b4"),
                              ("roll", "pitch", "yaw"))]
    ax2.legend(loc="upper right", fontsize=8)

    info = fig.text(0.01, 0.01, "waiting for data ...", fontsize=9,
                    family="monospace")
    try:
        fig.tight_layout(rect=(0, 0.04, 1, 1))
    except Exception:
        LOG.debug("tight_layout 失败(可忽略):\n%s", traceback.format_exc())
    return dict(fig=fig, ax3=ax3, ax2=ax2, model=args.model, cube=cube,
                plane=plane, axes_lines=axes_lines, curves=curves, info=info)


def update_artists(g, last, bufs, args, sign, stats):
    """刷新所有 artist。任何异常由调用方捕获并去重记录。"""
    import numpy as np
    R = quat_to_R(last["quat"])
    # 立方体
    if g.get("cube") is not None:
        set3d(g["cube"], *edges_xyz((R @ cube_vertices().T).T))
    # 飞机模型：每个部件绕同一条 R 旋转
    for ln, pts in g.get("plane", ()):
        P = pts @ R.T
        set3d(ln, P[:, 0], P[:, 1], P[:, 2])
    # 机体三轴
    for i, ln in enumerate(g["axes_lines"]):
        a = R[:, i] * 0.95
        set3d(ln, [0.0, a[0]], [0.0, a[1]], [0.0, a[2]])

    bt, br, bp, by = bufs
    if len(bt) >= 2:
        ts = np.fromiter(bt, float)
        ts = ts - ts[-1]
        arrs = []
        for i, d in enumerate((br, bp, by)):
            arr = np.fromiter(d, float) * sign[i]
            if i == 2:
                arr = np.rad2deg(np.unwrap(np.deg2rad(arr)))
            arrs.append(arr)
        lo = min(float(a.min()) for a in arrs)
        hi = max(float(a.max()) for a in arrs)
        pad = max(1.0, (hi - lo) * 0.12)
        g["ax2"].set_ylim(lo - pad, hi + pad)
        g["ax2"].set_xlim(-args.window, 0.2)
        for ln, arr in zip(g["curves"], arrs):
            step = max(1, len(arr) // args.max_points)
            ln.set_data(ts[::step], arr[::step])

    e = last["eul_deg"]
    g["info"].set_text(
        f"{stats['fps']:6.1f} Hz | frames {stats['frames']:<7d} "
        f"crc_err {stats['crc']:<4d} resync {stats['resync']:<6d}B | "
        f"roll {e[0]:+7.2f} pitch {e[1]:+7.2f} yaw {e[2]:+8.2f} deg | "
        f"|q| {last['qnorm']:.4f} T {last['temp']:>3d}C "
        f"static {last['is_static']} cal {last['is_calibrate']}")


def make_sign(flip_spec):
    import numpy as np
    f = {s.strip().lower() for s in flip_spec.split(",") if s.strip()}
    return np.array([-1.0 if "roll" in f else 1.0,
                     -1.0 if "pitch" in f else 1.0,
                     -1.0 if "yaw" in f else 1.0])


def check_3d_artist_types(g):
    """回归检查: matplotlib>=3.11 的 Line3D.draw() 会访问 verts[0].shape，
    若 artist 内部存的是 list 就会抛 AttributeError 并导致白屏。
    这里显式验证，把"白屏"变成日志里一条明确结论。
    """
    bad = []
    try:
        for ln in g["ax3"].lines:
            verts = getattr(ln, "_verts3d", None)
            if verts and not hasattr(verts[0], "shape"):
                bad.append(type(ln).__name__)
    except Exception:
        LOG.debug("3D artist 类型检查跳过:\n%s", traceback.format_exc())
        return True
    if bad:
        LOG.error("3D artist 校验失败: %d 条 Line3D 内部是 list 而非 ndarray "
                  "(matplotlib>=3.11 会白屏)", len(bad))
        return False
    LOG.info("3D artist 校验通过: 所有 Line3D 均持有 numpy 数组")
    return True


# ---------------------------------------------------------------- 自检
def run_diag(args, plt):
    """不接硬件，用合成帧走完绘图管线，验证 GUI 环境是否正常。"""
    import rmu_reader as R
    LOG.info("== diag 模式: 合成数据 %.1f 秒, %d 帧/步 ==", args.diag_seconds,
             DIAG_BURST)
    g = build_figure(args, plt)
    bufs = [deque(maxlen=int(args.window * 1000)) for _ in range(4)]
    sign = make_sign(args.flip)
    parser = R.FrameParser()
    stats = dict(fps=args.fps, frames=0, crc=0, resync=0)
    ok, i, last = 0, 0, None
    plt.ion()
    plt.show()

    for _ in range(max(1, int(args.diag_seconds * args.fps))):
        # 每步灌入一小段 1kHz 数据，让曲线填满窗口（否则只有几毫秒看不到波形）
        for _ in range(DIAG_BURST):
            yaw = ((i * 0.05) % 360.0) - 180.0
            frame = R.build_frame(
                ts_ms=i,
                eul=tuple(math.radians(v) for v in
                          (25 * math.sin(i / 300.0), 15 * math.cos(i / 250.0), yaw)),
                quat=(math.cos(math.radians(12)), 0, 0,
                      math.sin(math.radians(12))))
            for f in parser.feed(frame):
                e = f["eul_deg"]
                bufs[0].append(i / 1000.0)
                bufs[1].append(e[0]); bufs[2].append(e[1]); bufs[3].append(e[2])
                stats["frames"] = parser.frames
                stats["crc"] = parser.crc_errors
                stats["resync"] = parser.resync_bytes
                last = f
            i += 1
        if last is None:
            continue
        try:
            update_artists(g, last, bufs, args, sign, stats)
            if ok == 0:
                check_3d_artist_types(g)
            plt.pause(1.0 / args.fps)
            ok += 1
        except Exception:
            LOG.error("diag 绘制失败(仅记录一次):\n%s", traceback.format_exc())
            break

    out = args.snapshot or "diag.png"
    try:
        g["fig"].savefig(out, dpi=90)
        LOG.info("diag 出图成功 -> %s (%d 帧, 刷新 %d 次)", out,
                 stats["frames"], ok)
        print(f"[diag] OK: 绘图管线正常，已保存 {out}")
        rc = 0
    except Exception:
        LOG.error("diag 保存图片失败:\n%s", traceback.format_exc())
        print("[diag] FAIL: 无法出图，请看日志")
        rc = 3
    plt.close("all")
    return rc


# ---------------------------------------------------------------- 实时
def run_live(args, plt):
    import serial
    try:
        ser = serial.Serial(args.port, args.baud, timeout=0)
    except Exception:
        LOG.error("打不开 %s:\n%s", args.port, traceback.format_exc())
        print(f"打不开 {args.port}（权限/USB 直通/设备号？详见 {args.log}）",
              file=sys.stderr)
        return 2

    g = build_figure(args, plt)
    bufs = [deque(maxlen=int(args.window * 1000)) for _ in range(4)]
    sign = make_sign(args.flip)
    parser = FrameParser()
    stats = dict(fps=0.0, frames=0, crc=0, resync=0)

    headless = args.headless
    if not headless:
        plt.ion()
        plt.show()
        try:
            g["fig"].canvas.draw()          # 先强绘一次，暴露首帧问题
        except Exception:
            LOG.error("首帧绘制失败:\n%s", traceback.format_exc())

    t0 = time.time()
    last = None
    last_draw = 0.0
    last_status = 0.0
    last_snap = 0.0
    win_t0 = t0
    win_n = 0
    draw_fail = 0
    snap_n = 0
    checked = False

    try:
        while True:
            if not headless and not plt.fignum_exists(g["fig"].number):
                LOG.info("窗口被关闭，退出")
                break

            chunk = ser.read(4096)
            if chunk:
                for f in parser.feed(chunk):
                    e = f["eul_deg"]
                    bufs[0].append(f["ts_ms"] / 1000.0)
                    bufs[1].append(e[0]); bufs[2].append(e[1]); bufs[3].append(e[2])
                    last = f
                    win_n += 1

            now = time.time()
            if now - win_t0 >= 1.0:
                stats.update(fps=win_n / (now - win_t0), frames=parser.frames,
                             crc=parser.crc_errors, resync=parser.resync_bytes)
                win_n = 0
                win_t0 = now
                LOG.debug("fps=%.1f frames=%d crc_err=%d resync=%dB last_eul=%s",
                          stats["fps"], stats["frames"], stats["crc"],
                          stats["resync"],
                          None if last is None else [round(v, 2) for v in last["eul_deg"]])

            if now - last_draw < 1.0 / args.fps:
                time.sleep(0.002)
                continue
            last_draw = now

            if last is None:
                if now - t0 > 2.0 and not args.headless:
                    g["info"].set_text(f"2 秒内未解析出帧 | 丢弃 {parser.resync_bytes} B")
                if args.snapshot and now - last_snap >= args.snapshot_every:
                    last_snap = now
                    snap_n += 1
                    g["fig"].savefig(f"{args.snapshot}", dpi=90)
                if not headless:
                    plt.pause(0.001)
                continue

            try:
                update_artists(g, last, bufs, args, sign, stats)
                if not checked:
                    checked = True
                    check_3d_artist_types(g)
                if headless:
                    if args.snapshot and now - last_snap >= args.snapshot_every:
                        last_snap = now
                        snap_n += 1
                        g["fig"].savefig(args.snapshot, dpi=90)
                        LOG.debug("快照 %s (#%d)", args.snapshot, snap_n)
                else:
                    plt.pause(0.001)
                draw_fail = 0
            except Exception:
                draw_fail += 1
                if draw_fail == 1:
                    LOG.error("绘制异常(仅记录第一次完整栈):\n%s",
                              traceback.format_exc())
                    print(f"[warn] 绘制异常: 见 {args.log}", file=sys.stderr)
                if draw_fail >= 5 and not headless:
                    LOG.error("连续 %d 次绘制失败，改为 headless(界面不再刷新)",
                              draw_fail)
                    print("[warn] 连续绘制失败，已切到 headless 模式，"
                          "数据仍在记录，详见日志", file=sys.stderr)
                    headless = True
                time.sleep(0.05)
    except KeyboardInterrupt:
        LOG.info("用户中断 (Ctrl-C)")
    except Exception:
        LOG.error("主循环异常退出:\n%s", traceback.format_exc())
    finally:
        try:
            ser.close()
        except Exception:
            pass
        if not headless:
            try:
                plt.close("all")
            except Exception:
                pass

    summary = (f"帧 {parser.frames} | CRC 错 {parser.crc_errors} | "
               f"重同步 {parser.resync_bytes}B | 绘制异常 {draw_fail} 次 | "
               f"运行 {time.time() - t0:.1f}s")
    LOG.info("==== 结束: %s ====", summary)
    print(summary)
    return 0


# ---------------------------------------------------------------- 入口
def build_parser():
    ap = argparse.ArgumentParser(description="rmu 姿态实时可视化")
    ap.add_argument("--port", default="/dev/rmu")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--window", type=float, default=10.0, help="曲线时间窗口(秒)")
    ap.add_argument("--fps", type=float, default=20.0, help="图形刷新率")
    ap.add_argument("--max-points", type=int, default=1500)
    ap.add_argument("--flip", default="", help="取反轴: roll,pitch,yaw")
    ap.add_argument("--model", default="cube", choices=["cube", "plane", "both"],
                    help="3D 显示模型：cube=立方体, plane=飞机, both=两者叠加")
    ap.add_argument("--view", default="", help="视角覆盖, 形如 '30,-120'")
    ap.add_argument("--log", default="rmu_vis.log")
    ap.add_argument("--backend", default="auto",
                    choices=["auto", "tkagg", "qtagg", "agg"],
                    help="绘图后端；白屏时可换 qtagg 试试")
    ap.add_argument("--snapshot", default="", help="定期保存 PNG 的路径")
    ap.add_argument("--snapshot-every", type=float, default=2.0)
    ap.add_argument("--headless", action="store_true", help="不开窗口")
    ap.add_argument("--diag", action="store_true", help="不接硬件自检绘图管线")
    ap.add_argument("--diag-seconds", type=float, default=3.0)
    ap.add_argument("-v", "--verbose", action="store_true", help="控制台打详细日志")
    return ap


def main():
    args = build_parser().parse_args()
    # 后端选择必须在 import matplotlib 之前通过环境变量决定
    if args.headless or args.diag or args.backend == "agg":
        os.environ["MPLBACKEND"] = "Agg"
    elif args.backend == "tkagg":
        os.environ["MPLBACKEND"] = "TkAgg"
    elif args.backend == "qtagg":
        os.environ["MPLBACKEND"] = "QtAgg"

    setup_logging(args.log, args.verbose)
    LOG.info("==== rmu_vis start ====")
    LOG.info("args = %s", vars(args))
    log_environment()
    install_tk_exception_hook()

    import matplotlib.pyplot as plt
    plt.rcParams["axes.unicode_minus"] = False

    try:
        return run_diag(args, plt) if args.diag else run_live(args, plt)
    except Exception:
        LOG.error("未捕获异常:\n%s", traceback.format_exc())
        print(f"程序异常退出，详见 {args.log}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
