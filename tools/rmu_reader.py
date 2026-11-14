#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
rmu_reader.py -- 读取 rmu (H7_IMU_With_EKF) 的 1kHz 姿态数据流

固件侧来源:
    User/Callback/Timer_Callback/tim_callback.cpp   (IMU_Data, 结构体 62-77 行)
    User/BSP/USB/bsp_usb.cpp                        (每 1ms 一次的发送调度)
    通信协议.md  2.1 节

帧格式 (82 字节, packed, 小端):
    0   H  header    0xA55A          (字节序 5A A5)
    2   H  len       0x004C          (= 82 - 6, 注意不是整帧长度)
    4   H  crc       CRC-16/XMODEM   (范围 = byte[0:4] + byte[6:82])
    6   B  tag       0x91
    7   B  is_calibrate_gyro
    8   B  is_static
    9   b  temp      int8, 摄氏度
    10  f  prs       固定 0.0
    14  I  ts        ms (uint32, 约 49.7 天回绕)
    18  3f acc       m/s^2
    30  3f gyr       rad/s   (已减零偏)
    42  3f offset    陀螺零偏 rad/s
    54  3f eul       roll, pitch, yaw  (rad)   <-- 姿态
    66  4f quat      q0(w), q1, q2, q3

用法:
    python3 rmu_reader.py --selftest                  # 无硬件自检(先跑这个)
    python3 rmu_reader.py                             # 默认 /dev/ttyACM0
    python3 rmu_reader.py --port /dev/rmu             # 用 udev 软链接
    python3 rmu_reader.py --csv log.csv               # 同时录 CSV
    python3 rmu_reader.py --raw                       # 前 20 帧十六进制

依赖: pyserial  (pip install pyserial -i https://mirrors.ustc.edu.cn/pypi/web/simple)
"""

import argparse
import binascii
import math
import struct
import sys
import time

FRAME_LEN = 82
HEADER = b"\x5a\xa5"          # 0xA55A little-endian
LEN_FIELD = 0x004C            # 76 = FRAME_LEN - 6
TAG_EXPECTED = 0x91

# 与 firmware 的 __attribute__((packed)) 结构体一一对应; '<' 前缀保证无对齐填充
_FRAME = struct.Struct("<HHHBBBBfI3f3f3f3f4f")
assert _FRAME.size == FRAME_LEN, _FRAME.size


def crc16_xmodem(data: bytes) -> int:
    """init 0x0000 / poly 0x1021 / 不反射 / 无 xorout，等价固件里的 crc16_update()。"""
    return binascii.crc_hqx(data, 0)


def frame_crc(frame: bytes) -> int:
    """避开 crc 字段自身：前 4 字节 + 第 6 字节到帧尾。"""
    return crc16_xmodem(frame[0:4] + frame[6:FRAME_LEN])


def parse_frame(frame: bytes) -> dict:
    (header, length, crc, tag, calib, static, temp, prs,
     ts_ms, ax, ay, az, gx, gy, gz, ox, oy, oz,
     roll, pitch, yaw, q0, q1, q2, q3) = _FRAME.unpack(frame)

    return {
        "header": header,
        "len": length,
        "crc": crc,
        "tag": tag,
        "is_calibrate": calib,
        "is_static": static,
        "temp": temp,
        "prs": prs,
        "ts_ms": ts_ms,
        "acc": (ax, ay, az),
        "gyr": (gx, gy, gz),
        "offset": (ox, oy, oz),
        "eul_rad": (roll, pitch, yaw),
        "eul_deg": tuple(math.degrees(v) for v in (roll, pitch, yaw)),
        "quat": (q0, q1, q2, q3),
        "qnorm": math.sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3),
    }


def build_frame(ts_ms=1234, acc=(0.0, 0.0, 9.81), gyr=(0.01, -0.02, 0.03),
                eul=(0.1, -0.2, 1.3), quat=(0.99, 0.01, -0.02, 0.03),
                temp=31, static=1, calib=0) -> bytes:
    """自检用：按固件同样的布局拼一帧并写入正确的 CRC。"""
    payload = _FRAME.pack(
        0xA55A, LEN_FIELD, 0, TAG_EXPECTED, calib, static, temp, 0.0,
        ts_ms, *acc, *gyr, *((0.0, 0.0, 0.0)), *eul, *quat)
    crc = frame_crc(payload)
    return payload[0:4] + struct.pack("<H", crc) + payload[6:]


class FrameParser:
    """字节流 -> 帧。负责滑窗找帧头、长度校验、CRC 校验、统计。"""

    def __init__(self):
        self.buf = bytearray()
        self.frames = 0
        self.crc_errors = 0
        self.len_errors = 0
        self.resync_bytes = 0

    def feed(self, data: bytes):
        self.buf += data
        out = []
        while True:
            idx = self.buf.find(HEADER)
            if idx < 0:
                # 保留结尾可能的半个帧头 '5A'
                keep = 1 if self.buf.endswith(b"\x5a") else 0
                self.resync_bytes += len(self.buf) - keep
                del self.buf[: len(self.buf) - keep]
                return out
            if idx:
                self.resync_bytes += idx
                del self.buf[:idx]
            if len(self.buf) < FRAME_LEN:
                return out
            if self.buf[2] | (self.buf[3] << 8) != LEN_FIELD:
                self.len_errors += 1
                self.resync_bytes += 1
                del self.buf[:1]
                continue
            frame = bytes(self.buf[:FRAME_LEN])
            if frame_crc(frame) != (frame[4] | (frame[5] << 8)):
                self.crc_errors += 1
                self.resync_bytes += 1
                del self.buf[:1]
                continue
            del self.buf[:FRAME_LEN]
            self.frames += 1
            parsed = parse_frame(frame)
            parsed["_raw"] = frame          # --raw 模式需要原始字节
            out.append(parsed)
        return out


def selftest() -> int:
    ok = True

    # 1) CRC 算法标准校验值: CRC-16/XMODEM("123456789") == 0x31C3
    got = crc16_xmodem(b"123456789")
    print(f"[1] CRC-16/XMODEM(\"123456789\") = 0x{got:04X}  期望 0x31C3  "
          f"{'OK' if got == 0x31C3 else 'FAIL'}")
    ok &= got == 0x31C3

    # 2) 往返：造帧 -> 解析 -> 字段一致
    eul = (0.123, -0.456, 2.345)
    frame = build_frame(ts_ms=98765, eul=eul, quat=(0.5, 0.5, 0.5, 0.5))
    p = FrameParser()
    got_frames = p.feed(frame)
    f = got_frames[0] if got_frames else None
    passed = (f is not None and f["len"] == LEN_FIELD and f["tag"] == TAG_EXPECTED
              and f["ts_ms"] == 98765
              and all(abs(a - b) < 1e-6 for a, b in zip(f["eul_rad"], eul)))
    print(f"[2] 单帧解析: len/tag/ts/eul 一致  {'OK' if passed else 'FAIL'}")
    ok &= passed

    # 3) 抗干扰：前面塞垃圾字节 + 尾部截断半帧，仍应解出唯一帧
    p = FrameParser()
    out = p.feed(b"\x00\x11\x5a\x33junk" + frame + frame[:40])
    passed = len(out) == 1 and p.resync_bytes > 0
    print(f"[3] 重同步: 垃圾前导+半帧尾部 -> 解出 {len(out)} 帧, "
          f"丢弃 {p.resync_bytes} 字节  {'OK' if passed else 'FAIL'}")
    ok &= passed

    # 4) 坏 CRC 必须被拒绝
    bad = bytearray(frame)
    bad[20] ^= 0xFF
    p = FrameParser()
    out = p.feed(bytes(bad))
    passed = len(out) == 0 and p.crc_errors >= 1
    print(f"[4] 坏 CRC 拒绝: 解出 {len(out)} 帧, crc_errors={p.crc_errors}  "
          f"{'OK' if passed else 'FAIL'}")
    ok &= passed

    print("\n自检" + ("全部通过 ✅" if ok else "存在失败 ❌"))
    return 0 if ok else 1


def main() -> int:
    ap = argparse.ArgumentParser(description="rmu IMU 姿态数据读取器")
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200,
                    help="CDC 虚拟串口此值无实际意义，但必须设置")
    ap.add_argument("--csv", metavar="PATH", help="同时写入 CSV")
    ap.add_argument("--raw", action="store_true", help="打印前 20 帧十六进制")
    ap.add_argument("--fps-window", type=float, default=1.0)
    ap.add_argument("--print-hz", type=float, default=10.0,
                    help="终端刷新频率（计数仍按全速统计）")
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()

    if args.selftest:
        return selftest()

    try:
        import serial
    except ImportError:
        print("缺少 pyserial，请先安装:\n"
              "  pip install pyserial -i https://mirrors.ustc.edu.cn/pypi/web/simple",
              file=sys.stderr)
        return 2

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0)
    except serial.SerialException as e:
        msg = str(e)
        print(f"打不开 {args.port}: {msg}", file=sys.stderr)
        if "ermission denied" in msg or "Errno 13" in msg:
            print("\n这是权限问题（/dev/ttyACM* 属于 root:dialout, 660）。三选一：\n"
                  "  1) 立刻可用（重插后失效）:\n"
                  "       sudo chmod a+rw /dev/ttyACM0\n"
                  "  2) 永久生效（推荐，顺便固定成 /dev/rmu）:\n"
                  "       sudo tee /etc/udev/rules.d/99-rmu.rules >/dev/null <<'EOF'\n"
                  "       SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"0483\", "
                  "ATTRS{idProduct}==\"6666\", MODE=\"0666\", SYMLINK+=\"rmu\"\n"
                  "       EOF\n"
                  "       sudo udevadm control --reload-rules\n"
                  "       sudo udevadm trigger --subsystem-match=tty\n"
                  "       # 之后统一用 --port /dev/rmu\n"
                  "  3) 让已加入的 dialout 组在当前 shell 生效:\n"
                  "       sg dialout -c 'python3 tools/rmu_reader.py ...'"
                  "   (或注销后重新登录)", file=sys.stderr)
        else:
            print("排查: lsusb 是否有 0483:6666 / sudo dmesg | tail / "
                  "VMware 是否已把设备连到虚拟机", file=sys.stderr)
        return 2

    csv_fp = None
    if args.csv:
        csv_fp = open(args.csv, "w", encoding="utf-8", newline="")
        csv_fp.write("ts_ms,roll_deg,pitch_deg,yaw_deg,q0,q1,q2,q3,"
                     "ax,ay,az,gx,gy,gz,temp,is_static,is_calibrate,qnorm\n")

    parser = FrameParser()
    t0 = time.time()
    win_t0 = t0
    win_frames = 0
    fps = 0.0
    last_print = 0.0
    last = None
    raw_left = 20 if args.raw else 0
    print(f"已打开 {args.port}，等待帧头 0xA55A ... (Ctrl-C 结束)\n")

    try:
        while True:
            chunk = ser.read(4096)
            if not chunk:
                time.sleep(0.001)
                continue
            frames = parser.feed(chunk)
            win_frames += len(frames)
            if not frames:
                continue
            last = frames[-1]

            for f in frames:
                if raw_left:
                    raw_left -= 1
                    raw = f["_raw"]
                    head = " ".join(f"{b:02X}" for b in raw[:20])
                    print(f"[raw] {head} ...  len=0x{f['len']:04X} "
                          f"tag=0x{f['tag']:02X} crc=0x{f['crc']:04X} "
                          f"ts={f['ts_ms']}ms")
                if csv_fp:
                    e, q, a, g = f["eul_deg"], f["quat"], f["acc"], f["gyr"]
                    csv_fp.write(
                        f"{f['ts_ms']},{e[0]:.6f},{e[1]:.6f},{e[2]:.6f},"
                        f"{q[0]:.6f},{q[1]:.6f},{q[2]:.6f},{q[3]:.6f},"
                        f"{a[0]:.4f},{a[1]:.4f},{a[2]:.4f},"
                        f"{g[0]:.4f},{g[1]:.4f},{g[2]:.4f},"
                        f"{f['temp']},{f['is_static']},{f['is_calibrate']},"
                        f"{f['qnorm']:.6f}\n")

            now = time.time()
            if now - win_t0 >= args.fps_window:
                fps = win_frames / (now - win_t0)
                win_frames = 0
                win_t0 = now

            if now - last_print >= 1.0 / args.print_hz:
                last_print = now
                e = last["eul_deg"]
                g = last["gyr"]
                print(
                    f"#{parser.frames:<8d} t={now - t0:7.2f}s {fps:6.1f}Hz  "
                    f"roll={e[0]:+7.2f} pitch={e[1]:+7.2f} yaw={e[2]:+8.2f}  "
                    f"|q|={last['qnorm']:.4f}  "
                    f"T={last['temp']:>3d}C st={last['is_static']} cal={last['is_calibrate']}  "
                    f"gyr=({g[0]:+.2f},{g[1]:+.2f},{g[2]:+.2f})  "
                    f"crc_err={parser.crc_errors} resync={parser.resync_bytes}B")
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        if csv_fp:
            csv_fp.close()
            print(f"\nCSV 已写入 {args.csv}")

    elapsed = max(time.time() - t0, 1e-6)
    total = parser.frames + parser.crc_errors + parser.len_errors
    print("\n===== 统计 =====")
    print(f"运行 {elapsed:.2f}s, 收到 {parser.frames} 帧, "
          f"平均 {parser.frames / elapsed:.1f} 帧/s")
    print(f"CRC 错 {parser.crc_errors} 帧 ({parser.crc_errors / max(total, 1) * 100:.3f}%)")
    print(f"长度字段错 {parser.len_errors} 次, 重同步丢弃 {parser.resync_bytes} 字节")
    if last:
        e = last["eul_deg"]
        print(f"最后一帧: roll={e[0]:+.2f}° pitch={e[1]:+.2f}° yaw={e[2]:+.2f}°")
    if parser.frames and abs(parser.frames / elapsed - 1000) > 100:
        print("提示: 帧率明显偏离 1kHz —— 固件 CDC_Transmit_HS 在 BUSY 时会静默丢帧，"
              "或虚拟机 USB 轮询延迟过大")
    return 0


if __name__ == "__main__":
    sys.exit(main())
