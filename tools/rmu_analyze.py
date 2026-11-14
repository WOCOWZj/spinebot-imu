#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
rmu_analyze.py -- 对 rmu_reader.py --csv 录下的姿态数据做验收分析

用法:
    python3 rmu_analyze.py ~/rmu_static.csv
    python3 rmu_analyze.py ~/rmu_static.csv --expect-hz 1000

输出四组结论:
    1) 时基/帧率: 平均帧率、ts 间隔抖动、丢帧数
    2) 静止精度: roll/pitch 均值与抖动(标准差)、四元数模长偏差
    3) 陀螺噪声: 零偏(均值)与噪声(标准差), 单位 rad/s 与 deg/s
    4) 加速度: 模长是否≈9.81, 各轴均值(判断安装方向)

依赖: 仅标准库
"""

import argparse
import csv
import math
import statistics as st
import sys


def load(path):
    rows = []
    with open(path, newline="", encoding="utf-8") as fp:
        for r in csv.DictReader(fp):
            try:
                rows.append({k: float(v) for k, v in r.items()})
            except (TypeError, ValueError):
                continue  # 跳过残缺行
    return rows


def col(rows, name):
    return [r[name] for r in rows]


def fmt(v, unit="", nd=3):
    return f"{v:+.{nd}f}{unit}"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--expect-hz", type=float, default=1000.0)
    args = ap.parse_args()

    rows = load(args.csv)
    if len(rows) < 10:
        print(f"样本太少({len(rows)} 行)，先录 10 秒以上再看")
        return 1

    ts = col(rows, "ts_ms")
    span_s = (ts[-1] - ts[0]) / 1000.0
    fps = (len(rows) - 1) / span_s if span_s > 0 else 0
    print(f"== 1) 时基 / 帧率 ==")
    print(f"  样本 {len(rows)} 行, 时长 {span_s:.2f}s, 平均 {fps:.1f} 帧/s "
          f"(期望 {args.expect_hz:.0f})")
    gaps = [b - a for a, b in zip(ts, ts[1:])]
    gaps = [g for g in gaps if g >= 0]
    if gaps:
        exp_ms = 1000.0 / args.expect_hz
        dropped = sum(max(0, round(g / exp_ms) - 1) for g in gaps)
        print(f"  ts 间隔: 中位 {st.median(gaps):.1f}ms, 最大 {max(gaps):.1f}ms, "
              f"估算丢帧 {dropped} 帧 ({dropped / max(len(rows), 1) * 100:.2f}%)")
        print(f"  抖动(间隔标准差): {st.pstdev(gaps):.2f}ms  "
              f"{'OK' if st.pstdev(gaps) < 2 * exp_ms else '偏大'}")

    # 静止段: 优先用固件自带的 is_static 标记
    static = [r for r in rows if r["is_static"] >= 0.5] or rows
    tag = "固件标记静止段" if static is not rows else "全部样本(未标记静止)"
    print(f"\n== 2) 姿态稳定性 [{tag}] ==")
    for name in ("roll_deg", "pitch_deg", "yaw_deg"):
        v = [r[name] for r in static]
        print(f"  {name:10s} mean={fmt(st.mean(v), '°', 2)}  "
              f"std={st.pstdev(v):.3f}°  range=[{min(v):+.2f}, {max(v):+.2f}]")

    qn = col(rows, "qnorm")
    dev = max(abs(x - 1.0) for x in qn)
    print(f"  四元数模长最大偏差 {dev:.2e}  {'OK' if dev < 1e-3 else '归一化异常'}")

    yaw = [r["yaw_deg"] for r in rows]
    drift = (yaw[-1] - yaw[0]) / span_s if span_s > 0 else 0
    print(f"  yaw 全程漂移 {yaw[-1] - yaw[0]:+.2f}° ({drift:+.3f}°/s)"
          f"  [无磁力计, 缓慢漂移属正常]")

    print(f"\n== 3) 陀螺: 零偏与噪声 [{tag}] ==")
    for ax in ("gx", "gy", "gz"):
        v = [r[ax] for r in static]
        print(f"  {ax}: 零偏 {fmt(st.mean(v), ' rad/s', 5)} = {st.mean(v) * 180 / math.pi:+.4f}°/s"
              f"   噪声 std {st.pstdev(v) * 180 / math.pi:.4f}°/s")

    print(f"\n== 4) 加速度 (模长应≈9.81) ==")
    norms = [math.sqrt(r["ax"] ** 2 + r["ay"] ** 2 + r["az"] ** 2) for r in static]
    print(f"  |a| mean={st.mean(norms):.3f} m/s²  std={st.pstdev(norms):.3f}"
          f"  {'OK' if abs(st.mean(norms) - 9.81) < 0.3 else '偏离 9.81, 检查量程/标定'}")
    for ax in ("ax", "ay", "az"):
        v = [r[ax] for r in static]
        print(f"  {ax}: mean={fmt(st.mean(v), ' m/s²')}")
    dom = max(("ax", "ay", "az"), key=lambda a: abs(st.mean([r[a] for r in static])))
    print(f"  主重力轴 = {dom}  (据此判断安装朝向)")

    print(f"\n== 判定 ==")
    checks = [
        (abs(fps - args.expect_hz) / args.expect_hz < 0.05, "帧率偏差 <5%"),
        (st.pstdev([r["roll_deg"] for r in static]) < 0.5, "静止 roll 抖动 <0.5°"),
        (st.pstdev([r["pitch_deg"] for r in static]) < 0.5, "静止 pitch 抖动 <0.5°"),
        (abs(st.mean(norms) - 9.81) < 0.3, "重力模长偏差 <0.3"),
        (dev < 1e-3, "四元数归一化完好"),
    ]
    for ok, desc in checks:
        print(f"  [{'PASS' if ok else 'FAIL'}] {desc}")
    return 0 if all(ok for ok, _ in checks) else 2


if __name__ == "__main__":
    sys.exit(main())
