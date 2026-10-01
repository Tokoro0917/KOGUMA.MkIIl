#!/usr/bin/env python3
"""最短走行のターンのシミュレーター。

Core/Src/Move.c の TurnV_Table を読み込み、Short_NANAME_MoveTurnV と同じ
手順(前オフセット -> Motor_Sula_COS -> 後オフセット)で機体の軌跡を計算する。

モデル
  - 並進速度 v はターン中一定。角速度 ω(t) は Motor_Sula_COS と同じ
    cos 型の加速 -> 等角速度 -> cos 型の減速(角度が足りなければ三角形)
  - 機体の向き θ はジャイロ制御で ω(t) を完全に追従する
  - タイヤの横滑り: 進む向きは θ からスリップ角 β だけ旋回の外側にずれる。
      β[deg] = k * a_lat[m/s^2],  a_lat = v * ω (横加速度)
    (pidream「ターン調整のルーティン」の、スリップ角は遠心力 m*v*ω に
     比例するというモデル。速度の違うターンでも同じ k が使えるよう v*ω で持つ)
  - 壁切れ補正は入れていない。オフセットは「距離だけ走る」として扱う

座標: ターン開始時の機体を原点、前を +x、左を +y とする(右ターンは左の鏡像)。
理想の終点は迷路の幾何(区画180mm)で決まる:
  大回り90 / 大回り180 / 斜め入り45 / 135 は区画中心から、
  斜め出45 / 135 / V90 は斜めの壁の中点から始まる(Shortest_Pass_Compression*)

使い方 (リポジトリ直下で):
  python3 sim/turn_sim.py table [--k K]
      TurnV_Table の全ターンを、表の速度で走ったときの終点のずれ
  python3 sim/turn_sim.py sim TURN --v 2400 [--k K] [--svg out.svg]
      1つのターンの終点のずれと、ずれを消す前後オフセット
  python3 sim/turn_sim.py design TURN --v 2400 [--k K] [--min-offset 5]
      角速度・角加速度を探して、横加速度が最小になるパラメータを出す
  python3 sim/turn_sim.py fit TURN --v 2400 --measured LAT [--decel 300]
      停止位置テスト(ターン -> 直線で停止)の横ずれ実測値から k を求める
  python3 sim/turn_sim.py selftest
      解析解との比較(CIで実行)
TURN は big90 big180 in45 in135 out45 out135 v90
"""
import argparse
import math
import os
import re
import sys

V_BASE = 2000.0  # Move.c の TURNV_V_BASE
CELL = 180.0
HALF = 90.0
DIAG = HALF * math.sqrt(2)  # 127.3

# (前方, 左, 角度[deg]) 左ターンの理想の終点
IDEAL_END = {
    "big90": (CELL, CELL, 90),
    "big180": (0.0, CELL, 180),
    "in45": (CELL, HALF, 45),
    "in135": (HALF, CELL, 135),
    "out45": ((HALF + CELL) / math.sqrt(2), (CELL - HALF) / math.sqrt(2), 45),
    "out135": ((CELL - HALF) / math.sqrt(2), (CELL + HALF) / math.sqrt(2), 135),
    "v90": (DIAG, DIAG, 90),
}
NAMES = {
    "big90": "大回り90", "big180": "大回り180", "in45": "斜め入り45",
    "in135": "斜め入り135", "out45": "斜め出45", "out135": "斜め出135",
    "v90": "V90",
}
ENUM = {
    "TV_BIG90": "big90", "TV_BIG180": "big180", "TV_IN45": "in45",
    "TV_IN135": "in135", "TV_OUT45": "out45", "TV_OUT135": "out135",
    "TV_V90": "v90",
}

MOVE_C = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..",
                      "Core", "Src", "Move.c")


def load_table(path=MOVE_C):
    """Move.c の TurnV_Table を読む"""
    with open(path, encoding="utf-8", errors="replace") as f:
        src = f.read()
    pat = re.compile(r"\[(TV_\w+)\]\s*=\s*\{([^{}\[]*)\}")
    table = {}
    for m in pat.finditer(src):
        if m.group(1) not in ENUM:
            continue
        f = [s.strip() for s in m.group(2).split(",")]
        table[ENUM[m.group(1)]] = {
            "v": float(f[0]), "angle": float(f[1]),
            "w_max": float(f[2]), "w_ac": float(f[3]),
            "st_kind": f[4], "offset_st": float(f[5]),
            "end_kind": f[6], "offset_end": float(f[7]),
        }
    if len(table) != len(ENUM):
        sys.exit("Move.c の TurnV_Table を読めなかった: " + path)
    return table


def omega_profile(angle, w_max, w_ac):
    """Motor_Sula_COS の角速度 ω(t)[deg/s] を返す関数と、終了時刻"""
    x13 = math.pi / (2 * w_ac) * w_max ** 2  # 加速+減速で回る角度
    if x13 > angle:
        w_max = math.sqrt(2 * w_ac * angle / math.pi)
        x13 = angle
    t1 = math.pi * w_max / (2 * w_ac)
    t_const = (angle - x13) / w_max
    t_end = 2 * t1 + t_const

    def w(t):
        if t < t1:
            return w_max / 2 * (1 - math.cos(math.pi * t / t1))
        if t < t1 + t_const:
            return w_max
        tt = t - t1 - t_const
        if tt < t1:
            return w_max / 2 * (1 + math.cos(math.pi * tt / t1))
        return 0.0
    return w, t_end, w_max


def run_turn(v, angle, w_max, w_ac, st, end, k=0.0, dt=1e-5, trace=None):
    """左ターンを走らせて終点 (x, y, θ) を返す。trace にはリストで軌跡を入れる"""
    x = st
    y = 0.0
    th = 0.0
    if trace is not None:
        trace.append((0.0, 0.0))
        trace.append((x, y))
    w, t_end, w_pk = omega_profile(angle, w_max, w_ac)
    n = int(math.ceil(t_end / dt))
    h = t_end / n
    vm = v / 1000.0
    for i in range(n):
        tm = (i + 0.5) * h
        wd = w(tm)
        th_m = th + math.radians(wd) * h / 2
        beta = math.radians(k * vm * math.radians(wd))  # k[deg/(m/s^2)]
        phi = th_m - beta  # 外側(右)にずれる
        x += v * math.cos(phi) * h
        y += v * math.sin(phi) * h
        th += math.radians(wd) * h
        if trace is not None and i % 50 == 0:
            trace.append((x, y))
    th = math.radians(angle)  # ジャイロ制御で角度は合う
    x += end * math.cos(th)
    y += end * math.sin(th)
    if trace is not None:
        trace.append((x, y))
    a_lat = vm * math.radians(w_pk)
    return x, y, th, a_lat, w_pk


def error_in_exit_frame(name, x, y):
    """理想の終点に対するずれを、出口の向きの (前後, 横) で返す。
    横は出口の進行方向に対して左が+"""
    ex, ey, ang = IDEAL_END[name]
    dx, dy = x - ex, y - ey
    a = math.radians(ang)
    along = dx * math.cos(a) + dy * math.sin(a)
    lat = -dx * math.sin(a) + dy * math.cos(a)
    return along, lat


def scaled(p, v):
    s = v / V_BASE
    return p["w_max"] * s, p["w_ac"] * s * s


def solve_offsets(name, v, angle, w_max, w_ac, k, dt=1e-5):
    """終点が理想の線上に来るように前後オフセットを解く。
    180度は前後が平行で解けないので、前オフセットを固定して後ろを解き、
    横ずれは残る(角速度で合わせる)"""
    ax, ay, _, _, _ = run_turn(v, angle, w_max, w_ac, 0.0, 0.0, k, dt)
    ex, ey, ang = IDEAL_END[name]
    a = math.radians(ang)
    # st*(1,0) + (ax,ay) + end*(cos a, sin a) = (ex, ey)
    det = math.sin(a)
    if abs(det) < 1e-6:
        return None, None
    end = (ey - ay) / det
    st = (ex - ax) - end * math.cos(a)
    return st, end


def st_run(p, offset_st=None):
    """前オフセットのうち、壁切れ検出の後に走る距離"""
    x = p["offset_st"] if offset_st is None else offset_st
    return 0.5 * x if p["st_kind"] == "WC_NORMAL" else x


def wallcut_pos(table, name, k):
    """表のオフセットが壁切れ込みで合っているとして、壁切れ検出点の位置を逆算する。
    180度は解けないので、同じ通常の壁切れを使うターンの平均を使う"""
    p = table[name]
    w_max, w_ac = scaled(p, p["v"])
    st, _ = solve_offsets(name, p["v"], p["angle"], w_max, w_ac, k, dt=1e-4)
    if st is not None:
        return st - st_run(p)
    ds = [wallcut_pos(table, n, k) for n in ("big90", "in45", "in135")]
    return sum(ds) / len(ds)


def describe(name, p, v, k, show_fix=True):
    w_max, w_ac = scaled(p, v)
    x, y, th, a_lat, w_pk = run_turn(v, p["angle"], w_max, w_ac,
                                     p["offset_st"], p["offset_end"], k)
    along, lat = error_in_exit_frame(name, x, y)
    line = "%-10s v=%5.0f  ω=%5.0f deg/s  横加速度=%5.1f m/s^2  終点のずれ: 前後%+6.1f 横%+6.1f mm" % (
        NAMES[name], v, w_pk, a_lat, along, lat)
    if show_fix:
        st, end = solve_offsets(name, v, p["angle"], w_max, w_ac, k)
        if st is None:
            line += "  (180度は横ずれを角速度で合わせる)"
        else:
            line += "  -> 合うオフセット: 前%6.1f 後%6.1f (表: %g / %g)" % (
                st, end, p["offset_st"], p["offset_end"])
            # Motor_Wallcut_ST は X/2 走ってから壁切れを待ち、残り X/2 を走る。
            # Motor_Wallcut_ST_NANAME は壁切れを待ってから X を走る。
            # 合うオフセットからその分を引くと、壁切れが起きる位置が逆算できる
            line += "  前の壁切れ位置(逆算) %5.1f" % (st - st_run(p))
    return line


def write_svg(path, name, runs):
    """runs: [(ラベル, 色, 軌跡)]"""
    pts = [pt for _, _, tr in runs for pt in tr]
    ex, ey, _ = IDEAL_END[name]
    a = math.radians(IDEAL_END[name][2])
    pts += [(0, 0), (ex, ey), (ex + 60 * math.cos(a), ey + 60 * math.sin(a))]
    pad = 40
    xmin = min(p[0] for p in pts) - pad
    xmax = max(p[0] for p in pts) + pad
    ymin = min(p[1] for p in pts) - pad
    ymax = max(p[1] for p in pts) + pad
    sc = 2.0
    W = (xmax - xmin) * sc
    H = (ymax - ymin) * sc

    def tx(p):
        return ((p[0] - xmin) * sc, (ymax - p[1]) * sc)
    out = ['<svg xmlns="http://www.w3.org/2000/svg" width="%.0f" height="%.0f" '
           'style="background:#fff;font-family:sans-serif">' % (W, H + 20 * len(runs) + 10)]
    # 半区画グリッド
    g0 = math.floor(xmin / HALF) * HALF
    while g0 <= xmax:
        a, b = tx((g0, ymin)), tx((g0, ymax))
        out.append('<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" stroke="#ddd"/>' % (a + b))
        g0 += HALF
    g0 = math.floor(ymin / HALF) * HALF
    while g0 <= ymax:
        a, b = tx((xmin, g0)), tx((xmax, g0))
        out.append('<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" stroke="#ddd"/>' % (a + b))
        g0 += HALF
    # 理想の入口の線と出口の線(破線)
    ea = math.radians(IDEAL_END[name][2])
    for (px, py, a) in [(0, 0, 0.0), (ex, ey, ea)]:
        p1 = tx((px - 150 * math.cos(a), py - 150 * math.sin(a)))
        p2 = tx((px + 150 * math.cos(a), py + 150 * math.sin(a)))
        out.append('<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" stroke="#36c" '
                   'stroke-dasharray="6 4"/>' % (p1 + p2))
    for (px, py) in [(0, 0), (ex, ey)]:
        c = tx((px, py))
        out.append('<circle cx="%.1f" cy="%.1f" r="4" fill="#000"/>' % c)
    for i, (label, color, tr) in enumerate(runs):
        d = " ".join("%.1f,%.1f" % tx(p) for p in tr)
        out.append('<polyline points="%s" fill="none" stroke="%s" stroke-width="2"/>' % (d, color))
        out.append('<text x="10" y="%.0f" fill="%s" font-size="14">%s</text>' % (
            H + 20 * (i + 1), color, label))
    out.append("</svg>")
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(out))


def cmd_table(args):
    table = load_table()
    print("k = %g deg/(m/s^2)  (壁切れ補正なし、距離だけで走った場合)" % args.k)
    print("前の壁切れ位置(逆算): 表のオフセットが壁切れ込みで合っているとしたときの、")
    print("  ターン開始点から壁切れ検出点までの距離。同じ種類の壁切れでそろえば幾何が合っている")
    for name in IDEAL_END:
        p = table[name]
        print(describe(name, p, p["v"], args.k))


def cmd_sim(args):
    table = load_table()
    p = table[args.turn]
    v = args.v if args.v else p["v"]
    print(describe(args.turn, p, v, args.k))
    if args.k != 0:
        print("(参考) k=0: " + describe(args.turn, p, v, 0.0, show_fix=False))
    if args.svg:
        runs = []
        for k, color in [(0.0, "#888"), (args.k, "#c00")]:
            tr = []
            w_max, w_ac = scaled(p, v)
            run_turn(v, p["angle"], w_max, w_ac, p["offset_st"], p["offset_end"], k, trace=tr)
            runs.append(("k=%g (青の破線が理想の入口・出口の線)" % k, color, tr))
        write_svg(args.svg, args.turn, runs)
        print("軌跡: " + args.svg)


def cmd_design(args):
    table = load_table()
    p = table[args.turn]
    v = args.v if args.v else p["v"]
    angle = p["angle"]
    ex, ey, _ = IDEAL_END[args.turn]
    # 実機は壁切れを検出してから曲がり始めるので、それより手前では曲がれない
    d_cut = wallcut_pos(table, args.turn, args.k)
    min_st = d_cut + args.min_offset
    best = None
    # 角速度を下げる(=横加速度を下げる)ほど旋回半径が大きくなり、
    # 前後オフセットが負になって入らなくなる。その境目を探す
    # 角加速度はモーターのトルクで頭打ちになる(速度によらない)ので上限を付ける。
    # 上限なしだと「一瞬で角速度を立てる」非現実的な解になる
    for w in range(200, 6001, 25):
        for wac_k in range(5, int(args.wac_max / 1000) + 1, 5):
            w_ac = wac_k * 1000.0
            if args.turn == "big180":
                ax, ay, _, a_lat, w_pk = run_turn(v, angle, w, w_ac, 0, 0, args.k, dt=2e-4)
                if abs(ay - ey) > 1.0:
                    continue
                st = min_st
                end = st + ax  # x: st + ax - end = 0
                if end < args.min_offset:
                    end = args.min_offset
                    st = end - ax
            else:
                st, end = solve_offsets(args.turn, v, angle, w, w_ac, args.k, dt=2e-4)
                _, _, _, a_lat, w_pk = run_turn(v, angle, w, w_ac, 0, 0, 0.0, dt=1e-3)
            if st < min_st - 1e-6 or end < args.min_offset:
                continue
            if best is None or a_lat < best[0] - 1e-9:
                best = (a_lat, w, w_ac, st, end, w_pk)
    if best is None:
        print("%s v=%.0f: 条件を満たすパラメータが見つからない。" % (NAMES[args.turn], v))
        print("  壁切れ(%.1f mm)+%.0f mm から曲がり始めて、角加速度 %.0f 以下では理想の線に乗れない。" % (
            d_cut, args.min_offset, args.wac_max))
        print("  --wac-max を上げる(モーターが出せるか要確認)か、--min-offset を下げる")
        return
    a_lat, w, w_ac, st, end, w_pk = best
    st, end = (st, end) if args.turn == "big180" else solve_offsets(
        args.turn, v, angle, w, w_ac, args.k)  # 細かい刻みで解き直す
    s = v / V_BASE
    print("%s v=%.0f k=%g: 横加速度が最小になるパラメータ" % (NAMES[args.turn], v, args.k))
    print("  ω_max=%.0f deg/s (実際のピーク %.0f)  ω_ac=%.0f deg/s^2  前%.1f 後%.1f mm  横加速度 %.1f m/s^2" % (
        w, w_pk, w_ac, st, end, a_lat))
    run = st - d_cut
    offset_st = 2 * run if p["st_kind"] == "WC_NORMAL" else run
    print("  前の壁切れ位置(表から逆算) %.1f mm -> offset_st = %.0f" % (d_cut, offset_st))
    print("  TurnV_Table に書く値(V_BASE=2000 に換算): w_max=%.0f w_ac=%.0f offset_st=%.0f offset_end=%.0f(距離)" % (
        w / s, w_ac / (s * s), offset_st, end))
    print("  ※ offset_end は距離だけで走った値。後ろの壁切れで早めに止まる分は実機で合わせる")
    print("  (参考) 今の表の値をこの速度に換算したとき:")
    print("    " + describe(args.turn, p, v, args.k))


def cmd_fit(args):
    """停止位置テスト: ターンのあと出口の向きに直線を走って止まる。
    角度はジャイロで合うので、出口の横ずれがそのまま停止位置の横ずれになる"""
    table = load_table()
    p = table[args.turn]
    v = args.v if args.v else p["v"]
    w_max, w_ac = scaled(p, v)

    def lat_at(k):
        x, y, _, _, _ = run_turn(v, p["angle"], w_max, w_ac,
                                 p["offset_st"], p["offset_end"], k, dt=1e-4)
        return error_in_exit_frame(args.turn, x, y)[1]

    lat0 = lat_at(0.0)
    target = lat0 + args.measured if args.relative else args.measured
    lo, hi = 0.0, 5.0
    f_lo, f_hi = lat_at(lo) - target, lat_at(hi) - target
    if f_lo * f_hi > 0:
        print("k=0〜5 の範囲で合わない(横ずれ k=0: %+.1f, k=5: %+.1f, 目標 %+.1f)" % (
            lat_at(lo), lat_at(hi), target))
        return
    for _ in range(50):
        mid = (lo + hi) / 2
        f = lat_at(mid) - target
        if f * f_lo > 0:
            lo, f_lo = mid, f
        else:
            hi = mid
    k = (lo + hi) / 2
    print("%s v=%.0f: 横ずれ %+.1f mm になる k = %.4f deg/(m/s^2)" % (
        NAMES[args.turn], v, target, k))
    print("  (k=0 での横ずれ %+.1f mm。外側が負)" % lat0)


def cmd_selftest(args):
    """解析解と比べてシミュレーターの計算を確かめる(CI用)"""
    fails = []
    v = 2000.0
    # 角加速度が無限大なら等角速度の円弧になり、半径が合えばオフセットは0
    for name, r in [("big90", CELL), ("v90", DIAG), ("big180", HALF)]:
        w = math.degrees(v / r)
        ang = IDEAL_END[name][2]
        x, y, _, _, _ = run_turn(v, ang, w, 1e12, 0.0, 0.0)
        along, lat = error_in_exit_frame(name, x, y)
        if abs(along) > 0.1 or abs(lat) > 0.1:
            fails.append("%s: 半径%.1fの円弧で終点がずれる (%.2f, %.2f)" % (name, r, along, lat))
    # 解いたオフセットで走ると理想の終点に乗る
    table = load_table()
    for name, p in table.items():
        w_max, w_ac = scaled(p, p["v"])
        st, end = solve_offsets(name, p["v"], p["angle"], w_max, w_ac, 0.1)
        if st is None:
            continue
        x, y, _, _, _ = run_turn(p["v"], p["angle"], w_max, w_ac, st, end, 0.1)
        along, lat = error_in_exit_frame(name, x, y)
        if abs(along) > 0.1 or abs(lat) > 0.1:
            fails.append("%s: 解いたオフセットで終点がずれる (%.2f, %.2f)" % (name, along, lat))
    # 横滑りで外側(右)に膨らむ
    p = table["big90"]
    x0, y0, _, _, _ = run_turn(v, 90, p["w_max"], p["w_ac"], 0, 0, 0.0)
    x1, y1, _, _, _ = run_turn(v, 90, p["w_max"], p["w_ac"], 0, 0, 0.2)
    if not (x1 > x0 and y1 < y0):
        fails.append("横滑りの向きが逆")
    # 角度が足りないときは三角形の角速度になり、指定角度ちょうど回る
    w, t_end, w_pk = omega_profile(45, 3000, 50000)
    n = 20000
    tot = sum(w((i + 0.5) * t_end / n) for i in range(n)) * t_end / n
    if abs(tot - 45) > 0.01 or w_pk >= 3000:
        fails.append("三角形の角速度の積分が %.3f deg" % tot)
    for f in fails:
        print("FAIL: " + f)
    print("OK" if not fails else "%d件の失敗" % len(fails))
    sys.exit(1 if fails else 0)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    turns = list(IDEAL_END)

    a = sub.add_parser("table")
    a.add_argument("--k", type=float, default=0.0)
    a.set_defaults(func=cmd_table)

    a = sub.add_parser("sim")
    a.add_argument("turn", choices=turns)
    a.add_argument("--v", type=float)
    a.add_argument("--k", type=float, default=0.0)
    a.add_argument("--svg")
    a.set_defaults(func=cmd_sim)

    a = sub.add_parser("design")
    a.add_argument("turn", choices=turns)
    a.add_argument("--v", type=float)
    a.add_argument("--k", type=float, default=0.0)
    a.add_argument("--min-offset", type=float, default=5.0,
                   help="壁切れ後に走る距離と後オフセットの下限[mm]")
    a.add_argument("--wac-max", type=float, default=130000.0,
                   help="角加速度の上限[deg/s^2]。既定は今の表で使っている最大値")
    a.set_defaults(func=cmd_design)

    a = sub.add_parser("fit")
    a.add_argument("turn", choices=turns)
    a.add_argument("--v", type=float)
    a.add_argument("--measured", type=float, required=True,
                   help="停止位置の横ずれ[mm]。出口の進行方向に対して左が+(外側に膨らむと負)")
    a.add_argument("--relative", action="store_true",
                   help="--measured を k=0 のシミュレーション結果からの差として扱う")
    a.set_defaults(func=cmd_fit)

    a = sub.add_parser("selftest")
    a.set_defaults(func=cmd_selftest)

    args = ap.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
