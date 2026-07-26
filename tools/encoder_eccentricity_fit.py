"""
AS5047磁気エンコーダの偏心(1次高調波)誤差を、LOG_print()が出力したCSVログから
推定するオフライン解析スクリプト。

使い方:
    Define.h の ENCODER_ECC_CALIBRATION_LOG を有効にしてビルド・書き込みし、
    ホイールが数回転する走行/テストモードを LOG_get_start()〜LOG_get_end() の
    区間で実行したのち、LOG_print() のシリアル出力をターミナルソフトでファイル保存する
    (1行9列の通常ログではなく、Encoder_R/Encoder_Lが追加された11列ログが必要)。

    python encoder_eccentricity_fit.py <保存したcsvファイル>

出力される encoder_*_ecc_amp / encoder_*_ecc_phase の値を
Core/Src/PL_encoder.c の該当定数にそのまま貼り付ける。

補正式(PL_encoder.c側と対応): corrected = raw - amp * sin(raw + phase)
"""

import argparse
import sys

import numpy as np

# LOG_print() の列順 (ENCODER_ECC_CALIBRATION_LOG有効時, 全11列):
# t, Tire_Speed, G_Motor_V_Target, Gyro_z, G_Motor_W_Target,
# Sensor_L, Sensor_R, Motor_Voltage_L, Motor_Voltage_R, Encoder_R, Encoder_L
COL_ENCODER_R = -2
COL_ENCODER_L = -1


def load_csv(path):
    rows = []
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                values = [float(v) for v in line.split(",")]
            except ValueError:
                continue  # 文字化けした行やヘッダ行はスキップ
            rows.append(values)
    if not rows:
        raise ValueError(f"{path} から有効な数値行を読み取れませんでした")
    ncols = len(rows[0])
    rows = [r for r in rows if len(r) == ncols]
    return np.array(rows)


def unwrap_deg(angle_deg):
    return np.degrees(np.unwrap(np.radians(angle_deg)))


def estimate_period_samples(unwrapped):
    """連続角度が360度を跨いだサンプル間隔の中央値から、1回転あたりのサンプル数を推定する"""
    revolution = np.floor(unwrapped / 360.0)
    crossings = np.where(np.diff(revolution) != 0)[0]
    if len(crossings) < 2:
        return None
    return int(round(np.median(np.diff(crossings))))


def find_constant_speed_segment(unwrapped, period_samples, tol_ratio=0.08):
    """走行ログの中から角速度がほぼ一定な区間(定常走行/巡航区間)を自動検出する。

    トラップ台形プロファイルの加減速区間は角速度が変化するため偏心誤差の
    切り分けが難しい。角速度がほぼ一定な区間だけを使うことで、トレンド除去を
    多項式フィットではなく単純な線形フィットにでき、加減速の影響を排除できる。
    """
    n = len(unwrapped)
    velocity = np.diff(unwrapped, prepend=unwrapped[0])

    window = max(3, period_samples if period_samples else n // 10)
    if window % 2 == 0:
        window += 1
    pad = window // 2
    padded = np.pad(velocity, pad, mode="edge")
    smoothed_velocity = np.convolve(padded, np.ones(window) / window, mode="valid")

    nonzero = smoothed_velocity[np.abs(smoothed_velocity) > 1e-6]
    if len(nonzero) == 0:
        return slice(0, n)
    typical_speed = float(np.median(nonzero))
    tol = tol_ratio * abs(typical_speed)
    mask = np.abs(smoothed_velocity - typical_speed) < tol

    best_start = best_len = cur_start = cur_len = 0
    for i, ok in enumerate(mask):
        if ok:
            if cur_len == 0:
                cur_start = i
            cur_len += 1
            if cur_len > best_len:
                best_start, best_len = cur_start, cur_len
        else:
            cur_len = 0
    if best_len == 0:
        return slice(0, n)
    return slice(best_start, best_start + best_len)


def fit_first_harmonic(raw_deg):
    """raw_deg: 0-360度の生角度時系列。1次高調波(振幅amp・位相phase[deg])を返す。

    corrected = raw - amp * sin(raw + phase) というPL_encoder.c側の補正式に
    対応する amp, phase を最小二乗的に推定する。

    手順:
    1. 生角度を連続角度に変換(ラップ解除)。
    2. 角速度がほぼ一定な区間(加減速の影響を受けない巡航区間)を自動検出。
    3. その区間内だけ線形(=等速)トレンドを引き、残差(偏心リップル)を抽出。
    4. 残差を生角度基準の sin/cos に投影し振幅・位相を求める。
    """
    unwrapped = unwrap_deg(raw_deg)
    period_samples = estimate_period_samples(unwrapped)
    if period_samples is None or period_samples < 4:
        raise ValueError(
            "ログ中に1回転分のデータが検出できませんでした。"
            "もっと長く(2回転以上)ホイールを回してからログを取得してください。"
        )

    segment = find_constant_speed_segment(unwrapped, period_samples)
    revolutions_in_segment = (unwrapped[segment][-1] - unwrapped[segment][0]) / 360.0
    if revolutions_in_segment < 2:
        raise ValueError(
            f"等速区間が{revolutions_in_segment:.1f}回転分しか検出できませんでした"
            "(2回転以上推奨)。ログの取得時間を延ばすか、"
            "より一定速度で走行/回転させてください。"
        )

    t_all = np.arange(len(unwrapped), dtype=float)
    t_seg = t_all[segment]
    u_seg = unwrapped[segment]
    raw_seg = raw_deg[segment]

    coeffs = np.polyfit(t_seg, u_seg, 1)
    trend = np.polyval(coeffs, t_seg)
    residual = u_seg - trend

    theta = np.radians(raw_seg)
    a = 2.0 * np.mean(residual * np.sin(theta))
    b = 2.0 * np.mean(residual * np.cos(theta))

    amp = float(np.hypot(a, b))
    phase = float(np.degrees(np.arctan2(b, a)))
    return amp, phase, residual, theta, segment, revolutions_in_segment


def report(name, raw_deg):
    amp, phase, residual, theta, segment, revs = fit_first_harmonic(raw_deg)
    predicted = amp * np.sin(theta + np.radians(phase))
    residual_after = residual - predicted
    print(f"--- {name} ---")
    print(f"  等速区間: サンプル{segment.start}〜{segment.stop} ({revs:.1f}回転分)")
    print(f"  補正前リップル RMS: {np.sqrt(np.mean(residual ** 2)):.4f} deg")
    print(f"  補正後リップル RMS: {np.sqrt(np.mean(residual_after ** 2)):.4f} deg")
    print(f"  amp   = {amp:.4f}")
    print(f"  phase = {phase:.4f}")
    return amp, phase, residual, residual_after, theta


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", help="LOG_print()の出力を保存したCSVファイル")
    parser.add_argument("--plot", action="store_true", help="補正前後のリップルを可視化する")
    args = parser.parse_args()

    data = load_csv(args.csv)
    if data.shape[1] < 11:
        print(
            f"警告: 列数が{data.shape[1]}しかありません。"
            "ENCODER_ECC_CALIBRATION_LOG を有効にしてログを取り直してください。",
            file=sys.stderr,
        )
        sys.exit(1)

    encoder_r = data[:, COL_ENCODER_R]
    encoder_l = data[:, COL_ENCODER_L]

    try:
        amp_r, phase_r, res_r, res_r_after, theta_r = report("Encoder_R", encoder_r)
        amp_l, phase_l, res_l, res_l_after, theta_l = report("Encoder_L", encoder_l)
    except ValueError as e:
        print(f"エラー: {e}", file=sys.stderr)
        sys.exit(1)

    print("\n--- PL_encoder.c に貼り付ける定数 ---")
    print(f"float encoder_R_ecc_amp = {amp_r:.4f}f, encoder_R_ecc_phase = {phase_r:.4f}f;")
    print(f"float encoder_L_ecc_amp = {amp_l:.4f}f, encoder_L_ecc_phase = {phase_l:.4f}f;")

    if args.plot:
        try:
            import matplotlib.pyplot as plt
        except ImportError:
            print("\nmatplotlibが見つからないため--plotはスキップします", file=sys.stderr)
            return

        fig, axes = plt.subplots(2, 1, figsize=(8, 6), sharex=False)
        for ax, name, theta, before, after in (
            (axes[0], "Encoder_R", theta_r, res_r, res_r_after),
            (axes[1], "Encoder_L", theta_l, res_l, res_l_after),
        ):
            order = np.argsort(theta)
            ax.scatter(np.degrees(theta)[order], before[order], s=2, label="before")
            ax.scatter(np.degrees(theta)[order], after[order], s=2, label="after")
            ax.set_title(name)
            ax.set_xlabel("raw angle [deg]")
            ax.set_ylabel("ripple [deg]")
            ax.legend()
        fig.tight_layout()
        plt.show()


if __name__ == "__main__":
    main()
