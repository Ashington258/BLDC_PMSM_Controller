import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt, hilbert


# ------------------------
# Matplotlib 中文与负号支持
# ------------------------
plt.rcParams["font.sans-serif"] = ["SimHei"]  # 中文字体（黑体）
plt.rcParams["axes.unicode_minus"] = False  # 正常显示负号


# ------------------------
# 工具函数
# ------------------------
def read_scope_csv(filename: str):
    """读取示波器 CSV，跳过元数据，返回 t, v"""
    with open(filename, "r") as f:
        lines = f.readlines()
    start_idx = None
    for i, line in enumerate(lines):
        if line.strip().startswith("Second"):
            start_idx = i + 1
            break
    df = pd.read_csv(filename, skiprows=start_idx, names=["t", "v"])
    return df["t"].values, df["v"].values


def lowpass_filter(x: np.ndarray, fs: float, cutoff: float = 2000, order: int = 4):
    """低通滤波，默认截止 2 kHz"""
    nyq = fs / 2
    b, a = butter(order, cutoff / nyq, btype="low")
    return filtfilt(b, a, x)


def get_phase(x: np.ndarray, fs: float, f0: float) -> float:
    """利用 Hilbert 变换提取相位"""
    analytic = hilbert(x)
    phase = np.unwrap(np.angle(analytic))
    t = np.arange(len(x)) / fs
    phase_ref = phase - 2 * np.pi * f0 * t
    return np.mean(phase_ref)  # 平均相位


# ------------------------
# 核心流程封装
# ------------------------
def process_channels(
    files: list[str],
    cutoff_hz: float = 2000.0,
    plot_points: int | None = 2000,
    plot_percent: float | None = 20,
):
    """
    主处理流程：
    - plot_points: 指定显示的点数（优先级低）
    - plot_percent: 指定显示百分比 (0~100)，优先级高
    """
    # 读取文件
    channels: list[tuple[np.ndarray, np.ndarray]] = []
    for f in files:
        t, v = read_scope_csv(f)
        channels.append((t, v))
    t = channels[0][0]
    fs = 1.0 / (t[1] - t[0])
    print("采样率:", fs)
    filtered = []
    plt.figure(1, figsize=(12, 6))
    for i, (ti, vi) in enumerate(channels):
        v_f = lowpass_filter(vi, fs, cutoff=cutoff_hz)
        filtered.append(v_f)
        # -------- 控制显示长度 --------
        if plot_percent is not None:
            n = int(len(ti) * (plot_percent / 100.0))
        elif plot_points is not None:
            n = min(plot_points, len(ti))
        else:
            n = len(ti)  # 全量显示
        plt.plot(ti[:n], vi[:n], alpha=0.5, label=f"CH{i+1} 原始信号")
        plt.plot(ti[:n], v_f[:n], label=f"CH{i+1} 滤波后")
    plt.legend()
    plt.title("原始波形与低通后的正弦包络")
    plt.xlabel("时间 [s]")
    plt.ylabel("电压 [V]")

    # FFT 估算基波频率（以 CH1 为准）
    v0 = filtered[0]
    N = len(v0)
    freqs = np.fft.rfftfreq(N, 1.0 / fs)
    fft_mag = np.abs(np.fft.rfft(v0))
    f0 = freqs[np.argmax(fft_mag[1:]) + 1]
    print("基波频率估计:", f0, "Hz")

    plt.figure(2, figsize=(12, 6))
    plt.semilogy(freqs, fft_mag)
    plt.title("FFT 频谱 (CH1)")
    plt.xlabel("频率 [Hz]")
    plt.ylabel("幅值")

    # 计算相位
    phases = []
    for v_f in filtered:
        ph = get_phase(v_f, fs, f0)
        phases.append(ph)

    phases = np.array(phases)
    phases_deg = np.rad2deg(phases)
    phases_deg = (phases_deg - phases_deg[0]) % 360
    print("三相相对相位 [deg]:", phases_deg)

    expected = [0, 120, 240]
    for i, ph in enumerate(phases_deg):
        print(f"CH{i+1} 相位: {ph:.1f}° (期望 {expected[i]}° 附近)")

    plt.show()

    return phases_deg, f0, fs


# ------------------------
# main 入口
# ------------------------
def main():
    files = [
        # "data/wavedata/SDS824X_HD_CSV_C1_1.csv",
        # "data/wavedata/SDS824X_HD_CSV_C2_1.csv",
        # "data/wavedata/SDS824X_HD_CSV_C3_1.csv",
        "data/wavedata/svpwm_C1_1.csv",
        "data/wavedata/svpwm_C2_1.csv",
        "data/wavedata/svpwm_C3_1.csv",
    ]
    process_channels(files, cutoff_hz=2000.0, plot_points=20000)


if __name__ == "__main__":
    main()
