import pandas as pd
import matplotlib.pyplot as plt
import argparse
import os

plt.rcParams['font.sans-serif'] = ['Times New Roman']

def main(csv_path):
    # 读取 CSV 数据
    df = pd.read_csv(csv_path)

    # 删除前五秒的row
    df = df[df["t"] > 5]

    time = df['t']
    
    # 找出所有距离误差列（dij_error）
    dij_columns = [col for col in df.columns if col.endswith('_err')]
    o_error = df['o_error']
    o_error_deg = df['o_error_deg']

    # 获取输出路径
    output_dir = os.path.dirname(csv_path)
    distance_plot_path = os.path.join(output_dir, 'distance_errors.png')
    orientation_plot_path = os.path.join(output_dir, 'orientation_error.png')
    orientation_deg_plot_path = os.path.join(output_dir, 'orientation_error_deg.png')

    # 图1：距离误差
    plt.figure(figsize=(4,3))
    for col in dij_columns:
        plt.plot(time, df[col], label=col, linewidth=0.7, alpha=0.7)
    plt.xlabel("Time (s)")
    plt.ylabel("Distance Error (m)")

    plt.grid(True, linestyle='--', linewidth=0.7, alpha=0.7)
    plt.tick_params(axis='both', which='major', direction='in')

    plt.savefig(distance_plot_path, dpi=300, bbox_inches='tight', pad_inches=0.1)
    plt.close()
    print(f"Saved plots to:\n  {distance_plot_path}")

    # 图2：方向误差(norm+deg)
    plt.figure(figsize=(4,3))
    plt.plot(time, o_error, color='blue', label='o_error', linewidth=0.8)
    plt.xlabel("Time (s)")
    plt.ylabel("Orientation Error")

    plt.grid(True, linestyle='--', alpha=0.7)
    plt.tick_params(axis='both', which='major', direction='in')

    plt.savefig(orientation_plot_path, dpi=300, bbox_inches='tight', pad_inches=0.1)
    plt.close()
    print(f"  {orientation_plot_path}")
    # next
    plt.figure(figsize=(4,3))
    plt.plot(time, o_error_deg, color='blue', label='o_error', linewidth=0.8)
    plt.xlabel("Time (s)")
    plt.ylabel("Orientation Error (deg)")

    plt.grid(True, linestyle='--', alpha=0.7)
    plt.tick_params(axis='both', which='major', direction='in')

    plt.savefig(orientation_deg_plot_path, dpi=300, bbox_inches='tight', pad_inches=0.1)
    plt.close()
    print(f"  {orientation_deg_plot_path}")



    # 图3： 画稳态误差，画图用原始数据，计算均值和标准差时取绝对值（所有数据合在一起）
    mask = (time >= 12) & (time <= 25)
    plt.figure(figsize=(4,3))

    mean_list = []
    std_list = []
    for col in dij_columns:
        segment = df.loc[mask, col]
        abs_segment = segment.abs()
        mean = abs_segment.mean()
        std = segment.std()
        mean_list.append(mean)
        std_list.append(std)
        plt.plot(time[mask], segment, label=col, linewidth=0.7, alpha=0.7)
    mean_val = sum(mean_list) / len(mean_list)
    std_val = sum(std_list) / len(std_list)

    plt.xlabel("Time (s)")
    plt.ylabel("Distance Error (m)")
    plt.title(f"Mean: {mean_val:.3f} / Std: {std_val:.3e}", fontsize=10)
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.tick_params(axis='both', which='major', direction='in')

    segment_plot_path = os.path.join(output_dir, 'distance_errors_steady.png')
    plt.savefig(segment_plot_path, dpi=300, bbox_inches='tight', pad_inches=0.1)
    plt.close()
    print(f"  {segment_plot_path}")

    # 图4：实时频率
    # plt.figure(figsize=(4,3))
    # plt.plot(time, df['freq'], color='green', label='Frequency', linewidth=0.7, alpha=0.7)
    # plt.xlabel("Time (s)")
    # plt.ylabel("Frequency (Hz)")
    # plt.grid(True, linestyle='--', alpha=0.7)
    # plt.tick_params(axis='both', which='major', direction='in')

    # freq_plot_path = os.path.join(output_dir, 'frequency.png')
    # plt.savefig(freq_plot_path, dpi=300, bbox_inches='tight', pad_inches=0.1)
    # plt.close()
    # print(f"  {freq_plot_path}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Plot formation control errors from CSV.")
    parser.add_argument("csv_path", help="Path to the CSV file containing error data.")
    args = parser.parse_args()
    main(args.csv_path)
